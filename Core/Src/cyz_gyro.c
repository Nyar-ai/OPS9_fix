/**
  ******************************************************************************
  * @file    cyz_gyro.c
  * @brief   CY-Z (QMM01) gyro access layer - USART1 binding and interrupt rx.
  *
  * This is the only file that touches HAL UART objects for the gyro. All frame
  * knowledge stays in cyz_protocol.c, which is pure C and unit tested on a PC
  * (tests/cyz_protocol_test.c); here one received byte is handed to it. What the
  * rest of the firmware gets from this file:
  *   - the two values the receive interrupt publishes (latest telemetry sample,
  *     last module reply),
  *   - the report-rate negotiation: a polled state machine that asks for 200 Hz
  *     and walks down 100 -> 50 Hz until both confirmation channels agree (T4),
  *   - the PC13 field indicator: three states, decided from the arrival time of
  *     the last valid frame (T5).
  * No printf redirection, no task, no queue - the periodic work rides on the 1 ms
  * task that already exists (StartDefaultTask in Core/Src/freertos.c).
  ******************************************************************************
  */

#include "cyz_gyro.h"
#include "cyz_protocol.h" /* pure-C protocol layer: parsing is all it does here */

/* The gyro link, bound by cyz_gyro_init(). NULL until then, so a callback that
   arrives before init is ignored instead of dereferencing a wild handle. */
static UART_HandleTypeDef *s_uart;

/* One byte in flight: HAL fills it, the callback moves it to the parser. */
static uint8_t s_rx_byte;

/* Parser state machine, fed one byte per receive interrupt. */
static cyz_proto_t s_parser;

/* Last completed frame, produced by the parser and dispatched by type below. */
static cyz_proto_frame_t s_frame;

/* Latest telemetry sample, published by the receive interrupt with a seqlock.
   Three words cannot ride on one single-copy atomic access the way the reply
   word below does, so the tick goes odd while they are copied in and even again
   once the sample is whole: a reader that picked up a mixed copy sees the tick
   move and retries. The writer never waits for a reader and the reader never
   masks interrupts, which is what keeps a read out of the 86.8 us byte budget.
   tick == 0 means "nothing published yet"; after that it advances by two. */
static volatile cyz_gyro_sample_t s_sample;
static volatile uint32_t s_sample_tick;

/* Last reply from the module (ACK / scale response), packed into one aligned
   32-bit word: kind << 24 | cmd << 16 | result << 8 | seq. The interrupt stores
   that word and a reader loads it - a single-copy atomic access on Cortex-M3,
   so the four fields can never be seen half updated and no lock or retry is
   needed. A stored reply always has a non-zero kind, so word == 0 means
   "nothing yet". */
static volatile uint32_t s_reply_word;

/* Telemetry frames published since reset. The rate negotiation measures its
   secondary confirmation over a window of these - telemetry only, because an ACK
   is not a reported sample. Written by the receive interrupt. */
static volatile uint32_t s_telemetry_frames;

/* HAL_GetTick() of the last valid frame of any type, 0 until one arrives: what
   the PC13 indicator decides from. The receive interrupt only stamps it. */
static volatile uint32_t s_last_rx_ms;

/* Bumped after every stored reply, so a task can tell "the module answered
   something new" by watching this counter instead of clearing the reply word -
   which would race with the interrupt that writes it. */
static volatile uint32_t s_reply_serial;

/* ---- report-rate negotiation (private to this file, T4) ---------------- */
/* Every number below is a bound rather than a guess:
   - 8 bytes on the wire at 115200 8N1 take 8 x 10 / 115200 = 0.69 ms, so a 10 ms
     transmit timeout is two orders of magnitude of head room;
   - the module answers within one report period (5 ms at 200 Hz), so a 100 ms ACK
     window leaves room for a busy module and still bounds the whole chain;
   - the secondary confirmation counts telemetry over one second - about 200
     frames at 200 Hz, so its resolution is one frame per second.
   Worst case (module absent) the three rungs cost 3 x (two ACK windows + 1 s) in
   the polling task, and nothing else is delayed by it. */
#define CYZ_GYRO_TX_TIMEOUT_MS        10u
#define CYZ_GYRO_ACK_WINDOW_MS       100u
#define CYZ_GYRO_MEASURE_WINDOW_MS  1000u

typedef enum
{
  CYZ_NEG_SEND_SET = 0, /* ask for the gear being tried */
  CYZ_NEG_WAIT_SET_ACK,
  CYZ_NEG_SEND_QUERY,   /* primary confirmation channel: read the gear back */
  CYZ_NEG_WAIT_QUERY_ACK,
  CYZ_NEG_MEASURE,      /* secondary confirmation channel: count frames */
  CYZ_NEG_DONE
} cyz_gyro_neg_stage_t;

/* Only the polling task touches these, so they stay plain data: no lock, and the
   receive interrupt never writes any of them. */
static cyz_gyro_rate_report_t s_rate;
static cyz_gyro_neg_stage_t   s_neg_stage;           /* CYZ_NEG_SEND_SET after init */
static uint8_t                s_neg_target;          /* gear the current attempt asks for */
static uint8_t                s_cmd_seq;             /* sequence of the command in flight */
static uint32_t               s_neg_wait_ms;         /* start of the window being waited on */
static uint32_t               s_neg_reply_seen;      /* reply serial snapshot */
static uint32_t               s_neg_frames_at_start; /* telemetry count at window start */

/* ---- PC13 field indicator (private to this file, T5) ------------------- */
/* 1 Hz means a full second per cycle: 500 ms lit, 500 ms dark. The decision
   itself lives in the protocol layer (cyz_proto_link_state), where a PC test
   pins all three states; this layer only drives the pin. */
#define CYZ_GYRO_LED_BLINK_HALF_MS 500u

static cyz_proto_link_state_t s_led_state; /* CYZ_LINK_OFF after reset */
static uint32_t               s_led_blink_ms;
static uint8_t                s_led_level;

/* Publishes one decoded telemetry frame. Runs in the receive interrupt. */
static void cyz_gyro_publish_sample(const cyz_proto_telemetry_t *telemetry)
{
  s_sample_tick++; /* odd: a reader copying right now will retry */
  s_sample.angle_deg = telemetry->angle_deg;
  s_sample.gyro_dps  = telemetry->gyro_dps;
  s_sample.seq       = telemetry->seq;
  s_sample_tick++; /* even again: those three fields are one frame's sample */

  /* Telemetry only: this is what the rate negotiation measures, and an ACK must
     never count as a reported sample. */
  s_telemetry_frames++;
}

/* Stores one reply. Runs in the receive interrupt. */
static void cyz_gyro_store_reply(cyz_gyro_reply_kind_t kind, uint8_t cmd,
                                 uint8_t result, uint8_t seq)
{
  s_reply_word = ((uint32_t)kind << 24) | ((uint32_t)cmd << 16) |
                 ((uint32_t)result << 8) | (uint32_t)seq;

  /* Word first, then the serial: a task that sees a new serial is guaranteed to
     read the new word. */
  s_reply_serial++;
}

/* Handle dispatch: only the gyro link's own handle is ours. Keeps the USART2
   robot link (a different handle) out of this file entirely, and makes an event
   that arrives before init harmless. */
static uint8_t cyz_gyro_is_own_handle(const UART_HandleTypeDef *huart)
{
  return (uint8_t)((s_uart != NULL) && (huart == s_uart));
}

/* Arms exactly one byte of interrupt reception.
   Precondition: s_uart is bound (cyz_gyro_init() has run). Must be called on
   every completed or aborted reception - a missed re-arm silences the link
   until the next reset, with no way to tell it from an unplugged module. */
static void cyz_gyro_arm(void)
{
  (void)HAL_UART_Receive_IT(s_uart, &s_rx_byte, 1u);
}

void cyz_gyro_init(void)
{
  uint8_t i;

  s_uart    = &huart1; /* the gyro link; huart2 stays untouched */
  s_rx_byte = 0u;

  cyz_proto_init(&s_parser);

  /* Cleared so a debugger sees zeros before the first frame arrives; what
     actually gates a read is the tick, 0 until a frame is published. */
  s_sample.angle_deg = 0.0f;
  s_sample.gyro_dps  = 0.0f;
  s_sample.seq       = 0u;
  s_sample_tick      = 0u;
  s_reply_word       = 0u;
  s_frame.type = CYZ_FRAME_NONE;

  /* Everything the two periodic jobs below read starts from its documented zero
     state: no frame seen yet, no reply serial, nothing negotiated. */
  s_telemetry_frames = 0u;
  s_last_rx_ms       = 0u;
  s_reply_serial     = 0u;

  s_rate.state       = CYZ_GYRO_RATE_IDLE;
  s_rate.attempts    = 0u;
  s_rate.final_code  = (uint8_t)CYZ_GYRO_RATE_NONE;
  s_rate.measured_hz = 0u;
  for (i = 0u; i < CYZ_GYRO_RATE_ATTEMPTS; i++)
  {
    s_rate.log[i].code        = 0u;
    s_rate.log[i].ack_result  = 0u;
    s_rate.log[i].readback    = 0u;
    s_rate.log[i].measured_hz = 0u;
  }

  s_neg_stage           = CYZ_NEG_SEND_SET;
  s_neg_target          = (uint8_t)CYZ_RATE_200HZ;
  s_cmd_seq             = 0u;
  s_neg_wait_ms         = 0u;
  s_neg_reply_seen      = 0u;
  s_neg_frames_at_start = 0u;

  /* The indicator starts where the pin already is after reset (MX_GPIO_Init
     drives PC13 low): nothing has arrived yet, so "dark". */
  s_led_state    = CYZ_LINK_OFF;
  s_led_level    = 0u;
  s_led_blink_ms = 0u;

  cyz_gyro_arm();
}

void cyz_gyro_on_rx_complete(UART_HandleTypeDef *huart)
{
  uint8_t byte;
  cyz_proto_result_t result;

  if (cyz_gyro_is_own_handle(huart) == 0u)
  {
    return;
  }

  /* Copy the byte out before re-arming: HAL writes the next one into the same
     slot, but only once the receiver has been armed again and the next byte has
     actually arrived, which cannot happen while this interrupt is running. */
  byte = s_rx_byte;

  /* Re-arm first so the following byte is never missed, then do the parsing
     work. Both steps together are far below the 86.8 us byte period. */
  cyz_gyro_arm();
  result = cyz_proto_feed(&s_parser, byte, &s_frame);
  if (result != CYZ_OK)
  {
    return; /* CYZ_NO_FRAME while a frame is still incomplete, or a malformed
               frame the parser dropped: nothing is published either way, so a
               bad frame can never overwrite the sample of a good one. */
  }

  /* A valid frame of any type just arrived. This single store is everything the
     interrupt contributes to the field indicator and to the frame age a task can
     read out - no GPIO, no decision, no work beyond the stamp. */
  s_last_rx_ms = HAL_GetTick();

  /* Hand the frame to the slot that matches its type: telemetry to the sample,
     replies to the reply word. An ACK can therefore never turn up in a sample -
     which is what reading an angle right after sending a command depends on,
     because the same Result byte means a status for one command and a report
     rate for another. */
  switch (s_frame.type)
  {
    case CYZ_FRAME_TELEMETRY:
      cyz_gyro_publish_sample(&s_frame.u.telemetry);
      break;

    case CYZ_FRAME_ACK:
      cyz_gyro_store_reply(CYZ_GYRO_REPLY_ACK, s_frame.u.ack.cmd,
                           s_frame.u.ack.result, s_frame.u.ack.seq);
      break;

    case CYZ_FRAME_SCALE:
      cyz_gyro_store_reply(CYZ_GYRO_REPLY_SCALE, s_frame.u.scale.cmd, 0u,
                           s_frame.u.scale.seq);
      break;

    default:
      break; /* CYZ_FRAME_NONE: not reachable when the feed returned CYZ_OK */
  }
}

void cyz_gyro_on_uart_error(UART_HandleTypeDef *huart)
{
  if (cyz_gyro_is_own_handle(huart) == 0u)
  {
    return;
  }

  /* A byte was lost or corrupted, so the frame being assembled cannot be
     trusted any more: drop it and let the parser resynchronise on the next
     header byte. Only the fixed-size FSM state is cleared - the counters stay
     monotonic, so the field diagnostics keep adding up across the error. */
  s_parser.cnt  = 0u;
  s_parser.need = 0u;

  /* Overrun is the blocking case: HAL_UART_IRQHandler runs UART_EndRxTransfer,
     which disables the RXNE/PE/ERR interrupts and puts RxState back to READY
     (stm32f1xx_hal_uart.c:2411-2416, 3335-3349), and only then reports the
     error - without the re-arm below the link would stay silent forever. Noise
     and framing errors are non-blocking: the byte has already gone through the
     normal receive path (which re-armed reception), RXNEIE is still enabled and
     RxState is still BUSY_RX, where a re-arm would only answer HAL_BUSY - so
     that case is skipped on purpose. */
  if (huart->RxState != HAL_UART_STATE_BUSY_RX)
  {
    cyz_gyro_arm();
  }
}

uint8_t cyz_gyro_read_sample(cyz_gyro_sample_t *out)
{
  uint32_t tick_before;
  cyz_gyro_sample_t copy;

  if (out == NULL)
  {
    return 0u;
  }

  for (;;)
  {
    tick_before = s_sample_tick;
    if (tick_before == 0u)
    {
      return 0u; /* no telemetry frame has arrived yet */
    }
    if ((tick_before & 1u) != 0u)
    {
      continue; /* the interrupt is publishing right now: look again */
    }

    copy.angle_deg = s_sample.angle_deg;
    copy.gyro_dps  = s_sample.gyro_dps;
    copy.seq       = s_sample.seq;

    if (s_sample_tick == tick_before)
    {
      /* The tick did not move across the copy, so no publish started or
         finished in between: those three fields are one frame's sample. */
      *out = copy;
      return 1u;
    }

    /* A frame was published while copying, so take the newer one. The writer is
       busy for a handful of instructions every 5 ms at 200 Hz, which is how
       often this can repeat: no interrupt is disabled, nothing is waited for -
       only this reader redoes its own work. */
  }
}

uint8_t cyz_gyro_read_reply(cyz_gyro_reply_t *out)
{
  uint32_t word = s_reply_word; /* one aligned 32-bit load: cannot be torn */

  if ((out == NULL) || (word == 0u))
  {
    return 0u;
  }

  out->kind   = (cyz_gyro_reply_kind_t)(uint8_t)((word >> 24) & 0xFFu);
  out->cmd    = (uint8_t)((word >> 16) & 0xFFu);
  out->result = (uint8_t)((word >> 8) & 0xFFu);
  out->seq    = (uint8_t)(word & 0xFFu);
  return 1u;
}

/* Counter reads: these counters are written by the receive interrupt and read
   from tasks, so the volatile access below forces a fresh load instead of a value
   the compiler cached before the interrupt advanced it. One aligned 32-bit word
   is atomic on Cortex-M3, so the reader needs no lock either - which is why the
   parameter is a pointer to volatile rather than a plain one. */
static uint32_t cyz_gyro_read_counter(const volatile uint32_t *counter)
{
  return *counter;
}

uint32_t cyz_gyro_rx_frame_count(void)
{
  return cyz_gyro_read_counter(&s_parser.frames_ok);
}

/* ==== report-rate negotiation (T4) ====================================== */

/* Sends one 8 byte command frame. Returns 1 only when the frame really left the
   UART. HAL_UART_Transmit polls the TX flags with a bounded timeout: USART1 has
   no DMA channel in this CubeMX generation and no TX callback is involved, and
   all eight bytes are on their way once the TC flag is up - about 0.7 ms at
   115200, so the polling task is never held up for longer than that. */
static uint8_t cyz_gyro_send_command(uint8_t cmd, uint8_t param)
{
  uint8_t frame[CYZ_PROTO_COMMAND_LEN];

  if (s_uart == NULL)
  {
    return 0u; /* cyz_gyro_init() has not run: there is nothing to send on */
  }

  s_cmd_seq++; /* the module echoes Seq back in its ACK, which is how a reply is
                  recognised among answers to earlier commands */
  if (cyz_proto_pack_command(cmd, param, s_cmd_seq, frame,
                             (uint16_t)sizeof(frame)) != CYZ_OK)
  {
    return 0u; /* undocumented combination: no half frame goes on the wire */
  }

  /* Snapshot before transmitting: from here on, a reply that arrives is an answer
     to this command - or a stale one, which the sequence check rejects. */
  s_neg_reply_seen = s_reply_serial;

  return (uint8_t)(HAL_UART_Transmit(s_uart, frame, (uint16_t)sizeof(frame),
                                     CYZ_GYRO_TX_TIMEOUT_MS) == HAL_OK);
}

/* Takes the reply that answers the command in flight (same command number and
   same sequence). Returns 1 with *out filled when it is ours, 0 while the module
   has not answered yet. Anything else that turns up - an ACK for an earlier
   command, a scale response - is consumed and ignored, so waiting always makes
   progress instead of re-reading the same reply forever. */
static uint8_t cyz_gyro_take_reply(uint8_t cmd, uint8_t seq, cyz_gyro_reply_t *out)
{
  cyz_gyro_reply_t reply;

  if (s_reply_serial == s_neg_reply_seen)
  {
    return 0u; /* nothing new arrived */
  }
  s_neg_reply_seen = s_reply_serial; /* this reply is consumed either way */

  if (cyz_gyro_read_reply(&reply) == 0u)
  {
    return 0u;
  }
  if ((reply.kind != CYZ_GYRO_REPLY_ACK) || (reply.cmd != cmd) || (reply.seq != seq))
  {
    return 0u;
  }

  *out = reply;
  return 1u;
}

/* Closes the attempt whose entry in the log is complete, and either starts the
   next rung or ends the negotiation. */
static void cyz_gyro_rate_decide(void)
{
  uint8_t index = (uint8_t)(s_rate.attempts - 1u);
  uint8_t code  = s_rate.log[index].code;
  uint8_t next  = cyz_proto_rate_next_lower(code);

  s_rate.measured_hz = s_rate.log[index].measured_hz;

  /* Both channels must agree: the readback is the module's own answer to
     Cmd=0x02 (primary), the measured frame rate is what actually arrived on the
     wire (secondary). Anything else - no answer, a different gear, or the right
     gear framed at a rate that is not that gear - leaves this rung unconfirmed. */
  if ((s_rate.log[index].readback == code) &&
      (cyz_proto_rate_matches(code, s_rate.log[index].measured_hz) != 0u))
  {
    s_rate.state      = CYZ_GYRO_RATE_CONFIRMED;
    s_rate.final_code = code;
    s_neg_stage       = CYZ_NEG_DONE;
    return;
  }

  if (next != (uint8_t)CYZ_PROTO_RATE_NONE)
  {
    s_neg_target = next;
    s_neg_stage  = CYZ_NEG_SEND_SET;
    return;
  }

  /* Chain exhausted: no gear could be confirmed, which is what a module that is
     not connected, not powered or not in UART mode looks like from here. The log
     keeps one entry per rung, so the reason stays readable afterwards. */
  s_rate.state      = CYZ_GYRO_RATE_FAILED;
  s_rate.final_code = (uint8_t)CYZ_GYRO_RATE_NONE;
  s_neg_stage       = CYZ_NEG_DONE;
}

void cyz_gyro_rate_negotiate_poll(void)
{
  cyz_gyro_reply_t reply;
  uint32_t         now = HAL_GetTick();
  uint32_t         elapsed;
  uint32_t         frames;
  uint8_t          index;

  switch (s_neg_stage)
  {
    case CYZ_NEG_SEND_SET:
      if (s_rate.attempts >= (uint8_t)CYZ_GYRO_RATE_ATTEMPTS)
      {
        /* Defensive: the chain has exactly as many rungs as the log has entries,
           so a fourth attempt is unreachable - but a full log must never be
           written past its end, whatever a future edit does to the chain. */
        s_rate.state      = CYZ_GYRO_RATE_FAILED;
        s_rate.final_code = (uint8_t)CYZ_GYRO_RATE_NONE;
        s_neg_stage       = CYZ_NEG_DONE;
        break;
      }

      /* 200 Hz is the gear this firmware wants; a fallback decision has already
         moved s_neg_target one rung down before re-entering here. A module that
         only boots after this first command is picked up by the lower rungs. */
      if (s_rate.attempts == 0u)
      {
        s_neg_target = (uint8_t)CYZ_RATE_200HZ;
      }

      index = s_rate.attempts;
      s_rate.attempts = (uint8_t)(index + 1u);
      s_rate.log[index].code        = s_neg_target;
      s_rate.log[index].ack_result  = (uint8_t)CYZ_GYRO_ACK_NONE;
      s_rate.log[index].readback    = (uint8_t)CYZ_GYRO_RATE_NONE;
      s_rate.log[index].measured_hz = 0u;

      s_rate.state  = CYZ_GYRO_RATE_RUNNING;
      s_neg_wait_ms = now;

      (void)cyz_gyro_send_command((uint8_t)CYZ_CMD_SET_RATE, s_neg_target);
      s_neg_stage = CYZ_NEG_WAIT_SET_ACK;
      break;

    case CYZ_NEG_WAIT_SET_ACK:
      index = (uint8_t)(s_rate.attempts - 1u);
      if (cyz_gyro_take_reply((uint8_t)CYZ_CMD_SET_RATE, s_cmd_seq, &reply) != 0u)
      {
        /* Result is a status code for Cmd=0x03: 0x00 accepted, 0x02 the module
           does not take this command or parameter. Recorded either way - the
           readback below says which gear the module actually holds. */
        s_rate.log[index].ack_result = reply.result;
        s_neg_wait_ms = now;
        s_neg_stage   = CYZ_NEG_SEND_QUERY;
      }
      else if ((uint32_t)(now - s_neg_wait_ms) >= CYZ_GYRO_ACK_WINDOW_MS)
      {
        s_neg_wait_ms = now; /* rung unanswered: still ask where the module is */
        s_neg_stage   = CYZ_NEG_SEND_QUERY;
      }
      else
      {
        /* still waiting for the ACK */
      }
      break;

    case CYZ_NEG_SEND_QUERY:
      (void)cyz_gyro_send_command((uint8_t)CYZ_CMD_QUERY_RATE, 0x00u);
      s_neg_wait_ms = now;
      s_neg_stage   = CYZ_NEG_WAIT_QUERY_ACK;
      break;

    case CYZ_NEG_WAIT_QUERY_ACK:
      index = (uint8_t)(s_rate.attempts - 1u);
      if (cyz_gyro_take_reply((uint8_t)CYZ_CMD_QUERY_RATE, s_cmd_seq, &reply) != 0u)
      {
        /* For Cmd=0x02 the Result byte is the report-rate code itself. */
        s_rate.log[index].readback = reply.result;
        s_neg_stage                = CYZ_NEG_MEASURE;
        s_neg_wait_ms              = now;
        s_neg_frames_at_start      = cyz_gyro_read_counter(&s_telemetry_frames);
      }
      else if ((uint32_t)(now - s_neg_wait_ms) >= CYZ_GYRO_ACK_WINDOW_MS)
      {
        s_neg_stage           = CYZ_NEG_MEASURE; /* nothing to read back */
        s_neg_wait_ms         = now;
        s_neg_frames_at_start = cyz_gyro_read_counter(&s_telemetry_frames);
      }
      else
      {
        /* still waiting for the readback */
      }
      break;

    case CYZ_NEG_MEASURE:
      elapsed = (uint32_t)(now - s_neg_wait_ms);
      if (elapsed >= CYZ_GYRO_MEASURE_WINDOW_MS)
      {
        index  = (uint8_t)(s_rate.attempts - 1u);
        frames = cyz_gyro_read_counter(&s_telemetry_frames) - s_neg_frames_at_start;
        /* Normalised to one second: the window ends on a poll boundary and is a
           millisecond or two long, where a raw frame count would read high. */
        s_rate.log[index].measured_hz = (uint16_t)((frames * 1000u) / elapsed);
        cyz_gyro_rate_decide();
      }
      break;

    case CYZ_NEG_DONE:
    default:
      break; /* nothing left to do: the report holds the outcome */
  }
}

uint8_t cyz_gyro_read_rate_report(cyz_gyro_rate_report_t *out)
{
  if ((out == NULL) || (s_rate.state == CYZ_GYRO_RATE_IDLE))
  {
    return 0u;
  }

  *out = s_rate; /* written and read by the polling task only: no lock needed */
  return 1u;
}

/* ==== PC13 field indicator (T5) ========================================= */

/* PC13, configured as a push-pull output by CubeMX (Core/Src/gpio.c): this layer
   only drives the level, it never reconfigures the pin. */
static void cyz_gyro_led_write(uint8_t on)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (on != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void cyz_gyro_indicator_tick(void)
{
  uint32_t               now   = HAL_GetTick();
  cyz_proto_link_state_t state = cyz_proto_link_state(now, cyz_gyro_last_rx_ms());

  if (state != s_led_state)
  {
    /* Entering a state always starts from the level that state means: dark for
       "off", lit for both live states - so the blinking state can never be taken
       for the dark one at a glance. */
    s_led_state    = state;
    s_led_level    = (uint8_t)((state == CYZ_LINK_OFF) ? 0u : 1u);
    s_led_blink_ms = now;
    cyz_gyro_led_write(s_led_level);
    return;
  }

  if ((state == CYZ_LINK_SLOW_BLINK) &&
      ((uint32_t)(now - s_led_blink_ms) >= CYZ_GYRO_LED_BLINK_HALF_MS))
  {
    s_led_blink_ms = now;
    s_led_level   ^= 1u;
    cyz_gyro_led_write(s_led_level);
  }
}

uint32_t cyz_gyro_last_rx_ms(void)
{
  return cyz_gyro_read_counter(&s_last_rx_ms);
}
