/**
  ******************************************************************************
  * @file    cyz_gyro.c
  * @brief   CY-Z (QMM01) gyro access layer - USART1 binding and interrupt rx.
  *
  * This is the only file that touches HAL UART objects for the gyro. All frame
  * knowledge stays in cyz_protocol.c, which is pure C and unit tested on a PC
  * (tests/cyz_protocol_test.c); here one received byte is handed to it and that
  * is the whole integration: what a task reads out of this file are the two
  * values published from it (latest telemetry sample, last module reply).
  * No printf redirection, no task, no queue.
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

/* Publishes one decoded telemetry frame. Runs in the receive interrupt. */
static void cyz_gyro_publish_sample(const cyz_proto_telemetry_t *telemetry)
{
  s_sample_tick++; /* odd: a reader copying right now will retry */
  s_sample.angle_deg = telemetry->angle_deg;
  s_sample.gyro_dps  = telemetry->gyro_dps;
  s_sample.seq       = telemetry->seq;
  s_sample_tick++; /* even again: those three fields are one frame's sample */
}

/* Stores one reply. Runs in the receive interrupt. */
static void cyz_gyro_store_reply(cyz_gyro_reply_kind_t kind, uint8_t cmd,
                                 uint8_t result, uint8_t seq)
{
  s_reply_word = ((uint32_t)kind << 24) | ((uint32_t)cmd << 16) |
                 ((uint32_t)result << 8) | (uint32_t)seq;
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

/* Counter reads: the parser counters are written by the receive interrupt and
   read from tasks, so the volatile access below forces a fresh load instead of
   a value the compiler cached before the interrupt advanced it. One aligned
   32-bit word is atomic on Cortex-M3, so the reader needs no lock either. */
static uint32_t cyz_gyro_read_counter(const uint32_t *counter)
{
  const volatile uint32_t *live = counter;
  return *live;
}

uint32_t cyz_gyro_rx_frame_count(void)
{
  return cyz_gyro_read_counter(&s_parser.frames_ok);
}
