/**
  ******************************************************************************
  * @file    cyz_gyro.h
  * @brief   CY-Z (QMM01) gyro access layer: USART1 binding, interrupt reception
  *          and byte-stream hand-off to the protocol layer, the two values a task
  *          reads from it (latest telemetry sample, last module reply), the
  *          report-rate negotiation and the PC13 link indicator.
  *
  * One gyro, one HAL handle: USART1 (PB6/PB7, remapped, 115200 8N1). The robot
  * link keeps USART2 and its own handle - this layer never touches it, which is
  * enforced by dispatching every callback on the handle.
  *
  * Reception is single byte in interrupt mode, re-armed after every byte. That
  * is what this CubeMX generation offers: USART1 has no DMA channel configured,
  * so interrupt mode is the project's fact (spec: "interrupt vs DMA").
  *
  * Wiring (all call sites are CubeMX user areas, so a regeneration keeps them):
  *   main.c USER CODE BEGIN 2 -> cyz_gyro_init()   (after MX_USART1_UART_Init(),
  *                                                  before the scheduler starts)
  *   main.c USER CODE BEGIN 4 -> forward HAL_UART_RxCpltCallback /
  *                               HAL_UART_ErrorCallback to this layer
  *   StartDefaultTask         -> cyz_gyro_rate_negotiate_poll() and
  *                               cyz_gyro_indicator_tick(), on the 1 ms task that
  *                               already exists (Core/Src/freertos.c)
  *
  * Interrupt budget: at 115200 a byte takes 86.8 us on the wire, so the ISR
  * path here only moves one byte into the protocol state machine - no blocking
  * call, no FreeRTOS API, no GPIO toggling.
  ******************************************************************************
  */

#ifndef CYZ_GYRO_H
#define CYZ_GYRO_H

#include <stdint.h>
#include "usart.h" /* HAL UART types and the generated huart1/huart2 handles */

/* Binds the gyro link to huart1 and arms interrupt reception. Call once, after
   MX_USART1_UART_Init() and before the RTOS scheduler starts. Works with the
   module absent or miswired: it only prepares the receiver and returns. */
void cyz_gyro_init(void);

/* Handle-dispatched HAL callbacks: forward the two HAL UART callbacks here from
   main.c. Both run in interrupt context and ignore every handle except the gyro
   one, so the USART2 robot link keeps its own reception state untouched. */
void cyz_gyro_on_rx_complete(UART_HandleTypeDef *huart);
void cyz_gyro_on_uart_error(UART_HandleTypeDef *huart);

/* ---- latest telemetry snapshot ----------------------------------------- */
/* One sample per received telemetry frame. The receive interrupt is the only
   writer and a task is the reader: the reader copies, validates and retries
   instead of locking, so it never disables interrupts and never waits for the
   interrupt - a read cannot delay reception or disturb the frame counter. */
typedef struct
{
  float    angle_deg; /* integrated angle, deg */
  float    gyro_dps;  /* filtered angular rate, deg/s */
  uint16_t seq;       /* module frame counter, wraps at 65536 */
} cyz_gyro_sample_t;

/* Copies the latest telemetry snapshot - all three fields of one and the same
   frame, never a mix of two. Returns 1 when a sample has been published, 0 when
   none has arrived yet (or out is NULL); out is left untouched in that case.

   Dropped-frame detection: the module increments seq by one per frame, so with
   prev from the previous read, delta = (uint16_t)(seq - prev) is 1 for
   consecutive frames and delta - 1 is the number of frames that never reached
   this firmware (bad CRC, noise, an unplugged wire); delta == 0 means the very
   same frame as last time. */
uint8_t cyz_gyro_read_sample(cyz_gyro_sample_t *out);

/* ---- last reply from the module (ACK / scale factor) ------------------- */
/* Replies never touch the snapshot above. Their Result byte is a status code
   for most commands but a report-rate code for the query command 0x02, so it is
   handed over raw next to cmd and the caller decides what it means. */
typedef enum
{
  CYZ_GYRO_REPLY_ACK   = 1, /* command acknowledged: result is its Result byte */
  CYZ_GYRO_REPLY_SCALE = 2  /* scale-factor response for command 0x08 */
} cyz_gyro_reply_kind_t;

typedef struct
{
  cyz_gyro_reply_kind_t kind;
  uint8_t               cmd;    /* the command this reply answers */
  uint8_t               result; /* ACK only: raw Result byte */
  uint8_t               seq;    /* command sequence echoed back */
} cyz_gyro_reply_t;

/* Copies the last reply the module returned. Returns 1 when one has been
   stored, 0 when none has arrived yet (or out is NULL); out is left untouched
   in that case. */
uint8_t cyz_gyro_read_reply(cyz_gyro_reply_t *out);

/* Read-only frame counter for the 30 second field check "is the link alive?".
   Counts every frame decoded since init - telemetry, ACK and scale responses
   alike - so at the default setting it follows the module's report rate.
   Written by the receive interrupt and read by any task: a single aligned
   32-bit word, so a reader never needs to lock anything. Malformed frames are
   deliberately not counted here. */
uint32_t cyz_gyro_rx_frame_count(void);

/* ---- report-rate negotiation (T4) --------------------------------------- */
/* The firmware fixes the report rate at 200 Hz and walks the documented chain
   down (200 -> 100 -> 50 Hz) when the module does not take it. The module ACKs
   every command, and one gear is only accepted when both confirmation channels
   agree: the Cmd=0x02 readback - the module's own answer, the primary channel -
   and the measured telemetry frame rate, what actually arrived, the secondary
   one. */
#define CYZ_GYRO_RATE_NONE      0xFFu /* no rate code / not a rate code */
#define CYZ_GYRO_ACK_NONE       0xFFu /* the module never answered this command */
#define CYZ_GYRO_RATE_ATTEMPTS     3u /* 200, 100, 50 Hz: one entry per rung */

typedef enum
{
  CYZ_GYRO_RATE_IDLE      = 0, /* nothing asked yet */
  CYZ_GYRO_RATE_RUNNING   = 1, /* asking, reading back, measuring */
  CYZ_GYRO_RATE_CONFIRMED = 2, /* a gear passed both channels */
  CYZ_GYRO_RATE_FAILED    = 3  /* chain exhausted: absent or unanswering module */
} cyz_gyro_rate_state_t;

/* One rung of the chain, with every value the two channels produced for it, so a
   failed negotiation says which rung failed and why instead of just "no rate". */
typedef struct
{
  uint8_t  code;        /* rate code this attempt asked for: 0x05 / 0x04 / 0x00 */
  uint8_t  ack_result;  /* raw ACK Result byte, CYZ_GYRO_ACK_NONE when no ACK came */
  uint8_t  readback;    /* rate code the Cmd=0x02 readback returned, NONE if none */
  uint16_t measured_hz; /* telemetry frames per second in the window (0 = none) */
} cyz_gyro_rate_attempt_t;

typedef struct
{
  cyz_gyro_rate_state_t   state;
  uint8_t                 attempts;    /* entries of log[] that are valid */
  uint8_t                 final_code;  /* confirmed gear, CYZ_GYRO_RATE_NONE if none */
  uint16_t                measured_hz; /* rate measured for the final attempt */
  cyz_gyro_rate_attempt_t log[CYZ_GYRO_RATE_ATTEMPTS];
} cyz_gyro_rate_report_t;

/* Advances the negotiation when it has something to do, returns at once
   otherwise. Call it from an existing periodic task: one command at a time, a
   bounded transmit timeout, HAL_GetTick() windows for the waits - no task, no
   queue, nothing that blocks, and no RTOS API, so the same call would also work
   before the scheduler starts. A module that never answers costs only time. */
void cyz_gyro_rate_negotiate_poll(void);

/* Copies the negotiation report (state, confirmed gear, one entry per attempt).
   Returns 1, or 0 while nothing has been attempted yet or when out is NULL - out
   is untouched then. Task-context data: the receive interrupt never writes it. */
uint8_t cyz_gyro_read_rate_report(cyz_gyro_rate_report_t *out);

/* ---- PC13 field indicator (T5) ----------------------------------------- */
/* Three states, decided only by the arrival time of the last valid frame:
     dark       = no valid frame since reset (not wired / module unpowered /
                  interface-select pin pulled to I2C)
     slow blink = frames arrived, but none for more than a second (link dropped,
                  or the module was set to polling mode 0x03)
     lit        = a valid frame arrived within the last second
   Call it from an existing periodic task: the level is decided and driven here,
   never in the receive interrupt, which only stamps the arrival time. */
void cyz_gyro_indicator_tick(void);

/* Arrival time (HAL_GetTick) of the last valid frame, 0 = none yet: the raw input
   of the decision above, which the indicator reads the same way a debugger or a
   task can - the receive interrupt only writes it. */
uint32_t cyz_gyro_last_rx_ms(void);

#endif /* CYZ_GYRO_H */
