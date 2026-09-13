/**
  ******************************************************************************
  * @file    cyz_gyro.c
  * @brief   CY-Z (QMM01) gyro access layer - USART1 binding and interrupt rx.
  *
  * This is the only file that touches HAL UART objects for the gyro. All frame
  * knowledge stays in cyz_protocol.c, which is pure C and unit tested on a PC
  * (tests/cyz_protocol_test.c); here one received byte is handed to it and that
  * is the whole integration. No printf redirection, no task, no queue.
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

/* Last completed frame, the hand-off point for the value semantics of the next
   ticket; for now the parser only needs somewhere to put its output. */
static cyz_proto_frame_t s_frame;

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
  s_frame.type = CYZ_FRAME_NONE;

  cyz_gyro_arm();
}

void cyz_gyro_on_rx_complete(UART_HandleTypeDef *huart)
{
  uint8_t byte;

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
  (void)cyz_proto_feed(&s_parser, byte, &s_frame);
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

  /* Overrun aborts the transfer (HAL_UART_IRQHandler -> UART_EndRxTransfer puts
     RxState back to READY) and would leave the link silent forever unless we
     re-arm here. Noise and framing errors are non-blocking: the HAL keeps
     receiving, RxState is still BUSY_RX, and a re-arm would only answer
     HAL_BUSY - so that case is skipped on purpose. */
  if (huart->RxState != HAL_UART_STATE_BUSY_RX)
  {
    cyz_gyro_arm();
  }
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
