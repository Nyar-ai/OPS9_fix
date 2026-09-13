/**
  ******************************************************************************
  * @file    cyz_gyro.h
  * @brief   CY-Z (QMM01) gyro access layer: USART1 binding, interrupt reception
  *          and byte-stream hand-off to the protocol layer.
  *
  * One gyro, one HAL handle: USART1 (PB6/PB7, remapped, 115200 8N1). The robot
  * link keeps USART2 and its own handle - this layer never touches it, which is
  * enforced by dispatching every callback on the handle.
  *
  * Reception is single byte in interrupt mode, re-armed after every byte. That
  * is what this CubeMX generation offers: USART1 has no DMA channel configured,
  * so interrupt mode is the project's fact (spec: "interrupt vs DMA").
  *
  * Wiring (both call sites are CubeMX user areas, see Core/Src/main.c):
  *   USER CODE BEGIN 2  ->  cyz_gyro_init()        (after MX_USART1_UART_Init(),
  *                                                  before the scheduler starts)
  *   USER CODE BEGIN 4  ->  forward HAL_UART_RxCpltCallback /
  *                          HAL_UART_ErrorCallback to this layer
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

/* Read-only frame counter for the 30 second field check "is the link alive?".
   Counts every frame decoded since init - telemetry, ACK and scale responses
   alike - so at the default setting it follows the module's report rate.
   Written by the receive interrupt and read by any task: a single aligned
   32-bit word, so a reader never needs to lock anything. Malformed frames are
   deliberately not counted here. */
uint32_t cyz_gyro_rx_frame_count(void);

#endif /* CYZ_GYRO_H */
