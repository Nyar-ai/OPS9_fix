/**
  ******************************************************************************
  * @file    cyz_protocol.c
  * @brief   CY-Z (QMM01) gyro UART protocol layer - implementation.
  *
  * Pure C99: includes only <stdint.h> (through cyz_protocol.h) and <string.h>,
  * so it builds and unit-tests on a PC - see tests/cyz_protocol_test.c.
  * Field layout and CRC rules: docs/CY-Z protocol doc (2026-09-11).
  ******************************************************************************
  */

#include "cyz_protocol.h"
#include <string.h>

/* Byte range covered by the CRC and position of the CRC field, per frame type. */
#define TELEMETRY_CRC_FIRST   2u
#define TELEMETRY_CRC_COUNT  10u /* Seq[2] + AngleDeg[4] + GyroDps[4] */
#define TELEMETRY_CRC_POS    12u
#define ACK_CRC_FIRST         2u
#define ACK_CRC_COUNT         3u /* Cmd + Result + Seq */
#define ACK_CRC_POS           5u
#define SCALE_CRC_FIRST       2u
#define SCALE_CRC_COUNT       7u /* Cmd + Seq + Factor[4] + Reserved */
#define SCALE_CRC_POS         9u
#define COMMAND_CRC_FIRST     2u
#define COMMAND_CRC_COUNT     3u /* Cmd + Param + Seq */
#define COMMAND_CRC_POS       5u

/* ---- little-endian field access ---------------------------------------- */
static uint16_t read_u16_le(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)((uint16_t)p[1] << 8)));
}

static float read_f32_le(const uint8_t *p)
{
  uint32_t raw = ((uint32_t)p[0]) |
                 ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) |
                 ((uint32_t)p[3] << 24);
  float value;
  memcpy(&value, &raw, sizeof(value)); /* reinterpret the IEEE-754 bit pattern */
  return value;
}

/* ---- CRC-16/MODBUS ----------------------------------------------------- */
uint16_t cyz_proto_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFu;
  uint16_t i;
  uint8_t  bit;

  if (data == NULL)
  {
    return crc; /* nothing to fold in: the initial value is the result */
  }

  for (i = 0u; i < len; i++)
  {
    crc ^= (uint16_t)data[i];
    for (bit = 0u; bit < 8u; bit++)
    {
      if ((crc & 0x0001u) != 0u)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001u);
      }
      else
      {
        crc = (uint16_t)(crc >> 1);
      }
    }
  }
  return crc;
}

uint8_t cyz_proto_frame_len(cyz_proto_frame_type_t type)
{
  switch (type)
  {
    case CYZ_FRAME_TELEMETRY: return (uint8_t)CYZ_PROTO_TELEMETRY_LEN;
    case CYZ_FRAME_ACK:       return (uint8_t)CYZ_PROTO_ACK_LEN;
    case CYZ_FRAME_SCALE:     return (uint8_t)CYZ_PROTO_SCALE_LEN;
    default:                  return 0u;
  }
}

uint8_t cyz_proto_cmd_param_valid(uint8_t cmd, uint8_t param)
{
  switch (cmd)
  {
    case CYZ_CMD_ZERO_ANGLE:  /* 0x01: 0x01 = clear angle, 0x02 = re-estimate bias */
      return (uint8_t)((param == 0x01u) || (param == 0x02u));

    case CYZ_CMD_QUERY_RATE:  /* 0x02 */
      return (uint8_t)(param == 0x00u);

    case CYZ_CMD_SET_RATE:    /* 0x03: this project only uses the three rates it
                                 negotiates. 0x01/0x02 exist in the protocol but
                                 are unused here, and 0x03 (polling mode) would
                                 silence the module - indistinguishable from a
                                 broken link - so it must never be sent. */
      return (uint8_t)((param == CYZ_RATE_50HZ) ||
                       (param == CYZ_RATE_100HZ) ||
                       (param == CYZ_RATE_200HZ));

    case CYZ_CMD_READ_DATA:   /* 0x04 */
      return (uint8_t)(param == 0x00u);

    case CYZ_CMD_SCALE_START: /* 0x05: number of revolutions */
      return (uint8_t)((param == 1u) || (param == 2u) ||
                       (param == 3u) || (param == 6u));

    case CYZ_CMD_SCALE_FINISH: /* 0x06, 0x07, 0x08 */
    case CYZ_CMD_SCALE_CANCEL:
    case CYZ_CMD_READ_SCALE:
      return (uint8_t)(param == 0x00u);

    default:
      return 0u;
  }
}

/* ---- command packing --------------------------------------------------- */
cyz_proto_result_t cyz_proto_pack_command(uint8_t cmd, uint8_t param, uint8_t seq,
                                          uint8_t *out, uint16_t out_len)
{
  uint16_t crc;

  if (out == NULL)
  {
    return CYZ_ERR_NULL;
  }
  if (out_len < CYZ_PROTO_COMMAND_LEN)
  {
    return CYZ_ERR_LENGTH;
  }
  if (cyz_proto_cmd_param_valid(cmd, param) == 0u)
  {
    return CYZ_ERR_PARAM;
  }

  out[0] = CYZ_PROTO_HDR_A5;
  out[1] = CYZ_PROTO_HDR_COMMAND_1;
  out[COMMAND_CRC_FIRST] = cmd;
  out[3] = param;
  out[4] = seq;
  crc = cyz_proto_crc16(&out[COMMAND_CRC_FIRST], COMMAND_CRC_COUNT);
  out[COMMAND_CRC_POS] = (uint8_t)(crc & 0xFFu);
  out[COMMAND_CRC_POS + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
  out[7] = CYZ_PROTO_TAIL_COMMAND;

  return CYZ_OK;
}

/* ---- decoders ---------------------------------------------------------- */
cyz_proto_result_t cyz_proto_decode_telemetry(const uint8_t *buf, uint16_t len,
                                              cyz_proto_telemetry_t *out)
{
  if ((buf == NULL) || (out == NULL))
  {
    return CYZ_ERR_NULL;
  }
  if (len != CYZ_PROTO_TELEMETRY_LEN)
  {
    return CYZ_ERR_LENGTH;
  }
  if ((buf[0] != CYZ_PROTO_HDR_TELEMETRY_0) || (buf[1] != CYZ_PROTO_HDR_TELEMETRY_1))
  {
    return CYZ_ERR_HEADER;
  }
  if ((buf[14] != CYZ_PROTO_TAIL_TELEMETRY_0) || (buf[15] != CYZ_PROTO_TAIL_TELEMETRY_1))
  {
    return CYZ_ERR_TAIL;
  }
  if (cyz_proto_crc16(&buf[TELEMETRY_CRC_FIRST], TELEMETRY_CRC_COUNT) !=
      read_u16_le(&buf[TELEMETRY_CRC_POS]))
  {
    return CYZ_ERR_CRC;
  }

  out->seq       = read_u16_le(&buf[2]);
  out->angle_deg = read_f32_le(&buf[4]);
  out->gyro_dps  = read_f32_le(&buf[8]);
  return CYZ_OK;
}

cyz_proto_result_t cyz_proto_decode_ack(const uint8_t *buf, uint16_t len,
                                        cyz_proto_ack_t *out)
{
  if ((buf == NULL) || (out == NULL))
  {
    return CYZ_ERR_NULL;
  }
  if (len != CYZ_PROTO_ACK_LEN)
  {
    return CYZ_ERR_LENGTH;
  }
  if ((buf[0] != CYZ_PROTO_HDR_A5) || (buf[1] != CYZ_PROTO_HDR_ACK_1))
  {
    return CYZ_ERR_HEADER;
  }
  if (buf[7] != CYZ_PROTO_TAIL_ACK)
  {
    return CYZ_ERR_TAIL;
  }
  if (cyz_proto_crc16(&buf[ACK_CRC_FIRST], ACK_CRC_COUNT) != read_u16_le(&buf[ACK_CRC_POS]))
  {
    return CYZ_ERR_CRC;
  }

  out->cmd    = buf[2];
  out->result = buf[3];
  out->seq    = buf[4];
  /* The same Result byte is a report-rate code for the query command and a
     status code for every other command - resolve it here so callers never
     have to guess which one they are looking at. */
  out->kind = (out->cmd == CYZ_CMD_QUERY_RATE) ? CYZ_ACK_KIND_RATE_CODE
                                               : CYZ_ACK_KIND_STATUS;
  return CYZ_OK;
}

cyz_proto_result_t cyz_proto_decode_scale(const uint8_t *buf, uint16_t len,
                                          cyz_proto_scale_t *out)
{
  if ((buf == NULL) || (out == NULL))
  {
    return CYZ_ERR_NULL;
  }
  if (len != CYZ_PROTO_SCALE_LEN)
  {
    return CYZ_ERR_LENGTH;
  }
  if ((buf[0] != CYZ_PROTO_HDR_A5) || (buf[1] != CYZ_PROTO_HDR_SCALE_1))
  {
    return CYZ_ERR_HEADER;
  }
  if (buf[11] != CYZ_PROTO_TAIL_SCALE)
  {
    return CYZ_ERR_TAIL;
  }
  if (cyz_proto_crc16(&buf[SCALE_CRC_FIRST], SCALE_CRC_COUNT) != read_u16_le(&buf[SCALE_CRC_POS]))
  {
    return CYZ_ERR_CRC;
  }

  out->cmd               = buf[2];
  out->seq               = buf[3];
  out->gyro_scale_factor = read_f32_le(&buf[4]);
  out->reserved          = buf[8];
  return CYZ_OK;
}

/* ---- byte-stream parser ------------------------------------------------ */
static cyz_proto_frame_type_t frame_type_for_header(uint8_t b0, uint8_t b1)
{
  if ((b0 == CYZ_PROTO_HDR_TELEMETRY_0) && (b1 == CYZ_PROTO_HDR_TELEMETRY_1))
  {
    return CYZ_FRAME_TELEMETRY;
  }
  if (b0 == CYZ_PROTO_HDR_A5)
  {
    if (b1 == CYZ_PROTO_HDR_ACK_1)
    {
      return CYZ_FRAME_ACK;
    }
    if (b1 == CYZ_PROTO_HDR_SCALE_1)
    {
      return CYZ_FRAME_SCALE;
    }
  }
  return CYZ_FRAME_NONE; /* A5 5A is our own command frame: never received */
}

static uint8_t is_header_start(uint8_t b)
{
  return (uint8_t)((b == CYZ_PROTO_HDR_TELEMETRY_0) || (b == CYZ_PROTO_HDR_A5));
}

void cyz_proto_init(cyz_proto_t *p)
{
  if (p == NULL)
  {
    return;
  }
  memset(p, 0, sizeof(*p));
}

cyz_proto_result_t cyz_proto_feed(cyz_proto_t *p, uint8_t byte, cyz_proto_frame_t *frame)
{
  cyz_proto_frame_type_t type;
  cyz_proto_result_t result;

  if ((p == NULL) || (frame == NULL))
  {
    return CYZ_ERR_NULL;
  }
  p->bytes_seen++;

  if (p->cnt == 0u)
  {
    /* Looking for the first header byte; anything else is noise and is dropped. */
    if (is_header_start(byte) != 0u)
    {
      p->buf[0] = byte;
      p->cnt = 1u;
    }
    return CYZ_NO_FRAME;
  }

  if (p->cnt == 1u)
  {
    /* Second header byte: fixes the frame type, or resynchronises on this byte. */
    type = frame_type_for_header(p->buf[0], byte);
    if (type != CYZ_FRAME_NONE)
    {
      p->buf[1] = byte;
      p->cnt = 2u;
      p->need = cyz_proto_frame_len(type);
    }
    else if (is_header_start(byte) != 0u)
    {
      p->buf[0] = byte; /* this byte may start a frame of its own */
    }
    else
    {
      p->cnt = 0u;
    }
    return CYZ_NO_FRAME;
  }

  /* Collecting the rest of a frame whose length is already known. */
  if (p->cnt < CYZ_PROTO_FRAME_MAX_LEN)
  {
    p->buf[p->cnt] = byte;
    p->cnt++;
  }
  if (p->cnt < p->need)
  {
    return CYZ_NO_FRAME;
  }

  /* Frame complete: validate it and let the matching decoder fill the result. */
  memset(frame, 0, sizeof(*frame));
  type = frame_type_for_header(p->buf[0], p->buf[1]);
  switch (type)
  {
    case CYZ_FRAME_TELEMETRY:
      result = cyz_proto_decode_telemetry(p->buf, (uint16_t)p->need, &frame->u.telemetry);
      break;

    case CYZ_FRAME_ACK:
      result = cyz_proto_decode_ack(p->buf, (uint16_t)p->need, &frame->u.ack);
      break;

    case CYZ_FRAME_SCALE:
      result = cyz_proto_decode_scale(p->buf, (uint16_t)p->need, &frame->u.scale);
      break;

    default:
      result = CYZ_ERR_HEADER;
      break;
  }

  /* Drop the frame either way and resynchronise on the next byte, so a
     malformed frame never blocks the frames that follow it. */
  p->cnt = 0u;
  p->need = 0u;
  if (result == CYZ_OK)
  {
    frame->type = type;
    p->frames_ok++;
  }
  else
  {
    frame->type = CYZ_FRAME_NONE;
    p->frames_bad++;
  }
  return result;
}

/* ---- link health (T5) --------------------------------------------------
   The decision the PC13 indicator makes, kept here because this file is the
   project's PC-testable seam: same input, same output, no HAL, no clock. */
cyz_proto_link_state_t cyz_proto_link_state(uint32_t now_ms, uint32_t last_rx_ms)
{
  if (last_rx_ms == 0u)
  {
    return CYZ_LINK_OFF; /* nothing valid has arrived since reset */
  }

  /* Difference form on purpose: the true age of the last frame falls out of
     unsigned arithmetic across a wrap, as long as it is below 2^31 ms (24 days),
     where "now > last + timeout" would answer wrongly for the whole wrap. */
  if ((uint32_t)(now_ms - last_rx_ms) > CYZ_PROTO_LINK_TIMEOUT_MS)
  {
    return CYZ_LINK_SLOW_BLINK;
  }
  return CYZ_LINK_ON;
}

/* ---- report-rate negotiation helpers (T4) ------------------------------ */
static uint16_t rate_nominal_hz(uint8_t code)
{
  switch (code)
  {
    case CYZ_RATE_200HZ: return 200u;
    case CYZ_RATE_100HZ: return 100u;
    case CYZ_RATE_50HZ:  return 50u;
    default:             return 0u; /* 20/10 Hz and polling mode are not in the chain */
  }
}

uint8_t cyz_proto_rate_next_lower(uint8_t code)
{
  switch (code)
  {
    case CYZ_RATE_200HZ: return (uint8_t)CYZ_RATE_100HZ;
    case CYZ_RATE_100HZ: return (uint8_t)CYZ_RATE_50HZ;
    default:             return (uint8_t)CYZ_PROTO_RATE_NONE;
  }
}

uint8_t cyz_proto_rate_matches(uint8_t code, uint16_t measured_hz)
{
  uint16_t nominal = rate_nominal_hz(code);
  uint32_t low;
  uint32_t high;

  if ((nominal == 0u) || (measured_hz == 0u))
  {
    return 0u; /* no such gear, or nothing arrived to measure */
  }

  low  = ((uint32_t)nominal * 3u) / 4u;
  high = ((uint32_t)nominal * 5u) / 4u;
  return (uint8_t)(((uint32_t)measured_hz >= low) && ((uint32_t)measured_hz <= high));
}




