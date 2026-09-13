/**
  ******************************************************************************
  * @file    cyz_protocol.h
  * @brief   CY-Z (QMM01) gyro UART protocol layer - pure C, no HAL dependency.
  *
  * Frame layout, field widths and CRC rules follow docs/CY-Z protocol doc
  * (2026-09-11), the single source of truth for this protocol in this repository.
  *
  * Frames (multi-byte fields little-endian, CRC-16/MODBUS):
  *   telemetry (module -> host) 16 B : AA 55 | Seq u16 | AngleDeg f32 | GyroDps f32 | CRC u16 | 55 AA
  *   ack       (module -> host)  8 B : A5 5B | Cmd u8 | Result u8 | Seq u8 | CRC u16 | 5B
  *   scale     (module -> host) 12 B : A5 5C | Cmd u8 | Seq u8 | Factor f32 | Reserved u8 | CRC u16 | 5C
  *   command   (host -> module)  8 B : A5 5A | Cmd u8 | Param u8 | Seq u8 | CRC u16 | 5A
  *
  * This header depends on <stdint.h> only, so the layer can be built and unit
  * tested on a PC without any HAL/CMSIS header - see tests/cyz_protocol_test.c.
  ******************************************************************************
  */

#ifndef CYZ_PROTOCOL_H
#define CYZ_PROTOCOL_H

#include <stdint.h>

/* ---- frame sizes ------------------------------------------------------- */
#define CYZ_PROTO_TELEMETRY_LEN  16u
#define CYZ_PROTO_ACK_LEN         8u
#define CYZ_PROTO_SCALE_LEN      12u
#define CYZ_PROTO_COMMAND_LEN     8u
#define CYZ_PROTO_FRAME_MAX_LEN  16u

/* ---- header / tail constants ------------------------------------------- */
#define CYZ_PROTO_HDR_TELEMETRY_0 0xAAu
#define CYZ_PROTO_HDR_TELEMETRY_1 0x55u
#define CYZ_PROTO_HDR_A5          0xA5u
#define CYZ_PROTO_HDR_COMMAND_1   0x5Au /* host -> module: never emitted by the rx parser */
#define CYZ_PROTO_HDR_ACK_1       0x5Bu
#define CYZ_PROTO_HDR_SCALE_1     0x5Cu
#define CYZ_PROTO_TAIL_TELEMETRY_0 0x55u
#define CYZ_PROTO_TAIL_TELEMETRY_1 0xAAu
#define CYZ_PROTO_TAIL_COMMAND    0x5Au
#define CYZ_PROTO_TAIL_ACK        0x5Bu
#define CYZ_PROTO_TAIL_SCALE      0x5Cu

/* ---- commands (host -> module) ----------------------------------------- */
#define CYZ_CMD_ZERO_ANGLE    0x01u
#define CYZ_CMD_QUERY_RATE    0x02u
#define CYZ_CMD_SET_RATE      0x03u
#define CYZ_CMD_READ_DATA     0x04u
#define CYZ_CMD_SCALE_START   0x05u
#define CYZ_CMD_SCALE_FINISH  0x06u
#define CYZ_CMD_SCALE_CANCEL  0x07u
#define CYZ_CMD_READ_SCALE    0x08u

/* ---- report-rate codes (Cmd=0x03 Param, and Cmd=0x02 Result) ----------- */
#define CYZ_RATE_50HZ    0x00u
#define CYZ_RATE_20HZ    0x01u
#define CYZ_RATE_10HZ    0x02u
#define CYZ_RATE_POLLING 0x03u /* polling mode: module stops reporting on its own */
#define CYZ_RATE_100HZ   0x04u
#define CYZ_RATE_200HZ   0x05u

/* ---- ACK status codes (only valid when Cmd != 0x02) -------------------- */
#define CYZ_ACK_STATUS_OK        0x00u /* same byte as CYZ_RATE_50HZ - meaning depends on Cmd */
#define CYZ_ACK_STATUS_NOT_STILL 0x01u
#define CYZ_ACK_STATUS_BAD_CMD   0x02u
#define CYZ_ACK_STATUS_FAILED    0x03u

/* ---- result codes ------------------------------------------------------ */
typedef enum
{
  CYZ_OK         = 0,  /* frame decoded / command packed */
  CYZ_NO_FRAME   = 1,  /* one byte absorbed, no complete frame yet (streaming, not an error) */
  CYZ_ERR_NULL   = -1, /* null pointer argument */
  CYZ_ERR_LENGTH = -2, /* wrong buffer/frame length */
  CYZ_ERR_HEADER = -3, /* header bytes do not match the frame being decoded */
  CYZ_ERR_TAIL   = -4, /* tail byte(s) do not match */
  CYZ_ERR_CRC    = -5, /* CRC-16/MODBUS mismatch */
  CYZ_ERR_PARAM  = -6  /* command/parameter combination rejected */
} cyz_proto_result_t;

/* ---- decoded frames ---------------------------------------------------- */
typedef enum
{
  CYZ_FRAME_NONE      = 0,
  CYZ_FRAME_TELEMETRY = 1,
  CYZ_FRAME_ACK       = 2,
  CYZ_FRAME_SCALE     = 3
} cyz_proto_frame_type_t;

typedef struct
{
  uint16_t seq;       /* wraps around at 65535 */
  float    angle_deg; /* integrated angle, deg */
  float    gyro_dps;  /* filtered angular rate, deg/s */
} cyz_proto_telemetry_t;

/* The Result byte means different things depending on Cmd. Never interpret it
   without checking kind first: 0x00 is "50 Hz" for Cmd=0x02 but "success" for
   every other command. */
typedef enum
{
  CYZ_ACK_KIND_STATUS    = 0, /* Cmd != 0x02: Result is a status code */
  CYZ_ACK_KIND_RATE_CODE = 1  /* Cmd == 0x02: Result is a report-rate code */
} cyz_proto_ack_kind_t;

typedef struct
{
  uint8_t              cmd;    /* original command number */
  uint8_t              result; /* raw Result byte */
  uint8_t              seq;    /* original command sequence */
  cyz_proto_ack_kind_t kind;   /* derived from cmd by the decoder */
} cyz_proto_ack_t;

typedef struct
{
  uint8_t cmd;              /* original command number (0x08) */
  uint8_t seq;
  float   gyro_scale_factor;
  uint8_t reserved;         /* documented as 0x00; carried through, not enforced */
} cyz_proto_scale_t;

typedef struct
{
  cyz_proto_frame_type_t type;
  union
  {
    cyz_proto_telemetry_t telemetry;
    cyz_proto_ack_t       ack;
    cyz_proto_scale_t     scale;
  } u;
} cyz_proto_frame_t;

/* ---- CRC-16/MODBUS (init 0xFFFF, poly 0xA001, low byte first on the wire) */
uint16_t cyz_proto_crc16(const uint8_t *data, uint16_t len);

/* ---- decoders for complete frames -------------------------------------- */
cyz_proto_result_t cyz_proto_decode_telemetry(const uint8_t *buf, uint16_t len,
                                              cyz_proto_telemetry_t *out);
cyz_proto_result_t cyz_proto_decode_ack(const uint8_t *buf, uint16_t len,
                                        cyz_proto_ack_t *out);
cyz_proto_result_t cyz_proto_decode_scale(const uint8_t *buf, uint16_t len,
                                          cyz_proto_scale_t *out);

/* ---- command packing --------------------------------------------------- */
/* Reports whether (cmd, param) is a documented, accepted combination. */
uint8_t cyz_proto_cmd_param_valid(uint8_t cmd, uint8_t param);

/* Packs an 8 byte command frame into out (out_len >= CYZ_PROTO_COMMAND_LEN).
   Rejects undocumented combinations with CYZ_ERR_PARAM - in particular the
   rate parameter 0x03 (polling mode), which would silence the module and is
   indistinguishable from a broken link. */
cyz_proto_result_t cyz_proto_pack_command(uint8_t cmd, uint8_t param, uint8_t seq,
                                          uint8_t *out, uint16_t out_len);

/* Expected on-wire length of a frame type (0 for CYZ_FRAME_NONE). */
uint8_t cyz_proto_frame_len(cyz_proto_frame_type_t type);

/* ---- byte-stream parser ------------------------------------------------ */
typedef struct
{
  uint8_t  buf[CYZ_PROTO_FRAME_MAX_LEN]; /* bytes of the frame being assembled */
  uint8_t  cnt;                          /* how many of them are already there */
  uint8_t  need;                         /* expected frame length, 0 = header unresolved */
  uint32_t frames_ok;                    /* valid frames decoded */
  uint32_t frames_bad;                   /* malformed frames dropped (length/tail/CRC) */
  uint32_t bytes_seen;                   /* bytes fed in */
} cyz_proto_t;

void cyz_proto_init(cyz_proto_t *p);

/* Feeds one received byte. Returns CYZ_NO_FRAME until a frame completes, then
   CYZ_OK with *frame filled - or CYZ_ERR_LENGTH / CYZ_ERR_TAIL / CYZ_ERR_CRC
   for a malformed frame. Malformed frames are dropped at their fixed length and
   the parser resynchronises on the next header byte, so it always self-heals. */
cyz_proto_result_t cyz_proto_feed(cyz_proto_t *p, uint8_t byte, cyz_proto_frame_t *frame);

#endif /* CYZ_PROTOCOL_H */
