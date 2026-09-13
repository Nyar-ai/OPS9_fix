/**
  ******************************************************************************
  * @file    tests/cyz_protocol_test.c
  * @brief   PC-side unit test for the CY-Z protocol layer.
  *
  * Build & run (no HAL, no framework, no dependency beyond a C99 compiler):
  *   gcc -std=c99 -Wall -Wextra -pedantic -I Core/Inc Core/Src/cyz_protocol.c \
  *       tests/cyz_protocol_test.c -o cyz_protocol_test
  *
  * Golden byte strings come from docs/CY-Z protocol doc (2026-09-11) or were
  * recomputed independently - never from the code under test.
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>

#include "cyz_protocol.h"

static int g_pass;
static int g_fail;

#define CHECK(cond)                                          \
  do                                                         \
  {                                                          \
    if (cond)                                                \
    {                                                        \
      g_pass++;                                              \
    }                                                        \
    else                                                     \
    {                                                        \
      g_fail++;                                              \
      printf("FAIL line %d: %s\n", __LINE__, #cond);         \
    }                                                        \
  } while (0)

/* ---- helpers ----------------------------------------------------------- */
static uint32_t f32_bits(float v)
{
  uint32_t bits;
  memcpy(&bits, &v, sizeof(bits));
  return bits;
}

/* Feeds a byte array in one go and returns the result of the last byte. */
static cyz_proto_result_t feed_bytes(cyz_proto_t *p, const uint8_t *data,
                                     uint16_t len, cyz_proto_frame_t *frame)
{
  cyz_proto_result_t r = CYZ_NO_FRAME;
  uint16_t i;

  for (i = 0u; i < len; i++)
  {
    r = cyz_proto_feed(p, data[i], frame);
  }
  return r;
}

/* Builds an 8 byte ACK frame; the CRC comes from the layer (whose CRC is pinned
   by the golden vectors below), only the field bytes are assembled here. */
static void make_ack(uint8_t cmd, uint8_t result, uint8_t seq, uint8_t *out)
{
  uint16_t crc;

  out[0] = 0xA5u;
  out[1] = 0x5Bu;
  out[2] = cmd;
  out[3] = result;
  out[4] = seq;
  crc = cyz_proto_crc16(&out[2], 3u);
  out[5] = (uint8_t)(crc & 0xFFu);
  out[6] = (uint8_t)((crc >> 8) & 0xFFu);
  out[7] = 0x5Bu;
}

/* ---- golden frames ----------------------------------------------------- */
/* 16 B telemetry: Seq=1, AngleDeg=1.0f, GyroDps=0.0f, CRC=0x072A - byte for
   byte the example frame printed in the protocol document. */
static const uint8_t k_telemetry_ok[16] = {
  0xAA, 0x55, 0x01, 0x00, 0x00, 0x00, 0x80, 0x3F,
  0x00, 0x00, 0x00, 0x00, 0x2A, 0x07, 0x55, 0xAA
};
/* 16 B telemetry: Seq=0x1234, AngleDeg=-12.5f, GyroDps=-0.25f, CRC=0x6436. */
static const uint8_t k_telemetry_2[16] = {
  0xAA, 0x55, 0x34, 0x12, 0x00, 0x00, 0x48, 0xC1,
  0x00, 0x00, 0x80, 0xBE, 0x36, 0x64, 0x55, 0xAA
};
/* 8 B ACK: Cmd=0x02 (query rate) Result=0x05 (200 Hz) Seq=0, CRC=0x50D3. */
static const uint8_t k_ack_rate_200hz[8] = {
  0xA5, 0x5B, 0x02, 0x05, 0x00, 0xD3, 0x50, 0x5B
};
/* 8 B ACK: Cmd=0x03 (set rate) Result=0x02 (bad command/param) Seq=1, CRC=0x6041. */
static const uint8_t k_ack_status_badcmd[8] = {
  0xA5, 0x5B, 0x03, 0x02, 0x01, 0x41, 0x60, 0x5B
};
/* 12 B scale factor: Cmd=0x08, Seq=1, GyroScaleFactor=1.0f, Reserved=0, CRC=0x0983. */
static const uint8_t k_scale_ok[12] = {
  0xA5, 0x5C, 0x08, 0x01, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x83, 0x09, 0x5C
};
/* 8 B command: Cmd=0x03 (set rate) Param=0x05 (200 Hz) Seq=0, CRC=0x9082. */
static const uint8_t k_cmd_set_200hz[8] = {
  0xA5, 0x5A, 0x03, 0x05, 0x00, 0x82, 0x90, 0x5A
};

/* ==== tests: CRC, frame length, telemetry ============================== */
static void test_crc16(void)
{
  CHECK(cyz_proto_crc16(&k_telemetry_ok[2], 10u) == 0x072Au);
  CHECK(cyz_proto_crc16(&k_telemetry_ok[2], 10u) ==
        (uint16_t)((uint16_t)k_telemetry_ok[12] |
                   (uint16_t)((uint16_t)k_telemetry_ok[13] << 8)));
  CHECK(cyz_proto_crc16(&k_telemetry_2[2], 10u) == 0x6436u);
  CHECK(cyz_proto_crc16(&k_ack_rate_200hz[2], 3u) == 0x50D3u);
  CHECK(cyz_proto_crc16(&k_scale_ok[2], 7u) == 0x0983u);
  CHECK(cyz_proto_crc16(&k_cmd_set_200hz[2], 3u) == 0x9082u);
  CHECK(cyz_proto_crc16(NULL, 5u) == 0xFFFFu);       /* nothing folded in */
  CHECK(cyz_proto_crc16(k_telemetry_ok, 0u) == 0xFFFFu);
}

static void test_frame_len(void)
{
  CHECK(cyz_proto_frame_len(CYZ_FRAME_TELEMETRY) == 16u);
  CHECK(cyz_proto_frame_len(CYZ_FRAME_ACK) == 8u);
  CHECK(cyz_proto_frame_len(CYZ_FRAME_SCALE) == 12u);
  CHECK(cyz_proto_frame_len(CYZ_FRAME_NONE) == 0u);
}

static void test_telemetry_decode(void)
{
  cyz_proto_telemetry_t t;
  uint8_t bad[16];

  CHECK(cyz_proto_decode_telemetry(k_telemetry_ok, 16u, &t) == CYZ_OK);
  CHECK(t.seq == 1u);
  CHECK(f32_bits(t.angle_deg) == f32_bits(1.0f));
  CHECK(f32_bits(t.gyro_dps) == f32_bits(0.0f));

  CHECK(cyz_proto_decode_telemetry(k_telemetry_2, 16u, &t) == CYZ_OK);
  CHECK(t.seq == 0x1234u);
  CHECK(f32_bits(t.angle_deg) == f32_bits(-12.5f));
  CHECK(f32_bits(t.gyro_dps) == f32_bits(-0.25f));

  /* every defect class reports its own result code */
  CHECK(cyz_proto_decode_telemetry(k_telemetry_ok, 15u, &t) == CYZ_ERR_LENGTH);
  CHECK(cyz_proto_decode_telemetry(NULL, 16u, &t) == CYZ_ERR_NULL);
  CHECK(cyz_proto_decode_telemetry(k_telemetry_ok, 16u, NULL) == CYZ_ERR_NULL);

  memcpy(bad, k_telemetry_ok, 16u);
  bad[0] = 0x00u;
  CHECK(cyz_proto_decode_telemetry(bad, 16u, &t) == CYZ_ERR_HEADER);

  memcpy(bad, k_telemetry_ok, 16u);
  bad[15] = 0x00u;
  CHECK(cyz_proto_decode_telemetry(bad, 16u, &t) == CYZ_ERR_TAIL);

  memcpy(bad, k_telemetry_ok, 16u);
  bad[12] ^= 0xFFu;
  CHECK(cyz_proto_decode_telemetry(bad, 16u, &t) == CYZ_ERR_CRC);

  memcpy(bad, k_telemetry_ok, 16u);
  bad[4] ^= 0x01u;                    /* payload tampered with, CRC left alone */
  CHECK(cyz_proto_decode_telemetry(bad, 16u, &t) == CYZ_ERR_CRC);
}

/* ==== tests: ACK semantics, scale factor, command packing ============== */
static void test_ack_decode(void)
{
  cyz_proto_ack_t a;
  uint8_t frame[8];

  /* Cmd=0x02 -> Result is a report-rate code */
  CHECK(cyz_proto_decode_ack(k_ack_rate_200hz, 8u, &a) == CYZ_OK);
  CHECK(a.cmd == CYZ_CMD_QUERY_RATE);
  CHECK(a.seq == 0u);
  CHECK(a.kind == CYZ_ACK_KIND_RATE_CODE);
  CHECK(a.result == CYZ_RATE_200HZ);

  /* Cmd=0x03 -> the very same field is a status code */
  CHECK(cyz_proto_decode_ack(k_ack_status_badcmd, 8u, &a) == CYZ_OK);
  CHECK(a.cmd == CYZ_CMD_SET_RATE);
  CHECK(a.seq == 1u);
  CHECK(a.kind == CYZ_ACK_KIND_STATUS);
  CHECK(a.result == CYZ_ACK_STATUS_BAD_CMD);

  /* The documented trap: Result 0x00 means "50 Hz" for Cmd=0x02 but "success"
     for any other Cmd. Both frames below carry the identical Result byte. */
  make_ack(CYZ_CMD_QUERY_RATE, 0x00u, 0u, frame);
  CHECK(cyz_proto_decode_ack(frame, 8u, &a) == CYZ_OK);
  CHECK(a.kind == CYZ_ACK_KIND_RATE_CODE);
  CHECK(a.result == CYZ_RATE_50HZ);

  make_ack(CYZ_CMD_SET_RATE, 0x00u, 0u, frame);
  CHECK(cyz_proto_decode_ack(frame, 8u, &a) == CYZ_OK);
  CHECK(a.kind == CYZ_ACK_KIND_STATUS);
  CHECK(a.result == CYZ_ACK_STATUS_OK);

  /* error paths */
  CHECK(cyz_proto_decode_ack(k_ack_rate_200hz, 7u, &a) == CYZ_ERR_LENGTH);
  CHECK(cyz_proto_decode_ack(NULL, 8u, &a) == CYZ_ERR_NULL);
  CHECK(cyz_proto_decode_ack(k_ack_rate_200hz, 8u, NULL) == CYZ_ERR_NULL);

  memcpy(frame, k_ack_rate_200hz, 8u);
  frame[7] = 0x00u;
  CHECK(cyz_proto_decode_ack(frame, 8u, &a) == CYZ_ERR_TAIL);

  memcpy(frame, k_ack_rate_200hz, 8u);
  frame[5] ^= 0x01u;
  CHECK(cyz_proto_decode_ack(frame, 8u, &a) == CYZ_ERR_CRC);
}

static void test_scale_decode(void)
{
  cyz_proto_scale_t s;
  uint8_t bad[12];

  CHECK(cyz_proto_decode_scale(k_scale_ok, 12u, &s) == CYZ_OK);
  CHECK(s.cmd == CYZ_CMD_READ_SCALE);
  CHECK(s.seq == 1u);
  CHECK(f32_bits(s.gyro_scale_factor) == f32_bits(1.0f));
  CHECK(s.reserved == 0x00u);

  CHECK(cyz_proto_decode_scale(k_scale_ok, 11u, &s) == CYZ_ERR_LENGTH);
  CHECK(cyz_proto_decode_scale(NULL, 12u, &s) == CYZ_ERR_NULL);
  CHECK(cyz_proto_decode_scale(k_scale_ok, 12u, NULL) == CYZ_ERR_NULL);

  memcpy(bad, k_scale_ok, 12u);
  bad[11] = 0x00u;
  CHECK(cyz_proto_decode_scale(bad, 12u, &s) == CYZ_ERR_TAIL);

  memcpy(bad, k_scale_ok, 12u);
  bad[9] ^= 0x01u;
  CHECK(cyz_proto_decode_scale(bad, 12u, &s) == CYZ_ERR_CRC);
}

static void test_pack_command(void)
{
  /* expected bytes recomputed independently from the protocol document */
  static const uint8_t expect_50hz[8]  = { 0xA5, 0x5A, 0x03, 0x00, 0x00, 0x81, 0xC0, 0x5A };
  static const uint8_t expect_100hz[8] = { 0xA5, 0x5A, 0x03, 0x04, 0x00, 0x83, 0x00, 0x5A };
  static const uint8_t expect_200hz[8] = { 0xA5, 0x5A, 0x03, 0x05, 0x00, 0x82, 0x90, 0x5A };
  static const uint8_t expect_query[8] = { 0xA5, 0x5A, 0x02, 0x00, 0x07, 0x91, 0xC2, 0x5A };
  uint8_t out[8];
  uint8_t guard[8];

  CHECK(cyz_proto_pack_command(CYZ_CMD_SET_RATE, CYZ_RATE_50HZ, 0u, out, 8u) == CYZ_OK);
  CHECK(memcmp(out, expect_50hz, 8u) == 0);
  CHECK(cyz_proto_pack_command(CYZ_CMD_SET_RATE, CYZ_RATE_100HZ, 0u, out, 8u) == CYZ_OK);
  CHECK(memcmp(out, expect_100hz, 8u) == 0);
  CHECK(cyz_proto_pack_command(CYZ_CMD_SET_RATE, CYZ_RATE_200HZ, 0u, out, 8u) == CYZ_OK);
  CHECK(memcmp(out, expect_200hz, 8u) == 0);
  CHECK(cyz_proto_pack_command(CYZ_CMD_QUERY_RATE, 0x00u, 7u, out, 8u) == CYZ_OK);
  CHECK(memcmp(out, expect_query, 8u) == 0);

  /* polling mode would silence the module and look exactly like a broken link,
     so it must never be packable; 20/10 Hz are documented but unused here */
  CHECK(cyz_proto_pack_command(CYZ_CMD_SET_RATE, CYZ_RATE_POLLING, 0u, out, 8u) == CYZ_ERR_PARAM);
  CHECK(cyz_proto_cmd_param_valid(CYZ_CMD_SET_RATE, CYZ_RATE_POLLING) == 0u);
  CHECK(cyz_proto_cmd_param_valid(CYZ_CMD_SET_RATE, CYZ_RATE_20HZ) == 0u);
  CHECK(cyz_proto_cmd_param_valid(CYZ_CMD_SET_RATE, CYZ_RATE_10HZ) == 0u);
  CHECK(cyz_proto_cmd_param_valid(CYZ_CMD_SET_RATE, 0x06u) == 0u);
  CHECK(cyz_proto_cmd_param_valid(0x09u, 0x00u) == 0u); /* undocumented command */

  /* documented combinations stay valid */
  CHECK(cyz_proto_cmd_param_valid(CYZ_CMD_ZERO_ANGLE, 0x01u) == 1u);
  CHECK(cyz_proto_cmd_param_valid(CYZ_CMD_ZERO_ANGLE, 0x02u) == 1u);
  CHECK(cyz_proto_cmd_param_valid(CYZ_CMD_SCALE_START, 6u) == 1u);
  CHECK(cyz_proto_cmd_param_valid(CYZ_CMD_READ_SCALE, 0x00u) == 1u);

  /* a rejected argument must leave the caller's buffer untouched, so no half
     frame can ever be put on the wire */
  memcpy(guard, out, 8u);
  CHECK(cyz_proto_pack_command(CYZ_CMD_SET_RATE, CYZ_RATE_POLLING, 0u, out, 8u) == CYZ_ERR_PARAM);
  CHECK(memcmp(guard, out, 8u) == 0);

  CHECK(cyz_proto_pack_command(CYZ_CMD_SET_RATE, CYZ_RATE_200HZ, 0u, NULL, 8u) == CYZ_ERR_NULL);
  CHECK(cyz_proto_pack_command(CYZ_CMD_SET_RATE, CYZ_RATE_200HZ, 0u, out, 7u) == CYZ_ERR_LENGTH);
}

/* ==== tests: byte-stream parser ======================================== */
static void test_fsm_init_and_null(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;

  cyz_proto_init(&p);
  CHECK(p.cnt == 0u);
  CHECK(p.need == 0u);
  CHECK(p.frames_ok == 0u);
  CHECK(p.frames_bad == 0u);
  CHECK(p.bytes_seen == 0u);

  cyz_proto_init(NULL); /* documented no-op */

  CHECK(cyz_proto_feed(NULL, 0xAAu, &f) == CYZ_ERR_NULL);
  CHECK(cyz_proto_feed(&p, 0xAAu, NULL) == CYZ_ERR_NULL);
}

/* Half a frame in one call, the rest in the next: the FSM must stitch it. */
static void test_fsm_half_frame(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;

  cyz_proto_init(&p);

  CHECK(feed_bytes(&p, k_telemetry_ok, 8u, &f) == CYZ_NO_FRAME);
  CHECK(p.frames_ok == 0u);

  CHECK(feed_bytes(&p, &k_telemetry_ok[8], 8u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_TELEMETRY);
  CHECK(f.u.telemetry.seq == 1u);
  CHECK(f32_bits(f.u.telemetry.angle_deg) == f32_bits(1.0f));
  CHECK(f32_bits(f.u.telemetry.gyro_dps) == f32_bits(0.0f));
  CHECK(p.frames_ok == 1u);
  CHECK(p.frames_bad == 0u);
  CHECK(p.bytes_seen == 16u);
}

/* Two frames glued together: the first must complete on its last byte and the
   next byte must already belong to the following frame. */
static void test_fsm_two_frames_in_one_stream(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;
  uint8_t glued[24];

  memcpy(&glued[0], k_telemetry_ok, 16u);
  memcpy(&glued[16], k_ack_rate_200hz, 8u);

  cyz_proto_init(&p);

  CHECK(feed_bytes(&p, glued, 16u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_TELEMETRY);

  CHECK(feed_bytes(&p, &glued[16], 8u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_ACK);
  CHECK(f.u.ack.kind == CYZ_ACK_KIND_RATE_CODE);
  CHECK(f.u.ack.result == CYZ_RATE_200HZ);

  CHECK(p.frames_ok == 2u);
  CHECK(p.frames_bad == 0u);
}

/* Noise before a frame - including a stray 0xA5 that is not a valid header -
   must be absorbed without inventing a broken frame. */
static void test_fsm_noise_between_frames(void)
{
  static const uint8_t noise[4] = { 0x00u, 0xFFu, 0x11u, 0xA5u };
  cyz_proto_t p;
  cyz_proto_frame_t f;
  uint8_t stream[20];

  memcpy(&stream[0], noise, 4u);
  memcpy(&stream[4], k_telemetry_ok, 16u);

  cyz_proto_init(&p);

  CHECK(feed_bytes(&p, stream, 20u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_TELEMETRY);
  CHECK(f.u.telemetry.seq == 1u);
  CHECK(p.frames_ok == 1u);
  CHECK(p.frames_bad == 0u);
  CHECK(p.bytes_seen == 20u);
}

/* ==== tests: parser recovery =========================================== */
/* A repeated header-start byte must not eat the real frame that follows. */
static void test_fsm_resync_on_repeated_header_start(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;
  uint8_t stream[18];

  stream[0] = 0xAAu; /* looks like a header start ... */
  stream[1] = 0xAAu; /* ... but the pair is invalid: the second byte restarts it */
  memcpy(&stream[2], k_telemetry_ok, 16u);

  cyz_proto_init(&p);

  CHECK(feed_bytes(&p, stream, 18u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_TELEMETRY);
  CHECK(f.u.telemetry.seq == 1u);
  CHECK(f32_bits(f.u.telemetry.angle_deg) == f32_bits(1.0f));
  CHECK(p.frames_ok == 1u);
  CHECK(p.frames_bad == 0u);
}

/* A frame with a broken CRC is dropped, and the next good frame still decodes. */
static void test_fsm_self_heals_after_bad_crc(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;
  uint8_t corrupt[16];

  memcpy(corrupt, k_telemetry_ok, 16u);
  corrupt[13] ^= 0xFFu; /* CRC high byte broken */

  cyz_proto_init(&p);

  CHECK(feed_bytes(&p, corrupt, 16u, &f) == CYZ_ERR_CRC);
  CHECK(p.frames_bad == 1u);
  CHECK(p.frames_ok == 0u);

  CHECK(feed_bytes(&p, k_telemetry_ok, 16u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_TELEMETRY);
  CHECK(p.frames_ok == 1u);
  CHECK(p.frames_bad == 1u);
  CHECK(p.bytes_seen == 32u);
}

/* Bad tail is reported as a tail error (not as a CRC error), and the parser
   recovers right after the frame's fixed length. */
static void test_fsm_bad_tail(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;
  uint8_t bad[16];

  memcpy(bad, k_telemetry_ok, 16u);
  bad[15] = 0x00u;

  cyz_proto_init(&p);

  CHECK(feed_bytes(&p, bad, 16u, &f) == CYZ_ERR_TAIL);
  CHECK(p.frames_bad == 1u);

  CHECK(feed_bytes(&p, k_ack_rate_200hz, 8u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_ACK);
  CHECK(p.frames_ok == 1u);
}

/* A5 5A is the frame *we* send. If it ever comes back it is noise, never a
   received frame - and the telemetry behind it must still decode. */
static void test_fsm_ignores_own_command_frames(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;
  uint8_t stream[24];

  memcpy(&stream[0], k_cmd_set_200hz, 8u);
  memcpy(&stream[8], k_telemetry_ok, 16u);

  cyz_proto_init(&p);

  CHECK(feed_bytes(&p, stream, 24u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_TELEMETRY);
  CHECK(p.frames_ok == 1u);
  CHECK(p.frames_bad == 0u);
}

/* 12 B scale factor response through the parser. */
static void test_fsm_scale_frame(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;

  cyz_proto_init(&p);

  CHECK(feed_bytes(&p, k_scale_ok, 12u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_SCALE);
  CHECK(f.u.scale.cmd == CYZ_CMD_READ_SCALE);
  CHECK(f.u.scale.seq == 1u);
  CHECK(f32_bits(f.u.scale.gyro_scale_factor) == f32_bits(1.0f));
  CHECK(f.u.scale.reserved == 0x00u);
  CHECK(p.frames_ok == 1u);
  CHECK(p.frames_bad == 0u);
}

/* The access layer dispatches on frame.type after every return from the parser,
   so a frame that failed validation must be marked as "nothing": a stale type
   would let a bad frame overwrite the published snapshot. */
static void test_frame_out_marked_none_on_bad_frame(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;
  uint8_t corrupt[16];

  memcpy(corrupt, k_telemetry_ok, 16u);
  corrupt[13] ^= 0xFFu; /* CRC high byte broken */

  cyz_proto_init(&p);

  CHECK(feed_bytes(&p, k_telemetry_ok, 16u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_TELEMETRY);

  CHECK(feed_bytes(&p, corrupt, 16u, &f) == CYZ_ERR_CRC);
  CHECK(f.type == CYZ_FRAME_NONE);        /* marked, not left stale */

  /* What the access layer acts on is the type, so nothing here asserts how the
     parser clears the payload of a rejected frame. */
}

/* Bytes of a frame that is still incomplete must not touch the caller's frame:
   the access layer keeps serving the last published sample while they arrive. */
static void test_frame_out_untouched_until_frame_completes(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;
  uint8_t partial[15];

  memcpy(partial, k_telemetry_ok, 15u); /* everything but the closing tail byte */

  cyz_proto_init(&p);

  f.type                  = CYZ_FRAME_TELEMETRY;
  f.u.telemetry.seq       = 0xBEEFu;
  f.u.telemetry.angle_deg = 42.0f;
  f.u.telemetry.gyro_dps  = -1.5f;

  CHECK(feed_bytes(&p, partial, 15u, &f) == CYZ_NO_FRAME);
  CHECK(f.type == CYZ_FRAME_TELEMETRY);
  CHECK(f.u.telemetry.seq == 0xBEEFu);
  CHECK(f32_bits(f.u.telemetry.angle_deg) == f32_bits(42.0f));
  CHECK(f32_bits(f.u.telemetry.gyro_dps) == f32_bits(-1.5f));
  CHECK(p.frames_ok == 0u);
}

/* A byte lost on the wire leaves the parser waiting inside a frame; the bytes
   that follow splice into it, so that one frame is rejected as malformed - and
   the frame after it decodes again. This is the streaming half of the self
   healing the receive interrupt relies on after a UART error. */
static void test_fsm_recovers_after_lost_byte(void)
{
  cyz_proto_t p;
  cyz_proto_frame_t f;

  cyz_proto_init(&p);

  /* only the first half of the frame arrives ... */
  CHECK(feed_bytes(&p, k_telemetry_ok, 8u, &f) == CYZ_NO_FRAME);
  CHECK(p.frames_ok == 0u);
  CHECK(p.frames_bad == 0u);

  /* ... so the next frame's bytes complete this one, which is rejected */
  CHECK(feed_bytes(&p, k_telemetry_2, 16u, &f) == CYZ_NO_FRAME);
  CHECK(p.frames_bad == 1u);
  CHECK(p.frames_ok == 0u);

  /* the parser is empty again: the next complete frame decodes */
  CHECK(feed_bytes(&p, k_telemetry_ok, 16u, &f) == CYZ_OK);
  CHECK(f.type == CYZ_FRAME_TELEMETRY);
  CHECK(f.u.telemetry.seq == 1u);
  CHECK(p.frames_ok == 1u);
  CHECK(p.frames_bad == 1u);
}

int main(void)
{
  test_crc16();
  test_frame_len();
  test_telemetry_decode();
  test_ack_decode();
  test_scale_decode();
  test_pack_command();
  test_fsm_init_and_null();
  test_fsm_half_frame();
  test_fsm_two_frames_in_one_stream();
  test_fsm_noise_between_frames();
  test_fsm_resync_on_repeated_header_start();
  test_fsm_self_heals_after_bad_crc();
  test_fsm_bad_tail();
  test_fsm_ignores_own_command_frames();
  test_fsm_scale_frame();
  test_frame_out_marked_none_on_bad_frame();
  test_frame_out_untouched_until_frame_completes();
  test_fsm_recovers_after_lost_byte();

  printf("cyz_protocol_test: %d checks passed, %d failed\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}




