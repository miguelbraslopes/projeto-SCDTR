#pragma once
#include <stdint.h>
#include "mcp2515.h"

// ── msg type ──────────────────────────
#define INTERNAL 0   // pwm-message  (high priority)
#define FSERIAL  1   // serial message (lower priority)

// ── Field masks & shifts ─────────────────────
#define CAN_PRIORITY_MASK    0x400   // bit 10
#define CAN_PRIORITY_SHIFT   10

#define CAN_SRC_MASK         0x300   // bits 9–8
#define CAN_SRC_SHIFT        8

#define CAN_DST_MASK         0x0C0   // bits 7–6
#define CAN_DST_SHIFT        6

#define CAN_MSG_TYPE_MASK    0x03F   // bits 5–0
#define CAN_MSG_TYPE_SHIFT   0

#define CAN_FLOAT_PAYLOAD_LEN 4

enum CANSerialMsgType : uint8_t {
    CAN_MSG_SET_DUTY              = 0x00,//
    CAN_MSG_GET_DUTY              = 0x01,//
    CAN_MSG_SET_ILLUM_REF         = 0x02,//
    CAN_MSG_GET_ILLUM_REF         = 0x03,//
    CAN_MSG_GET_LUX               = 0x04,//
    CAN_MSG_GET_LDR_VOLTAGE       = 0x05,//
    CAN_MSG_SET_OCCUPANCY         = 0x06,//
    CAN_MSG_GET_OCCUPANCY         = 0x07,//
    CAN_MSG_SET_ANTI_WINDUP       = 0x08,//
    CAN_MSG_GET_ANTI_WINDUP       = 0x09,//
    CAN_MSG_SET_FEEDBACK          = 0x0A,//
    CAN_MSG_GET_FEEDBACK          = 0x0B,//
    CAN_MSG_GET_EXT_ILLUM         = 0x0C,
    CAN_MSG_GET_INST_POWER        = 0x0D,
    CAN_MSG_GET_ELAPSED_TIME      = 0x0E,
    CAN_MSG_START_STREAM_Y        = 0x0F,
    CAN_MSG_START_STREAM_U        = 0x10,
    CAN_MSG_STOP_STREAM_Y         = 0x11,
    CAN_MSG_STOP_STREAM_U         = 0x12,
    CAN_MSG_GET_STREAM_BUFFER_Y   = 0x13,
    CAN_MSG_GET_STREAM_BUFFER_U   = 0x14,
    CAN_MSG_GET_AVG_ENERGY        = 0x15,
    CAN_MSG_GET_AVG_VIS_ERROR     = 0x16,
    CAN_MSG_GET_AVG_FLICKER       = 0x17,
    CAN_MSG_GET_REF_BOUND_HIGH    = 0x18,
    CAN_MSG_SET_REF_BOUND_HIGH    = 0x19,
    CAN_MSG_GET_REF_BOUND_LOW     = 0x1A,
    CAN_MSG_SET_REF_BOUND_LOW     = 0x1B,
    CAN_MSG_GET_CURR_REF_BOUND    = 0x1C,
    CAN_MSG_GET_ENERGY_COST       = 0x1D,
    CAN_MSG_SET_ENERGY_COST       = 0x1E,
    CAN_MSG_RESTART               = 0x1F,
    CAN_MSG_ACK                   = 0x20,
    CAN_MSG_ERR                   = 0x21
};

extern MCP2515 can0;

void encode_and_send(uint8_t type, uint8_t srcID, uint8_t dstID, uint8_t msgType, float value);
void encode_and_send_byte(uint8_t type, uint8_t srcID, uint8_t dstID, uint8_t msgType, uint8_t value);
void encode_and_send_status(uint8_t srcID, uint8_t dstID, bool success);
canid_t encodeID(uint8_t type, uint8_t srcID, uint8_t dstID, uint8_t msgType);
void CAN_packFloat(float value, uint8_t outPayload[CAN_FLOAT_PAYLOAD_LEN]);

struct CANDecodedID {
    uint8_t type;
    uint8_t srcID;
    uint8_t dstID;
    uint8_t msgType;
};

bool decodeID(canid_t canID, CANDecodedID& out);
bool decodeCommand(canid_t canID, CANSerialMsgType& outCommand);

void processirq();
void init_can();



