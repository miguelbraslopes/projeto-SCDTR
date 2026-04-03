#include "canv2.h"
#include "serial.h"
#include <string.h>

MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};

void init_can(){
    can0.reset();
    can0.setBitrate(CAN_1000KBPS);
    can0.setNormalMode();
    //can0.setLoopbackMode();        
}

static float unpackFloatPayload(const can_frame& frame) {
    float value = 0.0f;
    if (frame.can_dlc >= CAN_FLOAT_PAYLOAD_LEN) {
        memcpy(&value, frame.data, CAN_FLOAT_PAYLOAD_LEN);
    }
    return value;
}

static bool buildCommandFromSerialType(const CANSerialMsgType type, const CANDecodedID& decoded, const can_frame& frame, Command& cmd) {
    cmd = {};
    cmd.luminaireId = decoded.srcID;
    cmd.origin = ORIGIN_CAN;
    cmd.sourceLuminaireId = decoded.srcID;
    if (debug){
        Serial.print("Decoded CAN ID - Type: "); Serial.print(decoded.type);
        Serial.print(", SrcID: "); Serial.print(decoded.srcID);
        Serial.print(", DstID: "); Serial.print(decoded.dstID);
        Serial.print(", MsgType: "); Serial.println(decoded.msgType);
    }
    switch (type) {
        case CAN_MSG_SET_DUTY:
            cmd.mainCmd = "u";
            cmd.value = unpackFloatPayload(frame);
            return true;
        case CAN_MSG_GET_DUTY:
            cmd.mainCmd = "g";
            cmd.subCmd = "u";
            return true;
        case CAN_MSG_SET_ILLUM_REF:
            cmd.mainCmd = "r";
            cmd.value = unpackFloatPayload(frame);
            return true;
        case CAN_MSG_GET_ILLUM_REF:
            cmd.mainCmd = "g";
            cmd.subCmd = "r";
            return true;
        case CAN_MSG_GET_LUX:
            cmd.mainCmd = "g";
            cmd.subCmd = "y";
            return true;
        case CAN_MSG_GET_LDR_VOLTAGE:
            cmd.mainCmd = "g";
            cmd.subCmd = "v";
            return true;
        case CAN_MSG_SET_OCCUPANCY:
            cmd.mainCmd = "o";
            if (frame.can_dlc >= 1) {
                cmd.charValue = static_cast<char>(frame.data[0]);
            }
            return true;
        case CAN_MSG_GET_OCCUPANCY:
            cmd.mainCmd = "g";
            cmd.subCmd = "o";
            return true;
        case CAN_MSG_SET_ANTI_WINDUP:
            cmd.mainCmd = "a";
            cmd.value = (frame.can_dlc >= 1 && frame.data[0] != 0) ? 1.0f : 0.0f;
            return true;
        case CAN_MSG_GET_ANTI_WINDUP:
            cmd.mainCmd = "g";
            cmd.subCmd = "a";
            return true;
        case CAN_MSG_SET_FEEDBACK:
            cmd.mainCmd = "f";
            cmd.value = (frame.can_dlc >= 1 && frame.data[0] != 0) ? 1.0f : 0.0f;
            return true;
        case CAN_MSG_GET_FEEDBACK:
            cmd.mainCmd = "g";
            cmd.subCmd = "f";
            return true;
        default:
            // Message exists in enum but is not mapped to executeCommand yet.
            return false;
    }
}

void encode_and_send(uint8_t type, uint8_t srcID, uint8_t dstID, uint8_t msgType, float value){
    //Build ID
    canid_t id = encodeID(type, srcID, dstID, msgType);
    //build message
    can_frame msg{};
    msg.can_id = id;
    CAN_packFloat(value, msg.data);
    msg.can_dlc = CAN_FLOAT_PAYLOAD_LEN;
    can0.sendMessage(&msg);
}

void encode_and_send_byte(uint8_t type, uint8_t srcID, uint8_t dstID, uint8_t msgType, uint8_t value) {
    canid_t id = encodeID(type, srcID, dstID, msgType);
    can_frame msg{};
    msg.can_id = id;
    msg.data[0] = value;
    msg.can_dlc = 1;
    can0.sendMessage(&msg);
}

void encode_and_send_status(uint8_t srcID, uint8_t dstID, bool success) {
    const uint8_t msgType = success ? CAN_MSG_ACK : CAN_MSG_ERR;
    canid_t id = encodeID(FSERIAL, srcID, dstID, msgType);
    can_frame msg{};
    msg.can_id = id;
    msg.can_dlc = 0;
    can0.sendMessage(&msg);
}

void CAN_packFloat(float value, uint8_t outPayload[CAN_FLOAT_PAYLOAD_LEN]) {
    memcpy(outPayload, &value, CAN_FLOAT_PAYLOAD_LEN);
}

canid_t encodeID(uint8_t type, uint8_t srcID, uint8_t dstID, uint8_t msgType){
    if (type != INTERNAL && type != FSERIAL) {
        return 0;
    }

    if (srcID > (CAN_SRC_MASK >> CAN_SRC_SHIFT)) {
        return 0;
    }

    canid_t id = 0;

    id |= (static_cast<canid_t>(type)  << CAN_PRIORITY_SHIFT) & CAN_PRIORITY_MASK;
    id |= (static_cast<canid_t>(srcID) << CAN_SRC_SHIFT) & CAN_SRC_MASK;

    if (type == FSERIAL) {
        if (dstID > (CAN_DST_MASK >> CAN_DST_SHIFT) || msgType > CAN_MSG_TYPE_MASK) {
            return 0;
        }

        id |= (static_cast<canid_t>(dstID)   << CAN_DST_SHIFT) & CAN_DST_MASK;
        id |= (static_cast<canid_t>(msgType) << CAN_MSG_TYPE_SHIFT) & CAN_MSG_TYPE_MASK;
    }

    // Return 32-bit canid_t with standard 11-bit identifier.
    // Bits 0-10: CAN identifier (11 bits)
    // Bits 11-28: reserved (always 0)
    // Bit 29: error message frame flag = 0 (data frame)
    // Bit 30: remote transmission request flag = 0 (data frame)
    // Bit 31: frame format flag = 0 (standard 11-bit, not extended 29-bit)
    return (id & CAN_SFF_MASK) & ~(CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG);
}

bool decodeID(canid_t canID, CANDecodedID& out) {
    if ((canID & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) {
        return false;
    }

    canid_t id = canID & CAN_SFF_MASK;

    out.type = static_cast<uint8_t>((id & CAN_PRIORITY_MASK) >> CAN_PRIORITY_SHIFT);
    out.srcID = static_cast<uint8_t>((id & CAN_SRC_MASK) >> CAN_SRC_SHIFT);

    if (out.type != INTERNAL && out.type != FSERIAL) {
        return false;
    }

    if (out.srcID > (CAN_SRC_MASK >> CAN_SRC_SHIFT)) {
        return false;
    }

    if (out.type == FSERIAL) {
        out.dstID = static_cast<uint8_t>((id & CAN_DST_MASK) >> CAN_DST_SHIFT);
        out.msgType = static_cast<uint8_t>((id & CAN_MSG_TYPE_MASK) >> CAN_MSG_TYPE_SHIFT);

        if (out.dstID > (CAN_DST_MASK >> CAN_DST_SHIFT) || out.msgType > CAN_MSG_TYPE_MASK) {
            return false;
        }
    } else {
        out.dstID = 0;
        out.msgType = 0;
    }

    return true;
}

bool decodeCommand(canid_t canID, CANSerialMsgType& outCommand) {
    CANDecodedID decoded{};
    if (!decodeID(canID, decoded)) {
        return false;
    }

    if (decoded.type != FSERIAL) {
        return false;
    }

    if (decoded.msgType > static_cast<uint8_t>(CAN_MSG_ERR)) {
        return false;
    }

    outCommand = static_cast<CANSerialMsgType>(decoded.msgType);
    return true;
}

static bool isGetMsgType(uint8_t msgType) {
    return msgType == CAN_MSG_GET_DUTY ||
           msgType == CAN_MSG_GET_ILLUM_REF ||
           msgType == CAN_MSG_GET_LUX ||
           msgType == CAN_MSG_GET_LDR_VOLTAGE ||
           msgType == CAN_MSG_GET_OCCUPANCY ||
           msgType == CAN_MSG_GET_ANTI_WINDUP ||
           msgType == CAN_MSG_GET_FEEDBACK;
}

static void processFrame(const can_frame& frm) {
    //decode message type
    CANDecodedID receivedFrame{};

    if (decodeID(frm.can_id, receivedFrame) == false) {
        return;
    }

    if (debugger_can){
        Serial.println("Received CAN message");
        Serial.print("ID: ");
        Serial.println(frm.can_id, DEC);
        Serial.print(" Decoded as - Type: ");
        Serial.println(receivedFrame.type);
        Serial.print(" SrcID: ");
        Serial.println(receivedFrame.srcID);
        Serial.print(" DstID: ");
        Serial.println(receivedFrame.dstID);
        Serial.print(" MsgType: ");
        Serial.println(receivedFrame.msgType);
        Serial.print("Data: ");
        Serial.println(unpackFloatPayload(frm));
        Serial.println("---------------------------------------");
    }

    //"callback" function to msg type
    if (receivedFrame.type == INTERNAL) {   //update pwms of other luminaires
        //check src id and update pwm
        if (receivedFrame.srcID < 4 && receivedFrame.srcID >=0 && receivedFrame.srcID != _luminaireId)
        {
            gPending.haspwm = true;
            gPending.newpwm[receivedFrame.srcID] = unpackFloatPayload(frm);
            return;
        }
    }
    else if (receivedFrame.type == FSERIAL) {
        //check if serial command in can is for me or other luminaire, if for me execute, if for other luminaire forward

        if (receivedFrame.dstID==_luminaireId) {    //comando para esta luminária, executar
            if (receivedFrame.msgType == CAN_MSG_ACK) {
                if (waiting_can) {
                    waiting_can = false;
                    Serial.println("ack");
                }
                return;
            }
            if (receivedFrame.msgType == CAN_MSG_ERR) {
                if (waiting_can) {
                    waiting_can = false;
                    Serial.println("err");
                }
                return;
            }

            // If we are waiting for a remote GET, this frame is the response data.
            // Print once to serial and do not execute as a command (avoids CAN ping-pong loop).
            if (waiting_can && isGetMsgType(receivedFrame.msgType)) {
                waiting_can = false;
                if (receivedFrame.msgType == CAN_MSG_GET_OCCUPANCY) {
                    char occupancy = '\0';
                    if (frm.can_dlc >= 1) {
                        occupancy = static_cast<char>(frm.data[0]);
                    } else {
                        // Backward compatibility with older float payload format.
                        occupancy = static_cast<char>(static_cast<int>(unpackFloatPayload(frm)));
                    }
                    Serial.print("o ");
                    Serial.print(receivedFrame.srcID);
                    Serial.print(" ");
                    Serial.println(occupancy);
                } else {
                    Serial.print("g ");
                    Serial.print(receivedFrame.srcID);
                    Serial.print(" ");
                    Serial.println(unpackFloatPayload(frm));
                }
                return;
            }

            if (debugger_can) {
                Serial.println("Processing CAN command for this luminaire");
            }


            Command cmd;
            if (buildCommandFromSerialType(static_cast<CANSerialMsgType>(receivedFrame.msgType), receivedFrame, frm, cmd)) {
                if (waiting_can==true) {
                    // response for something we sent
                    waiting_can = false;
                }
                executeCommand(cmd);
            }
        }
    }
}

void processirq(){
    uint8_t irq = can0.getInterrupts();

    //get messages from both buffers (if available) and process each one
    can_frame frm{};
    if(irq & MCP2515::CANINTF_RX0IF) {
        can0.readMessage(MCP2515::RXB0, &frm);
        processFrame(frm);
    }
    if(irq & MCP2515::CANINTF_RX1IF) {
        can0.readMessage(MCP2515::RXB1, &frm);
        processFrame(frm);
    }
}