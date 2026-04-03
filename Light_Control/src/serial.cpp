#include "serial.h"

static bool mapCommandToCanMsgType(const Command& cmd, uint8_t& outMsgType) {
  if (cmd.mainCmd == "u") {
    outMsgType = CAN_MSG_SET_DUTY;
    return true;
  }
  if (cmd.mainCmd == "r") {
    outMsgType = CAN_MSG_SET_ILLUM_REF;
    return true;
  }
  if (cmd.mainCmd == "o") {
    outMsgType = CAN_MSG_SET_OCCUPANCY;
    return true;
  }
  if (cmd.mainCmd == "a") {
    outMsgType = CAN_MSG_SET_ANTI_WINDUP;
    return true;
  }
  if (cmd.mainCmd == "f") {
    outMsgType = CAN_MSG_SET_FEEDBACK;
    return true;
  }
  if (cmd.mainCmd == "g") {
    if (cmd.subCmd == "u") {
      outMsgType = CAN_MSG_GET_DUTY;
      return true;
    }
    if (cmd.subCmd == "r") {
      outMsgType = CAN_MSG_GET_ILLUM_REF;
      return true;
    }
    if (cmd.subCmd == "y") {
      outMsgType = CAN_MSG_GET_LUX;
      return true;
    }
    if (cmd.subCmd == "v") {
      outMsgType = CAN_MSG_GET_LDR_VOLTAGE;
      return true;
    }
    if (cmd.subCmd == "o") {
      outMsgType = CAN_MSG_GET_OCCUPANCY;
      return true;
    }
    if (cmd.subCmd == "a") {
      outMsgType = CAN_MSG_GET_ANTI_WINDUP;
      return true;
    }
    if (cmd.subCmd == "f") {
      outMsgType = CAN_MSG_GET_FEEDBACK;
      return true;
    }
  }

  return false;
}

// Helper function to send GET command responses (via Serial or CAN)
static void sendGetResponse(const Command& cmd, float responseValue) {
  if (cmd.origin == ORIGIN_SERIAL) {
    // Send response via Serial (original behavior)
    Serial.print(cmd.mainCmd);
    Serial.print(" ");
    Serial.print(cmd.luminaireId);
    Serial.print(" ");
    Serial.println(responseValue);
  } else if (cmd.origin == ORIGIN_CAN) {
    // Send response via CAN
    uint8_t msgType = 0;
    if (!mapCommandToCanMsgType(cmd, msgType)) {
      // Cannot map command type - should not happen
      Serial.println("err");
      return;
    }
    encode_and_send(FSERIAL, _luminaireId, cmd.sourceLuminaireId, msgType, responseValue);
  }
}

static void sendGetCharResponse(const Command& cmd, char responseValue) {
  if (cmd.origin == ORIGIN_SERIAL) {
    // Occupancy GET response format: o <i> <val>
    Serial.print("o");
    Serial.print(" ");
    Serial.print(cmd.luminaireId);
    Serial.print(" ");
    Serial.println(responseValue);
  } else if (cmd.origin == ORIGIN_CAN) {
    uint8_t msgType = 0;
    if (!mapCommandToCanMsgType(cmd, msgType)) {
      Serial.println("err");
      return;
    }
    encode_and_send_byte(FSERIAL, _luminaireId, cmd.sourceLuminaireId, msgType, static_cast<uint8_t>(responseValue));
  }
}

// Helper function to send ACK/ERR responses (via Serial or CAN)
static void sendCommandResponse(const Command& cmd, bool success) {
  if (cmd.origin == ORIGIN_SERIAL) {
    // Send response via Serial
    if (success) {
      Serial.println("ack");
    } else {
      Serial.println("err");
    }
  } else if (cmd.origin == ORIGIN_CAN) {
    // For CAN: ACK/ERR is encoded in CAN msgType (no payload needed)
    encode_and_send_status(_luminaireId, cmd.sourceLuminaireId, success);
  }
}

static void printStatus() {
  float duty;
  float referenceLux;
  char occupancyState;
  bool antiWindupEnabled;
  bool feedbackEnabled;
  float pwmValues[3];

  critical_section_enter_blocking(&gStateLock);
  duty = gOutputs.duty;
  referenceLux = gInputs.referenceLux;
  occupancyState = gInputs.occupancyState;
  antiWindupEnabled = gInputs.antiWindupEnabled;
  feedbackEnabled = gInputs.feedbackEnabled;
  for (int i = 0; i < 3; i++) {
    pwmValues[i] = gInputs.pwm[i];
  }
  critical_section_exit(&gStateLock);

  Serial.println("-------------------------------------------------------");
  Serial.println("Status");
  Serial.print("Duty: ");
  Serial.println(duty);
  Serial.print("Reference Lux: ");
  Serial.println(referenceLux);
  Serial.print("Occupancy State: ");
  Serial.println(occupancyState);
  Serial.print("Anti-windup Enabled: ");
  Serial.println(antiWindupEnabled ? "true" : "false");
  Serial.print("Feedback Enabled: ");
  Serial.println(feedbackEnabled ? "true" : "false");

  if (pwmValues[0] != 0.0f || pwmValues[1] != 0.0f || pwmValues[2] != 0.0f) {
    Serial.print("PWM: [");
    for (int i = 0; i < 3; i++) {
      Serial.print(pwmValues[i]);
      if (i < 2) {
        Serial.print(", ");
      }
    }
    Serial.println("]");
  }
}

static void printExternalPwm() {
  float pwmValues[3];

  critical_section_enter_blocking(&gStateLock);
  for (int i = 0; i < 3; i++) {
    pwmValues[i] = gInputs.pwm[i];
  }
  critical_section_exit(&gStateLock);

  Serial.println("-------------------------------------------------------");
  Serial.println("PWM received from other luminaires");
  for (int i = 0; i < 3; i++) {
    if (i == _luminaireId) {
      continue;
    }
    Serial.print("PWM[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(pwmValues[i]);
  }
}

void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() > 0) {
    Command cmd = parseCommand(input);

    // Local-only command: print PWM values received from other nodes.
    if (cmd.mainCmd == "g" && cmd.subCmd == "p") {
      cmd.luminaireId = _luminaireId;
    }

    if (cmd.luminaireId < 0 || cmd.luminaireId >= 3) {
      Serial.println("err");
    } else if (cmd.luminaireId == _luminaireId ) {
      executeCommand(cmd);
    } else {
      // Encode message and forward to destination luminaire via CAN
      uint8_t msgType = 0;
      if (!mapCommandToCanMsgType(cmd, msgType)) {
        Serial.println("err");
        return;
      }
      if (cmd.mainCmd == "o") {
        // Occupancy set carries a character state ('o', 'l', 'h').
        encode_and_send_byte(FSERIAL, _luminaireId, cmd.luminaireId, msgType, static_cast<uint8_t>(cmd.charValue));
      } else if (cmd.mainCmd == "a" || cmd.mainCmd == "f") {
        // Boolean set commands are encoded as a single byte (0/1).
        encode_and_send_byte(FSERIAL, _luminaireId, cmd.luminaireId, msgType, (cmd.value == 1.0f) ? 1 : 0);
      } else {
        encode_and_send(FSERIAL, _luminaireId, cmd.luminaireId, msgType, cmd.value);
      }
      waiting_can = true; // Set flag to indicate we're waiting for a CAN response
    }
  }
}

Command parseCommand(String input) {
  
  Command cmd;
  cmd.mainCmd = "";
  cmd.subCmd = "";
  cmd.luminaireId = -1;
  cmd.value = 0.0f;
  cmd.charValue = '\0';
  cmd.origin = ORIGIN_SERIAL;
  cmd.sourceLuminaireId = _luminaireId;
  
  // Tokenize input
  int tokens[5];
  int tokenCount = 0;
  String token = "";
  
  for (int i = 0; i < input.length() && tokenCount < 5; i++) {
    if (input[i] == ' ') {
      if (token.length() > 0) {
        tokens[tokenCount] = token.length();
        tokenCount++;
        token = "";
      }
    } else {
      token += input[i];
    }
  }
  if (token.length() > 0 && tokenCount < 5) {
    tokens[tokenCount] = token.length();
    tokenCount++;
  }
  
  // Extract tokens from input
  String tokens_str[5];
  int pos = 0;
  for (int i = 0; i < tokenCount; i++) {
    tokens_str[i] = input.substring(pos, pos + tokens[i]);
    pos += tokens[i] + 1;
  }
  
  // Hierarchical decoding
  if (tokenCount >= 1) {
    cmd.mainCmd = tokens_str[0];
    
    // 'g' commands have a subcommand
    if (cmd.mainCmd == "g" && tokenCount >= 2) {
      cmd.subCmd = tokens_str[1];
      
      if (tokenCount >= 3) {
        cmd.luminaireId = tokens_str[2].toInt();
      }
      if (tokenCount >= 4) {
        cmd.value = tokens_str[3].toFloat();
      }
    }
    // 's' (stream start) and 'S' (stream stop)
    else if ((cmd.mainCmd == "s" || cmd.mainCmd == "S") && tokenCount >= 2) {
      cmd.subCmd = tokens_str[1];
      if (tokenCount >= 3) {
        cmd.luminaireId = tokens_str[2].toInt();
      }
    }
    // Other commands
    else if (tokenCount >= 2) {
      cmd.luminaireId = tokens_str[1].toInt();
      if (tokenCount >= 3) {
        String valStr = tokens_str[2];
        // Check if it's a single character (for occupancy)
        if (valStr.length() == 1 && !isDigit(valStr[0])) {
          cmd.charValue = valStr[0];
        } else {
          cmd.value = valStr.toFloat();
        }
      }
    }
  }
  
  return cmd;
}

void executeCommand(Command cmd) {

  if (cmd.mainCmd == "status" || cmd.mainCmd == "p") {
    printStatus();
  }

  // GET commands (main command 'g')
  else if (cmd.mainCmd == "g") {
    
    if (cmd.subCmd == "u") {
      // Get duty cycle: g u <i>
      float dutyValue;

      critical_section_enter_blocking(&gStateLock);
      dutyValue = gOutputs.duty;
      critical_section_exit(&gStateLock);
      
      sendGetResponse(cmd, dutyValue);
    }

    else if (cmd.subCmd == "r") {
      // Get reference lux: g r <i>
      float refValue;
      
      critical_section_enter_blocking(&gStateLock);
      refValue = gInputs.referenceLux;
      critical_section_exit(&gStateLock);
      
      sendGetResponse(cmd, refValue);
    }

    else if (cmd.subCmd == "y") {
      // Get measured lux: g y <i>
      float lux;
      
      critical_section_enter_blocking(&gStateLock);
      lux = gOutputs.luxMeasured;
      critical_section_exit(&gStateLock);
      
      sendGetResponse(cmd, lux);
    }

    else if (cmd.subCmd == "v") {
      // Get voltage at LDR: g v <i>
      float v_out;
      
      critical_section_enter_blocking(&gStateLock);
      v_out = gOutputs.ldrVoltage;
      critical_section_exit(&gStateLock);
      
      sendGetResponse(cmd, v_out);
    }

    else if (cmd.subCmd == "o") {
      // Get occupancy state: g o <i>
      char occupancyState;

      critical_section_enter_blocking(&gStateLock);
      occupancyState = gInputs.occupancyState;
      critical_section_exit(&gStateLock);

      sendGetCharResponse(cmd, occupancyState);
    }

    else if (cmd.subCmd == "a") {
      // Get anti-windup state: g a <i>
      bool antiWindupEnabled;
      critical_section_enter_blocking(&gStateLock);
      antiWindupEnabled = gInputs.antiWindupEnabled;
      critical_section_exit(&gStateLock);
      sendGetResponse(cmd, antiWindupEnabled ? 1.0f : 0.0f);
    }

    else if (cmd.subCmd == "f") {
      // Get feedback control state: g f <i>
      bool feedbackEnabled;
      critical_section_enter_blocking(&gStateLock);
      feedbackEnabled = gInputs.feedbackEnabled;
      critical_section_exit(&gStateLock);
      sendGetResponse(cmd, feedbackEnabled ? 1.0f : 0.0f);
    }

    else if (cmd.subCmd == "w") {
      // Get setpoint weighting: g w <i>
      float beta = controller.getWeight(PID::BETA);
      sendGetResponse(cmd, beta);
    }

    else if (cmd.subCmd == "d") {
      // Get external luminance: g d <i>
      float externalLuminance = controller.getExternalLuminance();
      sendGetResponse(cmd, externalLuminance);
    }

    else if (cmd.subCmd == "p") {
      // Local command: print received PWM values from other luminaires.
      printExternalPwm();
      
    }
  }
  // SET commands
  else if (cmd.mainCmd == "u") {
    // Set duty cycle: u <i> <val>
    // Activates manual override mode (independent of feedback state).
    if (cmd.value >= 0.0f && cmd.value <= 1.0f) {
      critical_section_enter_blocking(&gStateLock);
      gPending.hasDuty = true;
      gPending.newDuty = cmd.value;
      gPending.hasManualOverride = true;
      gPending.newManualOverride = true;
      critical_section_exit(&gStateLock);
      sendCommandResponse(cmd, true);
    } else {
      sendCommandResponse(cmd, false);
    }
  }
  else if (cmd.mainCmd == "r") {
    // Set reference lux: r <i> <val>
    if (cmd.luminaireId < 3) {
      critical_section_enter_blocking(&gStateLock);
      gPending.hasReferenceLux = true;
      gPending.newReferenceLux = cmd.value;
      critical_section_exit(&gStateLock);
      sendCommandResponse(cmd, true);
    } else {
      sendCommandResponse(cmd, false);
    }
  }
  else if (cmd.mainCmd == "o") {
    // Set occupancy state: o <i> <val>
    if (cmd.luminaireId < 3 && (cmd.charValue == 'o' || cmd.charValue == 'l' || cmd.charValue == 'h')) {
      critical_section_enter_blocking(&gStateLock);
      gPending.hasOccupancyState = true;
      gPending.newOccupancyState = cmd.charValue;
      critical_section_exit(&gStateLock);
      sendCommandResponse(cmd, true);
    } else {
      sendCommandResponse(cmd, false);
    }
  }
  else if (cmd.mainCmd == "a") {
    // Set anti-windup: a <i> <val>
    if (cmd.luminaireId < 3 && (cmd.value == 0.0f || cmd.value == 1.0f)) {
      critical_section_enter_blocking(&gStateLock);
      gPending.hasAntiWindupEnabled = true;
      gPending.newAntiWindupEnabled = (cmd.value == 1.0f);
      critical_section_exit(&gStateLock);
      sendCommandResponse(cmd, true);
    } else {
      sendCommandResponse(cmd, false);
    }
  }
  else if (cmd.mainCmd == "f") {
    // Set feedback control: f <i> <val>
    // Feedback is independent of manual override.
    if (cmd.luminaireId < 3 && (cmd.value == 0.0f || cmd.value == 1.0f)) {
      critical_section_enter_blocking(&gStateLock);
      gPending.hasFeedbackEnabled = true;
      gPending.newFeedbackEnabled = (cmd.value == 1.0f);
      critical_section_exit(&gStateLock);
      sendCommandResponse(cmd, true);
    } else {
      sendCommandResponse(cmd, false);
    }
  }
  else if (cmd.mainCmd == "m") {
    // Set manual override: m <i> <val>
    // m <i> 0 = disable override, m <i> 1 = enable override
    if (cmd.luminaireId < 3 && (cmd.value == 0.0f || cmd.value == 1.0f)) {
      critical_section_enter_blocking(&gStateLock);
      gPending.hasManualOverride = true;
      gPending.newManualOverride = (cmd.value == 1.0f);
      critical_section_exit(&gStateLock);
      sendCommandResponse(cmd, true);
    } else {
      sendCommandResponse(cmd, false);
    }
  }
  /*else if (cmd.mainCmd == "w") {
    // Set setpoint weighting: w <i> <val>
    if (cmd.luminaireId == 0) {
      critical_section_enter_blocking(&gStateLock);
      gPending.hasBeta = true;
      gPending.newBeta = cmd.value;
      critical_section_exit(&gStateLock);
      sendCommandResponse(cmd, true);
    } else {
      sendCommandResponse(cmd, false);
    }
  }*/
  else {
    sendCommandResponse(cmd, false);
  }
}



void print_to_serial() {
  uint32_t timestamp;
  float luxRef;
  float luxMeasured;
  float ldrVoltage;
  float ldrResistance;
  float duty;
  bool antiWindup;
  float beta = controller.getWeight(PID::BETA);

  critical_section_enter_blocking(&gStateLock);
  timestamp = gOutputs.timestampMs;
  luxRef = gInputs.referenceLux;
  luxMeasured = gOutputs.luxMeasured;
  ldrVoltage = gOutputs.ldrVoltage;
  ldrResistance = gOutputs.ldrResistance;
  duty = gOutputs.duty;
  antiWindup = gInputs.antiWindupEnabled;
  critical_section_exit(&gStateLock);

  //time,luminaire_ID,lux_ref,lux_meas,LDRvoltage,LDRresistance,duty_cycle,windup_state,setpoint-weight
  Serial.print(timestamp);
  Serial.print(",");
  Serial.print(_luminaireId);
  Serial.print(",");
  Serial.print(luxRef);
  Serial.print(",");
  Serial.print(luxMeasured);
  Serial.print(",");
  Serial.print(ldrVoltage);
  Serial.print(",");
  Serial.print(ldrResistance);
  Serial.print(",");
  Serial.print(duty);
  Serial.print(",");
  Serial.print(antiWindup);
  Serial.print(",");
  Serial.println(beta);
}