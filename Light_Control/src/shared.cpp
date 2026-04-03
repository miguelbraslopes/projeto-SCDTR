#include "shared.h"

volatile ControlInputs gInputs = {20.0f, 'o', true, true, false, {0.0f, 0.0f, 0.0f}};
volatile ControlOutputs gOutputs = {0, 0.0f, 0.0f, 0.0f, 0.0f};
volatile PendingCommands gPending = {};
critical_section_t gStateLock;
bool waiting_can = false;

void initSharedState() {
  critical_section_init(&gStateLock);
}