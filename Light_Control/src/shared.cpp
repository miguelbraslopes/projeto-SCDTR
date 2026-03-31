#include "shared.h"

volatile ControlInputs gInputs = {20.0f, 1.0f, true, true, true, 'o'};
volatile ControlOutputs gOutputs = {0, 0.0f, 0.0f, 0.0f, 0.0f};
volatile PendingCommands gPending = {};
critical_section_t gStateLock;
bool waiting_can = false;

void initSharedState() {
  critical_section_init(&gStateLock);
}