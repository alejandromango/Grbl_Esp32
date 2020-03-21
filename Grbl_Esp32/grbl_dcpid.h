#ifndef grbl_dcpid_h
#define grbl_dcpid_h

#include "grbl.h"
#include "config.h"
#include "maslow_v2.h"

machine *mill;

void PID_Timer_WritePeriod(uint64_t alarm_val);
void PID_Timer_Start();
void PID_Timer_Stop();

void pid_init(machine *device);
void update_motors_pid(uint8_t step_mask, uint8_t dir_mask);


// -- Task handles for use in the notifications
// Why are these here? Just copied format from stepper timer, they're not used
void IRAM_ATTR onPIDTimer();
void IRAM_ATTR onPIDOffTimer();

#define PID_TIMER_GROUP TIMER_GROUP_0
#define PID_TIMER_INDEX TIMER_1

#endif

