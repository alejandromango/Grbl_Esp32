
#include grbl.h

#ifdef USE_PIDCONTROL
#include "grbl_dcpid.h"
void IRAM_ATTR onPIDDriverTimer(void *para){
    compute_pid();
}


void update_motors_pid(uint8_t step_mask, uint8_t dir_mask){
    dc_motor_step(step_mask, dir_mask);
}

void pid_init(){
    timer_init(PID_TIMER_GROUP, PID_TIMER_INDEX, &config);
    timer_set_counter_value(PID_TIMER_GROUP, PID_TIMER_INDEX, 0x00000000ULL);
    timer_enable_intr(PID_TIMER_GROUP, PID_TIMER_INDEX);
    timer_isr_register(PID_TIMER_GROUP, PID_TIMER_INDEX, onPIDDriverTimer, NULL, 0, NULL);
    uint32_t PID_cycles = TICKS_PER_MICROSECOND * 10000; // Run PID cycle every 10 ms
    PID_Timer_WritePeriod(PID_cycles);
}

void IRAM_ATTR PID_Timer_WritePeriod(uint64_t alarm_val)
{
	timer_set_alarm_value(PID_TIMER_GROUP, PID_TIMER_INDEX, alarm_val);
}

void IRAM_ATTR PID_Timer_Start()
{
#ifdef ESP_DEBUG
	//Serial.println("ST Start");
#endif

	timer_set_counter_value(PID_TIMER_GROUP, PID_TIMER_INDEX, 0x00000000ULL);

	timer_start(PID_TIMER_GROUP, PID_TIMER_INDEX);
	TIMERG0.hw_timer[PID_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;

}

void IRAM_ATTR PID_Timer_Stop()
{
#ifdef ESP_DEBUG
	//Serial.println("ST Stop");
#endif

	timer_pause(PID_TIMER_GROUP, PID_TIMER_INDEX);

}
#endif
