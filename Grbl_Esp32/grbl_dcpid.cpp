#include "grbl.h"

#ifdef USE_PIDCONTROL

bool pid_ready_state = true;
bool grbl_pid_idle = false;
bool grbl_pid_running = false;
bool pid_busy = false;

void IRAM_ATTR onPIDDriverTimer(void *para){

    TIMERG0.int_clr_timers.t1 = 1;
    if(pid_busy){
        Serial.println("Attempted to enter PID interrupt too early");
        return;
    }
    pid_busy = true;
#ifndef MASLOW_DEBUG
    compute_pid();
    pid_ready_state = machine_regulation();
#else
    Serial.println("PID triggered");
#endif
    TIMERG0.hw_timer[PID_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;
    // timer_set_alarm(PID_TIMER_GROUP, PID_TIMER_INDEX);
    pid_busy = false;
}

bool pid_ready(){
    return pid_ready_state;
}

void pid_go_idle(){
    grbl_pid_idle = true;
}

void pid_wake_up(){
    grbl_pid_idle = false;
}

void update_motors_pid(uint8_t step_mask, uint8_t dir_mask){
#ifndef MASLOW_DEBUG
    pid_step(step_mask, dir_mask);
#else
    Serial.printf("Steps: %X, Directions: %X\n", step_mask, dir_mask);
#endif
}

void update_motors_state(){
    pid_get_state();
}

void pid_init(){
    timer_config_t config;
    config.divider     = F_TIMERS / F_STEPPER_TIMER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en  = TIMER_PAUSE;
    config.alarm_en    = TIMER_ALARM_EN;
    config.intr_type   = TIMER_INTR_LEVEL;
    config.auto_reload = true;

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
    if(!grbl_pid_running){
        timer_set_counter_value(PID_TIMER_GROUP, PID_TIMER_INDEX, 0x00000000ULL);
        timer_start(PID_TIMER_GROUP, PID_TIMER_INDEX);
        TIMERG0.hw_timer[PID_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;
        // timer_set_alarm(PID_TIMER_GROUP, PID_TIMER_INDEX);
        grbl_pid_running = true;
        Serial.println("PID Start");
    }

}

void IRAM_ATTR PID_Timer_Stop()
{
    if(grbl_pid_running){
        Serial.println("PID Stop");
        timer_pause(PID_TIMER_GROUP, PID_TIMER_INDEX);
        motor_stop();
        grbl_pid_running = false;
    }

}
#endif
