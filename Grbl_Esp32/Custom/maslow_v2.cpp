#ifdef maslow_v2_h
esp_adc_cal_characteristics_t *adc_1_characterisitics = (esp_adc_cal_characteristics_t*) calloc(1, sizeof(esp_adc_cal_characteristics_t));
esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, adc_1_characterisitics);
esp_err_t config_err_0 = adc1_config_width(ADC_WIDTH_BIT_12);
esp_err_t config_err_1 = adc1_config_channel_atten(MOTOR_1_ADC, ADC_ATTEN_DB_2_5);
esp_err_t config_err_2 = adc1_config_channel_atten(MOTOR_2_ADC, ADC_ATTEN_DB_2_5);
esp_err_t config_err_3 = adc1_config_channel_atten(MOTOR_3_ADC, ADC_ATTEN_DB_2_5);
esp_err_t config_err_4 = adc1_config_channel_atten(MOTOR_4_ADC, ADC_ATTEN_DB_2_5);
esp_err_t config_err_5 = adc1_config_channel_atten(MOTOR_5_ADC, ADC_ATTEN_DB_2_5);
TLC59711 tlc(NUM_TLC59711, TLC_CLOCK, TLC_DATA); // MASLOW_TODO: Will this get garbage collected?
MotorUnit motor1(&tlc, MOTOR_1_FORWARD, MOTOR_1_BACKWARD, MOTOR_1_ADC, RSENSE, adc_1_characterisitics, MOTOR_1_CS, DC_TOP_LEFT_MM_PER_REV, 1);
MotorUnit motor2(&tlc, MOTOR_2_FORWARD, MOTOR_2_BACKWARD, MOTOR_2_ADC, RSENSE, adc_1_characterisitics, MOTOR_2_CS, DC_TOP_RIGHT_MM_PER_REV, 1);
MotorUnit motor3(&tlc, MOTOR_3_FORWARD, MOTOR_3_BACKWARD, MOTOR_3_ADC, RSENSE, adc_1_characterisitics, MOTOR_3_CS, DC_BOTTOM_LEFT_MM_PER_REV, 1);
MotorUnit motor4(&tlc, MOTOR_4_FORWARD, MOTOR_4_BACKWARD, MOTOR_4_ADC, RSENSE, adc_1_characterisitics, MOTOR_4_CS, DC_BOTTOM_RIGHT_MM_PER_REV, 1);
MotorUnit motor5(&tlc, MOTOR_5_FORWARD, MOTOR_5_BACKWARD, MOTOR_5_ADC, RSENSE, adc_1_characterisitics, MOTOR_5_CS, DC_Z_AXIS_MM_PER_REV, 1);

static float x_max = 2438.4; // 8 feet in mm
static float y_max = 1219.2; // 4 feet in mm


void machine_init(){
    tlc.begin();
    tlc.write();
    Serial.println("machine setup complete");

}

/*
 Apply inverse kinematics for maslow CNC
 Take X, Y, and Z value and

*/
void inverse_kinematics(float *target, plan_line_data_t *pl_data, float *position) {
    //static float last_angle = 0;
    //static float last_radius = 0;
    float x, y, z;                  // distances in each cartesian axis
    float maslow_target[N_AXIS];    // target location in maslow cable lengths
    //grbl_sendf(CLIENT_SERIAL, "Position: %4.2f %4.2f %4.2f \r\n", position[X_AXIS] - x_offset, position[Y_AXIS], position[Z_AXIS]);
    //grbl_sendf(CLIENT_SERIAL, "Target: %4.2f %4.2f %4.2f \r\n", target[X_AXIS] - x_offset, target[Y_AXIS], target[Z_AXIS]);
    x = target[X_AXIS];
    y = target[Y_AXIS];
    z = target[Z_AXIS];

    // MASLOWTODO: Drop equations for each of the cable lengths here.
    // maslow_target[DC_TOP_LEFT] = sqrt((x - X_TL_OFFSET)^2 + (y - Y_TL_OFFSET)^2);
    // maslow_target[DC_TOP_RIGHT] = sqrt((x_max - x - X_TR_OFFSET)^2 + (y - Y_TR_OFFSET)^2);
    // maslow_target[DC_BOTTOM_LEFT] = sqrt((x - X_BL_OFFSET)^2 + (y - y_max - Y_BL_OFFSET)^2);
    // maslow_target[DC_BOTTOM_RIGHT] = sqrt((x- x_max - X_BR_OFFSET)^2 + (y - y_max - Y_BR_OFFSET)^2);
    // maslow_target[DC_Z_AXIS] = z;

    mc_line(maslow_target, pl_data);
}

/*
 Forward kinematics for maslow CNC
 MASLOWTODO: Back calculate the XYZ position based on the actual cable lengths.
*/
void forward_kinematics(float *position) {
    float original_position[N_AXIS];  // temporary storage of original
    float print_position[N_AXIS];
    int32_t current_position[N_AXIS];  // Copy current state of the system position variable
    memcpy(current_position, sys_position, sizeof(sys_position));
    system_convert_array_steps_to_mpos(print_position, current_position);

}

void pid_step(uint8_t step_mask, uint8_t dir_mask){
    motor1.step(step_mask & (1<<DC_TOP_LEFT), dir_mask & (1<<DC_TOP_LEFT), 1.0/DC_TOP_LEFT_STEPS_PER_MM);
    motor2.step(step_mask & (1<<DC_TOP_RIGHT), dir_mask & (1<<DC_TOP_RIGHT), 1.0/DC_TOP_RIGHT_STEPS_PER_MM);
    motor3.step(step_mask & (1<<DC_BOTTOM_LEFT), dir_mask & (1<<DC_BOTTOM_LEFT), 1.0/DC_BOTTOM_LEFT_STEPS_PER_MM);
    motor4.step(step_mask & (1<<DC_BOTTOM_RIGHT), dir_mask & (1<<DC_BOTTOM_RIGHT), 1.0/DC_BOTTOM_RIGHT_STEPS_PER_MM);
    motor5.step(step_mask & (1<<DC_Z_AXIS), dir_mask & (1<<DC_Z_AXIS), 1.0/DC_Z_AXIS_STEPS_PER_MM);

}

void pid_get_state(){
    motor1.updateControllerState();
    motor2.updateControllerState();
    motor3.updateControllerState();
    motor4.updateControllerState();
    motor5.updateControllerState();
}

void motor_stop(){
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    motor5.stop();
}

void compute_pid(){
    motor1.computePID();
    motor2.computePID();
    motor3.computePID();
    motor4.computePID();
    motor5.computePID();
}

void update_setpoints(double setpoint_1,
                               double setpoint_2,
                               double setpoint_3,
                               double setpoint_4,
                               double setpoint_5){
    motor1.setSetpoint(setpoint_1);
    motor2.setSetpoint(setpoint_2);
    motor3.setSetpoint(setpoint_3);
    motor4.setSetpoint(setpoint_4);
    motor5.setSetpoint(setpoint_5);
}

bool machine_regulation(){
    return (motor1.getRegulationState() &
            motor2.getRegulationState() &
            motor3.getRegulationState() &
            motor4.getRegulationState() &
            motor5.getRegulationState());
}

void print_setpoints(){
    Serial.printf("Motor setpoints: %g, %g, %g, %g, %g\n",
                                    motor1.getSetpoint(),
                                    motor2.getSetpoint(),
                                    motor3.getSetpoint(),
                                    motor4.getSetpoint(),
                                    motor5.getSetpoint());
}

void update_pid_tunes(double new_p, double new_i, double new_d){
    motor1.setPIDTune(new_p, new_i, new_d);
    motor2.setPIDTune(new_p, new_i, new_d);
    motor3.setPIDTune(new_p, new_i, new_d);
    motor4.setPIDTune(new_p, new_i, new_d);
    motor5.setPIDTune(new_p, new_i, new_d);
}

void update_control_mode(pid_mode new_mode){
    motor1.setControlMode(new_mode);
    motor2.setControlMode(new_mode);
    motor3.setControlMode(new_mode);
    motor4.setControlMode(new_mode);
    motor5.setControlMode(new_mode);
}
#endif
