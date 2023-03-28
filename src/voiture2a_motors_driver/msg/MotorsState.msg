# Values of the PWM send to the servo and the engine (in 5*ms)
uint16[2] pwm_value

# Values of the Remote Control channel (SBUS protocol)
uint16[18] channels

# Additional values of the Remote Control
uint8 failsafe
uint8 lost

# Battery voltage
float32 battery