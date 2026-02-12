
#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

void mg50s_set_speed(int8_t speed_percent);
esp_err_t servo_mcpwm_init(void);
void mg50s_stop(void);

#endif