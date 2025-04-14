/*
 * pi.h
 *
 *  Created on: Jan 6, 2025
 *      Author: kajpa
 */

#ifndef SRC_PI_H_
#define SRC_PI_H_

typedef struct
{
    float kp;
    float ki;
    float kc;
    float dt;
}pi_parameters_t;

typedef struct
{
    pi_parameters_t p;
    float previous_error;
    float previous_u;
    float previous_u_lim;
    float previous_I;
    float setpoint;
}pi_t;

float calculate_control(pi_t* pi, float measured);

extern pi_t pi;

#endif /* SRC_PI_H_ */
