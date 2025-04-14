/*
 * pi.c
 *
 *  Created on: Jan 6, 2025
 *      Author: kajpa
 */

#include "pi.h"

float calculate_control(pi_t* pi, float measured)
{
    float u, u_lim, error, P, I;

    error = pi->setpoint - measured;

    P = pi->p.kp * error;

//    integral = pi->previous_integral + (error + pi->previous_error) * pi->p.dt * 0.5;

    I = (pi->p.ki * (error + pi->previous_error) + pi->p.kc * (pi->previous_u_lim-pi->previous_u)) * pi->p.dt * 0.5 + pi->previous_I;

    u = P + I;
    u_lim = u;
    if(u>1.0)
        u_lim = 1.0;
    else if(u<-1.0)
        u_lim = -1.0;

    pi->previous_error = error;
    pi->previous_I = I;
    pi->previous_u = u;
    pi->previous_u_lim = u_lim;
    return u_lim;
}

pi_t pi =
{
        .p.kp = 2.6,
        .p.ki = 0.1,
        .p.kc = 1.5,
        .p.dt = 0.5,
        .previous_error = 0.0,
        .previous_u = 0.0,
        .previous_u_lim = 0.0,
        .previous_I = 0.0,
		.setpoint = 21.0
};
