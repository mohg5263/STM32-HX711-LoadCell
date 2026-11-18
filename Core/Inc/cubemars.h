/*
 * cubemars.h
 *
 *  Created on: Nov 18, 2026
 *      Author: mghan
 */


#ifndef CUBEMARS_H
#define CUBEMARS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define MOTOR_CAN_ID 9U

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;

    uint32_t last_rx_ms;
    bool feedback_valid;

} CubeMars_Feedback_t;

typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float kp;
    float kd;
    float torque_nm;

} CubeMars_Command_t;

bool CubeMars_Enable(void);
bool CubeMars_Disable(void);
bool CubeMars_SetZero(void);

bool CubeMars_SendMIT(
    const CubeMars_Command_t *command
);

void CubeMars_ProcessFeedback(
    const CAN_RxHeaderTypeDef *header,
    const uint8_t data[8]
);

#endif
