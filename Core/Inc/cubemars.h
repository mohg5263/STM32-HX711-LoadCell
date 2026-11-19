#ifndef CUBEMARS_H
#define CUBEMARS_H
#include "main.h"
#include <stdint.h>

#define MOTOR_CAN_ID 9U

/* Shared application variables. Units are radians, MIT motor position.
 * motor_position_rad is written ONLY by CAN RX ISR.
 * motor_target_rad is assigned by application code / serial command.
 * Valid flag and last_rx_ms distinguish stale/unknown from current data.
 * These are RAM variables, not persistent after MCU power-off. */
extern volatile float motor_position_rad;
extern volatile float motor_target_rad;
extern volatile uint8_t motor_position_valid;
extern volatile uint32_t motor_last_rx_ms;
extern volatile uint32_t motor_rx_count;

uint8_t CubeMars_Enable(CAN_HandleTypeDef *hcan);
uint8_t CubeMars_Disable(CAN_HandleTypeDef *hcan);
uint8_t CubeMars_SendPassive(CAN_HandleTypeDef *hcan);
uint8_t CubeMars_SendPosition(CAN_HandleTypeDef *hcan, float target_rad,
                              float kp, float kd);
void CubeMars_ProcessRx(const CAN_RxHeaderTypeDef *header,
                        const uint8_t data[8]);
#endif
