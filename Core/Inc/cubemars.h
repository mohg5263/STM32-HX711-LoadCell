#ifndef CUBEMARS_H
#define CUBEMARS_H

#include "main.h"
#include <stdint.h>

#define MOTOR_CAN_ID 9U

/* Legacy MIT feedback monitor with explicit enable/disable test commands. */
typedef struct {
    float position_rad;   /* Decoded MIT position, NOT raw absolute encoder counts */
    float velocity_rad_s;
    float torque_nm;
    uint8_t temperature_raw; /* Temperature encoding not established by this manual */
    uint8_t error_raw;
    uint32_t last_rx_ms;
    uint32_t matching_frames;
    uint32_t all_frames;
    uint32_t invalid_frames;
    uint32_t other_frames;
    uint32_t last_can_id;
    uint8_t last_ide;
    uint8_t last_dlc;
    uint8_t feedback_valid;
} CubeMars_Feedback_t;

/* Non-blocking one-shot legacy MIT special commands for CAN ID 9. */
uint8_t CubeMars_EnableOnce(CAN_HandleTypeDef *hcan);
uint8_t CubeMars_DisableOnce(CAN_HandleTypeDef *hcan);

/* Called in CAN RX interrupt; does not transmit. */
void CubeMars_ProcessRx(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);
/* Atomic snapshot for main-loop reporting. */
void CubeMars_GetFeedback(CubeMars_Feedback_t *out);

#endif /* CUBEMARS_H */
