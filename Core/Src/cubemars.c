#include "cubemars.h"

/* AK Series Driver Manual V1.0.18, pages 60-63, legacy MIT mode.
 * AK40-10 ranges. Decoded position is not raw encoder counts.
 * Motor only receives a special enable/disable command on explicit UART request.
 */
#define AK40_P_MIN (-12.5f)
#define AK40_P_MAX (12.5f)
#define AK40_V_MIN (-45.5f)
#define AK40_V_MAX (45.5f)
#define AK40_T_MIN (-5.0f)
#define AK40_T_MAX (5.0f)

static volatile CubeMars_Feedback_t feedback;

static float decode_u(uint32_t encoded, float min, float max, uint32_t max_encoded)
{
    return min + ((float)encoded * (max - min) / (float)max_encoded);
}

void CubeMars_ProcessRx(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    if (header == NULL || data == NULL) return;
    feedback.all_frames++;
    feedback.last_ide = (uint8_t)(header->IDE == CAN_ID_EXT);
    feedback.last_dlc = (uint8_t)header->DLC;
    feedback.last_can_id = (header->IDE == CAN_ID_STD) ? header->StdId : header->ExtId;

    /* Earlier working eight-motor firmware identifies replies by data[0],
     * not by header.StdId. Accept any standard-ID 8-byte frame with payload
     * motor ID 9; preserve header ID in diagnostics for verification.
     */
    if (header->IDE != CAN_ID_STD || header->RTR != CAN_RTR_DATA)
    {
        feedback.other_frames++;
        return;
    }
    if (header->DLC != 8U)
    {
        feedback.invalid_frames++;
        return;
    }
    if (data[0] != MOTOR_CAN_ID)
    {
        feedback.other_frames++;
        return;
    }

    uint32_t p = ((uint32_t)data[1] << 8) | data[2];
    uint32_t v = ((uint32_t)data[3] << 4) | (data[4] >> 4);
    uint32_t t = (((uint32_t)data[4] & 0xFU) << 8) | data[5];

    feedback.position_rad = decode_u(p, AK40_P_MIN, AK40_P_MAX, 65535U);
    feedback.velocity_rad_s = decode_u(v, AK40_V_MIN, AK40_V_MAX, 4095U);
    feedback.torque_nm = decode_u(t, AK40_T_MIN, AK40_T_MAX, 4095U);
    feedback.temperature_raw = data[6];
    feedback.error_raw = data[7];
    feedback.last_rx_ms = HAL_GetTick();
    feedback.matching_frames++;
    feedback.feedback_valid = 1U;
}

void CubeMars_GetFeedback(CubeMars_Feedback_t *out)
{
    if (out == NULL) return;
    /* Very short critical section to prevent partially copied ISR fields. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out->position_rad = feedback.position_rad;
    out->velocity_rad_s = feedback.velocity_rad_s;
    out->torque_nm = feedback.torque_nm;
    out->temperature_raw = feedback.temperature_raw;
    out->error_raw = feedback.error_raw;
    out->last_rx_ms = feedback.last_rx_ms;
    out->matching_frames = feedback.matching_frames;
    out->all_frames = feedback.all_frames;
    out->invalid_frames = feedback.invalid_frames;
    out->other_frames = feedback.other_frames;
    out->last_can_id = feedback.last_can_id;
    out->last_ide = feedback.last_ide;
    out->last_dlc = feedback.last_dlc;
    out->feedback_valid = feedback.feedback_valid;
    __set_PRIMASK(primask);
}

/* Transmit one legacy-MIT special command; NEVER wait for a mailbox.
 * FC enable, FD disable. Queueing is not proof of delivery or execution.
 */
static uint8_t CubeMars_SendSpecial(CAN_HandleTypeDef *hcan, uint8_t command)
{
    if (hcan == NULL) return 0U;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) return 0U;

    CAN_TxHeaderTypeDef header = {0};
    uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint32_t mailbox;
    header.StdId = MOTOR_CAN_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    data[7] = command;
    return (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) == HAL_OK) ? 1U : 0U;
}

uint8_t CubeMars_EnableOnce(CAN_HandleTypeDef *hcan)
{
    return CubeMars_SendSpecial(hcan, 0xFCU);
}

uint8_t CubeMars_DisableOnce(CAN_HandleTypeDef *hcan)
{
    return CubeMars_SendSpecial(hcan, 0xFDU);
}

/* Reference: user's working eight-actuator legacy MIT protocol:
 * p16 | v12 | kp12 | kd12 | torque12, motor CAN standard ID = 9.
 * This command requests ZERO kp, kd, velocity and feedforward torque.
 * Position field is set to the center of the allowed encoding, but with
 * KP=0 it does not command a position-holding torque.
 * All fields within 0..(2^bits-1). Do not transmit from an interrupt.
 */
static uint16_t CubeMars_Encode(float val, float low, float high, uint32_t mask)
{
    if (val < low) val = low;
    if (val > high) val = high;
    float f = (val - low) * (float)mask / (high - low);
    uint32_t enc = (uint32_t)(f + 0.5f);
    if (enc > mask) enc = mask;
    return (uint16_t)enc;
}

uint8_t CubeMars_SendPassiveMIT(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
        return 0U;

    uint16_t p = CubeMars_Encode(0.0f, AK40_P_MIN, AK40_P_MAX, 65535U);
    uint16_t v = CubeMars_Encode(0.0f, AK40_V_MIN, AK40_V_MAX, 4095U);
    uint16_t t = CubeMars_Encode(0.0f, AK40_T_MIN, AK40_T_MAX, 4095U);
    uint16_t kp = 0U, kd = 0U;

    uint8_t data[8] = {
        (uint8_t)(p >> 8),
        (uint8_t)(p & 0xFFU),
        (uint8_t)(v >> 4),
        (uint8_t)(((v & 0xFU) << 4) | (kp >> 8)),
        (uint8_t)(kp & 0xFFU),
        (uint8_t)(kd >> 4),
        (uint8_t)(((kd & 0xFU) << 4) | (t >> 8)),
        (uint8_t)(t & 0xFFU)
    };
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox = 0;
    header.StdId = MOTOR_CAN_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    return (HAL_CAN_AddTxMessage(hcan, &header, data, &mailbox) == HAL_OK) ? 1U : 0U;
}
