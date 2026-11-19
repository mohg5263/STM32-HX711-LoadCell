#include "cubemars.h"
#include <math.h>

/* AK Series Driver Manual V1.0.18: AK40-10 legacy standard-ID MIT. */
#define AK40_P_MIN (-12.5f)
#define AK40_P_MAX (12.5f)
#define AK40_V_MIN (-45.5f)
#define AK40_V_MAX (45.5f)
#define AK40_T_MIN (-5.0f)
#define AK40_T_MAX (5.0f)

volatile float motor_position_rad = 0.0f;
volatile float motor_target_rad = 0.0f;
volatile float motor_kp = 1.0f;
volatile float motor_kd = 0.2f;
volatile uint8_t motor_position_valid = 0U;
volatile uint32_t motor_last_rx_ms = 0U;
volatile uint32_t motor_rx_count = 0U;

static uint16_t encode(float x, float lo, float hi, uint32_t maxval)
{
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    uint32_t raw = (uint32_t)(((x - lo) * (float)maxval) / (hi - lo) + 0.5f);
    return (uint16_t)(raw > maxval ? maxval : raw);
}

static uint8_t send_frame(CAN_HandleTypeDef *hcan, const uint8_t bytes[8])
{
    if (hcan == NULL || HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
        return 0U;
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    header.StdId = MOTOR_CAN_ID;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)bytes, &mailbox) == HAL_OK;
}

static uint8_t special(CAN_HandleTypeDef *hcan, uint8_t last_byte)
{
    uint8_t frame[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    frame[7] = last_byte;
    return send_frame(hcan, frame);
}
uint8_t CubeMars_Enable(CAN_HandleTypeDef *hcan) { return special(hcan, 0xFCU); }
uint8_t CubeMars_Disable(CAN_HandleTypeDef *hcan) { return special(hcan, 0xFDU); }

static uint8_t send_mit(CAN_HandleTypeDef *hcan, float pos, float kp, float kd)
{
    uint16_t p = encode(pos, AK40_P_MIN, AK40_P_MAX, 65535U);
    uint16_t v = encode(0.0f, AK40_V_MIN, AK40_V_MAX, 4095U);
    uint16_t k = encode(kp, 0.0f, 500.0f, 4095U);
    uint16_t d = encode(kd, 0.0f, 5.0f, 4095U);
    uint16_t t = encode(0.0f, AK40_T_MIN, AK40_T_MAX, 4095U);
    uint8_t frame[8] = {
        (uint8_t)(p >> 8), (uint8_t)p, (uint8_t)(v >> 4),
        (uint8_t)(((v & 15U) << 4) | (k >> 8)), (uint8_t)k,
        (uint8_t)(d >> 4), (uint8_t)(((d & 15U) << 4) | (t >> 8)),
        (uint8_t)t
    };
    return send_frame(hcan, frame);
}
uint8_t CubeMars_SendPassive(CAN_HandleTypeDef *hcan)
{
    return send_mit(hcan, 0.0f, 0.0f, 0.0f);
}
uint8_t CubeMars_SendPosition(CAN_HandleTypeDef *hcan, float pos, float kp, float kd)
{
    if (!isfinite(pos) || pos < AK40_P_MIN || pos > AK40_P_MAX ||
        !isfinite(kp) || kp < 0.0f || kp > 500.0f ||
        !isfinite(kd) || kd < 0.0f || kd > 5.0f)
        return 0U;
    return send_mit(hcan, pos, kp, kd);
}

/* Decode feedback, 8-byte legacy MIT: motor ID + p16 + v12 + torque12 + extras.
 * Reported position is NOT necessarily raw single-turn encoder position. */
void CubeMars_ProcessRx(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    if (header == NULL || data == NULL || header->IDE != CAN_ID_STD ||
        header->RTR != CAN_RTR_DATA || header->DLC != 8U ||
        data[0] != MOTOR_CAN_ID)
        return;
    uint16_t p = ((uint16_t)data[1] << 8) | (uint16_t)data[2];
    motor_position_rad = AK40_P_MIN +
        ((float)p * (AK40_P_MAX - AK40_P_MIN) / 65535.0f);
    motor_last_rx_ms = HAL_GetTick();
    motor_rx_count++;
    motor_position_valid = 1U;
}
