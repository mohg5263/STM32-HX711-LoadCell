/*
 * hx711.h
 *
 *  Created on: Nov 16, 2026
 *      Author: mghan
 */


#ifndef HX711_H
#define HX711_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>


typedef struct
{
    int32_t offset;
    float scale;  // ADC counts per pound
} HX711_t;

bool HX711_ReadRaw(int32_t *value);
bool HX711_ReadAverage(uint8_t samples, int32_t *average);

bool HX711_Tare(HX711_t *hx);
bool HX711_Calibrate(HX711_t *hx, float known_weight_lb);
bool HX711_GetWeight(HX711_t *hx, float *weight_lb);

#endif

