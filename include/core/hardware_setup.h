#pragma once
#include <stdint.h>
#include "hardware/gpio.h"

bool init_hardware();
void mpu_isr(uint gpio, uint32_t events);
