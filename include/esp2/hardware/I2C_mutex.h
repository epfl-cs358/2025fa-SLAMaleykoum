#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern SemaphoreHandle_t i2c_mutex;
void i2cMutexInit();


// Use these around any I2C access
inline void i2c_lock() {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
}

inline void i2c_unlock() {
    xSemaphoreGive(i2c_mutex);
}