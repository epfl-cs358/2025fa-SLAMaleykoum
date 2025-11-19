#include "I2C_mutex.h"
SemaphoreHandle_t i2c_mutex = nullptr;

void i2cMutexInit() {
  if (!i2c_mutex) i2c_mutex = xSemaphoreCreateMutex(); // mutex = priorité héritée
}

