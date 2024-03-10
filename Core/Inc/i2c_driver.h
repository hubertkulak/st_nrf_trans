#include "stm32l4xx_hal.h"
#include "i2c.h"
#include <stdlib.h>
#include <string.h>

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);

void user_delay_ms(uint32_t period);

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);

