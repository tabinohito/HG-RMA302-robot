#ifndef POWER_INA226_H
#define POWER_INA226_H
#include "hardware/i2c.h"
#include "power_config.h"
#include "monitor.h"

typedef struct { uint8_t address; uint16_t die_id; bool found; } ina226_t;

static inline bool ina226_read(uint8_t address, uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    if (i2c_write_timeout_us(POWER_I2C_ID, address, &reg, 1, true, POWER_I2C_TIMEOUT_US) != 1 ||
        i2c_read_timeout_us(POWER_I2C_ID, address, data, 2, false, POWER_I2C_TIMEOUT_US) != 2)
        return false;
    *value = (uint16_t)((uint16_t)data[0] << 8 | data[1]);
    return true;
}

static inline bool ina226_find(ina226_t *sensor) {
    sensor->found = false;
    for (uint8_t address = 0x40; address <= 0x4f; ++address) {
        uint16_t manufacturer, die;
        if (!ina226_read(address, 0xfe, &manufacturer) || manufacturer != 0x5449 ||
            !ina226_read(address, 0xff, &die) || (die & 0xfff0) != 0x2260) continue;
        // 16-sample averaging, 1.1ms bus + 1.1ms shunt, continuous both.
        uint8_t config[] = {0x00, 0x45, 0x27};
        if (i2c_write_timeout_us(POWER_I2C_ID, address, config, 3, false, POWER_I2C_TIMEOUT_US) != 3)
            return false;
        sensor->address = address; sensor->die_id = die; sensor->found = true;
        return true;
    }
    return false;
}

static inline bool ina226_sample(ina226_t *sensor, power_measurement_t *measurement) {
    uint16_t bus, shunt;
    if (!sensor->found || !ina226_read(sensor->address, 0x02, &bus) || (bus & 0x8000) ||
        !ina226_read(sensor->address, 0x01, &shunt)) return false;
    *measurement = power_measure(bus, shunt, POWER_SHUNT_UOHM);
    return true;
}
#endif
