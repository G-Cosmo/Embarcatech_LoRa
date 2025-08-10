#ifndef ALERTS_H
#define ALERTS_H

#include <stdbool.h>
#include "bmp280.h"
#include "aht20.h"

// Struct para as flags de alerta
typedef struct {
    bool aht20_temperature;
    bool aht20_humidity;
    bool bmp280_pressure;
    bool bmp280_temperature;
} Sensor_alerts_t;

typedef struct{
    float min;
    float max;
    float offset;
} SensorParams_t;

typedef struct{
    SensorParams_t AHT20_temperature;
    SensorParams_t AHT20_humidity;
    SensorParams_t BMP280_pressure;
    SensorParams_t BMP280_temperature;
} ConfigParams_t;

void alerts_handle(Sensor_alerts_t *sensor_alerts, ConfigParams_t config_params, BMP280_data_t BMP280_data, AHT20_data_t AHT20_data);

#endif