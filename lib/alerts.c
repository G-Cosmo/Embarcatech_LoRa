#include "alerts.h"
#include "bmp280.h"
#include "aht20.h"
#include "hardware/pwm.h"

bool alert_check(float param, float value, bool isMin){
    if(isMin){
        if(value<param) return true;
        return false;
    }
    else{
        if(value>param) return true;
        return false;
    }
}

void alerts_handle(Sensor_alerts_t *sensor_alerts, ConfigParams_t config_params, BMP280_data_t BMP280_data, AHT20_data_t AHT20_data){
    sensor_alerts->aht20_temperature = alert_check(config_params.AHT20_temperature.min, AHT20_data.temperature, true) || alert_check(config_params.AHT20_temperature.max, AHT20_data.temperature, false);
    sensor_alerts->aht20_humidity = alert_check(config_params.AHT20_humidity.min, AHT20_data.humidity, true) || alert_check(config_params.AHT20_humidity.max, AHT20_data.humidity, false);
    sensor_alerts->bmp280_pressure = alert_check(config_params.BMP280_pressure.min, BMP280_data.pressure, true) || alert_check(config_params.BMP280_pressure.max, BMP280_data.pressure, false);
    sensor_alerts->bmp280_temperature = alert_check(config_params.BMP280_temperature.min, BMP280_data.temperature, true) || alert_check(config_params.BMP280_temperature.max, BMP280_data.temperature, false);
}