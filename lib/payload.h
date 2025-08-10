#ifndef PAYLOAD_H
#define PAYLOAD_H

#include "aht20.h"
#include "bmp280.h"
#include "alerts.h"

#define BUFFER_SIZE 30

typedef struct {
    float temperature[BUFFER_SIZE];
    float humidity[BUFFER_SIZE];
} AHT20_buffer_t;

typedef struct {
    float temperature[BUFFER_SIZE];
    float pressure[BUFFER_SIZE];
} BMP280_buffer_t;

typedef struct {
    size_t json_size;
    size_t aht20_humi_size;
    size_t aht20_temp_size;
    size_t bmp280_press_size;
    size_t bmp280_temp_size;
} Payload_sizes_t;

void payload_buffers_init(AHT20_buffer_t *AHT20_buffer, BMP280_buffer_t *BMP280_buffer);
void payload_buffers_update(AHT20_data_t AHT20_data, BMP280_data_t BMP280_data, AHT20_buffer_t *AHT20_buffer, BMP280_buffer_t *BMP280_buffer);
int payload_generate_json(char *json_buffer, Sensor_alerts_t sensor_alerts, AHT20_buffer_t *AHT20_buffer, BMP280_buffer_t *BMP280_buffer, Payload_sizes_t payload_sizes);

#endif