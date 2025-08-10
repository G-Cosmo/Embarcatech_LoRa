#include "payload.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "aht20.h"
#include "bmp280.h"
#include "alerts.h"

// Inicializa os buffers com valores zerados
void payload_buffers_init(AHT20_buffer_t *AHT20_buffer, BMP280_buffer_t *BMP280_buffer){
    for(int i=0; i<BUFFER_SIZE; i++){
        AHT20_buffer->humidity[i] = 0.0f;
        AHT20_buffer->temperature[i] = 0.0f;
        BMP280_buffer->pressure[i] = 0.0f;
        BMP280_buffer->temperature[i] = 0.0f;
    }
}

// Atualiza os buffers com o valor mais recente
void payload_buffers_update(AHT20_data_t AHT20_data, BMP280_data_t BMP280_data, AHT20_buffer_t *AHT20_buffer, BMP280_buffer_t *BMP280_buffer){
    for(int i=0; i<BUFFER_SIZE-1; i++){
        AHT20_buffer->humidity[i] = AHT20_buffer->humidity[i+1];
        AHT20_buffer->temperature[i] = AHT20_buffer->temperature[i+1];
        BMP280_buffer->pressure[i] = BMP280_buffer->pressure[i+1];
        BMP280_buffer->temperature[i] = BMP280_buffer->temperature[i+1];
    }

    AHT20_buffer->humidity[BUFFER_SIZE-1] = AHT20_data.humidity;
    AHT20_buffer->temperature[BUFFER_SIZE-1] = AHT20_data.temperature;
    BMP280_buffer->pressure[BUFFER_SIZE-1] = BMP280_data.pressure;
    BMP280_buffer->temperature[BUFFER_SIZE-1] = BMP280_data.temperature;
}

// Gera as strings no formato do JSON para os dados
int payload_generate_json(char *json_buffer, Sensor_alerts_t sensor_alerts, AHT20_buffer_t *AHT20_buffer, BMP280_buffer_t *BMP280_buffer, Payload_sizes_t payload_sizes){
    // Lembra de fazer isso aí no código principal para ter os valores atualizados
    // (Mas para o payload_sizes!!!!)
    // size_t json_size = sizeof(json_buffer);
    // size_t aht20_humi_size = sizeof(AHT20_buffer->humidity);
    // size_t aht20_temp_size = sizeof(AHT20_buffer->temperature);
    // size_t bmp280_press_size = sizeof(BMP280_buffer->pressure);
    // size_t bmp280_temp_size = sizeof(BMP280_buffer->temperature);

    int offset = 0;

    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "{");

    // AHT20 - Temperatura
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "\"AHT20_temperature\":[");
    for(size_t i = 0; i < BUFFER_SIZE; i++){
        // Nessa notação faz número e vírgula para todos, menos para o último valor
        offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "%.2f%s", AHT20_buffer->temperature[i], (i < BUFFER_SIZE - 1) ? "," : "");
    }
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "],");
    // Alerta
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "\"alert_AHT20_temperature\":\"%s\",", sensor_alerts.aht20_temperature ? "on" : "off");


    // AHT20 - Humidade
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "\"AHT20_humidity\":[");
    for(size_t i = 0; i < BUFFER_SIZE; i++){
        offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "%.2f%s", AHT20_buffer->humidity[i], (i < BUFFER_SIZE - 1) ? "," : "");
    }
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "],");
    // Alerta
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "\"alert_AHT20_humidity\":\"%s\",", sensor_alerts.aht20_humidity ? "on" : "off");


    // BMP280 - Pressão
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "\"BMP280_pressure\":[");
    for(size_t i = 0; i < BUFFER_SIZE; i++){
        offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "%.2f%s", BMP280_buffer->pressure[i], (i < BUFFER_SIZE - 1) ? "," : "");
    }
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "],");
    // Alerta
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "\"alert_BMP280_pressure\":\"%s\",", sensor_alerts.bmp280_pressure ? "on" : "off");


    // BMP280 - Temperatura
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "\"BMP280_temperature\":[");
    for(size_t i = 0; i < BUFFER_SIZE; i++){
        offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "%.2f%s", BMP280_buffer->temperature[i], (i < BUFFER_SIZE - 1) ? "," : "");
    }
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "],");
    // Alerta
    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "\"alert_BMP280_temperature\":\"%s\"", sensor_alerts.bmp280_temperature ? "on" : "off");


    offset += snprintf(json_buffer + offset, payload_sizes.json_size - offset, "}\r\n");

    return offset;
}
