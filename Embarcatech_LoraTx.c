#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "lib/aht20.h"
#include "lib/alerts.h"
#include "lib/bmp280.h"
#include "lib/payload.h"
#include "lib/spi_funcs.h"
#include "lib/ssd1306.h"


// Configurações da I2C do display
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C
bool cor = true;
ssd1306_t ssd;

// Configurações da I2C dos sensores
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa

#define BUTTON_A 5
#define BUTTON_B 6

// Estrutura para armazenar os dados dos sensores
// Dados individuais
AHT20_data_t AHT20_data;
BMP280_data_t BMP280_data;
int32_t raw_temp_bmp;
int32_t raw_pressure;

uint32_t last_sensor_read = 0;


// -> ISR dos Botões =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Tratamento de interrupções 
int display_page = 1;
int num_pages = 4;
uint32_t last_isr_time = 0;
void gpio_irq_handler(uint gpio, uint32_t events){
    uint32_t current_isr_time = to_us_since_boot(get_absolute_time());
    if(current_isr_time-last_isr_time > 200000){ // Debounce
        last_isr_time = current_isr_time;
        
        if(gpio==BUTTON_A) {
            display_page--;
        }
        else{
            display_page++;
        }
        

        if(display_page>num_pages) display_page=num_pages;
        if(display_page<1) display_page=1;
    }
}


int main()
{
    stdio_init_all();
    sleep_ms(1000);

    // Iniciando o display
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Iniciando Wi-Fi", 0, 0, false);
    ssd1306_draw_string(&ssd, "Aguarde...", 0, 30, false);    
    ssd1306_send_data(&ssd);

    // Iniciando o I2C dos sensores
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializando o BMP280
    bmp280_init(I2C_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C_PORT, &params);

    // Inicializando o AHT20
    aht20_reset(I2C_PORT);
    aht20_init(I2C_PORT);

    // Iniciando os botões
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    while (true) {

        uint32_t current_sensor_read = to_us_since_boot(get_absolute_time());

        if(current_sensor_read-last_sensor_read > 2000000){

            last_sensor_read = current_sensor_read;
            // Leitura do BMP280
            bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
            int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
            int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

            printf("[DEBUG] Leitura de sensores\n");
            printf("BMP280.pressure = %.3f kPa\n", BMP280_data.pressure);
            printf("BMP280.temperature = %.2f C\n", BMP280_data.temperature);

            // Leitura do AHT20
            if (aht20_read(I2C_PORT, &AHT20_data)){
                printf("AHT20_data.temperature = %.2f C\n", AHT20_data.temperature);
                printf("AHT20_data.humidity = %.2f %%\n", AHT20_data.humidity);
            }
            else{
                printf("Erro na leitura do AHT10!\n");
            }
            printf("\n\n");
        }

        // Strings com os valores
        char str_tmp_aht[5];
        char str_humi_aht[5];
        char str_press_bmp[5];
        char str_temp_bmp[5];

        // Atualizando as strings
        sprintf(str_tmp_aht, "%.1f C", AHT20_data.temperature);
        sprintf(str_humi_aht, "%.1f %%", AHT20_data.humidity);
        sprintf(str_press_bmp, "%.1f kPa", BMP280_data.pressure);
        sprintf(str_temp_bmp, "%.1f C", BMP280_data.temperature);

        // Atualiza o Display LCD
        // Frame que será reutilizado
        ssd1306_fill(&ssd, false);
        ssd1306_rect(&ssd, 0, 0, 128, 64, cor, !cor);
        // Mensagem superior (Nome do projeto e vagas ocupadas/totais)
        ssd1306_rect(&ssd, 0, 0, 128, 12, cor, cor); // Fundo preenchido
        ssd1306_draw_string(&ssd, "DogAtmos", 4, 3, true);

        switch(display_page){
            // AHT20 - Temperatura
            case 1:
                ssd1306_draw_string(&ssd, "1/5", 95, 3, true);
                ssd1306_draw_string(&ssd, "ATUAL: ", 4, 18, false);
                ssd1306_draw_string(&ssd, str_tmp_aht, 12+7*8, 18, false);
                ssd1306_draw_string(&ssd, "STATUS: ", 4, 28, false);
                // Simbolo de alerta
                // make_alert_display(sensor_alerts.aht20_temperature, 4 + 8*8, 28);
                // // Offset atual
                // ssd1306_draw_string(&ssd, "OFFSET: ", 4, 38, false);
                // ssd1306_draw_string(&ssd, str_offset_tmp_aht, 4 + 8*8, 38, false);
                // Indicação inferior
                ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor); // Fundo preenchido
                ssd1306_draw_string(&ssd, "AHT-TEMPERATURA", 4, 53, true);
                break;

            // AHT20 - Umidade
            case 2:
                ssd1306_draw_string(&ssd, "2/5", 95, 3, true);
                ssd1306_draw_string(&ssd, "ATUAL: ", 4, 18, false);
                ssd1306_draw_string(&ssd, str_humi_aht, 12+7*8, 18, false);
                ssd1306_draw_string(&ssd, "STATUS: ", 4, 28, false);
                // Simbolo de alerta
                // make_alert_display(sensor_alerts.aht20_humidity, 4 + 8*8, 28);
                // // Offset atual
                // ssd1306_draw_string(&ssd, "OFFSET: ", 4, 38, false);
                // ssd1306_draw_string(&ssd, str_offset_humi_aht, 4 + 8*8, 38, false);
                // Indicação inferior
                ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor); // Fundo preenchido
                ssd1306_draw_string(&ssd, "AHT - UMIDADE", 4, 53, true);
                break;

            // BMP - Pressão
            case 3:
                ssd1306_draw_string(&ssd, "3/5", 95, 3, true);
                ssd1306_draw_string(&ssd, "ATUAL: ", 4, 18, false);
                ssd1306_draw_string(&ssd, str_press_bmp, 12+7*8, 18, false);
                ssd1306_draw_string(&ssd, "STATUS: ", 4, 28, false);
                // Simbolo de alerta
                // make_alert_display(sensor_alerts.bmp280_pressure, 4 + 8*8, 28);
                // // Offset atual
                // ssd1306_draw_string(&ssd, "OFFSET: ", 4, 38, false);
                // ssd1306_draw_string(&ssd, str_offset_press_bmp, 4 + 8*8, 38, false);
                // Indicação inferior
                ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor); // Fundo preenchido
                ssd1306_draw_string(&ssd, "BMP - PRESSAO", 4, 53, true);
                break;

            // BMP - Temperatura
            case 4:
                ssd1306_draw_string(&ssd, "4/5", 95, 3, true);
                ssd1306_draw_string(&ssd, "ATUAL: ", 4, 18, false);
                ssd1306_draw_string(&ssd, str_temp_bmp, 12+7*8, 18, false);
                ssd1306_draw_string(&ssd, "STATUS: ", 4, 28, false);
                // // Simbolo de alerta
                // make_alert_display(sensor_alerts.bmp280_temperature, 4 + 8*8, 28);
                // // Offset atual
                // ssd1306_draw_string(&ssd, "OFFSET: ", 4, 38, false);
                // ssd1306_draw_string(&ssd, str_offset_temp_bmp, 4 + 8*8, 38, false);
                // Indicação inferior
                ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor); // Fundo preenchido
                ssd1306_draw_string(&ssd, "BMP-TEMPERATURA", 4, 53, true);
                break;

            // Tela de IP
            // case 5:
            //     ssd1306_draw_string(&ssd, "5/5", 95, 3, true);
            //     ssd1306_draw_string(&ssd, "IP do servidor", 4, 24, false);
            //     ssd1306_draw_string(&ssd, ip_str, 4, 34, false);
            //     break;
        }
    }
}
