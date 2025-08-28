#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "lib/spi_funcs.h"
#include "lib/ssd1306.h"
#include "lib/lora.h"

// --- Configurações SPI ---
#define MOSI_PIN 19
#define MISO_PIN 16
#define SPI_SCL_PIN 18
#define CSN_PIN 17
#define PIN_DIO0 8
#define BAUD_RATE 500*1000
#define RST_PIN 20

// --- Configurações I2C (Display) ---
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define DISP_ADDR 0x3C
ssd1306_t ssd;

// --- Estrutura do payload ---
typedef struct __attribute__ ((packed)) {
    float temp_aht20;
    float humidity_aht20;
    float temp_bmp280;
    float pressure_bmp280;
} sensor_payload_t;

// --- Parâmetros LoRa ---
uint16_t preamble_len = 8;
uint16_t sf = 7;
bool crc_mode = true;
bool ih = false;
uint32_t bw = 125000;
uint16_t cr = 1;
bool ldro = true;
double freq = 915; // MHz

// --- Protótipos ---
void setFrequency(double Frequency);
void setLora();

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    // --- Inicializa display ---
    i2c_init(I2C_PORT_DISP, 400*1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, DISP_ADDR, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "LoRa RX...", 0, 0, true);
    ssd1306_send_data(&ssd);

    // --- Inicializa SPI ---
    spi_init(spi_default, BAUD_RATE);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCL_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);

    gpio_init(CSN_PIN);
    gpio_set_dir(CSN_PIN, GPIO_OUT);
    gpio_put(CSN_PIN, 1);

    // Reset do SX1276
    gpio_init(RST_PIN);
    gpio_set_dir(RST_PIN, GPIO_OUT);
    gpio_put(RST_PIN, 1);
    sleep_ms(10);
    gpio_put(RST_PIN, 0);
    sleep_ms(10);
    gpio_put(RST_PIN, 1);

    // DIO0
    gpio_init(PIN_DIO0);
    gpio_set_dir(PIN_DIO0, GPIO_IN);

    // Configura LoRa
    setLora();

    // Configurar DIO0 para RxDone
    writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 = RxDone
    writeRegister(REG_DIO_MAPPING_2, 0x00);

    // Coloca em modo RX contínuo
    setMode(6);

    printf("Receptor pronto. Aguardando pacotes...\n");

    while (true) {
        // Verifica IRQ Flags
        uint8_t irqFlags = readRegister(REG_IRQ_FLAGS);
        if (irqFlags & 0x40) { // RxDone
            // Lê tamanho do payload
            uint8_t len = readRegister(REG_RX_NB_BYTES);

            // Pega endereço base do RX FIFO
            uint8_t fifoAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
            writeRegister(REG_FIFO_ADDR_PTR, fifoAddr);

            // Lê os bytes recebidos
            uint8_t buffer[sizeof(sensor_payload_t)];
            for (int i = 0; i < len; i++) {
                buffer[i] = readRegister(REG_FIFO);
            }

            // Interpreta payload
            if (len == sizeof(sensor_payload_t)) {
                sensor_payload_t payload;
                memcpy(&payload, buffer, sizeof(sensor_payload_t));

                printf("Pacote recebido:\n");
                printf("Temp AHT20: %.2f C\n", payload.temp_aht20);
                printf("Umid AHT20: %.2f %%\n", payload.humidity_aht20);
                printf("Temp BMP280: %.2f C\n", payload.temp_bmp280);
                printf("Press BMP280: %.3f kPa\n\n", payload.pressure_bmp280);

                // Atualiza display com dados recebidos
                char line[32];
                ssd1306_fill(&ssd, false);
                snprintf(line, sizeof(line), "T1: %.1fC", payload.temp_aht20);
                ssd1306_draw_string(&ssd, line, 0, 0, true);
                snprintf(line, sizeof(line), "H: %.1f%%", payload.humidity_aht20);
                ssd1306_draw_string(&ssd, line, 0, 12, true);
                snprintf(line, sizeof(line), "T2: %.1fC", payload.temp_bmp280);
                ssd1306_draw_string(&ssd, line, 0, 24, true);
                snprintf(line, sizeof(line), "P: %.1fkPa", payload.pressure_bmp280);
                ssd1306_draw_string(&ssd, line, 0, 36, true);
                ssd1306_send_data(&ssd);

            } else {
                // Tratamento de pacote inválido
                printf("ERRO: Pacote recebido com tamanho inesperado (%d bytes)\n", len);

                ssd1306_fill(&ssd, false);
                ssd1306_draw_string(&ssd, "ERRO RX!", 0, 0, true);
                ssd1306_draw_string(&ssd, "Pacote invalido", 0, 16, true);
                ssd1306_send_data(&ssd);
            }

            // Limpa flag RxDone
            writeRegister(REG_IRQ_FLAGS, 0xFF);
        }
        sleep_ms(10);
    }
}

// --- Definir frequência ---
void setFrequency(double Frequency)
{
    unsigned long FrequencyValue;
    Frequency = Frequency * 7110656 / 434;
    FrequencyValue = (unsigned long)(Frequency);

    writeRegister(REG_FRF_MSB, (FrequencyValue >> 16) & 0xFF);
    writeRegister(REG_FRF_MID, (FrequencyValue >> 8) & 0xFF);
    writeRegister(REG_FRF_LSB, FrequencyValue & 0xFF);
}

// --- Configuração LoRa (igual ao TX) ---
void setLora()
{
    setMode(1); // Sleep
    writeRegisterBit(REG_OPMODE, 7, 1); // LoRa mode
    writeRegisterBit(REG_OPMODE, 6, 0);
    writeRegisterBit(REG_OPMODE, 3, 1);

    setFrequency(freq);
    setBW(bw);
    setSF(sf);

    // Coding rate
    switch(cr) {
        case 1: writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b001); break;
        case 2: writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b010); break;
        case 3: writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b011); break;
        case 4: writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b100); break;
        default: writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b001); break;
    }

    writeRegisterBit(REG_MODEM_CONFIG, 0, ih ? 1 : 0);  //Implicit or explicit header
    //writeRegisterBit(REG_MODEM_CONFIG2, 2, crc_mode ? 1 : 0);
    writeRegisterBit(REG_MODEM_CONFIG2, 2, 0);
    writeRegisterField(REG_DETECT_OPT, 0, 3, 0x03);// Para sf 7 até 12
    writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
    writeRegisterBit(REG_MODEM_CONFIG3, 3, ldro ? 1 : 0);

    // Preamble
    writeRegister(REG_PREAMBLE_MSB, (preamble_len >> 8) & 0xFF);
    writeRegister(REG_PREAMBLE_LSB, preamble_len & 0xFF);

    printf("Config LoRa RX pronta.\n");
}
