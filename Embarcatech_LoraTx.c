#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "lib/aht20.h"
#include "lib/alerts.h"     
#include "lib/bmp280.h"
#include "lib/payload.h"    // Esse payload é do html (não está sendo utilizado)
#include "lib/spi_funcs.h"
#include "lib/ssd1306.h"
#include "lib/lora.h"

//LoRa Mode Register Map página 108 do datasheet do sx1276

// Configurações do SPI
#define MOSI_PIN 19
#define MISO_PIN 16
#define SPI_SCL_PIN 18 // pino do clock
#define CSN_PIN 17  //chip select
#define PIN_DIO0 8
#define BAUD_RATE 500*1000 //define o baud rate como 0,5 MHZ

#define RST_PIN 20


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

// buffers para impressão dos dados dos sensores
char str_tmp_aht[5];
char str_humi_aht[5];
char str_press_bmp[5];
char str_temp_bmp[5];

// Estrutura para representar o payload
typedef struct __attribute__ ((packed)) {
    float temp_aht20;      // 4 bytes
    float humidity_aht20;  // 4 bytes
    float temp_bmp280;     // 4 bytes
    float pressure_bmp280; // 4 bytes
} sensor_payload_t;  


uint32_t last_sensor_read = 0;

// Configurações do - TX:
uint16_t payload_len = sizeof(sensor_payload_t); // Tamanho do payload em byte
uint16_t preamble_len = 8; // Tamanho do preâmbulo
uint16_t sf = 7; // fator de espalhamento (Spreading Factor)
bool crc_mode = true; // flag que indica se o CRC deve ou não ser ativado
bool ih = false; // Cabeçalho implícito (Implicit Header) (false = explicit header mode)
uint32_t bw = 125000; // largura de banda (Bandwidth) em Hz
uint16_t cr = 1; // Taxa de codificação (Coding Rate)
bool ldro = false; // ldro = true => LowDataRateOptimize ativado
double freq = 915; // frequencia em MHz

// -> ISR dos Botões =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Tratamento de interrupções 
int volatile display_page = 1;
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

void setFrequency(double Frequency);
void setLora();
void sendSensorData();

int main()
{
    stdio_init_all();
    sleep_ms(2000);   
    
    // Iniciando os botões
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Iniciando o display
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_draw_string(&ssd, "Iniciando", 0, 0, false);
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

    //Inicializando o SPI
    spi_init(spi_default, BAUD_RATE);  //define a frequência do SPI como 0,5 MHZ
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCL_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);

    // Inicializando o Chip Select em driven-high state, ele é ativado em low
    gpio_init(CSN_PIN);
    gpio_set_dir(CSN_PIN, GPIO_OUT);
    gpio_put(CSN_PIN, 1);

    // Inicializando o pino de reset, que é ativado em nível baixo
    gpio_init(RST_PIN);
    gpio_set_dir(RST_PIN, GPIO_OUT);
    gpio_put(RST_PIN, 1);

    // DIO0 - input
    gpio_init(PIN_DIO0);
    gpio_set_dir(PIN_DIO0, GPIO_IN);

    printf("\n Check 1");

    setLora();

    printf("\n Check Lora"); 

    while (true) {

        uint32_t current_sensor_read = to_us_since_boot(get_absolute_time());

        if(current_sensor_read-last_sensor_read > 2000000){

            last_sensor_read = current_sensor_read;
            // Leitura do BMP280
            bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
            int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
            int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

            // Leitura do AHT20
            if (!aht20_read(I2C_PORT, &AHT20_data)){
                printf("Erro na leitura do AHT10!\n");
            }
            printf("\n\n");

            BMP280_data.pressure = pressure/1000.0f;
            BMP280_data.temperature = temperature/100.0f;
        }

        // Atualizando as strings
        sprintf(str_tmp_aht, "%.1f C", AHT20_data.temperature);
        sprintf(str_humi_aht, "%.1f %%", AHT20_data.humidity);
        sprintf(str_press_bmp, "%.1f kPa", BMP280_data.pressure);
        sprintf(str_temp_bmp, "%.1f C", BMP280_data.temperature);

        // Atualiza o Display LCD
        // Frame que será reutilizado
        ssd1306_fill(&ssd, false);
        ssd1306_rect(&ssd, 0, 0, 128, 64, cor, !cor);
        ssd1306_rect(&ssd, 0, 0, 128, 12, cor, cor); // Fundo preenchido
        ssd1306_draw_string(&ssd, "DogAtmos", 4, 3, true);

        switch(display_page){
            // AHT20 - Temperatura
            case 1:
                ssd1306_draw_string(&ssd, "1/4", 95, 3, true);
                ssd1306_draw_string(&ssd, "ATUAL: ", 4, 18, false);
                ssd1306_draw_string(&ssd, str_tmp_aht, 12+7*8, 18, false);
                ssd1306_draw_string(&ssd, "STATUS: ", 4, 28, false);
                ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor); // Fundo preenchido
                ssd1306_draw_string(&ssd, "AHT-TEMPERATURA", 4, 53, true);
                break;

            // AHT20 - Umidade
            case 2:
                ssd1306_draw_string(&ssd, "2/4", 95, 3, true);
                ssd1306_draw_string(&ssd, "ATUAL: ", 4, 18, false);
                ssd1306_draw_string(&ssd, str_humi_aht, 12+7*8, 18, false);
                ssd1306_draw_string(&ssd, "STATUS: ", 4, 28, false);
                ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor); // Fundo preenchido
                ssd1306_draw_string(&ssd, "AHT - UMIDADE", 4, 53, true);
                break;

            // BMP - Pressão
            case 3:
                ssd1306_draw_string(&ssd, "3/4", 95, 3, true);
                ssd1306_draw_string(&ssd, "ATUAL: ", 4, 18, false);
                ssd1306_draw_string(&ssd, str_press_bmp, 12+7*8, 18, false);
                ssd1306_draw_string(&ssd, "STATUS: ", 4, 28, false);
                ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor); // Fundo preenchido
                ssd1306_draw_string(&ssd, "BMP - PRESSAO", 4, 53, true);
                break;

            // BMP - Temperatura
            case 4:
                ssd1306_draw_string(&ssd, "4/4", 95, 3, true);
                ssd1306_draw_string(&ssd, "ATUAL: ", 4, 18, false);
                ssd1306_draw_string(&ssd, str_temp_bmp, 12+7*8, 18, false);
                ssd1306_draw_string(&ssd, "STATUS: ", 4, 28, false);
                ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor); // Fundo preenchido
                ssd1306_draw_string(&ssd, "BMP-TEMPERATURA", 4, 53, true);
                break;
        }

        ssd1306_send_data(&ssd);

        sendSensorData(str_humi_aht); // Função que faz o envio do payload

        sleep_ms(2000);
    }
}

// Definir frequência de operação
void setFrequency(double Frequency)
{
    unsigned long FrequencyValue; //Frequência corrida - valor para os registradores do SX1276

    printf("Frequência escolhida %.3f MHz\n", Frequency);

    Frequency = Frequency * 7110656 / 434; //Fórmula para correção da frequência - considerando o padrão de 434 MHz - 6C 80 00
    FrequencyValue = (unsigned long)(Frequency);
    printf("Frequencia calculada para ajuste dos registradores: %lu\n", FrequencyValue);

    writeRegister(REG_FRF_MSB, (FrequencyValue >> 16) & 0xFF); // Escrita no registrador RegFrMsb (0x06)
    writeRegister(REG_FRF_MID, (FrequencyValue >> 8) & 0xFF); // Escrita no registrador RegFrMid (0x07)
    writeRegister(REG_FRF_LSB, FrequencyValue & 0xFF); // Escrita no registrador RegFrLsb (0x08)

    // Teste de leitura dos registradores RegFrMsb, RegFrMid e RegFrLsb.
    readRegister(REG_FRF_MSB);
    readRegister(REG_FRF_MID);
    readRegister(REG_FRF_LSB);
}


void setLora()
{
    printf("\n=== INICIANDO CONFIGURAÇÃO LORA ===\n");
    
    // Configuração inicial - modo sleep
    printf("\nRegOpMode antes do sleep: 0x%02X", readRegister(REG_OPMODE));
    setMode(RF95_MODE_SLEEP); // Coloca o registrador RegOpMode no modo sleep
    printf("\nRegOpMode após sleep: 0x%02X", readRegister(REG_OPMODE));

    // Configuração do modo LoRa
    printf("\nRegOpMode antes config LoRa: 0x%02X", readRegister(REG_OPMODE));
    writeRegisterBit(REG_OPMODE, 7, 1); // Escreve o valor 1 no bit 7 de REG_OPMODE (coloca no modo LoRa)
    writeRegisterBit(REG_OPMODE, 6, 0); // Desativa o acesso ao registrador compartilhado com o modo FSK
    writeRegisterBit(REG_OPMODE, 3, 1); // Ativa o modo de alta frequência
    printf("\nRegOpMode após config LoRa: 0x%02X", readRegister(REG_OPMODE));

    // Configuração de frequência
    printf("\n\n--- CONFIGURAÇÃO DE FREQUÊNCIA ---");
    printf("\nRegFrMsb antes: 0x%02X", readRegister(REG_FRF_MSB));
    printf("\nRegFrMid antes: 0x%02X", readRegister(REG_FRF_MID));
    printf("\nRegFrLsb antes: 0x%02X", readRegister(REG_FRF_LSB));
    setFrequency(freq); // Define a frequencia
    printf("\nRegFrMsb após: 0x%02X", readRegister(REG_FRF_MSB));
    printf("\nRegFrMid após: 0x%02X", readRegister(REG_FRF_MID));
    printf("\nRegFrLsb após: 0x%02X", readRegister(REG_FRF_LSB));

    // Configuração da largura de banda
    printf("\n\n--- CONFIGURAÇÃO DE BANDWIDTH ---");
    printf("\nRegModemConfig antes BW: 0x%02X", readRegister(REG_MODEM_CONFIG));
    setBW(bw); // Define a largura de banda
    printf("\nRegModemConfig após BW: 0x%02X", readRegister(REG_MODEM_CONFIG));

    // Configuração do spreading factor
    printf("\n\n--- CONFIGURAÇÃO DE SPREADING FACTOR ---");
    printf("\nRegModemConfig2 antes SF: 0x%02X", readRegister(REG_MODEM_CONFIG2));
    setSF(sf);  // Define o spreading factor
    printf("\nRegModemConfig2 após SF: 0x%02X", readRegister(REG_MODEM_CONFIG2));

    // Configuração do coding rate
    printf("\n\n--- CONFIGURAÇÃO DE CODING RATE ---");
    printf("\nRegModemConfig antes CR: 0x%02X", readRegister(REG_MODEM_CONFIG));
    switch(cr) 
    {
    case 1:
        writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b001);
        break;
    case 2:
        writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b010);
        break;
    case 3:
        writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b011);
        break;
    case 4:
        writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b100);
        break;
    default:
        writeRegisterField(REG_MODEM_CONFIG, 1, 3, 0b001);
        break;
    }
    printf("\nRegModemConfig após CR: 0x%02X", readRegister(REG_MODEM_CONFIG));

    // Configuração do implicit header mode
    printf("\n\n--- CONFIGURAÇÃO DE HEADER MODE ---");
    printf("\nRegModemConfig antes IH: 0x%02X", readRegister(REG_MODEM_CONFIG));
    if(ih){  // Verifica a flag do implicit header mode
        writeRegisterBit(REG_MODEM_CONFIG, 0, 1);   // Ativa o implicit header mode
        printf(" (Implicit Header ATIVADO)");
    }else
    {
        writeRegisterBit(REG_MODEM_CONFIG, 0, 0);   // Desativa o implicit header mode
        printf(" (Explicit Header ATIVADO)");
    }
    printf("\nRegModemConfig após IH: 0x%02X", readRegister(REG_MODEM_CONFIG));

    // Configuração do CRC
    printf("\n\n--- CONFIGURAÇÃO DE CRC ---");
    printf("\nRegModemConfig2 antes CRC: 0x%02X", readRegister(REG_MODEM_CONFIG2));
    if(crc_mode)    // Verifica a flag de ativação do CRC 
    {
        writeRegisterBit(REG_MODEM_CONFIG2, 2, 1); // Ativa o CRC
        printf(" (CRC ATIVADO)");
    }else
    {
        writeRegisterBit(REG_MODEM_CONFIG2, 2, 0); // Desativa o CRC
        printf(" (CRC DESATIVADO)");
    }
    printf("\nRegModemConfig2 após CRC: 0x%02X", readRegister(REG_MODEM_CONFIG2));

    // Configuração do detection optimize
    printf("\n\n--- CONFIGURAÇÃO DE DETECTION OPTIMIZE ---");
    printf("\nRegDetectOpt antes: 0x%02X", readRegister(REG_DETECT_OPT));
    writeRegisterField(REG_DETECT_OPT, 0, 3, 0x03);
    printf("\nRegDetectOpt após: 0x%02X", readRegister(REG_DETECT_OPT));

    // Configuração do detection threshold
    printf("\n\n--- CONFIGURAÇÃO DE DETECTION THRESHOLD ---");
    printf("\nRegDetectionThreshold antes: 0x%02X", readRegister(REG_DETECTION_THRESHOLD));
    writeRegisterField(REG_DETECTION_THRESHOLD, 0, 8, 0x0A);
    printf("\nRegDetectionThreshold após: 0x%02X", readRegister(REG_DETECTION_THRESHOLD));

    // Configuração do LDRO (Low Data Rate Optimize)
    printf("\n\n--- CONFIGURAÇÃO DE LDRO ---");
    printf("\nRegModemConfig3 antes LDRO: 0x%02X", readRegister(REG_MODEM_CONFIG3));
    if(ldro)   // Checa se o LowDataRateOptimize deve ser ligado
    {
        writeRegisterBit(REG_MODEM_CONFIG3, 3, 1); // ativa o LDRO
        printf(" (LDRO ATIVADO)");
    }else{
        writeRegisterBit(REG_MODEM_CONFIG3, 3, 0); // desativa o LDRO
        printf(" (LDRO DESATIVADO)");
    }
    printf("\nRegModemConfig3 após LDRO: 0x%02X", readRegister(REG_MODEM_CONFIG3));

    // Configuração do payload length
    printf("\n\n--- CONFIGURAÇÃO DE PAYLOAD LENGTH ---");
    printf("\nRegPayloadLength antes: 0x%02X", readRegister(REG_PAYLOAD_LENGTH));
    if(payload_len < 256 && payload_len > 0) // Verifica se o tamanho do payload está entre 1 e 255
    {
        writeRegister(REG_PAYLOAD_LENGTH, payload_len); // Escreve o valor decimal diretamente no registrador do payload
    }else{ // Se o valor estiver fora do limite ele será definido como 255 bytes que é o máximo
        printf("\nValor de tamanho do payload inválido, o valor será definido como 255 bytes");
        writeRegister(REG_PAYLOAD_LENGTH, 255);
    }
    printf("\nRegPayloadLength após: 0x%02X", readRegister(REG_PAYLOAD_LENGTH));

    // Configuração do preâmbulo
    printf("\n\n--- CONFIGURAÇÃO DE PREÂMBULO ---");
    printf("\nRegPreambleMsb antes: 0x%02X", readRegister(REG_PREAMBLE_MSB));
    printf("\nRegPreambleLsb antes: 0x%02X", readRegister(REG_PREAMBLE_LSB));
    if (preamble_len > 0) {
        writeRegister(REG_PREAMBLE_MSB, (preamble_len >> 8) & 0xFF);
        writeRegister(REG_PREAMBLE_LSB, preamble_len & 0xFF);
    } else {
        printf("\nValor de preâmbulo inválido, será definido como 8 (default)");
        writeRegister(REG_PREAMBLE_MSB, 0x00);
        writeRegister(REG_PREAMBLE_LSB, 0x08);
    }
    printf("\nRegPreambleMsb após: 0x%02X", readRegister(REG_PREAMBLE_MSB));
    printf("\nRegPreambleLsb após: 0x%02X", readRegister(REG_PREAMBLE_LSB));

    // Configuração da potência de transmissão
    printf("\n\n--- CONFIGURAÇÃO DE POTÊNCIA PA ---");
    printf("\nRegPaConfig antes: 0x%02X", readRegister(REG_PA_CONFIG));
    writeRegister(REG_PA_CONFIG, PA_MED_BOOST); // ou PA_MAX_BOOST se precisar de mais alcance
    printf("\nRegPaConfig após: 0x%02X", readRegister(REG_PA_CONFIG));

    // Configuração do mapeamento DIO
    printf("\n\n--- CONFIGURAÇÃO DE DIO MAPPING ---");
    printf("\nRegDioMapping1 antes: 0x%02X", readRegister(REG_DIO_MAPPING_1));
    printf("\nRegDioMapping2 antes: 0x%02X", readRegister(REG_DIO_MAPPING_2));
    writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 = TxDone
    writeRegister(REG_DIO_MAPPING_2, 0x00);
    printf("\nRegDioMapping1 após: 0x%02X", readRegister(REG_DIO_MAPPING_1));
    printf("\nRegDioMapping2 após: 0x%02X", readRegister(REG_DIO_MAPPING_2));

    // Modo standby final
    printf("\n\n--- CONFIGURAÇÃO FINAL - MODO STANDBY ---");
    printf("\nRegOpMode antes standby: 0x%02X", readRegister(REG_OPMODE));
    setMode(RF95_MODE_STANDBY); // Coloca em modo stand by
    printf("\nRegOpMode após standby: 0x%02X", readRegister(REG_OPMODE));

    printf("\n\n=== CONFIGURAÇÃO LORA CONCLUÍDA ===\n");
}

void sendSensorData() {
    sensor_payload_t payload; // Criar estrutura do payload
    
    // Preencher o payload com os dados dos sensores
    payload.temp_aht20 = AHT20_data.temperature;
    payload.humidity_aht20 = AHT20_data.humidity;
    payload.temp_bmp280 = BMP280_data.temperature;
    payload.pressure_bmp280 = BMP280_data.pressure;
    
    setMode(RF95_MODE_STANDBY); // Standby
    
    // Limpar IRQ flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);

    writeRegister(REG_DIO_MAPPING_1, 0x40);		// 01 00 00 00 mapear DIO0 para o TxDone

    // Configurar FIFO
    writeRegister(REG_FIFO_TX_BASE_AD, 0x80);
    writeRegister(REG_FIFO_ADDR_PTR, 0x80);
    
    // Definir tamanho correto do payload
    writeRegister(REG_PAYLOAD_LENGTH, sizeof(sensor_payload_t));
    
    printf("\nTamanho do payload: %d bytes\n", sizeof(sensor_payload_t));
    
    // Escrever payload no FIFO byte a byte
    uint8_t *payload_bytes = (uint8_t*)&payload;
    for(int i = 0; i < sizeof(sensor_payload_t); i++) {
        writeRegister(REG_FIFO, payload_bytes[i]);
    }
    
    // Iniciar transmissão
    setMode(RF95_MODE_TX); // TX mode
    
    // Aguardar transmissão completar
    while(!(readRegister(REG_IRQ_FLAGS) & 0x08)) {
        sleep_ms(1);
    }
    
    // Limpar flag TxDone
    writeRegister(REG_IRQ_FLAGS, 0x08);
    
    // Voltar para standby
    setMode(RF95_MODE_STANDBY);
    
    printf("Dados enviados via LoRa!\n");
    printf("Temp AHT20: %.2f°C\n", payload.temp_aht20);
    printf("Umid AHT20: %.2f%%\n", payload.humidity_aht20);
    printf("Temp BMP280: %.2f°C\n", payload.temp_bmp280);
    printf("Press BMP280: %.3f kPa\n", payload.pressure_bmp280);
}