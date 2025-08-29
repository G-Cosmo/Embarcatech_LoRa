#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "lib/spi_funcs.h"
#include "lib/ssd1306.h"
#include "lib/lora.h"

// Configurações do SPI
#define MOSI_PIN 19
#define MISO_PIN 16
#define SPI_SCL_PIN 18
#define CSN_PIN 17
#define BAUD_RATE 500*1000

// // Definir para compatibilidade com spi_funcs.h
// #define PICO_DEFAULT_SPI_CSN_PIN CSN_PIN

// Pino de Reset
#define RST_PIN 20

// Configurações da I2C do display
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C
bool cor = true;
ssd1306_t ssd;

#define BUTTON_A 5
#define BUTTON_B 6

// Estrutura para representar o payload recebido
typedef struct {
    float temp_aht20;      // 4 bytes
    float humidity_aht20;  // 4 bytes
    float temp_bmp280;     // 4 bytes
    float pressure_bmp280; // 4 bytes
} sensor_payload_t;

// Configurações RX (devem ser iguais ao TX)
uint16_t rx_payload_len = sizeof(sensor_payload_t); // Tamanho do payload em bytes
uint16_t rx_preamble_len = 8; // Tamanho do preâmbulo
uint16_t rx_sf = 7; // fator de espalhamento (Spreading Factor)
bool rx_crc_mode = true; // flag que indica se o CRC deve ou não ser ativado
bool rx_ih = false; // Cabeçalho implícito (Implicit Header) (false = explicit header mode)
uint32_t rx_bw = 125000; // largura de banda (Bandwidth) em Hz
uint16_t rx_cr = 1; // Taxa de codificação (Coding Rate)
bool rx_ldro = false; // ldro = true => LowDataRateOptimize ativado
double rx_freq = 915; // frequencia em MHz

// Dados recebidos
sensor_payload_t received_data;
bool data_received = false;
uint32_t last_packet_time = 0;
uint32_t total_packets = 0;
uint32_t valid_packets = 0;

// Controle do display
int display_page = 1;
int num_pages = 5;
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
void setLoRaRX();
bool receiveLoraData();
void updateDisplay();

int main() {
    stdio_init_all();
    sleep_ms(1000);
    
    printf("Iniciando Receptor LoRa...\n");
    
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
    ssd1306_draw_string(&ssd, "Receptor LoRa", 0, 0, false);
    ssd1306_draw_string(&ssd, "Iniciando...", 0, 20, false);
    ssd1306_send_data(&ssd);

    // Inicializando o SPI
    spi_init(spi_default, BAUD_RATE);   // spi0
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCL_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);

    // Inicializando o Chip Select
    gpio_init(CSN_PIN);
    gpio_set_dir(CSN_PIN, GPIO_OUT);
    gpio_put(CSN_PIN, 1);

    // Inicializando o pino de reset
    gpio_init(RST_PIN);
    gpio_set_dir(RST_PIN, GPIO_OUT);
    gpio_put(RST_PIN, 1);

    // Configurar LoRa para recepção
    setLoRaRX();
    
    printf("Receptor LoRa iniciado. Aguardando dados...\n");

    while (true) {
        // Verificar se recebeu dados
        if(receiveLoraData()) {
            printf("Pacote processado com sucesso!\n\n");
            data_received = true;
        } else {
            printf("Falha na recepção do pacote.\n\n");
            data_received = false;
        }
        
        sleep_ms(100); // Pequeno delay antes da próxima tentativa
        
        // Atualizar display
        updateDisplay();
        
        sleep_ms(100);
    }
}

void setFrequency(double Frequency) {
    unsigned long FrequencyValue;
    
    printf("Configurando frequencia: %.3f MHz\n", Frequency);
    
    Frequency = Frequency * 7110656 / 434;
    FrequencyValue = (unsigned long)(Frequency);
    
    writeRegister(REG_FRF_MSB, (FrequencyValue >> 16) & 0xFF);
    writeRegister(REG_FRF_MID, (FrequencyValue >> 8) & 0xFF);
    writeRegister(REG_FRF_LSB, FrequencyValue & 0xFF);
    
    printf("Frequencia configurada!\n");
}

void setLoRaRX()
{
    printf("\n=== INICIANDO CONFIGURAÇÃO LORA RX ===\n");
    
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
    setFrequency(rx_freq); // Define a frequencia (mesma do TX)
    printf("\nRegFrMsb após: 0x%02X", readRegister(REG_FRF_MSB));
    printf("\nRegFrMid após: 0x%02X", readRegister(REG_FRF_MID));
    printf("\nRegFrLsb após: 0x%02X", readRegister(REG_FRF_LSB));

    // Configuração da largura de banda
    printf("\n\n--- CONFIGURAÇÃO DE BANDWIDTH ---");
    printf("\nRegModemConfig antes BW: 0x%02X", readRegister(REG_MODEM_CONFIG));
    setBW(rx_bw); // Define a largura de banda (mesma do TX)
    printf("\nRegModemConfig após BW: 0x%02X", readRegister(REG_MODEM_CONFIG));

    // Configuração do spreading factor
    printf("\n\n--- CONFIGURAÇÃO DE SPREADING FACTOR ---");
    printf("\nRegModemConfig2 antes SF: 0x%02X", readRegister(REG_MODEM_CONFIG2));
    setSF(rx_sf);  // Define o spreading factor (mesmo do TX)
    printf("\nRegModemConfig2 após SF: 0x%02X", readRegister(REG_MODEM_CONFIG2));

    // Configuração do coding rate
    printf("\n\n--- CONFIGURAÇÃO DE CODING RATE ---");
    printf("\nRegModemConfig antes CR: 0x%02X", readRegister(REG_MODEM_CONFIG));
    switch(rx_cr) 
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
    if(rx_ih){  // Verifica a flag do implicit header mode
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
    if(rx_crc_mode)    // Verifica a flag de ativação do CRC 
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
    if(rx_ldro)   // Checa se o LowDataRateOptimize deve ser ligado
    {
        writeRegisterBit(REG_MODEM_CONFIG3, 3, 1); // ativa o LDRO
        printf(" (LDRO ATIVADO)");
    }else{
        writeRegisterBit(REG_MODEM_CONFIG3, 3, 0); // desativa o LDRO
        printf(" (LDRO DESATIVADO)");
    }
    printf("\nRegModemConfig3 após LDRO: 0x%02X", readRegister(REG_MODEM_CONFIG3));

    // Configuração do payload length (para modo explicit header)
    printf("\n\n--- CONFIGURAÇÃO DE PAYLOAD LENGTH ---");
    printf("\nRegPayloadLength antes: 0x%02X", readRegister(REG_PAYLOAD_LENGTH));
    if(rx_payload_len < 256 && rx_payload_len > 0) // Verifica se o tamanho do payload está entre 1 e 255
    {
        writeRegister(REG_PAYLOAD_LENGTH, rx_payload_len); // Escreve o valor decimal diretamente no registrador do payload
    }else{ // Se o valor estiver fora do limite ele será definido como 255 bytes que é o máximo
        printf("\nValor de tamanho do payload inválido, o valor será definido como 255 bytes");
        writeRegister(REG_PAYLOAD_LENGTH, 255);
    }
    printf("\nRegPayloadLength após: 0x%02X", readRegister(REG_PAYLOAD_LENGTH));

    // Configuração do preâmbulo
    printf("\n\n--- CONFIGURAÇÃO DE PREÂMBULO ---");
    printf("\nRegPreambleMsb antes: 0x%02X", readRegister(REG_PREAMBLE_MSB));
    printf("\nRegPreambleLsb antes: 0x%02X", readRegister(REG_PREAMBLE_LSB));
    if (rx_preamble_len > 0) {
        writeRegister(REG_PREAMBLE_MSB, (rx_preamble_len >> 8) & 0xFF);
        writeRegister(REG_PREAMBLE_LSB, rx_preamble_len & 0xFF);
    } else {
        printf("\nValor de preâmbulo inválido, será definido como 8 (default)");
        writeRegister(REG_PREAMBLE_MSB, 0x00);
        writeRegister(REG_PREAMBLE_LSB, 0x08);
    }
    printf("\nRegPreambleMsb após: 0x%02X", readRegister(REG_PREAMBLE_MSB));
    printf("\nRegPreambleLsb após: 0x%02X", readRegister(REG_PREAMBLE_LSB));

    // Configuração do LNA (Low Noise Amplifier) para recepção
    printf("\n\n--- CONFIGURAÇÃO DE LNA ---");
    printf("\nRegLna antes: 0x%02X", readRegister(REG_LNA));
    writeRegister(REG_LNA, 0x23); // LNA boost on, Maximum gain
    printf("\nRegLna após: 0x%02X", readRegister(REG_LNA));

    // Configuração do mapeamento DIO para RX
    printf("\n\n--- CONFIGURAÇÃO DE DIO MAPPING RX ---");
    printf("\nRegDioMapping1 antes: 0x%02X", readRegister(REG_DIO_MAPPING_1));
    printf("\nRegDioMapping2 antes: 0x%02X", readRegister(REG_DIO_MAPPING_2));
    writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 = RxDone, DIO1 = RxTimeout
    writeRegister(REG_DIO_MAPPING_2, 0x00);
    printf("\nRegDioMapping1 após: 0x%02X", readRegister(REG_DIO_MAPPING_1));
    printf("\nRegDioMapping2 após: 0x%02X", readRegister(REG_DIO_MAPPING_2));

    // Configurar FIFO para recepção
    printf("\n\n--- CONFIGURAÇÃO DE FIFO RX ---");
    printf("\nRegFifoRxBaseAddr antes: 0x%02X", readRegister(REG_FIFO_RX_BASE_AD));
    printf("\nRegFifoAddrPtr antes: 0x%02X", readRegister(REG_FIFO_ADDR_PTR));
    writeRegister(REG_FIFO_RX_BASE_AD, 0x00); // Base address para RX
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);   // Pointer para início do FIFO
    printf("\nRegFifoRxBaseAddr após: 0x%02X", readRegister(REG_FIFO_RX_BASE_AD));
    printf("\nRegFifoAddrPtr após: 0x%02X", readRegister(REG_FIFO_ADDR_PTR));

    // Limpar IRQ flags
    printf("\n\n--- LIMPEZA DE IRQ FLAGS ---");
    printf("\nRegIrqFlags antes: 0x%02X", readRegister(REG_IRQ_FLAGS));
    writeRegister(REG_IRQ_FLAGS, 0xFF); // Limpa todas as flags de IRQ
    printf("\nRegIrqFlags após: 0x%02X", readRegister(REG_IRQ_FLAGS));

    // Modo standby final antes de entrar em RX
    printf("\n\n--- CONFIGURAÇÃO FINAL - MODO STANDBY ---");
    printf("\nRegOpMode antes standby: 0x%02X", readRegister(REG_OPMODE));
    setMode(RF95_MODE_STANDBY); // Coloca em modo stand by
    printf("\nRegOpMode após standby: 0x%02X", readRegister(REG_OPMODE));

    printf("\n\n=== CONFIGURAÇÃO LORA RX CONCLUÍDA ===\n");
}


bool receiveLoraData() {
    sensor_payload_t received_payload;
    
    printf("\n=== INICIANDO RECEPÇÃO ===\n");
    
    // Limpar IRQ flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    
    // Configurar FIFO para recepção
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));
    
    // Entrar em modo RX contínuo
    printf("Entrando em modo RX...\n");
    setMode(RF95_MODE_RX_CONTINUOUS);
    
    printf("Aguardando pacote LoRa...\n");
    
    // Aguardar recepção (timeout de 30 segundos)
    uint32_t timeout_start = to_us_since_boot(get_absolute_time());
    uint32_t timeout_us = 30000000; // 30 segundos
    
    while(!(readRegister(REG_IRQ_FLAGS) & 0x40)) { // Bit 6 = RxDone
        uint32_t current_time = to_us_since_boot(get_absolute_time());
        if(current_time - timeout_start > timeout_us) {
            printf("Timeout na recepção!\n");
            setMode(RF95_MODE_STANDBY);
            return false;
        }
        
        // Verificar se houve erro de CRC
        if(readRegister(REG_IRQ_FLAGS) & 0x20) { // Bit 5 = PayloadCrcError
            printf("Erro de CRC detectado!\n");
            writeRegister(REG_IRQ_FLAGS, 0xFF); // Limpar flags
            writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));
            continue; // Continuar aguardando próximo pacote
        }
        
        sleep_ms(10); // Pequeno delay para não sobrecarregar
    }
    
    printf("Pacote recebido!\n");
    
    // Verificar tamanho do payload recebido
    uint8_t rx_nb_bytes = readRegister(REG_RX_NB_BYTES);
    printf("Tamanho do payload recebido: %d bytes\n", rx_nb_bytes);
    
    // Verificar se o tamanho está correto
    if(rx_nb_bytes != sizeof(sensor_payload_t)) {
        printf("ERRO: Tamanho do payload incorreto! Esperado: %d, Recebido: %d\n", 
               sizeof(sensor_payload_t), rx_nb_bytes);
        writeRegister(REG_IRQ_FLAGS, 0xFF); // Limpar flags
        setMode(RF95_MODE_STANDBY);
        return false;
    }
    
    // Posicionar ponteiro do FIFO no início dos dados recebidos
    uint8_t fifo_rx_current_addr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
    writeRegister(REG_FIFO_ADDR_PTR, fifo_rx_current_addr);
    
    // Ler dados do FIFO byte a byte
    uint8_t *payload_bytes = (uint8_t*)&received_payload;
    for(int i = 0; i < sizeof(sensor_payload_t); i++) {
        payload_bytes[i] = readRegister(REG_FIFO);
    }
    
    // Limpar flags de IRQ
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    
    // Voltar para modo standby
    setMode(RF95_MODE_STANDBY);
    
    // Exibir dados recebidos
    printf("\n=== DADOS DOS SENSORES RECEBIDOS ===\n");
    printf("Temperatura AHT20: %.2f°C\n", received_payload.temp_aht20);
    printf("Umidade AHT20: %.2f%%\n", received_payload.humidity_aht20);
    printf("Temperatura BMP280: %.2f°C\n", received_payload.temp_bmp280);
    printf("Pressão BMP280: %.3f kPa\n", received_payload.pressure_bmp280);
    printf("=====================================\n");

    received_data = received_payload;
    
    return true;
}


void updateDisplay() {
    char str_temp_aht[20];
    char str_humi_aht[20];
    char str_temp_bmp[20];
    char str_press_bmp[20];
    char str_stats[20];
    
    // Preparar strings
    if(data_received) {
        sprintf(str_temp_aht, "%.1f C", received_data.temp_aht20);
        sprintf(str_humi_aht, "%.1f %%", received_data.humidity_aht20);
        sprintf(str_temp_bmp, "%.1f C", received_data.temp_bmp280);
        sprintf(str_press_bmp, "%.1f kPa", received_data.pressure_bmp280);
    } else {
        sprintf(str_temp_aht, "-- C");
        sprintf(str_humi_aht, "-- %%");
        sprintf(str_temp_bmp, "-- C");
        sprintf(str_press_bmp, "-- kPa");
    }
    
    sprintf(str_stats, "%lu/%lu", valid_packets, total_packets);
    
    // Frame base
    ssd1306_fill(&ssd, false);
    ssd1306_rect(&ssd, 0, 0, 128, 64, cor, !cor);
    
    // Cabeçalho
    ssd1306_rect(&ssd, 0, 0, 128, 12, cor, cor);
    ssd1306_draw_string(&ssd, "RX LoRa", 4, 3, true);
    
    switch(display_page) {
        case 1: // AHT20 - Temperatura
            ssd1306_draw_string(&ssd, "1/5", 95, 3, true);
            ssd1306_draw_string(&ssd, "TEMP AHT:", 4, 18, false);
            ssd1306_draw_string(&ssd, str_temp_aht, 4, 28, false);
            ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor);
            ssd1306_draw_string(&ssd, "AHT-TEMPERATURA", 4, 53, true);
            break;
            
        case 2: // AHT20 - Umidade
            ssd1306_draw_string(&ssd, "2/5", 95, 3, true);
            ssd1306_draw_string(&ssd, "UMID AHT:", 4, 18, false);
            ssd1306_draw_string(&ssd, str_humi_aht, 4, 28, false);
            ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor);
            ssd1306_draw_string(&ssd, "AHT - UMIDADE", 4, 53, true);
            break;
            
        case 3: // BMP280 - Temperatura
            ssd1306_draw_string(&ssd, "3/5", 95, 3, true);
            ssd1306_draw_string(&ssd, "TEMP BMP:", 4, 18, false);
            ssd1306_draw_string(&ssd, str_temp_bmp, 4, 28, false);
            ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor);
            ssd1306_draw_string(&ssd, "BMP-TEMPERATURA", 4, 53, true);
            break;
            
        case 4: // BMP280 - Pressão
            ssd1306_draw_string(&ssd, "4/5", 95, 3, true);
            ssd1306_draw_string(&ssd, "PRESS BMP:", 4, 18, false);
            ssd1306_draw_string(&ssd, str_press_bmp, 4, 28, false);
            ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor);
            ssd1306_draw_string(&ssd, "BMP - PRESSAO", 4, 53, true);
            break;
            
        case 5: // Estatísticas
            ssd1306_draw_string(&ssd, "5/5", 95, 3, true);
            ssd1306_draw_string(&ssd, "PACOTES:", 4, 18, false);
            ssd1306_draw_string(&ssd, str_stats, 4, 28, false);
            
            // Mostrar tempo desde último pacote
            if(data_received) {
                uint32_t time_since = (to_us_since_boot(get_absolute_time()) - last_packet_time) / 1000000;
                char time_str[20];
                sprintf(time_str, "Ultimo: %lus", time_since);
                ssd1306_draw_string(&ssd, time_str, 4, 38, false);
            }
            
            ssd1306_rect(&ssd, 51, 0, 128, 12, cor, cor);
            ssd1306_draw_string(&ssd, "ESTATISTICAS", 4, 53, true);
            break;
    }
    
    ssd1306_send_data(&ssd);
}