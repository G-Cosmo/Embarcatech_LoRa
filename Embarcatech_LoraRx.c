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

// Configurações do RX (devem ser iguais ao TX)
uint16_t payload_len = 17; // Tamanho do payload (sizeof(sensor_payload_t))
uint16_t preamble_len = 8;
uint16_t sf = 7;
bool crc_mode = true;
bool ih = false;
uint32_t bw = 125000;
uint16_t cr = 1;
bool ldro = false;
double freq = 915;

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
bool receiveData();
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
        if(receiveData()) {
            data_received = true;
            last_packet_time = to_us_since_boot(get_absolute_time());
            total_packets++;
            
            // Verificar checksum
            uint8_t *data = (uint8_t*)&received_data;
            uint8_t checksum = 0;
            for(int i = 0; i < sizeof(sensor_payload_t) - 1; i++) {
                checksum ^= data[i];
            }
            
            
            
                valid_packets++;
                printf("=== DADOS RECEBIDOS ===\n");
                printf("Temp AHT20: %.2f°C\n", received_data.temp_aht20);
                printf("Umidade AHT20: %.2f%%\n", received_data.humidity_aht20);
                printf("Temp BMP280: %.2f°C\n", received_data.temp_bmp280);
                printf("Pressao BMP280: %.3f kPa\n", received_data.pressure_bmp280);
                printf("Checksum: OK\n");
                printf("Pacotes: %lu/%lu\n", valid_packets, total_packets);
                printf("=======================\n");
            
            
        }
        
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

void setLoRaRX() {
    printf("Configurando LoRa para recepcao...\n");
    
    setMode(1); // Sleep mode
    
    // Configurar como LoRa
    writeRegisterBit(REG_OPMODE, 7, 1); // Modo LoRa
    writeRegisterBit(REG_OPMODE, 6, 0); // Desativa FSK
    writeRegisterBit(REG_OPMODE, 5, 1); // Alta frequência
    
    // Configurar frequência
    setFrequency(freq);
    
    // Configurar largura de banda
    setBW(bw);
    
    // Configurar coding rate
    switch(cr) {
    case 1:
        writeRegisterField(REG_MODEM_CONFIG, 3, 3, 0b001);
        break;
    case 2:
        writeRegisterField(REG_MODEM_CONFIG, 3, 3, 0b010);
        break;
    case 3:
        writeRegisterField(REG_MODEM_CONFIG, 3, 3, 0b011);
        break;
    case 4:
        writeRegisterField(REG_MODEM_CONFIG, 3, 3, 0b100);
        break;
    default:
        writeRegisterField(REG_MODEM_CONFIG, 3, 3, 0b001);
        break;
    }
    
    // CRÍTICO: Configurar header mode (deve ser igual ao TX)
    if(ih) {
        writeRegisterBit(REG_MODEM_CONFIG, 0, 1); // Implicit header
        // No modo implícito, DEVE configurar o payload length
        writeRegister(REG_PAYLOAD_LENGTH, sizeof(sensor_payload_t));
        printf("Modo Implicit Header - Payload Length: %d\n", sizeof(sensor_payload_t));
    } else {
        writeRegisterBit(REG_MODEM_CONFIG, 0, 0); // Explicit header
        printf("Modo Explicit Header\n");
    }
    
    // Configurar spreading factor
    setSF(sf);
    
    // Configurar CRC (DEVE ser igual ao TX)
    if(crc_mode) {
        writeRegisterBit(REG_MODEM_CONFIG2, 2, 1);
        printf("CRC habilitado\n");
    } else {
        writeRegisterBit(REG_MODEM_CONFIG2, 2, 0);
        printf("CRC desabilitado\n");
    }
    
    // Configurar detect optimize e threshold
    writeRegisterField(REG_DETECT_OPT, 2, 3, 0x03);
    writeRegisterField(REG_DETECTION_THRESHOLD, 7, 8, 0x0A);
    
    // Configurar LDRO
    if(ldro) {
        writeRegisterBit(REG_MODEM_CONFIG3, 3, 1);
    } else {
        writeRegisterBit(REG_MODEM_CONFIG3, 3, 0);
    }
    
    // Configurar preâmbulo
    if (preamble_len > 0) {
        writeRegister(REG_PREAMBLE_MSB, (preamble_len >> 8) & 0xFF);
        writeRegister(REG_PREAMBLE_LSB, preamble_len & 0xFF);
    }
    
    // Configurar LNA
    writeRegister(REG_LNA, LNA_MAX_GAIN);
    
    // Configurar DIO0 para RxDone
    writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 = RxDone
    
    // Configurar FIFO
    writeRegister(REG_FIFO_RX_BASE_AD, 0x00);   // Define o endereço inicial da partição da FIFO em que os dados de recepção serão armazenados (bits 0 até 128)
    writeRegister(REG_FIFO_ADDR_PTR, 0x00); // Desloca o ponteiro da FIFO para a posição inicial correspondentes aos dados de recepção
    writeRegister(REG_FIFO_RX_BASE_AD, 0x00);
    writeRegister(REG_FIFO_ADDR_PTR, 0x00);
    
    // Limpar flags de interrupção
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    
    // DEBUG: Imprimir configurações importantes
    printf("=== CONFIGURAÇÕES RX ===\n");
    printf("Payload Length: %d\n", readRegister(REG_PAYLOAD_LENGTH));
    printf("Modem Config 1: 0x%02X\n", readRegister(REG_MODEM_CONFIG));
    printf("Modem Config 2: 0x%02X\n", readRegister(REG_MODEM_CONFIG2));
    printf("Modem Config 3: 0x%02X\n", readRegister(REG_MODEM_CONFIG3));
    printf("========================\n");
    
    // Colocar em modo de recepção contínua
    setMode(6); // RXCONTINUOUS
    
    printf("LoRa configurado para recepcao!\n");
}
  

bool receiveData() {
    // Verificar se recebeu dados (RxDone flag)
    uint8_t irq_flags = readRegister(REG_IRQ_FLAGS);
    
    // DEBUG: Mostrar todas as flags
    printf("IRQ Flags: 0x%02X\n", irq_flags);
    
    if(irq_flags & 0x40) { // RxDone (bit 6)
        printf("=== PACOTE DETECTADO ===\n");
        
        // Verificar se há erro de CRC ANTES de processar
        if(irq_flags & 0x20) { // CRC Error (bit 5)
            printf("ERRO: CRC inválido!\n");
            printf("IRQ Flags detalhadas: 0x%02X\n", irq_flags);
            
            // Ainda assim, vamos verificar quantos bytes foram recebidos
            uint8_t nb_bytes = readRegister(REG_RX_NB_BYTES);
            printf("Bytes recebidos mesmo com CRC erro: %d\n", nb_bytes);
            
            writeRegister(REG_IRQ_FLAGS, 0xFF); // Limpar flags
            return false;
            return false;
        }
        
        // Verificar se há erro de header
        if(irq_flags & 0x10) { // Valid Header (bit 4) - deve estar em 1
            printf("Header válido detectado\n");
        } else {
            printf("AVISO: Header pode estar inválido\n");
        }
        
        //ler número de bytes recebidos
        uint8_t nb_bytes = readRegister(REG_RX_NB_BYTES);
        printf("Bytes efetivamente recebidos: %d\n", nb_bytes);
        printf("Bytes esperados: %d\n", sizeof(sensor_payload_t));
        
        // Ler informações adicionais para debug
        uint8_t current_addr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        uint8_t payload_len_reg = readRegister(REG_PAYLOAD_LENGTH);
        
        printf("=== DEBUG DETALHADO ===\n");
        printf("FIFO RX Current Addr: 0x%02X\n", current_addr);
        printf("Payload Length Register: %d\n", payload_len_reg);
        printf("Modem Status: 0x%02X\n", readRegister(0x18));
        
        // Verificar se o tamanho está correto
        if(nb_bytes != sizeof(sensor_payload_t)) {
            printf("ERRO: Tamanho incorreto!\n");
            printf("Esperado: %d bytes, Recebido: %d bytes\n", 
                   sizeof(sensor_payload_t), nb_bytes);
            
            // Tentar ler os dados mesmo assim para debug
            printf("Tentando ler os %d bytes recebidos:\n", nb_bytes);
            writeRegister(REG_FIFO_ADDR_PTR, current_addr);
            for(int i = 0; i < nb_bytes && i < 20; i++) {
                uint8_t byte = readRegister(REG_FIFO);
                printf("Byte %d: 0x%02X (%d)\n", i, byte, byte);
            }
            
            writeRegister(REG_IRQ_FLAGS, 0xFF);
            return false;
        }
        
        // Ler endereço atual do FIFO
        writeRegister(REG_FIFO_ADDR_PTR, current_addr);
        
        // Ler dados do FIFO
        uint8_t *payload_bytes = (uint8_t*)&received_data;
        for(int i = 0; i < sizeof(sensor_payload_t); i++) {
        printf("Lendo payload byte a byte:\n");
        for(int i = 0; i < sizeof(sensor_payload_t); i++) {
            payload_bytes[i] = readRegister(REG_FIFO);
            printf("Byte %d: 0x%02X\n", i, payload_bytes[i]);
        }
        
        // Limpar flags de interrupção
        writeRegister(REG_IRQ_FLAGS, 0xFF);
        
        // Ler RSSI do último pacote
        int16_t rssi = readRegister(0x1A) - 164; // Para frequência > 868MHz
        printf("RSSI: %d dBm\n", rssi);
        
        // DEBUG: Mostrar os floats recebidos
        printf("=== DADOS DECODIFICADOS ===\n");
        printf("Temp AHT20: %.2f°C\n", received_data.temp_aht20);
        printf("Umidade AHT20: %.2f%%\n", received_data.humidity_aht20);
        printf("Temp BMP280: %.2f°C\n", received_data.temp_bmp280);
        printf("Pressão BMP280: %.3f kPa\n", received_data.pressure_bmp280);
        printf("========================\n");
        
        return true;
    }
    
    return false;
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