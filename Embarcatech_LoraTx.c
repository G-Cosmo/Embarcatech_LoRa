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

// Estrutura para representar o payload
typedef struct {
    float temp_aht20;      // 4 bytes
    float humidity_aht20;  // 4 bytes
    float temp_bmp280;     // 4 bytes
    float pressure_bmp280; // 4 bytes
    uint8_t checksum;      // 1 byte
} sensor_payload_t;  

uint32_t last_sensor_read = 0;

// Configurações do - TX:
uint16_t payload_len = 10; // Tamanho do payload em byte
uint16_t preamble_len = 8; // Tamanho do preâmbulo
uint16_t sf = 7; // fator de espalhamento (Spreading Factor)
uint16_t crc = 0; // tamanho do CRC (Cyclic Redundancy Check) em bytes (normalmente 2 bytes) (não encontrei uso para essa variável, tamanho do CRC?)
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
    // // Make the SPI pins available to picotool
    // bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

    // Inicializando o Chip Select em driven-high state, ele é ativado em low
    gpio_init(CSN_PIN);
    gpio_set_dir(CSN_PIN, GPIO_OUT);
    gpio_put(CSN_PIN, 1);
    // Make the CS pin available to picotool
    // bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));

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

        //if(current_sensor_read-last_sensor_read > 2000000){

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

            BMP280_data.pressure = pressure/1000.0f;
            BMP280_data.temperature = temperature/100.0f;

        //}

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

        ssd1306_send_data(&ssd);

        sendSensorData(); // Função que faz o envio do payload

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
    //printf("Impressão de valor binário da frequência:\n");

    // imprimir_binario(FrequencyValue); // Imprimir o valor binário de 32 bits - FrequencyValue
    // printf("\n");
    // printf("Frequ. Regs. config:\n");

    // Teste de leitura dos registradores RegFrMsb, RegFrMid e RegFrLsb.
    readRegister(REG_FRF_MSB);
    readRegister(REG_FRF_MID);
    readRegister(REG_FRF_LSB);
}

void setLora()
{
    setMode(1); // Coloca o registrador RegOpMode no modo sleep, os modos estão descritos em comentários na função

    printf("\n RegOpMode:");
    readRegister(REG_OPMODE);

    writeRegisterBit(REG_OPMODE, 7, 1); // Escreve o valor 1 no bit 7 de REG_OPMODE sem alterar os outros valores (coloca no modo LoRa)
    writeRegisterBit(REG_OPMODE, 6, 0); // Desativa o acesso ao registrador compartilhado com o modo FSK
    writeRegisterBit(REG_OPMODE, 5, 1); // Ativa o modo de alta frequência (permitir configuração pelo usuario posteriormente)

    setFrequency(freq); // Define a frequencia com a função já feita pelo prof

    setBW(bw); // Define a largua de banda

    // Define o coding rate de acordo com a variável cr (deve ser entre 1 e 4 e o default é 1)
    switch(cr) 
    {
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

    if(ih){  // Verifica a flag do implicit header mode

        writeRegisterBit(REG_MODEM_CONFIG, 0, 1);   // Ativa o implicit header mode

    }else
    {
        writeRegisterBit(REG_MODEM_CONFIG, 0, 0);   // Desativa o implicit header mode
    }

    setSF(sf);  // Define o spreading factor (O caso do sf 6 não foi tratado, precisa de uma config especifica, portanto, ELE SÓ PODE SER DE 7 ATÉ 12)

    if(crc_mode)    // Verifica a flag de ativação do CRC 
    {
        writeRegisterBit(REG_MODEM_CONFIG2, 2, 1); // Ativa o CRC (deve ser ativado no receptor também)
    }else
    {
        writeRegisterBit(REG_MODEM_CONFIG2, 2, 0); // Desativa o CRC
    }
    // O spreading factor utiliza bits do registrador REG_MODEM_CONFIG2
    // Esse registrador também é responsável por outros parametros (TxContinuousMode por exemplo)
    // Tirando o do CRC, eles foram deixados como padrão nesse primeiro momento, mas podem precisar ser alterados depois

    // Escreve 0x03 nos três ultimos bits do registrador RegDetectOptmize (Lora detection optmize)
    // Esse valor (0x03) considera apenas os casos de espreading factor 7 até 12
    // O CASO DO SPREADING FACTOR 6 AINDA PRECISA SER TRATADO
    writeRegisterField(REG_DETECT_OPT, 2, 3, 0x03);

    // Escreve 0x0A no registrador RegDetectionThreshold (Lora detection threshold)
    // Esse valor (0x0a) considera apenas os casos de espreading factor 7 até 12
    // O CASO DO SPREADING FACTOR 6 AINDA PRECISA SER TRATADO
    writeRegisterField(REG_DETECTION_THRESHOLD, 7, 8, 0x0A);


    if(ldro)   // Checa se o LowDataRateOptimize deve ser ligado
    {
        writeRegisterBit(REG_MODEM_CONFIG3, 3, 1); // ativa o LDRO
    }else{

        writeRegisterBit(REG_MODEM_CONFIG3, 3, 0); // desativa o LDRO
    }

    if(payload_len < 256 && payload_len > 0) // Verifica se o tamanho do payload está entre 1 e 255
    {
        writeRegister(REG_PAYLOAD_LENGTH, payload_len); // Escreve o valor decimal diretamente no registrador do payload

    }else{ // Se o valor não estiver fora do limite ele será definido como 255 bytes que é o máximo
        printf("\n Valor de tamanho do payload inválido, o valor será definido como 255 bytes");
        writeRegister(REG_PAYLOAD_LENGTH, 255);
    }

    // Configura o preâmbulo de acordo com a variável global
    if (preamble_len > 0) {

        writeRegister(REG_PREAMBLE_MSB, (preamble_len >> 8) & 0xFF);
        writeRegister(REG_PREAMBLE_LSB, preamble_len & 0xFF);

    } else {

        printf("\nValor de preâmbulo inválido, será definido como 8 (default)");

        writeRegister(REG_PREAMBLE_MSB, 0x00);
        writeRegister(REG_PREAMBLE_LSB, 0x08);
    }

    // Configurar potência de transmissão
    writeRegister(REG_PA_CONFIG, PA_MED_BOOST); // ou PA_MAX_BOOST se precisar de mais alcance

    // Configurar DIO0 para TxDone (modo TX) e RxDone (modo RX)
    writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 = TxDone/RxDone
	writeRegister(REG_DIO_MAPPING_2,0x00);
    

    setMode(2); // Coloca em modo stand by (talvez essa função deva ser chamada logo depois de configurar como LoRa na linha 312)

    printf("\n Configuração realizada, modo stand by ativo.");

}

void sendSensorData() {

    sensor_payload_t payload; // Cria uma estrutura do payload
    
    // Preparar dados
    payload.temp_aht20 = AHT20_data.temperature;
    payload.humidity_aht20 = AHT20_data.humidity;
    payload.temp_bmp280 = BMP280_data.temperature;
    payload.pressure_bmp280 = BMP280_data.pressure;
    
    // Calcular checksum simples
    uint8_t *data = (uint8_t*)&payload;
    uint8_t checksum = 0;
    for(int i = 0; i < sizeof(sensor_payload_t) - 1; i++) {
        checksum ^= data[i];
    }
    payload.checksum = checksum;
    
    // Enviar via LoRa
    setMode(2); // Standby
    
    // Limpar IRQ flags
    writeRegister(REG_IRQ_FLAGS, 0xFF);
    
    // Configurar FIFO
    writeRegister(REG_FIFO_TX_BASE_AD, 0x00);   // Define o endereço inicial da partição da FIFO em que os dados para transmissão serão armazenados (bits 128 até 255)
    writeRegister(REG_FIFO_ADDR_PTR, 0x00); // Desloca o ponteiro da FIFO para a posição inicial correspondentes aos dados de transmissão

        
    // A função setLora() já define o tamanho do payload de acordo com a variável global
    // No entanto, a linha abaixo reescreve o tamanho do payload para garantir que seja exatamente do tamanho da estrutura
    // Isso evita que um tamanho maior do que o necessário seja alocado, mas essa linha pode ser comentada
    writeRegister(REG_PAYLOAD_LENGTH, sizeof(sensor_payload_t)); 

    printf("\n Tamanho desejado do payload: %d, Tamanho armazenado:", sizeof(sensor_payload_t));
    readRegister(REG_PAYLOAD_LENGTH);

    // Escrever payload no FIFO
    uint8_t *payload_bytes = (uint8_t*)&payload;
    for(int i = 0; i < sizeof(sensor_payload_t); i++) {
        writeRegister(REG_FIFO, payload_bytes[i]);
    }

    // Iniciar transmissão
    setMode(4); // TX mode
    
    // Aguardar transmissão completar
    while(!(readRegister(REG_IRQ_FLAGS) & 0x08)) {
        sleep_ms(1);
    }
    
    // Limpar flag TxDone
    writeRegister(REG_IRQ_FLAGS, 0x08);

    uint8_t bytes_actually_sent = readRegister(REG_PAYLOAD_LENGTH); // 0x22
    printf("Configurado para enviar: %d bytes\n", sizeof(sensor_payload_t));
    printf("Efetivamente enviado (REG_PAYLOAD_LENGHT): %d bytes\n", bytes_actually_sent);

    // Voltar para standby
    setMode(2);
    
    printf("Dados enviados via LoRa!\n");
}
