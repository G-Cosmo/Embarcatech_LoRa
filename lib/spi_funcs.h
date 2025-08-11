
#ifndef SPI_FUNCS_H
#define SPI_FUNCS_H

#include "pico/stdlib.h"
#include "lora.h"

#define READ_BIT 0x80

#define SPI_PORT spi0


#ifdef PICO_DEFAULT_SPI_CSN_PIN
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}
#endif

#if defined(spi_default) && defined(PICO_DEFAULT_SPI_CSN_PIN)

// Escrever no registrador do Rádio LoRa - SX1276 (função de Ricardo)
static void writeRegister(uint8_t reg, uint8_t data)
{
    uint8_t buf[2]; // Buffer para escrita no módulo LoRa

    buf[0] = reg | 0x80; // Registrador com operação de escrita
    buf[1] = data; // Byte de informação inserida no registrador

    cs_select(); // Iniciar comunicação SPI
    spi_write_blocking(SPI_PORT, buf, 2); // Transmitir informação do Buffer pela interface SPI
    cs_deselect(); // Finalizar comunicação SPI

    sleep_ms(1); // Delay de 1 ms
}


// Ler conteúdo registrador do Rádio LoRa - SX1276 (função de Ricardo)
static uint8_t readRegister(uint8_t addr)
{
    uint8_t buf[1]; // Buffer para leitura do módulo LoRa

    addr &= 0x7F; // Resgistrador com operação de leitura

    cs_select(); // Iniciar comunicação SPI
    spi_write_blocking(SPI_PORT, &addr, 1); // Transmitir informação do Buffer pela interface SPI - Endereço de leitura
    sleep_ms(1); // Delay de 1 ms
    spi_read_blocking(SPI_PORT, 0, buf, 1); // Receber informação pela interface SPI e armazenar no buffer
    cs_deselect(); // Finalizar comunicação SPI

    sleep_ms(1); // Delay de 1 ms

    printf("READ %02X\n", buf[0]); // Imprimir resultado obtido pela leitura do registrador
    return buf[0]; // Retornar resultado do valor armazenado no registrador
}


// Escreve 1 ou 0 em um bit especifico do registrado informado
static void writeRegisterBit(uint8_t reg, uint8_t bitPos, uint8_t bitVal)
{
    // Garante que bitPos está entre 0 e 7
    if (bitPos > 7) return;

    uint8_t value = readRegister(reg); // Lê valor atual do registrador

    if (bitVal) {
        value |= (1 << bitPos);  // Coloca o bit em 1
    } else {
        value &= ~(1 << bitPos); // Coloca o bit em 0
    }

    writeRegister(reg, value); // Escreve de volta
}

// Escreve um bloco de bits especifico em uma posição especifica do registrador
// Reg é o registrador escrito
// BitPos é a posição do bit mais significativo do bloco, ou seja, se quiser escrever nos bits 2 até 5, bitPos deve ser 5
// bitLen é o tamanho do bloco, se deseja escrever nos bits 2 até 5 então bitLen deve ser 4
// value é o valor a ser escrito, como 0b0101, por exemplo. Também pode ser usado o valor em hexadecimal ou em decimal
// Os outros bits permanecerão inalterados, seguindo o exemplo dos bits 2..5, os bits 0..1 e 6..7 permanecerão como estavam antes da escrita
static void writeRegisterField(uint8_t reg, uint8_t bitPos, uint8_t bitLen, uint8_t value) {
    
    if (bitPos > 7 || bitLen == 0 || bitLen > 8 || (bitPos + 1) < bitLen) return;

    uint8_t regValue = readRegister(reg);

    // Criar máscara para o campo (exemplo: se bitLen=3, mask=0b111)
    uint8_t mask = ((1 << bitLen) - 1) << (bitPos + 1 - bitLen);

    // Limpar os bits do campo
    regValue &= ~mask;

    // Colocar o valor alinhado na posição certa
    regValue |= (value << (bitPos + 1 - bitLen)) & mask;

    writeRegister(reg, regValue);
}

//altera apenas os 3 ultimos bits do registrador RegOpMode (muda o modo, sem mudar as outras configurações)
static void setMode(uint opt)
{
    switch (opt)
    {
    case 1:     //sleep mode  
        writeRegisterField(REG_OPMODE, 2, 3, 0b000);
        break;
    case 2:     //stdby mode   
        writeRegisterField(REG_OPMODE, 2, 3, 0b001);
        break;
    case 3:     //FSTX mode
        writeRegisterField(REG_OPMODE, 2, 3, 0b010);
        break;
    case 4:    //TX mode
        writeRegisterField(REG_OPMODE, 2, 3, 0b011); 
        break;
    case 5:    //FSRX mode
        writeRegisterField(REG_OPMODE, 2, 3, 0b100);
        break;
    case 6:    //RXCONTINUOUS mode
        writeRegisterField(REG_OPMODE, 2, 3, 0b101);  
        break;
    case 7:    //RXSINGLE mode
        writeRegisterField(REG_OPMODE, 2, 3, 0b110);
        break;
    case 8:    //CAD mode
        writeRegisterField(REG_OPMODE, 2, 3, 0b111);
        break;
    default:
        break;
    }
}


// Escreve nos 4 bits mais significativos do registrador RegModemConfig1 o binario equivalente à largura de banda escolhida
void setBW(uint32_t bw)
{  
    switch (bw)
    {
    case 7800:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b0000);
        break;
    case 10400:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b0001);
        break;
    case 15600:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b0010);
        break;
    case 20800:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b0011);
        break;
    case 31250:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b0100);
        break;
    case 41700:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b0101);
        break;
    case 62500:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b0110);
        break;
    case 125000:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b0111);
        break;
    case 250000:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b1000);
        break;
    case 500000:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b1001);
        break;
    default:
        writeRegisterField(REG_MODEM_CONFIG, 7, 4, 0b0111);
        break;
    }
}

void setSF(uint32_t sf)
{  
    switch (sf)
    {
    case 7:
        writeRegisterField(REG_MODEM_CONFIG2, 7, 4, 0x07);
        break;
    case 8:
        writeRegisterField(REG_MODEM_CONFIG2, 7, 4, 0x08);
        break;
    case 9:
        writeRegisterField(REG_MODEM_CONFIG2, 7, 4, 0x09);
        break;
    case 10:
        writeRegisterField(REG_MODEM_CONFIG2, 7, 4, 0x0A);
        break;
    case 11:
        writeRegisterField(REG_MODEM_CONFIG2, 7, 4, 0x0B);
        break;
    case 12:
        writeRegisterField(REG_MODEM_CONFIG2, 7, 4, 0x0C);
        break;
    default:
        writeRegisterField(REG_MODEM_CONFIG2, 7, 4, 0x07);
        break;
    }
}


#endif


#endif