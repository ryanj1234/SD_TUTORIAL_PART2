#include<avr/io.h>
#include<util/delay.h>

// pin definitions
#define DDR_SPI         DDRB
#define PORT_SPI        PORTB
#define CS              PINB2
#define MOSI            PINB3
#define MISO            PINB4
#define SCK             PINB5

// macros
#define CS_ENABLE()     PORT_SPI  &= ~(1 << CS)
#define CS_DISABLE()    PORT_SPI |= (1 << CS)

// command definitions
#define CMD0                0
#define CMD0_ARG            0x00000000
#define CMD0_CRC            0x94
#define CMD8                8
#define CMD8_ARG            0x0000001AA
#define CMD8_CRC            0x86
#define CMD58               58
#define CMD58_ARG           0x00000000
#define CMD58_CRC           0x00

// R1 responses
#define PARAM_ERROR(X)      X & 0b01000000
#define ADDR_ERROR(X)       X & 0b00100000
#define ERASE_SEQ_ERROR(X)  X & 0b00010000
#define CRC_ERROR(X)        X & 0b00001000
#define ILLEGAL_CMD(X)      X & 0b00000100
#define ERASE_RESET(X)      X & 0b00000010
#define IN_IDLE(X)          X & 0b00000001

#define POWER_UP_STATUS(X)  X & 0x40
#define CCS_VAL(X)          X & 0x40
#define VDD_2728(X)         X & 0b10000000
#define VDD_2829(X)         X & 0b00000001
#define VDD_2930(X)         X & 0b00000010
#define VDD_3031(X)         X & 0b00000100
#define VDD_3132(X)         X & 0b00001000
#define VDD_3233(X)         X & 0b00010000
#define VDD_3334(X)         X & 0b00100000
#define VDD_3435(X)         X & 0b01000000
#define VDD_3536(X)         X & 0b10000000

// R7 responses
#define CMD_VER(X)          ((X >> 4) & 0xF0)
#define VOL_ACC(X)          (X & 0x1F)
#define VOLTAGE_ACC_27_33   0b00000001
#define VOLTAGE_ACC_LOW     0b00000010
#define VOLTAGE_ACC_RES1    0b00000100
#define VOLTAGE_ACC_RES2    0b00001000

// UART functions
void UART_init(uint16_t baudRate);
void UART_putc(unsigned char data);
void UART_puts(char* charString);
void UART_puthex8(uint8_t val);
unsigned char UART_getc(void);

// SPI functions
void SPI_init(void);
uint8_t SPI_transfer(uint8_t data);

// SD functions
void SD_powerUpSeq(void);
void SD_command(uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t SD_readRes1(void);
void SD_readRes3_7(uint8_t *res);
uint8_t SD_goIdleState(void);
uint8_t *SD_sendIfCond(uint8_t *res);
uint8_t *SD_readOCR(uint8_t *res);
void SD_printR1(uint8_t res);
void SD_printR3(uint8_t *res);
void SD_printR7(uint8_t *res);

int main(void)
{
    // array to hold responses
    uint8_t res[5];

    // initialize UART
    UART_init(57600);

    // initialize SPI
    SPI_init();

    // start power up sequence
    SD_powerUpSeq();

    // received char from UART
    char c;

    while(1)
    {
        // print menu
        UART_puts("MENU\r\n");
        UART_puts("------------------\r\n");
        UART_puts("0 - Send CMD0\r\n1 - Send CMD8\r\n2 - Send CMD58\r\n");
        UART_puts("------------------\r\n");

        // get character from user
        c = UART_getc();

        if(c == '0')
        {
            // command card to idle
            UART_puts("Sending CMD0...\r\n");
            res[0] = SD_goIdleState();
            UART_puts("Response:\r\n");
            SD_printR1(res[0]);
        }
        else if(c == '1')
        {
            // send if conditions
            UART_puts("Sending CMD8...\r\n");
            SD_sendIfCond(res);
            UART_puts("Response:\r\n");
            SD_printR7(res);
        }
        else if(c == '2')
        {
            // send if conditions
            UART_puts("Sending CMD58...\r\n");
            SD_readOCR(res);
            UART_puts("Response:\r\n");
            SD_printR3(res);
        }
        else
        {
            UART_puts("Unrecognized command\r\n");
        }
    }
}

void SPI_init()
{
    // set CS, MOSI and SCK to output
    DDR_SPI |= (1 << CS) | (1 << MOSI) | (1 << SCK);

    // enable pull up resistor in MISO
    DDR_SPI |= (1 << MISO);

    // enable SPI, set as master, and clock to fosc/128
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
}

uint8_t SPI_transfer(uint8_t data)
{
    // load data into register
    SPDR = data;

    // Wait for transmission complete
    while(!(SPSR & (1 << SPIF)));

    // return SPDR
    return SPDR;
}

void SD_powerUpSeq()
{
    // make sure card is deselected
    CS_DISABLE();

    // give SD card time to power up
    _delay_ms(1);

    // select SD card
    SPI_transfer(0xFF);
    CS_ENABLE();

    // send 74 clock cycles to synchronize
    for(uint8_t i = 0; i < 74; i++)
        SPI_transfer(0xFF);

    // deselect SD card
    CS_DISABLE();
    SPI_transfer(0xFF);
}

void SD_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    // transmit command to sd card
    SPI_transfer(cmd|0x40);

    // transmit argument
    SPI_transfer((uint8_t)(arg >> 24));
    SPI_transfer((uint8_t)(arg >> 16));
    SPI_transfer((uint8_t)(arg >> 8));
    SPI_transfer((uint8_t)(arg));

    // transmit crc
    SPI_transfer(crc|0x01);
}

uint8_t SD_readRes1()
{
    uint8_t i = 0, res1;

    // keep polling until actual data received
    while((res1 = SPI_transfer(0xFF)) == 0xFF)
    {
        i++;

        // if no data received for 8 bytes, break
        if(i > 8) break;
    }

    return res1;
}

void SD_readRes3_7(uint8_t *res)
{
    // read response 1 in R7
    res[0] = SD_readRes1();

    // if error reading R1, return
    if(res[0] > 1) return;

    // read remaining bytes
    res[1] = SPI_transfer(0xFF);
    res[2] = SPI_transfer(0xFF);
    res[3] = SPI_transfer(0xFF);
    res[4] = SPI_transfer(0xFF);
}

uint8_t SD_goIdleState()
{
    // assert chip select
    SPI_transfer(0xFF);
    CS_ENABLE();
    SPI_transfer(0xFF);

    // send CMD0
    SD_command(CMD0, CMD0_ARG, CMD0_CRC);

    // read response
    uint8_t res1 = SD_readRes1();

    // deassert chip select
    SPI_transfer(0xFF);
    CS_DISABLE();
    SPI_transfer(0xFF);

    return res1;
}

uint8_t *SD_sendIfCond(uint8_t *res)
{
    // assert chip select
    SPI_transfer(0xFF);
    CS_ENABLE();
    SPI_transfer(0xFF);

    // send CMD8
    SD_command(CMD8, CMD8_ARG, CMD8_CRC);

    // read response
    SD_readRes3_7(res);

    // deassert chip select
    SPI_transfer(0xFF);
    CS_DISABLE();
    SPI_transfer(0xFF);

    return res;
}

uint8_t *SD_readOCR(uint8_t *res)
{
    // assert chip select
    SPI_transfer(0xFF);
    CS_ENABLE();
    SPI_transfer(0xFF);

    // send CMD58
    SD_command(CMD58, CMD58_ARG, CMD58_CRC);

    // read response
    SD_readRes3_7(res);

    // deassert chip select
    SPI_transfer(0xFF);
    CS_DISABLE();
    SPI_transfer(0xFF);

    return res;
}

void SD_printR1(uint8_t res)
{
    if(res & 0b10000000)
        UART_puts("\tError: MSB = 1\r\n");
    if(res == 0)
        UART_puts("\tCard Ready\r\n");
    if(PARAM_ERROR(res))
        UART_puts("\tParameter Error\r\n");
    if(ADDR_ERROR(res))
        UART_puts("\tAddress Error\r\n");
    if(ERASE_SEQ_ERROR(res))
        UART_puts("\tErase Sequence Error\r\n");
    if(CRC_ERROR(res))
        UART_puts("\tCRC Error\r\n");
    if(ILLEGAL_CMD(res))
        UART_puts("\tIllegal Command\r\n");
    if(ERASE_RESET(res))
        UART_puts("\tErase Reset Error\r\n");
    if(IN_IDLE(res))
        UART_puts("\tIn Idle State\r\n");
}

void SD_printR3(uint8_t *res)
{
    SD_printR1(res[0]);

    if(res[0] > 1) return;

    UART_puts("\tCard Power Up Status: ");
    if(POWER_UP_STATUS(res[1]))
    {
        UART_puts("READY\r\n");
        UART_puts("\tCCS Status: ");
        if(CCS_VAL(res[1])){ UART_puts("1\r\n"); }
        else UART_puts("0\r\n");
    }
    else
    {
        UART_puts("BUSY\r\n");
    }

    UART_puts("\tVDD Window: ");
    if(VDD_2728(res[3])) UART_puts("2.7-2.8, ");
    if(VDD_2829(res[2])) UART_puts("2.8-2.9, ");
    if(VDD_2930(res[2])) UART_puts("2.9-3.0, ");
    if(VDD_3031(res[2])) UART_puts("3.0-3.1, ");
    if(VDD_3132(res[2])) UART_puts("3.1-3.2, ");
    if(VDD_3233(res[2])) UART_puts("3.2-3.3, ");
    if(VDD_3334(res[2])) UART_puts("3.3-3.4, ");
    if(VDD_3435(res[2])) UART_puts("3.4-3.5, ");
    if(VDD_3536(res[2])) UART_puts("3.5-3.6");
    UART_puts("\r\n");
}

void SD_printR7(uint8_t *res)
{
    SD_printR1(res[0]);

    if(res[0] > 1) return;

    UART_puts("\tCommand Version: ");
    UART_puthex8(CMD_VER(res[1]));
    UART_puts("\r\n");

    UART_puts("\tVoltage Accepted: ");
    if(VOL_ACC(res[3]) == VOLTAGE_ACC_27_33)
        UART_puts("2.7-3.6V\r\n");
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_LOW)
        UART_puts("LOW VOLTAGE\r\n");
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_RES1)
        UART_puts("RESERVED\r\n");
    else if(VOL_ACC(res[3]) == VOLTAGE_ACC_RES2)
        UART_puts("RESERVED\r\n");
    else UART_puts("NOT DEFINED\r\n");

    UART_puts("\tEcho: ");
    UART_puthex8(res[4]);
    UART_puts("\r\n");
}

void UART_init(uint16_t baudRate)
{
	// calculate baud rate
	baudRate = (((F_CPU/(baudRate*16UL))) - 1);

	// set rate
	UBRR0H = (unsigned char)(baudRate >> 8);
	UBRR0L = (unsigned char) baudRate;

	// Enable reciever and transmitter
	UCSR0B |= (1 << RXEN0)|(1 << TXEN0);
}

void UART_putc(unsigned char data)
{
	// wait for empty transmit buffer
	while(!(UCSR0A & (1<<UDRE0)));

	// send data to output register
	UDR0 = data;
}

void UART_puts(char* charString)
{
	// iterate through string
	while(*charString > 0)
		// print character
		UART_putc(*charString++);
}

void UART_puthex8(uint8_t val)
{
    // extract upper and lower nibbles from input value
    uint8_t upperNibble = (val & 0xF0) >> 4;
    uint8_t lowerNibble = val & 0x0F;

    // convert nibble to its ASCII hex equivalent
    upperNibble += upperNibble > 9 ? 'A' - 10 : '0';
    lowerNibble += lowerNibble > 9 ? 'A' - 10 : '0';

    // print the characters
    UART_putc(upperNibble);
    UART_putc(lowerNibble);
}

unsigned char UART_getc(void)
{
	// wait for data to be received
	while(!(UCSR0A & (1 << RXC0)));

	// get data to output register
	return UDR0;
}
