/* 
 * File:   main.c
 * Author: kenshimizu
 *
 * Created on 2020/09/23, 23:44
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <xc.h>


// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#define _XTAL_FREQ  8000000 
#define LED0 RA0
#define LED1 RA1
#define LED2 RA2

#define I2C_ADDRESS 8

#define I2C_SEND_BUFFER_LENGTH  8
#define I2C_RECV_BUFFER_LENGTH  4
#define I2C_EOD                 0x00

#define I2C_CMD_SET_PWM  0x01
#define I2C_CMD_SET_LED  0x02

typedef unsigned char uchar;

void init_chip();
void start_interrupt();
void __interrupt() isr(void);

void _i2c_init_slave(unsigned char i2c_addr);
void _i2c_start_slave_interrupt();
void _i2c_slave_isr(void);
void _i2c_slave_recv_byte();
void _i2c_salve_bus_collision_isr(void);
void _i2c_on_cmd_received(uchar cmd);
void _i2c_reply_cmd(uchar cmd, char* ret);
void _i2c_slave_send_byte(void);

bool _pwm_init();
bool _pwm_set_lsb1(int per);
bool _pwm_set_lsb2(int per);
bool _pwm_set_lsb3(int per);
bool _pwm_set_lsb4(int per);

static char _i2c_send_buf[I2C_SEND_BUFFER_LENGTH + 1];  // send data buffer
static uchar _i2c_send_ptr = 0;                      // pointer of send data

static uchar _i2c_recv_buf[I2C_RECV_BUFFER_LENGTH + 1];  // send data buffer
static uchar _i2c_recv_ptr = 0;                      // pointer of receive data
static uchar _cmd = 0x00;

void Wait(unsigned int num){
    int i ;

    for (i=0 ; i<num ; i++){
        __delay_ms(10);
    }
}

/*
 * 
 */
int main(int argc, char** argv) { 
    init_chip();
    start_interrupt();
    
    if(!_pwm_init()){
        return 0;
    }

    LED1 = 0;
    LED2 = 0;
    
    while(1){
        Wait(1000);
    }
    
    return (EXIT_SUCCESS);
}

void init_chip() {
    OSCCON = 0b01101010;    // 4MHz
    /*
     * OSCCON - Oscillator Control Registers
     * 
     * bit 7
     * SPLLEN: Software PLL Enable bit
     * If PLLEN in Configuration Word 1 = 1:
     * SPLLEN bit is ignored. 4x PLL is always enabled (subject to oscillator requirements)
     * If PLLEN in Configuration Word 1 = 0:
     * 1 = 4x PLL Is enabled
     * 0 = 4x PLL is disabled
     * 
     * bit 6-3
     * IRCF<3:0>: Internal Oscillator Frequency Select bits
     * 000x = 31 kHz LF
     * 0010 = 31.25 kHz MF
     * 0011 = 31.25 kHz HF(1)
     * 0100 = 62.5 kHz MF
     * 0101 = 125 kHz MF
     * 0110 = 250 kHz MF
     * 0111 = 500 kHz MF (default upon Reset)
     * 1000 = 125 kHz HF(1)
     * 1001 = 250 kHz HF(1)
     * 1010 = 500 kHz HF(1)
     * 1011 = 1 MHz HF
     * 1100 = 2 MHz HF
     * 1101 = 4 MHz HF
     * 1110 = 8 MHz or 32 MHz HF (see Section 5.2.2.1 ?HFINTOSC?)
     * 1111 = 16 MHz HF
     * 
     * bit 2
     * Unimplemented: Read as ?0?
     * 
     * bit 1-0
     * SCS<1:0>: System Clock Select bits
     * 1x = Internal oscillator block
     * 01 = Timer1 oscillator
     * 00 = Clock determined by FOSC<2:0> in Configuration Word 1
     */
    
    APFCON0 = 0b00000000;
    APFCON1 = 0b00000000;
    /*
     * APFCON0: ALTERNATE PIN FUNCTION CONTROL REGISTER 0
     * APFCON1: ALTERNATE PIN FUNCTION CONTROL REGISTER 1
     */
    
    // No Analog used
    ANSELA = 0b00000000;
    ANSELB = 0b00000000;
    /*
     * ANSELA: PORTA ANALOG SELECT REGISTER
     * ANSELB: PORTB ANALOG SELECT REGISTER
     */
    
    // All for output
    TRISA = 0b00000000;
    TRISB = 0b00000000;
    //TRISB = 0b00011010;
    /*
     * TRISA: PORTA TRI-STATE REGISTER
     * TRISB: PORTB TRI-STATE REGISTER
     */
    
    _i2c_init_slave(I2C_ADDRESS);   // init i2c for slave as I2C_ADDRESS address

    PORTA  = 0b00000000;    // initialize all pins
    PORTB  = 0b00000000;    // initialize all pins
    /*
     * PORTA: PORTA REGISTER
     * PORTB: PORTB REGISTER
     */
}

void _i2c_init_slave(unsigned char i2c_addr) {
    TRISB |= 0b00010010;    // SCL/SDA for input (RA3 always for Input)

    SSP1STAT = 0b10000000 ; // Slew rate control disabled for standard speed mode (100 kHz and 1 MHz)
    SSP1CON1 = 0b00110110; 
    //SSP1CON1 = 0b00100110;  // Enables the serial port and configures the SDA and SCL pins as the source of the serial port pins
                            //  and I2C Slave mode, 7-bit address
    //SSP1CON2bits.GCEN = 1;  // Enable interrupt when a general call address (0x00 or 00h) is received in the SSP1SR
    //SSP1CON2bits.SEN = 1;   // Clock stretching is enabled for both slave transmit and slave receive (stretch enabled)
    //SSP1CON3bits.PCIE = ON;  // Enable interrupt on detection of Stop condition
    SSP1CON2 = 0b10000001;
    SSP1CON3 = 0x00; 
    SSP1ADD = I2C_ADDRESS<<1 ;   // 7-bit address, bit 0 Not used: Unused in this mode. Bit state is a ?don?t care?
    SSP1MSK = 0b11111110 ;   // bit 0: 2C Slave mode, 7-bit address, the bit is ignored
    
    // initialize pointer and buffer
    // _i2c_send_ptr = 0;
    // _i2c_send_buf[I2C_SEND_BUFFER_LENGTH] = I2C_EOD;
}

void start_interrupt() {   
    _i2c_start_slave_interrupt();
    
    PEIE = 1;      // Enables all active peripheral interrupts
    GIE = 1;       // Enables all active interrupts 
}

void _i2c_start_slave_interrupt() {
    SSP1IE = 1;    // Enables the MSSP(I2C) interrupt
    BCL1IE = 1;    // Enables the MSSP(I2C) Bus Collision Interrupt
    SSP1IF = 0;     // Reset Synchronous Serial Port (MSSP) Interrupt Flag bit 
    BCL1IF = 0;     // Reset MSSP Bus Collision Interrupt Flag bit
}

void __interrupt() isr(void) {
    if (SSP1IF == 1) {    // I2C intterupt
        _i2c_slave_isr();
    }
    if (BCL1IF == 1) {    // MSSP(I2C) Bus Collision Interrupt
        BCL1IE = 0;
        _i2c_salve_bus_collision_isr();
        BCL1IE = 1;
    }
}

void _i2c_slave_isr(void) {
    // I2C slave interrupt
    if (SSP1STATbits.R_nW == 0) {  // master write mode
        if (SSP1STATbits.D_nA == 0) {  // Address mode (start write session)
            unsigned char dummy;
            dummy = SSP1BUF;    // read dummy (address)
        } else {    // data mode
            _i2c_slave_recv_byte();     // read byte
        }
    } else {                        // master read mode
        if (SSP1STATbits.D_nA == 0) {
            unsigned char dummy;
            _i2c_send_ptr = 0;
            dummy = SSP1BUF;    // read as (address * 2 + 1)
        } else {
            _i2c_slave_send_byte();     // write byte
        }
    }
    SSP1IF = 0;   // clear interrupt flag
    SSP1CON1bits.CKP = 1;  // Enable clock
}

void _i2c_slave_recv_byte() {
    uchar c;
    // read data and copy to receive buffer
    c = SSP1BUF;
    _i2c_recv_buf[_i2c_recv_ptr % I2C_RECV_BUFFER_LENGTH] = c;
    if((_i2c_recv_ptr % I2C_RECV_BUFFER_LENGTH) == (I2C_RECV_BUFFER_LENGTH - 1)){
        _i2c_recv_buf[I2C_RECV_BUFFER_LENGTH] = 0;
        _i2c_on_cmd_received(_cmd);
    } else if((_i2c_recv_ptr % I2C_RECV_BUFFER_LENGTH) == 0){
        _cmd = c;
    }
    _i2c_recv_ptr++;
}
void _i2c_slave_send_byte(void) {
    if (_i2c_send_buf[_i2c_send_ptr] == I2C_EOD) {   // terminate character
        // over buffer, send 0x00 as no data
        SSP1BUF = 0x00;
    } else {
        // send next data
        SSP1BUF = _i2c_send_buf[_i2c_send_ptr];
        _i2c_send_ptr++;
    }
}

void _i2c_salve_bus_collision_isr(void) {
    BCL1IF = 0;   // clear interrupt flag
}

void _i2c_slave_prepare_send_data(uchar* data) {
    for(int i = 0; i < I2C_SEND_BUFFER_LENGTH; i++) {
        _i2c_send_buf[i] = data[i];
        if (data[i] == I2C_EOD)    break;      // terminate character
    }
    _i2c_send_ptr = 0;
}

void _i2c_on_cmd_received(uchar cmd) {
    char ret[8];
    _i2c_reply_cmd(cmd, ret);
    _i2c_slave_prepare_send_data(ret);
    uchar* len = strlen(ret);
    SSP1BUF = len;
}

void _i2c_reply_cmd(uchar cmd, char* ret) {
    // implements application code for process message
    // and returns reply message to i2c master

    //uchar cmd = msg[0];
    int v = 0;
    //char reply[I2C_SEND_BUFFER_LENGTH + 1];
    //char ret[8];
    
    if(cmd == I2C_CMD_SET_PWM){
        int id = _i2c_recv_buf[1];
        int v = _i2c_recv_buf[2];
        sprintf(ret, "%d-%d", id, v);
        if(id == 1){
            setPWM1(v);
        }else if(id == 2){
            _pwm_set_lsb2(v);
        }else if(id == 3){
            _pwm_set_lsb3(v);
        }else if(id == 4){
            _pwm_set_lsb4(v);
        }else{
            setPWM1(v);
        }
    }else if(cmd == I2C_CMD_SET_LED){
        int id = _i2c_recv_buf[1];
        int v = _i2c_recv_buf[2];
        if(id == 0){
            LED0 = v;
        }else if(id == 1){
            LED1 = v;
        } else if(id == 2){
            LED2 = v;
        }
        sprintf(ret, "%d-%d", id, v);
    }else{
        sprintf(ret, "na");
    }
}

/*
 * FOSC : The Frequency of the OSCilator - The FOSC bits determine the type of 
 * oscillator that will be used when the device is first powered. 
 * The oscilator is the clock that controls the execution of the instructions.
 * TOSC : The Time of OSC is the period of Fosc or 1/FOSC. (TOSC = 1/FOSC)
 * OSCCON : Oscillator Control Registers
 * PRx : The PWM period is specified by the PRx register ofTimer2/4/6. 
 * The PWM period can be calculated using the formula
 * PWM Period = [(PRx) + 1] * 4 * TOSC * (TMRx Prescale Value) 
 * Example:
 * OSCCON : 4MHz
 * PR2 : 255
 * TMR2 Prescaler : 64 **
 * Execution Cycle: (1s/4MHz) * 4 clock = 1 micro second
 * PWM Period : [(PRx) + 1] * 4 * Tosc * (TMRx Prescale Value)
 *              (255 + 1) * 4 * 1 * 64
 *              65535 micro second
 *              4.096 milli seconds
 * Palse Width : (CCPRxL:CCPxCON<5:4>) * Tosc * (TMRxPrescale Value)
 * CCPRxL : 10 -> 10 * 1 * 16 = 160 micro sec = 0.16ms
 * CCPRxL : 25 -> 25 * 1 * 16 = 400 micro sec = 0.4ms
 * CCPRxL : 40 -> 40 * 1 * 16 = 640 micro sec = 0.64ms
 * 
 * **TXCON
 * bit 7 Unimplemented: Read as ?0?
 * bit 6-3 TxOUTPS<3:0>: Timerx Output Postscaler Select bits
 * 0000 = 1:1 Postscaler
 * 0001 = 1:2 Postscaler
 * 0010 = 1:3 Postscaler
 * 0011 = 1:4 Postscaler
 * 0100 = 1:5 Postscaler
 * 0101 = 1:6 Postscaler
 * 0110 = 1:7 Postscaler
 * 0111 = 1:8 Postscaler
 * 1000 = 1:9 Postscaler
 * 1001 = 1:10 Postscaler
 * 1010 = 1:11 Postscaler
 * 1011 = 1:12 Postscaler
 * 1100 = 1:13 Postscaler
 * 1101 = 1:14 Postscaler
 * 1110 = 1:15 Postscaler
 * 1111 = 1:16 Postscaler
 * bit 2 TMRxON: Timerx On bit
 * 1 = Timerx is on
 * 0 = Timerx is off
 * bit 1-0 TxCKPS<1:0>: Timer2-type Clock Prescale Select bits
 * 00 = Prescaler is 1
 * 01 = Prescaler is 4
 * 10 = Prescaler is 16
 * 11 = Prescaler is 64
 * 
 * SG90 Tower Pro
 * Position "0" (1.5 ms pulse) is middle, "90" (~2ms pulse) is middle,
is all the way to the right, "-90" (~1ms pulse) is all the way to the left.
 * voltage: 4.8 - 6
 * 
 * SG90 DM DIY MORE
 * Operating voltage: 3.0V~7.2V
 * 
 * Memo:
 * Period : 20,000 mic sec
 * Duty Cycle : 1000 to 2000 mic sec.
 */
bool _pwm_init()
{
    CCP3CON = 0b00001100;
    CCP4CON = 0b00001100;
    CCP2CON = 0b00001100;
    
    CCPTMRS = 0b00000000;
    
    T2CON = 0b00000111;
    PR2 = 255;

    
    return true;
}
 
bool setPWM1(int per)
{
    CCPR1L = per;
    return true;
}
 
bool _pwm_set_lsb2(int per)
{
    CCPR2L = per;
    return true;
}

bool _pwm_set_lsb3(int per)
{
    CCPR3L = per;
    return true;
}

bool _pwm_set_lsb4(int per)
{
    CCPR4L = per;
    return true;
}