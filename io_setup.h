/* ************************************************************************** */
/** io_setup.h
  @Description
    Setting up IO, PPS, and buttons/LED's.
 
 */
/* ************************************************************************** */
#ifndef _IO_SETUP_H    /* Guard against multiple inclusion */
#define _IO_SETUP_H
#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/attribs.h>


#define LED1        LATDbits.LATD0
#define LED2        LATDbits.LATD1
#define LED3        LATDbits.LATD2
#define BUTTON1     PORTDbits.RD6
#define BUTTON2     PORTDbits.RD7
#define BUTTON3     PORTDbits.RD13
void system_reg_unlock(void);
void system_reg_lock(void);
void uart1_setup(void);
void io_setup(void);
void delay(int ms);                 // *only edit if clock changes* Delay function in ms
void buttons(void);                 // *DO NOT EDIT* Polls buttons and calls button off/on


//accel_define header
// Registers and their respective addresses are defined below

#define CTRL_REG1       0x20
#define CTRL_REG2       0x21
#define CTRL_REG3       0x22
#define CTRL_REG4       0x23
#define CTRL_REG5       0x24
#define CTRL_REG6       0x25
#define REFERENCE       0x26
#define INT1_THS        0x32
#define INT1_DUR        0x33
#define INT1_CFG        0x30
#define FIFO_CTRL       0x2E

#define OUT_X_L         0x28
#define OUT_X_H         0x29
#define OUT_Y_L         0x2A
#define OUT_Y_H         0x2B
#define OUT_Z_L         0x2C
#define OUT_Z_H         0x2D

// The chip select pin being used for the accelerometer

#define ACCEL_CS              LATGbits.LATG9
void spi2_setup(void);
void spi2_write_register(uint8_t address, uint8_t data);
int16_t spi2_read_register(uint8_t address);
void accel_setup(void);               
float accel_read_x(void);
float accel_read_y(void);   
float accel_read_z(void);
void test_accel(void);
void accel_print_data(char axis);
void accel_move_cursor(void);


#endif /* _IO_SETUP_H */
/* *****************************************************************************
 End of File
 */
