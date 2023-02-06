/* ************************************************************************** */
/** accel_define.c

  @Description
    Delay function and define statements.
 */
/* ************************************************************************** */

#include "accel_define.h"
#include <xc.h>
#include <stdio.h>
// this file is only necessary when you are creating your library

void spi2_setup(void)
{
    TRISGbits.TRISG6 = 0;     // set serial clock as an output
    TRISGbits.TRISG9 = 0;     // set Accelerometer Chip Select as an output
    // disable chip select "ACCEL_CS"
    //SPI2CONbits.SSEN = 0;   
    // clear SPI2CON register
    SPI2CON = 0;
    SPI2STAT = 0x0080;
    // clear SPI2BUF register
    SPI2BUF = 0;
    // SPI2BRG = ?   
    SPI2BRG = 511;        //Table 23-4     Page 23-29
    // clear overflow flag in SPI2STAT register
    SPI2STATbits.SPIROV = 0;
    // framed support off
    SPI2CONbits.FRMEN = 0;
    // 16 bit communication
    SPI2CONbits.MODE32 = 0;
    SPI2CONbits.MODE16 = 1;      //16 bit communication when AUDEN = 0
     
    // CKE
    SPI2CONbits.CKE = 0;    //0 = Serial output data changes on transition from idle clock state to active clock state
    // CKP
    SPI2CONbits.CKP = 1;     //1 = Idle state for clock is a high level; active state is a low level
    // master mode
    SPI2CONbits.MSTEN = 1;
    // sample data at the end of clock time
    SPI2CONbits.SMP = 1;
    // enable SPI2 (ON)
    SPI2CONbits.ON = 1;
}

void spi2_write_register(uint8_t address, uint8_t data)
{   
    uint16_t write_frame;
    uint16_t trash;
    
    write_frame = ((uint16_t)address << 8) | data;
            
    delay(1);
    // enable accelerometer chip select
    LATGbits.LATG9 = 0;
    // send "write_frame" to the SPI2 buffer
    SPI2BUF = write_frame;
    // wait for the SPI2 buffer full bit
    while(!SPI2STATbits.SPIRBF){};
    // clear SPI2 buffer using the "trash" variable
    trash = SPI2BUF;
    // disable accelerometer chip select
    LATGbits.LATG9 = 1;
}

int16_t spi2_read_register(uint8_t address)
{
    uint16_t read_frame;
    uint16_t value;
    
    read_frame = (((uint16_t)address|0x80) << 8) | 0x00;
    delay(1);
    // enable accelerometer chip select
    LATGbits.LATG9 = 0;
    // send "read_frame to SPI2 buffer
    SPI2BUF = read_frame;
    // wait for the SPI2 buffer full bit
    while(!SPI2STATbits.SPIRBF){};       //While SPI Receive Buffer is not full
    // read the SPI2 buffer contents with the "value" variable
    value = SPI2BUF;
    // disable accelerometer chip select
    LATGbits.LATG9 = 0;
    
    return value;
}



float accel_read_x(void)
{
   int16_t X_H; 
   int16_t X_L; 
   int16_t X;
   
   X_H = spi2_read_register(OUT_X_H)&0x00FF;
   X_L = spi2_read_register(OUT_X_L)&0x00FF;
   
   // Combine data from both registers
   // See Lab2 manual for instructions
   X = (X_H << 8) | X_L;

   float value = X * 0.000061;            // Convert to units of g
   return value;
}


float accel_read_y(void)
{
   int16_t Y_H; 
   int16_t Y_L; 
   int16_t Y;
   
   Y_H = spi2_read_register(OUT_Y_H)&0x00FF;
   Y_L = spi2_read_register(OUT_Y_L)&0x00FF;
   
   // Combine data from both registers
   // See Lab2 manual for instructions
   Y = (Y_H << 8) | Y_L;
   
   float value = Y * 0.000061;            // Convert to units of g
   return value;
}

float accel_read_z(void)
{
   int16_t Z_H; 
   int16_t Z_L; 
   int16_t Z;
   
   Z_H = spi2_read_register(OUT_Z_H)&0x00FF;
   Z_L = spi2_read_register(OUT_Z_L)&0x00FF;
   
   Z = (Z_H << 8) | Z_L;
   
   float value = Z * 0.000061;            // Convert to units of g
   return value;
}


void accel_setup(void)
{
    spi2_write_register(CTRL_REG1, 0x47);      //See Page 35 of Accelerometer manual
    spi2_write_register(CTRL_REG2, 0x00);
    spi2_write_register(CTRL_REG3, 0x00);
    spi2_write_register(CTRL_REG4, 0x88);
    spi2_write_register(CTRL_REG5, 0x40);
    spi2_write_register(CTRL_REG6, 0x00);
    spi2_write_register(REFERENCE, 0x00);
    spi2_write_register(INT1_THS, 0x00);
    spi2_write_register(INT1_DUR, 0x00);
    spi2_write_register(INT1_CFG, 0x00);
    spi2_write_register(FIFO_CTRL, 0x80);
}


void test_accel(void)
{   
    putchar(' ');
    
    int16_t test = spi2_read_register(0x0F);  // read WHO_AM_I register
    
    if (test == 0x0033)
    {
        printf("Pass");
    }
    else
    {
        printf("Fail");
    }
}
void accel_print_data(char axis)
{
    float data;
    
    if (axis == 'x')
    {
        data = accel_read_x();
    }
    else if (axis == 'y')
    {
        data = accel_read_y();
    }
    else if (axis == 'z')
    {
        data = accel_read_x();
    }
    
    char buffer[50]; 
    sprintf(buffer, "%f", data); 
    printf("%s \n", buffer);
}
void accel_move_cursor(void)
{
    if (accel_read_x() > 0.4)
    {
        putchar(' ');
    }
    else if (accel_read_x() < -0.4)
    {
        putchar(0x8);
    }
    
}


/* *****************************************************************************
 End of File
 */
