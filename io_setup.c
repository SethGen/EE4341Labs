/* ************************************************************************** */
/** io_setup.c
  @Description
    Has all pragma statements, PPS functions, and IO setup procedures.
 
 */
/* ************************************************************************** */
// DEVCFG3
#pragma config FSRSSEL = PRIORITY_7    // Shadow Register Set Priority Select->SRS :Priority 7
#pragma config PMDL1WAY = ON           // Peripheral Module Disable Configuration-Allow only one reconfiguration
#pragma config IOL1WAY = ON            // Peripheral Pin Select Configuration-Allow only one reconfiguration
#pragma config FUSBIDIO = ON           // USB USID Selection->Controlled by the USB Module
#pragma config FVBUSONIO = ON          // USB VBUS ON Selection->Controlled by USB Module
// DEVCFG2
#pragma config FPLLIDIV = DIV_2        // PLL Input Divider->2x Divider
#pragma config FPLLMUL = MUL_20        // PLL Multiplier->20x Multiplier
#pragma config UPLLIDIV = DIV_12       // USB PLL Input Divider->12x Divider
#pragma config UPLLEN = OFF            // USB PLL Enable->Disabled and Bypassed
#pragma config FPLLODIV = DIV_1        // System PLL Output Clock Divider->PLL Divide by 1
// DEVCFG1
#pragma config FNOSC = FRCPLL          // Oscillator Selection Bits->Fast RC Osc with PLL
#pragma config FSOSCEN = ON            // Secondary Oscillator Enable->Enabled
#pragma config IESO = ON               // Internal/External Switch Over->Enabled
#pragma config POSCMOD = OFF           // Primary Oscillator Configuration->Primary osc disabled
#pragma config OSCIOFNC = OFF          // CLKO Output Signal Active on the OSCO Pin->Disabled
#pragma config FPBDIV = DIV_2          // Peripheral Clock Divisor->Pb_Clk is Sys_Clk/2
#pragma config FCKSM = CSDCMD          // Clock Switching and Monitor Selection-Clock Switch Disable, FSCM Disabled
#pragma config WDTPS = PS1048576       // Watchdog Timer Postscaler->1:1048576
#pragma config WINDIS = OFF            // Watchdog Timer Window Enable->Watchdog Timer is in Non-Window Mode
#pragma config FWDTEN = ON             // Watchdog Timer Enable->WDT Enabled
#pragma config FWDTWINSZ = WINSZ_25    // Watchdog Timer Window Size->Window Size is 25%
// DEVCFG0
#pragma config DEBUG = OFF             // Background Debugger Enable->Debugger is Disabled
#pragma config JTAGEN = ON             // JTAG Enable->JTAG Port Enabled
#pragma config ICESEL = ICS_PGx2       // ICE/ICD Comm Channel Select->Communicate on PGEC2/PGED2
#pragma config PWP = OFF               // Program Flash Write Protect->Disable
#pragma config BWP = OFF               // Boot Flash Write Protect bit->Protection Disabled
#pragma config CP = OFF                // Code Protect->Protection Disabled
// SYSCLK = 80MHz, PBCLK = 40MHz //
#include "io_setup.h"
#include "accel_define.h"
void system_reg_unlock(void)
{
    SYSKEY = 0x12345678;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
}
void system_reg_lock(void)
{
    SYSKEY = 0x00000000; 
}

void uart1_setup(void)      //Sets up UART1 with a baud rate of 9600 bit/sec
{   
    // BRGH OFF; UEN TX_RX;
    U1MODEbits.BRGH = 0;          //Standard Speed mode ? 16x baud clock enabled
    U1MODEbits.UEN = 0;       //0b10 = UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
    // BaudRate = 9600; Frequency = 40000000 Hz; BRG 259;
    U1BRG = 259;
    // Enable transmit
    U1STAbits.UTXEN = 1;         //Enable UART1 Transmit
    // Enable receive
    U1STAbits.URXEN = 1;         //Enable UART1 Receive
    // Enable UART (ON bit)
    U1MODEbits.ON = 1;
    __XC_UART = 1;              // Code is configured to use UART1 for printf()
}

void io_setup(void)
{
// Below are the defaults , you will have to edit to make changes
// Add a comment for each edit you make, so you don't forget
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0000;
    LATD = 0x0000;
    LATE = 0x0000;
    LATF = 0x0000;
    LATG = 0x0000;
    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0xC6FF;     
    TRISB = 0xFFFF;
    TRISC = 0xF01E;
    TRISD = 0xFFF8;     
    TRISE = 0x03FF;
    TRISF = 0x313F;     
    TRISG = 0xF3C3;
    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPDA = 0x0000;
    CNPDB = 0x0000;
    CNPDC = 0x0000;
    CNPDD = 0x0000;
    CNPDE = 0x0000;
    CNPDF = 0x0000;
    CNPDG = 0x0000;
    CNPUA = 0x0000;
    CNPUB = 0x0000;
    CNPUC = 0x0000;
    CNPUD = 0x20C0;     //RD6, RD7 and RD13 Pulled Up 
    CNPUE = 0x0000;
    CNPUF = 0x0000;
    CNPUG = 0x0000;
    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;
    ODCD = 0x0000;
    ODCE = 0x0000;
    ODCF = 0x0000;
    ODCG = 0x0000;
    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSELA = 0x0000;
    ANSELB = 0x0000;
    ANSELD = 0x0000;
    ANSELE = 0x0000;
    ANSELG = 0x0000;
    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    system_reg_unlock();            // unlock PPS
    CFGCONbits.IOLOCK = 0;
    //UART1 Setup
    U1RXRbits.U1RXR = 0b0010;       //UART RX = RPF4(Pin 49) Values given in PIC32 Family Reference Manual
    RPF5Rbits.RPF5R = 0b0011;       //UART TX = RPF5(Pin 50)
    //SPI2 Setup
    SDI2Rbits.SDI2R = 0b0001;       //SDI2 = RPG7 (Pin 11)    Page 142
    RPG8Rbits.RPG8R = 0b0110;       //SD02 =  RPG8 (Pin 12)
//    SS2Rbits.SS2R = 0b0001;         //SS2 = RPG9 (Pin 14))
//    SPI2CONbits.SSEN = 1;
    CFGCONbits.IOLOCK = 1;          // lock   PPS
    system_reg_lock; 
}

void delay(int ms){
    ms = 2*ms;
    while(ms-- > 0){
        asm(".rept 19994");   //1 cycle = 25ns ; 1ms = 40,000 cycles
        asm("NOP");
        asm(".endr");   
    }
}

void button_on(int x)
{
    if (x == 1)
    {
        LED1 = 1;
// whatever else you want to do if button 1 is pressed
    }
    else if (x == 2)
    {
        LED2 = 1;
    }
    else if (x == 3)
    {
        LED3 = 1;
    }
}
void button_off(int x)
{
    if (x == 1)
    {
        LED1 = 0;
    }
    else if (x == 2)
    {
        LED2 = 0;
    }
    else if (x == 3)
    {
        LED3 = 0;
    }
}
void buttons(void)
{
    // Include code for debouncing every time a button is pressed
    // BUTTONs are pulled-up, so poll for low state
    
    if (!BUTTON1)
    {
        button_on(1);
        printf("\r x data: ");
        accel_print_data('x');
        delay(50);            //delay for debouncing when button is pressed
    }
    else
    {
        button_off(1);
    }
    
    if (!BUTTON2)
    {
        
        printf("\r y data: ");
        accel_print_data('y');
        delay(50);
    }
    else
    {
        button_off(2);
    }
    
    if (!BUTTON3)
    {
        
        button_on(3);
        accel_move_cursor();
        delay(50);
    }
    else
    {
        button_off(3);
    } 
}
/* *****************************************************************************
 accel_define functions
 */

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
    LATGbits.LATG9 = 1;
    
    return value;
}



float accel_read_x(void)
{
   int16_t X_H; 
   int16_t X_L; 
   int16_t X;
   
   X_H = spi2_read_register(OUT_X_H)&0x00FF;
   X_L = spi2_read_register(OUT_X_L)&0x00FF;
   float hPrint = (float)X_H;
   float lPrint = (float)X_L;
//   printf("High Byte(X): %f", hPrint);
//   printf("\n");
//   printf("Low Byte(X): %f ",lPrint);
//   printf("\n");
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
   float hPrint = (float)Y_H;
   float lPrint = (float)Y_L;
//   printf("High Byte(Y): %f", hPrint);
//   printf("\n");
//   printf("Low Byte(Y): %f ",lPrint);
//   printf("\n");
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
   
   Z_H = spi2_read_register(OUT_Z_H);
   Z_L = spi2_read_register(OUT_Z_L);
   
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
//    if (accel_read_x() > 0.4)
//    {
//        putchar(' ');
//    }
//    else if (accel_read_x() < -0.4)
//    {
//        putchar(0x8);
//    }
    putchar(0x8);
    
}


/* *****************************************************************************
 End of File
 */


