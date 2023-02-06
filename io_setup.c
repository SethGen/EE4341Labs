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
//        accel_print_data('x');
        delay(50);            //delay for debouncing when button is pressed
    }
    else
    {
        button_off(1);
    }
    
    if (!BUTTON2)
    {
        
        button_on(2);
//        accel_print_data('y');
        delay(50);
    }
    else
    {
        button_off(2);
    }
    
    if (!BUTTON3)
    {
        
        button_on(3);
//        accel_move_cursor();
        delay(50);
    }
    else
    {
        button_off(3);
    } 
}
/* *****************************************************************************
 End of File
 */

