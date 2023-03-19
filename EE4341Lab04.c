/*
** SDMMC.c SD card interface
*/
#include <xc.h>

// I/O definitions
// NOTE: Do not use the pins given
// by the MassStorage.pdf file. To find the 
// correct pinout, use the SDCardBoard and
// IO Expansion Board Information Sheet PDF's
// to map the pins to the top-right slot on the
// expansion board.
#define SDWP _RF1 // Write Protect input
#define SDCD _RF0 // Card Detect input
#define SDCS _RB1 // Card Select output


// Macros from reference file go here
#define readSPI() writeSPI( 0xFF)
#define clockSPI() writeSPI( 0xFF)

#define disableSD() SDCS = 1; clockSPI()
#define enableSD() SDCS = 0

typedef unsigned LBA; // logic block address, 32 bit wide


// SD card commands from reference file go here
#define RESET 0 // a.k.a. GO_IDLE (CMD0)
#define INIT 1 // a.k.a. SEND_OP_COND (CMD1)
#define READ_SINGLE 17
#define WRITE_SINGLE 24
#define DATA_START 0xFE
#define DATA_ACCEPT 0x05


#define I_TIMEOUT 10000
#define R_TIMEOUT 25000
#define W_TIMEOUT 250000



void delay(int ms){
    ms = 2*ms;
    while(ms-- > 0){
        asm(".rept 19994");   //1 cycle = 25ns ; 1ms = 40,000 cycles
        asm("NOP");
        asm(".endr");   
    }
}

void initSD(void){
    SDCS = 1; // initially keep the SD card disabled
    _TRISB1 = 0; // make Card select an output pin
    // init the SPI2 module for a slow (safe) clock speed first
    SPI1CON = 0x8120; // ON, CKE=1; CKP=0, sample middle
    SPI1BRG = 63; // Baud Rate = 312.5kHz
} // initSD

unsigned char writeSPI(unsigned char byte){  
    SPI1BUF= byte; // write to buffer for TX
    while( !SPI1STATbits.SPIRBF); // wait transfer complete
    return SPI1BUF; // read the received value
}// write a byte using SPI

int sendSDCmd(unsigned char c, unsigned a){
    // c command code
    // a byte address of data block
    int i, r;
    // enable SD card
    enableSD();
    // send a 6-byte command packet over SPI
    writeSPI( c | 0x40); // send command
    writeSPI( a>>24); // msb of the address
    writeSPI( a>>16);
    writeSPI( a>>8);
    writeSPI( a); // lsb
    writeSPI( 0x95); // send CMD0 CRC
    //6 Bytes ^^^
    // read 8 bytes over SPI
    // if any byte is not 0xFF, return that byte
    // now wait for a response, allow for up to 8 bytes delay
    for( i=0; i<8; i++)
        {
            r=readSPI();
            if ( r != 0xFF)
            break;
        }
    return r;
    // NOTE CSCD is still low!


} // sendSDCmd

int initMedia(void){
// returns 0 if successful
// E_COMMAND_ACK failed to acknowledge reset command
// E_INIT_TIMEOUT failed to initialize
    int i, r;
    // 1. with the card NOT selected
    disableSD();
    // 2. send 80 clock cycles start up
    for ( i=0; i<10; i++)
        clockSPI();
    // 3. now select the card
    enableSD();
    // 4. send a single RESET command
    r = sendSDCmd( RESET, 0); disableSD();
    if ( r != 1) // must return Idle
        return E_COMMAND_ACK; // comand rejected
    // 5. send repeatedly INIT until Idle terminates
    for (i=0; i<I_TIMEOUT; i++)
    {
        r = sendSDCmd( INIT, 0); disableSD();
        if ( !r)
        break;
    }
    if ( i == I_TIMEOUT)
        return E_INIT_TIMEOUT; // init timed out
    
    // 6. increase speed: disable SPI first, change settings and re-enable
    SPI1CON = 0; // disable the SPI2 module
    delay(1);
    SPI1BRG = 0; // Fpb/(2*(0+1))= 40/2 = 20 MHz
    SPI1CON = 0x8120; // re-enable the SPI2 module
    return 0;
} // init media

int readSECTOR( LBA a, char *p){

    // a LBA of sector requested
    // p pointer to sector buffer
    // returns TRUE if successful

    int r, i;

    // 1. send READ command
    r = sendSDCmd( READ_SINGLE, ( a << 9));
    if ( r == 0) // check if command was accepted
    {
    // 2. wait for a response
        for( i=0; i<R_TIMEOUT; i++)
        {
            r = readSPI();
            if ( r == DATA_START)
            break;
        }
    // 3. if it did not timeout, read 512 byte of data
        if ( i != R_TIMEOUT)
        {
            i = 512;
            do{
            *p++ = readSPI();
            } while (--i>0);
// 4. ignore CRC
        readSPI();
        readSPI();
        } // data arrived
    } // command accepted
// 5. remember to disable the card
    disableSD();
    return ( r == DATA_START); // return TRUE if successful
} // readSECTOR

int writeSECTOR(LBA a, char *p){

    // a LBA of sector requested
    // p pointer to sector buffer
    // returns TRUE if successful, FAIL if not

    unsigned r, i;

    int FAIL = 5; // arbitrary number
    if ( getWP())
        return FAIL;
    // 1. send WRITE command
    r = sendSDCmd( WRITE_SINGLE, ( a << 9));
    if ( r == 0) // check if command was accepted
    {
    // 2. send data
        writeSPI( DATA_START);
    // send 512 bytes of data
        for( i=0; i<512; i++)
            writeSPI( *p++);
    // 3. send dummy CRC
        clockSPI();
        clockSPI();
    // 4. check if data accepted
        r = readSPI();
        if ( (r & 0xf) == DATA_ACCEPT)
        {
    // 5. wait for write completion
            for( i=0; i<W_TIMEOUT; i++)
            {
                r = readSPI();
                if ( r != 0 )
                    break;
            }
        } // accepted
        else
            r = FAIL;
    } // command accepted
    // 6. remember to disable the card
    disableSD();
    return (r); // return TRUE if successful
}  // writeSECTOR

// SD card connector presence detection switch (1 line of code)
int getCD(void){
    // returns TRUE card present
    // FALSE card not present
    return !SDCD;
}

// card Write Protect tab detection switch (1 line of code)
int getWP(void){
    // returns TRUE write protect tab on LOCK
    // FALSE write protection tab OPEN
    return SDWP;
}

// The function below is used in the other .c file
void initData(void){
    int idx, b = 0;
    for (idx = 0; idx < B_SIZE; idx++) {
        data[idx] = b;
        b ^= 1;
    }
}


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
    CFGCONbits.IOLOCK = 1;          // lock   PPS
    system_reg_lock;
    _TRISF0 = 1; // make Card Detect an input pin
    _TRISF1 = 1; // make Write Protect Detect an input pin
}



