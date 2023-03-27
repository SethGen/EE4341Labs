/* CAN code Loopback Test */
/* data led on/off bit is transmitted via CAN */

/* CAN is done "bare metal" */
/* this routine uses a dumb counter */

#include <xc.h>
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#include <sys/kmem.h>

// Configuration bit settings
#pragma config FNOSC = PRIPLL, POSCMOD = XT
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_8, FWDTEN = OFF


// counter delay
#define DELAY 320000

// CAN definitions
#define MY_SID 0x123 //11bit CAN frame ID
#define FIFO_0_SIZE 4 // size of FIFO 0 (RX), in number of message buffers
#define FIFO_1_SIZE 2 // size of FIFO 1 (TX), in number of message buffers
#define MB_SIZE 4 //number of 4byte integers in a message buffer


// buffer for CAN FIFOs
static volatile unsigned int fifos[(FIFO_0_SIZE + FIFO_1_SIZE) * MB_SIZE];

void CAN_setup(void)
{       
    
    C1CONbits.ON = 1;	// turn on the CAN module
    
    C1CONbits.REQOP = 4;	// request configure mode
    
    while(C1CONbits.OPMOD != 4);	// wait to enter config mode
    
    C1FIFOCON0bits.FSIZE = FIFO_0_SIZE - 1; // set fifo 0 size
    
    C1FIFOCON0bits.TXEN = 0; // fifo 0 is an RX fifo
    
    C1FIFOCON1bits.FSIZE = FIFO_1_SIZE - 1; // set fifo 1 size
    C1FIFOCON1bits.TXEN = 1; // fifo 1 is a TX fifo
    
    C1FIFOBA = KVA_TO_PA(fifos); // fifo locations in SRAM

    C1RXM0bits.SID = 0x7FF; // mask 0 to match all SID bits
    
    C1FLTCON0bits.FSEL0 = 0; // filter 0 is for FIFO 0
    
    C1FLTCON0bits.MSEL0 = 0; // filter 0 uses mask 0
    
    C1RXF0bits.SID = 0x123; // filter 0 matches against SID 0x123
    
    C1FLTCON0bits.FLTEN0 = 1; // enable filter 0

    
    C1CONbits.REQOP = 2;	// request loopback mode
    
    while(C1CONbits.OPMOD != 2);	// wait for loopback mode
}

int main() { 

    // set up CAN peripheral in external function    
    CAN_setup();
    
    // set D0, D1, and D2 as outputs for the LEDs
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 0;
    
    int count;
    int to_send = 0xA5; //This can be any number really
    while(1) {
        
        /* First, you'll want to add the SID, number of bytes, and the 
         * data itself into the TX FIFO buffer: we do this by finding its
         * virtual address, then writing this information to that address */
         
        int* addr;
        addr = PA_TO_KVA1(C1FIFOUA1); // get FIFO 1 (the TX fifo) current message address
        addr[0] = MY_SID; // only the sid must be set for this example
        addr[1] = sizeof(to_send); // only DLC field must be set, we indicate 4 bytes
        addr[2] = to_send; // 4 bytes of actual data

        
        
        /* Tell the CAN peripheral to move onto the next entry in the TX FIFO, 
         * and that you want to send the current TX FIFO data */
        C1FIFOCON1SET = 0x2000; // setting UINC bit tells fifo to increment pointer
        C1FIFOCON1bits.TXREQ = 1; // request that data from the queue be sent

        
        /* Poll the correct flag to know when you have received data from the
         * buffer, then read data from there and move onto the next entry in the
         * RX FIFO */
        
        while(!C1FIFOINT0bits.RXNEMPTYIF);  // wait to receive data
        addr = PA_TO_KVA1(C1FIFOUA0);      // get the VA of current pointer to the RX FIFO
        C1FIFOCON0SET = 0x2000;           // setting UINC bit tells fifo to increment pointer

        
        /* Wait for some time (otherwise the LED blinking will be too fast),
         * then invert  */
        for(count=0; count < DELAY; count++); // wait
        to_send = ~to_send; // toggle led value
        LATDbits.LATD2 = to_send;
        
    }
}
