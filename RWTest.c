/*
** RWTest.c
**
*/
// configuration bit settings, Fcy=72 MHz, Fpb=36 MHz
#pragma config POSCMOD=XT, FNOSC=PRIPLL
#pragma config FPLLIDIV=DIV_2, FPLLMUL=MUL_18, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_2, FWDTEN=OFF, CP=OFF, BWP=OFF
#include "SDMMC.h"
#include <stdio.h>
#include <string.h>
#include <xc.h>
#define START_ADDRESS 0 // start block address
#define N_BLOCKS 1 // number of blocks 
#define LED3 _RD2  // visual feedback about SD usage status - fail
#define LED2 _RD1  // visual feedback about SD usage status - pass
 void delay(int ms){
    ms = 2*ms;
    while(ms-- > 0){
        asm(".rept 19994");   //1 cycle = 25ns ; 1ms = 40,000 cycles
        asm("NOP");
        asm(".endr");   
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
 
 
 
main(void){
    
          // code to unlock PPS
    system_reg_unlock();
    CFGCONbits.IOLOCK = 0;        
    
    SDI1Rbits.SDI1R = 0b1010;  // map RC4 to SDI1
   
    RPD9Rbits.RPD9R = 0b0111; // map SS1 to RD9
    RPD0Rbits.RPD0R = 0b1000; // map SDO1 to RD0
     // code to lock PPS
    CFGCONbits.IOLOCK = 1;          // lock   PPS
    system_reg_lock(); 
    // set RD1 and RD2 as outputs:
    TRISDbits.TRISD1 = 0;
    TRISDbits.TRISD2 = 0;
    TRISB = 0x0000;
    
    LED3 = 0;
    LED2 = 0;
    
    LBA addr;
    int i, j, r;

    // 1. initialize data
    initData();
    // 2. initialize SD/MMC module
    initSD();
    // 3. fill the buffer with pattern
    for( i=0; i<B_SIZE; i++)
        data[i]= i;

    // 4. wait for the card to be inserted
    printf( " Insert card.... \n \r" );
    while(!getCD()); // check CD switch
    delay(200); // wait contacts de-bounce
    if ( initMedia()) // init card
        { // if error code returned
        printf("Failed Init  \n \r" );
        goto End;
        }
    // 5. fill 16 groups of N_BLOCK sectors with data
    printf("Writing \n \r" );
    addr = START_ADDRESS;
    for( j=0; j<16; j++)
    {
        for( i=0; i<N_BLOCKS; i++)
        {
            if (!writeSECTOR( addr+i*j, data))
            { // writing failed
                printf("Failed to Write \n \r" );
                goto End;
            }
        } // i
    } // j 
    // 6. verify the contents of each sector written
    printf("\n \n \n");
    printf(" Verifying \n \r" );
    addr = START_ADDRESS;
    for( j=0; j<16; j++)
    {
        for( i=0; i<N_BLOCKS; i++)
        { // read back one block at a time
            if (!readSECTOR( addr+i*j, buffer))
            { // reading failed
                printf("Failed to Read \n \r" );
                goto End;
            }
    // verify each block content
            if ( memcmp( data, buffer, B_SIZE))
            { // mismatch
                printf("Failed to Match \n \r" );
                goto End;
            }
        } // i
    } // j
    // 7. indicate successful execution
    LED3 = 1;
    printf("Success! \n \r" );
    
    // main loop
    while(1);
    // If this is reached --> failure   
    End:
    
    LED2 = 1;
    
    
    //Week 2 Demo:

    
    while(!getWP());  //Wait for Write Protection to open
    
    
    // main loop
    while(1);
}


        //Week 2 Demo:
//    char wdata[B_SIZE] = {'W', 'B', '3', 'R', 'L', 'E', 'Q', 'D', '3', 'G', '0', 'Y', 'Y', 'L', 'B', 'X', 'S', 'R', '8', 'K', 'O', '3',
//    'T', 'G', '2', 'P', 'V', '8', 'N', 'M', 'M', 'X', '4', '7', '2', 'U', '0', '7', 'X', '9', '2', 'L', 'T', 'T', 'G', 'G',
//    'J', '2', 'I', 'W', 'Z', '8', '2', '9', '6', '8', '0', '0', 'C', '6', 'T', 'Z', 'O', '4', 'K', 'Y', '9', 'M', 'C', 'F',
//    'Z', 'B', '2', 'W', 'L', 'C', 'A', 'R', 'G', 'M', 'A', 'P', '1', 'P', 'T', 'I', '0', 'X', 'Z', 'N', 'W', 'V', 'S', 'S',
//    '7', 'B', 'E', 'I', '7', 'E', 'J', 'G', 'Y', 'X', '6', 'M', 'B', 'M', 'Y', '3', 'V', 'A', 'Y', 'V', 'J', 'N', 'R', '1',
//    'Y', '1', '6', 'X', 'Q', '8', 'E', '4', 'W', 'I', 'B', '5', 'R', 'J', 'N', '2', 'X', 'I', 'U', 'B', '2', 'H', 'O', 'Z',
//    '3', 'W', 'F', 'M', 'U', '2', 'I', 'B', 'G', 'C', 'D', 'E', '3', '0', 'C', '0', 'Y', 'D', '9', 'W', 'U', 'T', 'A', 'D',
//    'U', 'A', 'Q', 'Z', 'Z', 'H', 'J', 'R', 'T', 'Z', 'U', '7', '1', 'H', 'A', 'E', 'L', '4', '0', 'G', 'M', 'S', 'N', 'T',
//    '1', 'S', 'N', 'V', 'Z', 'I', 'C', 'X', 'I', 'Q', 'R', '1', 'N', 'E', 'I', 'J', 'B', 'B', 'P', 'V', 'A', 'J', 'J', 'J',
//    'F', 'Q', 'G', 'F', 'O', 'P', 'L', 'W', 'G', 'J', 'S', 'M', 'O', 'G', 'Y', '7', 'T', 'S', 'N', 'H', '6', '5', 'X', '3',
//    'Q', 'N', 'L', '0', '8', 'X', 'O', 'C', 'Z', 'R', 'D', 'M'};
