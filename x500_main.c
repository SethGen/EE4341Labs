/* ************************************************************************** */
/** x500_main.c
 *  EE 4341 Lab 1: RS-232
 *  Seth Gentry
 */
/* ************************************************************************** */
#include "io_setup.h"
#include "accel_define.h"




void setup(void)
{
    io_setup();
    uart1_setup();
    spi2_setup();
    // include any other setup functions here
}
int main(void)
{
    setup();
    while(1)
    { 
        delay(1);
        buttons();
    }
    return 0;
}
/* *****************************************************************************
 End of File
 */
