/* ************************************************************************** */
/** x500_main.c
 *  EE 4341 Lab 0: Setup
 *  Seth Gentry
 */
/* ************************************************************************** */
#include "io_setup.h"
void setup(void)
{
    io_setup();
    uart1_setup();
    // include any other setup functions here
}
int main(void)
{
    setup();
    
    delay(100);                  // 100 ms delay as precaution
    
    while(1)
    { 
        buttons();
    }
    return 0;
}
/* *****************************************************************************
 End of File
 */
