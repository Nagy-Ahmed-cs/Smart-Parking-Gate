#include <stdint.h>
#include "tm4c123gh6pm.h"

void delay(void) {
    for(uint32_t i = 0; i < 1000000; i++);
}

int main(void) {
    // Enable clock for Port F
    SYSCTL_RCGCGPIO_R |= 0x20;
    
    // Wait for clock to stabilize
    while((SYSCTL_PRGPIO_R & 0x20) == 0);
    
    // Set PF2 (Blue LED) as output
    GPIO_PORTF_DIR_R |= 0x08;
    
    // Enable digital function on PF2
    GPIO_PORTF_DEN_R |= 0x08;
    
    // Turn Blue LED ON
    GPIO_PORTF_DATA_R |= 0x08;
    
    while(1) {
        // Do nothing - LED stays blue
    }
}