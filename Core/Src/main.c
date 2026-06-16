#include "stm32h743xx.h"

volatile uint32_t counter = 0;

int main(void)
{
    // Simple boucle de vie
    while (1)
    {
        counter++;
    }
}