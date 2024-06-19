#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pico/stdio.h"
// #include "encoder.pio.h"


int main()
{
    stdio_init_all();
    while (true) {
        static int i = 0;
        printf("%d\n", i++);
        sleep_ms(100);
    }
}
