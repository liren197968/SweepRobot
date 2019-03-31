#include "mbed.h"
#include "USBHostGamepad.h"

USBHostGamepad *pad;

// LED
DigitalOut red(PB_5);
DigitalOut yellow(PA_10);

int main()
{    
    // USB Gmaepad Initialize
    pad = new USBHostGamepad();
    if (!pad->connect()) {
        printf("USB Gamepad not found.\r\n");
        while(1);
    }
    
    while(1)
    {
        USBHost::poll();
        
        red = pad->report[4] & 0x20;
        yellow = pad->report[4] & 0x40;
        
        printf("%02x %02x %02x %02x %02x %02x %02x %02x\r\n", pad->report[0],pad->report[1],pad->report[2],pad->report[3],pad->report[4],pad->report[5],pad->report[6],pad->report[7]);
        wait_ms(16);
    }
}
