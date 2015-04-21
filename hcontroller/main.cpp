#include "serial_com.h"
#include "optical_flow.h"
#include <unistd.h>
#include <stdio.h>
#include <time.h>

int main(void) 
{
    // initialize serial connections
    int fcontroller_fd = initSerialCom((char*)"/dev/fcontroller");
    int arduino_fd = initSerialCom((char*)"/dev/arduino");
    if(fcontroller_fd < 0)
    {
        printf("Failed to open fcontroller\n");
        return -1;
    }
    if(arduino_fd < 0)
    {
        printf("Failed to open arduino\n");
        return -1;
    }

    int of_status = opticalFlowInit(); // uses /dev/video0
    if (of_status < 0)
    {
        printf("optical flow failed\n");
        return -1;
    }

    //opticalFlowDemoFrameInit();
    while(1)  
    {
        int height, distanceToWindow;
        fcontroller_data_t in_data;
        commands_t out_data;

        readFController(fcontroller_fd, &in_data);
        readSonar(arduino_fd, height, distanceToWindow);

        calculateOpticalFlow(); 
    }

}
