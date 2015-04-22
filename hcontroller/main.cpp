#include "serial_com.h"
#include "optical_flow.h"
#include <unistd.h>
#include <stdio.h>
#include <time.h>

#define WARMUP_TIME 4
enum JetsonState
{
    JS_Idle,
    JS_WarmUp,
    JS_AltHold,
    JS_FindWindow
    JS_Error
};


int main(void) 
{
    JetsonState jState = JS_Idle;
    struct timespec warmup_start_time;

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
        float xFlowVelocity, yFlowVelocity;
        fcontroller_data_t in_data;
        commands_t out_data;
        struct timespec current_time;

        clock_gettime(CLOCK_REALTIME, &current_time);
        readFController(fcontroller_fd, &in_data);
        readSonar(arduino_fd, height, distanceToWindow);
        calculateOpticalFlow(xFlowVelocity, yFlowVelocity); 

        if (jState == Idle) 
        {
                out_data.ex = 0xFFFE;
                out_data.ey = 0xFDFC;
                out_data.ez = 0xFBFA;
                out_data.vx = 0xF9F8;
                out_data.vy = 0xF7F6;
                out_data.vz = 0xF5F4;
                out_data.heading = 0xF3F2;
            
            if (in_data.command == FC_ReceivedGo)
            {
                out_data.command - JC_WarmUp;
            }
            else if (in_data.command == FC_ReadyToFly)
            {
                jState = JS_WarmUp;
                clock_gettime(CLOCK_REALTIME, &warmup_start_time);
            }

        }
        else if (jState == JS_WarmUp)
        {
            out_data.command = JC_WarmUp;
            if (current_time.tv_sec - warmup_start_time.tv_sec > WARMUP_TIME) 
            {
                jState == JS_AltHold;
            }
        }
        else if (jState == JS_AltHold)
        {
            out_data.command = JC_AltHold;
            

        }


        out_data.header = out_data.ex ^ out_data.ey;
        out_data.footer = out_data.vx ^ out_data.vy;
        writeFController(fcontroller_fd, out_data);
    }

}
