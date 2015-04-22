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
    JS_FindWindow,
    JS_Error
};


int main(void) 
{
    JetsonState jState = JS_Idle;
    struct timespec warmup_start_time;

    // initialize serial connections
    //int fcontroller_fd = initSerialCom((char*)"/dev/fcontroller");
    int fcontroller_fd = initSerialCom((char*)"/dev/ttyUSB0");
    //int arduino_fd = initSerialCom((char*)"/dev/arduino");
    int arduino_fd = initSerialCom((char*)"/dev/ttyACM0");
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
    printf("sizeof(fcontroller_data_t) = %d\n", sizeof(fcontroller_data_t));
    printf("sizeof(commands_t) = %d\n", sizeof(commands_t));

    //opticalFlowDemoFrameInit();
    while(1)  
    {
        printf("entered while loop\n");
        int height, distanceToWindow;
        float xFlowVelocity, yFlowVelocity;
        fcontroller_data_t in_data;
        commands_t out_data;
        struct timespec current_time;

        clock_gettime(CLOCK_REALTIME, &current_time);
        readFController(fcontroller_fd, &in_data);
        printf("in_data.header = %x\n", in_data.header == in_data.rotation[0] ^ in_data.rotation[1]);
        printf("in_data.rotation[0] = %x\n", in_data.rotation[0]);
        printf("in_data.rotation[1] = %x\n", in_data.rotation[1]);
        printf("in_data.rotation[2] = %x\n", in_data.rotation[2]);
        printf("in_data.rVelocity[0] = %x\n", in_data.rVelocity[0]);
        printf("in_data.rVelocity[1] = %x\n", in_data.rVelocity[1]);
        printf("in_data.rVelocity[2] = %x\n", in_data.rVelocity[2]);
        printf("in_data.command = %d\n", in_data.command);
        printf("in_data.footer = %x\n", in_data.footer == (in_data.rVelocity[0] ^ in_data.rVelocity[1]));
        readSonar(arduino_fd, height, distanceToWindow);
        printf("height = %d\n", height);
        printf("distanceToWindow = %d\n", distanceToWindow);
        calculateOpticalFlow(xFlowVelocity, yFlowVelocity); 
        printf("xFlowVelocity = %f\n", xFlowVelocity);
        printf("yFlowVelocity = %f\n", yFlowVelocity);
    
        if (jState == JS_Idle) 
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
        writeFController(fcontroller_fd, &out_data);
    }
}
