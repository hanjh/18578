#ifndef SERIALCOMH
#define SERIALCOMH

#include <inttypes.h>

enum FCCommand : uint16_t
{
    FC_ReceivedGo,
    FC_ReadyToFly,
    FC_SerialError,
    FC_Failure,
    FC_Flying
};

enum JetCommand : uint16_t
{
    JC_Idle,
    JC_WarmUp,
    JC_AltHold,
    JC_ErrorStop
};

struct commands_t {
    uint16_t header;
    JetCommand command; // uint16_t
    //position errors
    int16_t ex;
    int16_t ey;
    int16_t ez;
    //inst. velocities
    int16_t vx;
    int16_t vy;
    int16_t vz;
    //heading
    int16_t heading; //Yaw
    uint16_t footer;
    // 10 * 2 = 20 bytes
    // 24 on fcontroller
};

struct fcontroller_data_t
{
    uint16_t header;
    uint16_t rotation[3];
    uint16_t rVelocity[3];
    FCCommand command; // uint16_t
    uint16_t footer; 
    // 9 * 2 = 18 bytes
    // 24 on fcontroller
};

int initSerialCom(char* deviceID);

void readFController(int fd,fcontroller_data_t* data);
void writeFController(int fd, commands_t* data);

void readSonar(int fd, int& height, int& distanceToGlass);


#endif
