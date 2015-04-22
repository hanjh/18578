#include "serial_com.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>

#if defined(DEVICE_ID) || defined(BAUD_RATE) 
    #error
#endif

//#define DEVICE_ID "/dev/fcontroller"
#define BAUD_RATE B115200
int initSerialCom(char* deviceID) 
{
    struct termios toptions;
    int fd;

    fd = open(deviceID, O_RDWR | O_NONBLOCK);// | O_SYNC);
    if (fd == -1) 
        printf("failed to open device %s \n", deviceID);
    if (tcgetattr(fd,&toptions) < 0) 
        printf("failed to get term attributes\n");
    
    //Set baud rate
    cfsetispeed(&toptions,BAUD_RATE);
    cfsetospeed(&toptions,BAUD_RATE);
    
    //A bunch of random things (who knows)
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    toptions.c_cflag &= ~CRTSCTS; // no flow control
    toptions.c_cflag |= CREAD | CLOCAL; // turn on READ and ignore control lines
    
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    toptions.c_oflag &= ~OPOST;

    toptions.c_cc[VMIN] = 0;
    toptions.c_cc[VTIME] = 0;
    
    //Set the bunch of random things as attributes
    tcsetattr(fd,TCSANOW,&toptions);
    if (tcsetattr(fd,TCSAFLUSH,&toptions) < 0) {
        printf("couldn't set term attributes\n");
    }
    return fd;
}

/*
int serialRead(int fd)
{
    int n;
    char inbuf[256];
    while(1) {
        //printf("looped\n");
        fgets(inbuf, 256,stdin); 
        //const char* str = "test\n";
        int num_w = write(fd,inbuf,strlen(inbuf));
        printf("%d\n",num_w);
        //sleep(2);
        //tcflush(fd,TCIOFLUSH);
        char buf[256];
        do {
            //char b[1];
            n = read(fd,buf,256);
            if (n > 0){
                buf[n] = '\0';
                printf("%s",buf);
            }
        } while (n != 0);
    }
    close(fd);
    return 0;
}
*/

void readFController(int fd, fcontroller_data_t* data)
{
    // fcontroller_data_t is 18 bytes
    //char temp_buffer[18];
    while(1)
    {
        tcflush(fd, TCIFLUSH);
        int n = 0;
        int i = 0;
        do
        {
            i += read(fd, ((char*)data) + i, sizeof(fcontroller_data_t) - i);
        } while(i < sizeof(fcontroller_data_t));

        // check that the data is okay
        if ((data->header == (data->rotation[0] ^ data->rotation[1])) && 
            (data->footer == (data->rVelocity[0] ^ data->rVelocity[1])))
        {   
            // data is okay, return 0;
            //tcflush(fd, TCIFLUSH);
            return;
        }
    }
}

void writeFController(int fd, commands_t* data)
{
    // commands_t is 20 bytes
    int n = 0;
    int i = 0;
    for(i = 0; i < sizeof(commands_t); i += n)
    {
        n = write(fd, ((char*) data) + i, sizeof(commands_t) - i);
    }
    //fflush(fd);
}

void readSonar(int fd, int& height, int& distanceToGlass)
{
    int read_values[2];
    tcflush(fd, TCIFLUSH);
    int i = 0;
    do
    {
        i += read(fd, ((char*)&read_values) + i, 8 - i);
    } while(i < 8);

    /*
    for(i = 0; i < 4; i += n)
    {
        n = read(fd, ((char*)read_values) + i, 4 - i);
    }
    */
    height = read_values[0];
    distanceToGlass = read_values[1];
    //tcflush(fd, TCIFLUSH);
}


/*
int serialWrite()
{
}

int serialClose()
{
}

*/




