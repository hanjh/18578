

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

    fd = open(deviceID, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        printf("failed to open device %s \n", deviceID);
    }
    if (tcgetattr(fd,&toptions) < 0) {
       printf("failed to get term attributes\n");
    }
    
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

int serialWrite()
{
}

int serialClose()
{
}



