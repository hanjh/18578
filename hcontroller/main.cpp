


#include "optical_flow.h"
#include <unistd.h>


int main(void) {
    opticalFlowInit();
    opticalFlowDemoFrameInit();
    while(1)    
        calculateOpticalFlow(); 
}
