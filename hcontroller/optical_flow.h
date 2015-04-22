#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

int opticalFlowInit(void);
int opticalFlowDemoFrameInit(void);
int calculateOpticalFlow(float& xFlowVelocity, float& yFlowVelocity);

#endif
