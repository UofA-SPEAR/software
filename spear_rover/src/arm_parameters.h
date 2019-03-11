
#ifndef ARM_PARAMS
#define ARM_PARAMS

//Placeholder values, although actual values shouldn't matter as long as the ratios are maintained (currently they are not)
const float BICEP_LENGTH = 0.37;
const float FOREARM_LENGTH = 0.17;
const float WRIST_LENGTH = 0.27;

//float SHOULDER_OFFSET = 1.0;  Should offset doesn't matter; move the coordinate system up by this amount and the result will be the same
const float ELBOW_OFFSET = 1.0;

const float ADJUSTED_SHOULDER_PITCH = atan2(ELBOW_OFFSET, BICEP_LENGTH);

//Constants for arm visualizer
const float BICEP_WIDTH = 0.05;
const float BICEP_HEIGHT = 0.05;
const float FOREARM_WIDTH = 0.05;
const float FOREARM_HEIGHT = 0.05;
const float WRIST_WIDTH = 0.05;
const float WRIST_HEIGHT = 0.05;

#endif
