
#ifndef ARM_PARAMS
#define ARM_PARAMS

//Placeholder values, although actual values shouldn't matter as long as the ratios are maintained (currently they are not)
const float BICEP_LENGTH = 1.0;
const float FOREARM_LENGTH = 1.0;
const float WRIST_LENGTH = 1.0;

//float SHOULDER_OFFSET = 1.0;  Should offset doesn't matter; move the coordinate system up by this amount and the result will be the same
const float ELBOW_OFFSET = 1.0;

const float ADJUSTED_SHOULDER_PITCH = atan2(ELBOW_OFFSET, BICEP_LENGTH);

#endif
