#include <math.h>

//Placeholder values, although actual values shouldn't matter as long as the ratios are maintained (currently they are not)
const float BICEP_LENGTH = 1.0;
const float FOREARM_LENGTH = 1.0;
const float WRIST_LENGTH = 1.0;

//float SHOULDER_OFFSET = 1f;  Should offset doesn't matter, move the coordinate system up by this amount and the result will be the same
const float ELBOW_OFFSET = 1.0;

const float ADJUSTED_SHOULDER_PITCH = atan2(ELBOW_OFFSET, BICEP_LENGTH);

/*
 * Calculate angles of the two inner arms using wrist position
 */
float* PosToAngArmsNoWrist(int x, int y, int z){
    //Normalize x, y, z, coordinates 
    float normX = (float)x/100.0;
    float normY = (float)y/100.0;
    float normZ = (float)z/100.0;

    float xy3DReach = hypot(normX, normY); //Needs C++ 11
    float shoulderYaw = atan2(normY, normX);

    float elbowPitch = acos(
        (pow(BICEP_LENGTH, 2) + pow(FOREARM_LENGTH, 2) - pow(xy3DReach, 2))
	/ (2*BICEP_LENGTH*FOREARM_LENGTH));

    float xy2DReach = hypot(xy3DReach, normZ);
    float shoulderPitch = acos(
        (pow(BICEP_LENGTH, 2) + pow(xy2DReach, 2) - pow(FOREARM_LENGTH, 2))
	/ (2*BICEP_LENGTH*xy2DReach)) - atan2(normZ, xy3DReach);

    static float angles[3] = {shoulderYaw,
	    shoulderPitch + ADJUSTED_SHOULDER_PITCH, elbowPitch};
    return angles;
}

/*
 * Calculate angles of arms using tip of wrist position and wrist orientation
 */
float* PosToAngAllArms(int x, int y, int z, float alpha){
    //Normalize x, y, z coordinates
    float normX = (float)x/100.0;
    float normY = (float)y/100.0;
    float normZ = (float)z/100.0;

    float x2DReach = sqrt(pow(x, 2) + pow(y, 2));
    float xy2DReachSquare = pow(x2DReach, 2) + pow(z, 2);
    float xyReachWrist = sqrt(xy2DReachSquare + pow(WRIST_LENGTH, 2)
        - (2*WRIST_LENGTH*(x2DReach*cos(alpha) + normZ*sin(alpha))));

    //Now use same calculations as above with the calculated xyReachWrist
    
    static float angles[3];
    return angles;
}
