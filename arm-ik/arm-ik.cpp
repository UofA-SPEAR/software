//Placeholder values, although actual values shouldn't matter as long as the ratios are maintained (currently they are not)
float BICEP_LENGTH = 1f;
float FOREARM_LENGTH = 1f;
float WRIST_LENGTH = 1f;

/*
 * Given the position of the wrist, calculate the motor angles of the two arms
 */
float[] PosToAngArmsNoWrist(int x, int y, int z){
    //float END_DISTANCE_2D = hypot(x, y); //Needs C++ 11
    float shoulderYaw = atan2(y, x);
    float elbowPitch = acos(( pow(BICEP_LENGTH, 2) + pow(FOREARM_LENGTH, 2) - pow(x, 2) - pow(y, 2) ) / (2*BICEP_LENGTH*FOREARM_LENGTH)) 
}
