#include <math.h>

#include <ros/ros.h>
#include <arm_controls/arm_position.h>
#include <rover2/ArmAngles.h>
#include <rover2/JointAngle.h>

ros::NodeHandle* node;
ros::Subscriber arm_coords_sub;
ros::Publisher arm_angles_pub;

//Placeholder values, although actual values shouldn't matter as long as the ratios are maintained (currently they are not)
const float BICEP_LENGTH = 1.0;
const float FOREARM_LENGTH = 1.0;
const float WRIST_LENGTH = 1.0;

//float SHOULDER_OFFSET = 1.0;  Should offset doesn't matter; move the coordinate system up by this amount and the result will be the same
const float ELBOW_OFFSET = 1.0;

const float ADJUSTED_SHOULDER_PITCH = atan2(ELBOW_OFFSET, BICEP_LENGTH);

struct angles{
    float shoulderYaw = 0.0;
    float shoulderPitch = 0.0;
    float elbowPitch = 0.0;
    float wristPitch = 0.0;
    float wristRoll = 0.0;
};

struct angles toAnglesInPlane(float x, float y); // Defined at bottom

/*
 * Calculate angles of the two inner arms using wrist position
 * TODO: Switch int inputs to unsigned char
 */
struct angles toAnglesFromPosition(int x, int y, int z){
    struct angles angles;
    //Normalize x, y, z, coordinates
    float xNorm = (float)x/100.0;
    float yNorm = (float)y/100.0;
    float zNorm = (float)z/100.0;

    if(pow(xNorm, 2) + pow(yNorm, 2) + pow(zNorm, 2) > 1){
	   //Project the position along it's radius onto a unit sphere
    }

    //Needs C++ 11
    angles = toAnglesInPlane(hypot(xNorm, yNorm), zNorm);
    angles.shoulderYaw = atan2(yNorm, xNorm);

    return angles;
}

/*
 * Calculate angles of arms using end effector position and wrist orientation
 * TODO: Switch int inputs to unsigned char
 */
struct angles toAnglesWithOrientation(int x, int y, int z, float alpha){
    struct angles angles;
    //Normalize x, y, z coordinates
    float xNorm = (float)x/100.0;
    float yNorm = (float)y/100.0;
    float zNorm = (float)z/100.0;

    if(pow(xNorm, 2) + pow(yNorm, 2) + pow(zNorm, 2) > 1){
	   //Project the position along it's radius onto a unit sphere
    }

    //Needs C++ 11
    angles = toAnglesInPlane(
        hypot(xNorm, yNorm) - WRIST_LENGTH*cos(alpha),
	zNorm - WRIST_LENGTH*sin(alpha));

    angles.wristPitch = alpha;
    angles.shoulderYaw = atan2(yNorm, xNorm);
    return angles;
}

/*
 * Calculate angles of arms given x, y coordinates of wrist in plane of arm
 *
 * Really should only be used by the other methods as it only sets the
 * shoulder and elbow pitch fields of the structure
 */
struct angles toAnglesInPlane(float x, float y){
    struct angles angles;

    float xyReachSquared = pow(x, 2) + pow(y, 2);

    angles.elbowPitch =
	acos((pow(BICEP_LENGTH, 2) + pow(FOREARM_LENGTH, 2) - xyReachSquared)
	    / (2*BICEP_LENGTH*FOREARM_LENGTH));

    angles.shoulderPitch =
        acos((pow(BICEP_LENGTH, 2) + xyReachSquared - pow(FOREARM_LENGTH, 2))
	    / (2*BICEP_LENGTH*sqrt(xyReachSquared)))
	- atan2(y, x) + ADJUSTED_SHOULDER_PITCH;

    return angles;
}

/**
 * On receiving arm coordinates, converts to arm angles and publishes
 */
void armCoordsCallback(const arm_controls::arm_position::ConstPtr& msg) {
  rover2::ArmAngles armAngles; // A vector of JointAngles
  rover2::JointAngle jointAngle;

  // Convert coords to angles
  // I think toAnglesWithOrientation takes wrist pitch which is msg->flick
  struct angles angles = toAnglesWithOrientation(msg->x, msg->y, msg->z, msg->flick);

  // Add angles to armAngles
  jointAngle.id = 0;
  jointAngle.angle = angles.shoulderYaw;
  armAngles.joints.push_back(jointAngle);

  jointAngle.id = 1;
  jointAngle.angle = angles.shoulderPitch;
  armAngles.joints.push_back(jointAngle);

  jointAngle.id = 2;
  jointAngle.angle = angles.elbowPitch;
  armAngles.joints.push_back(jointAngle);

  // I think msg->wrist is wrist roll
  jointAngle.id = 3;
  jointAngle.angle = msg->wrist;
  armAngles.joints.push_back(jointAngle);

  jointAngle.id = 4;
  jointAngle.angle = angles.wristPitch;
  armAngles.joints.push_back(jointAngle);

  jointAngle.id = 5;
  jointAngle.angle = msg->grab;
  armAngles.joints.push_back(jointAngle);

  arm_angles_pub.publish(armAngles);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "arm_ik");
  node = new ros::NodeHandle;
  arm_coords_sub = node->subscribe("/user_arm_controls", 1000, armCoordsCallback);
  arm_angles_pub = node->advertise<rover2::ArmAngles>("/arm/angles", 1000);
  ros::spin();
  return 0;
}
