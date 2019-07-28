#ifndef SKIDSTEER_HPP_
#define SKIDSTEER_HPP_

/*
 * Small library for dealing with skidsteer stuff.
 * Seperated out because I want changes in how I handle skidsteer to
 * reflect in both odometry and the steering itself.
 */

#include <geometry_msgs/Twist.h>
#include <math.h>

// Physical Rover Constants

// Takes the locations of the wheels.
// TODO: make these happen in compilation
//#define SKIDSTEER_ANGULAR_RATIO(x, y)       2 * PI * sqrt(x*x + y*y) / cos(atan(y/x))
#define SKIDSTEER_ANGULAR_RATIO(x, y)       sqrt(x*x + y*y) / cos(atan2(y, x))
#define SKIDSTEER_RATIO_FRONTRIGHT          SKIDSTEER_ANGULAR_RATIO(0.5, 0.5)
#define SKIDSTEER_RATIO_FRONTLEFT           SKIDSTEER_ANGULAR_RATIO(-0.5, 0.5)
#define SKIDSTEER_RATIO_BACKLEFT            SKIDSTEER_ANGULAR_RATIO(-0.5, -0.5)
#define SKIDSTEER_RATIO_BACKRIGHT           SKIDSTEER_ANGULAR_RATIO(0.5, -0.5)

typedef struct {
    struct {
        double back;
        double front;
    } left;

    struct {
        double back;
        double front;
    } right;
} skidsteer_t;

typedef struct {
    double x;
    double y;
    double yaw;
} odom_delta_t;


skidsteer_t twist_to_skidsteer(geometry_msgs::Twist twist) {
    skidsteer_t out;

    // We only care about linear x velocity, and angular z velocity

    // Handle linear velocity
    out.left.back   = twist.linear.x;
    out.left.front  = twist.linear.x;
    out.right.back  = twist.linear.x;
    out.right.front = twist.linear.x;

    /* Handle angular velocity
     *
     * I'm going to assume that the tangential velocity of the wheel to the circle
     * based on the centre of the robot will directly translate to rotational velocity.
     *
     *     y
     *     ^
     *   X | X
     *-x <----> x
     *   X | X
     *     v
     *    -y
     *
     */
    
    out.left.back   += twist.angular.z * SKIDSTEER_RATIO_BACKLEFT;
    out.left.front  += twist.angular.z * SKIDSTEER_RATIO_FRONTLEFT;
    out.right.back  += twist.angular.z * SKIDSTEER_RATIO_BACKRIGHT;
    out.right.front += twist.angular.z * SKIDSTEER_RATIO_FRONTRIGHT;

    return out;
}

odom_delta_t wheel_increments_to_odom_delta(skidsteer_t odom) {
    /* Assumptions:
     * - taken in small enough increments that it can be approximated as arc of circle.
     * - We have good data
     */
    
    odom_delta_t out;
    double left, right;
    
    // Calculate average linear motion.
    left  = (odom.left.back + odom.left.front) / 2;
    right = (odom.right.back + odom.right.front) / 2;

    // Forward is x I guess
    double linear = (left + right) / 2;

    // Remove linear component from each wheel's motion
    odom.left.back   -= linear;
    odom.left.front  -= linear;
    odom.right.back  -= linear;
    odom.right.front -= linear;

    double ang_left_back, ang_left_front, ang_right_back, ang_right_front;

    // Get angular motion according to each wheel
    ang_left_back   = odom.left.back / SKIDSTEER_RATIO_BACKLEFT;
    ang_left_front  = odom.left.front / SKIDSTEER_RATIO_FRONTLEFT;
    ang_right_back  = odom.right.back / SKIDSTEER_RATIO_BACKRIGHT;
    ang_right_front = odom.right.front / SKIDSTEER_RATIO_FRONTRIGHT;

    // Take yaw value as average angular motion
    out.yaw = (ang_left_back + ang_left_front + ang_right_back + ang_right_front) / 4;

    if (out.yaw == 0) {
        out.x = linear;
        out.y = 0;
    } else {
        // Radius of curvature of the path
        double radius = linear / out.yaw;

        // Just draw out an arc length with the angle in there and this will make sense.
        out.x = sin(out.yaw) * radius;
        out.y = radius - cos(out.yaw) * radius;
    }

    return out;
}


#endif
