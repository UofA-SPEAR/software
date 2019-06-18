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
#define SKIDSTEER_ANGULAR_RATIO(x, y)       2 * PI * sqrt(x*x + y*y) / cos(atan(y/x))
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
    double z;
    double yaw;
} tf_delta_t;


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

tf_delta_t odom_to_tf_delta(skidsteer_t odom) {
    // How do I translate the stuff I did above?
}


#endif
