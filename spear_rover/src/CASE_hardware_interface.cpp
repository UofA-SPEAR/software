// Actually implement hardware interface
//

#include <sstream>
#include <CASE_hardware_interface/CASE_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace CASE_hardware_interface
{
    CASEHardwareInterface::CASEHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
        init();
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/CASE/hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &CASEHardwareInterface::update, this);
    }

    CASEHardwareInterface::~CASEHardwareInterface() {

    }

    void CASEHardwareInterface::init() {
        // Get joint names
        nh_.getParam("/CASE/hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        // Initialize Controller 
        for (int i = 0; i < num_joints_; ++i) {
             // Create joint state interface
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
             joint_state_interface_.registerHandle(jointStateHandle);

            // Create position joint interface
            /*
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            JointLimits limits;
                SoftJointLimits softLimits;
            getJointLimits(joint.name, nh_, limits)
            PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
            positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
            position_joint_interface_.registerHandle(jointPositionHandle);
            */

            // Create effort joint interface
            /*
            JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(jointEffortHandle);
            */

            // Create joint velocity interface
            JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
            velocity_joint_interface_.registerHandle(jointVelocityHandle);
        }

        registerInterface(&joint_state_interface_);
        //registerInterface(&position_joint_interface_);
        //registerInterface(&effort_joint_interface_);
        registerInterface(&velocity_joint_interface_);
        //registerInterface(&velocityJointSoftLimitsInterface);
    }

    void CASEHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void CASEHardwareInterface::read() {
        for (int i = 0; i < num_joints_; i++) {
            //joint_position_[i] = CASE.getJoint(joint_names_[i]).read();
            // Read in all values from hardware
        }
        ROS_INFO("Reading values!");
    }

    void CASEHardwareInterface::write(ros::Duration elapsed_time) {
        velocityJointSoftLimitsInterface.enforceLimits(elapsed_time);
        for (int i = 0; i < num_joints_; i++) {
            // CASE.getJoint(joint_names_[i]).actuate(joint_effort_command_[i]);
            // Write values to hardware
            
        }
        ROS_INFO("Writing values");
    }
}
