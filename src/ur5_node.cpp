#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "modern_robotics.h"
#include "ur5_MR_description.h"
#include <Eigen/Dense>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_node");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "shoulder_pan_joint";
    joint_state.name[1] = "shoulder_lift_joint";
    joint_state.name[2] = "elbow_joint";
    joint_state.name[3] = "wrist_1_joint";
    joint_state.name[4] = "wrist_2_joint";
    joint_state.name[5] = "wrist_3_joint";

    Eigen::MatrixXd Slist;
    Slist = get_Slist();
    Eigen::MatrixXd Mlist = get_Mlist();
    Eigen::VectorXd joint_angles(6);
    joint_angles << 0, 0.0, 0, 0, 0, 0;
    bool solution_found;
    double x = 0.5;
    Eigen::MatrixXd Tsd(4,4);
    Eigen::MatrixXd end_effector(4,4);

    while(ros::ok())
    {
        Tsd << -1, 0, 0,   x,
                0, 0, 1, 0.5,
                0, 1, 0, 0.3,
                0, 0, 0, 1;
        solution_found = mr::IKinSpace(Slist, Mlist, Tsd, joint_angles, 0.0005, 0.0005);
        std::cout << "Solution found? " << solution_found << std::endl;
        end_effector = mr::FKinSpace(Mlist, Slist, joint_angles);
        std::cout << "Joint angles\n" << joint_angles << std::endl;
        std::cout << "MR FKinSpace position\n" << end_effector << std::endl;

        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = joint_angles[0];
        joint_state.position[1] = joint_angles[1];
        joint_state.position[2] = joint_angles[2];
        joint_state.position[3] = joint_angles[3];
        joint_state.position[4] = joint_angles[4];
        joint_state.position[5] = joint_angles[5];
        joint_pub.publish(joint_state);
        x = x - 0.001;
        std::cout << x << std::endl;
        loop_rate.sleep();
    }

    return 0;
}
