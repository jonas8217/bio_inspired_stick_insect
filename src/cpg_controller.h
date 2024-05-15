#ifndef CPG_CONTROLLER_H
#define CPG_CONTROLLER_H

// Includes
#include "cpg.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <vector>
#include <fstream>
#include "cmath"
#include <algorithm>

const int xl0 = 0.7*30;
const int xl1 = 0.1*30;
const int xl2 = 0.1*30;
const int xr1 = 0.1*30;
const int xr2 = 0.1*30;

class cpg_controller
{
public:
    // class initializing (initialize ros handle and assign it to our private variable)
    cpg_controller(ros::NodeHandle &node_handle_i);

    // publish cpgs function
    void publish_cpgs(std::vector<double> cpgs);

    // publish joint positions
    void publish_joint_positions(std::vector<double> joint_positions);

    
    void publish_joint_offsets();
    
    // initialize cpg's with weights
    void initialize_cpgs(double alpha, double frequency);

    // take cpg step
    void take_step();


    // provided premotor method
    std::vector<double> SimplePreMotor(double C1, double C2, int leg_index);

    int getSimState(){
        return sim_state;
    }

    void getSimState(const std_msgs::Float32::ConstPtr& data){
    if(data->data != -1)
        sim_state = data->data; 
    else 
        ROS_ERROR("Error in sim state ..");
    }


private:
    // initialize ros
    // happens in pre implemented controller :)

    // initialize nodes
    ros::NodeHandle node_handle;

    // initialize publishers
    ros::Publisher pub_cpg_values;
    ros::Publisher pub_joint_positions;
    ros::Publisher pub_joint_offsets;

    ros::Subscriber get_sim_state;

    int sim_state = 0;

    // initialize needed cpg's
    CPG cpg;

    // number of legs
    int number_of_legs = 6;
    int number_of_joints = 3;

    // CPG constants
    double frequency = 0.074*M_PI;
    double alpha = 1.25;

    // l = left, r = right
    double l0[xl0][2]={0};
    double l1[xl1][2];
    double l2[xl2][2];
    double r1[xr1][2];
    double r2[xr2][2];
    int step = 0;
    int max = xl0;//*xl1*xl2*xr1*xr2;

    // let cpg seetle before using it
    int settle_step = 0;
    int settle_limit = 100;

    // joint offsets
    double F_offset[3] = {0.523, 0.174, -1.047};
    double M_offset[3] = {0.0, 0.0, -1.047};
    double H_offset[3] = {-0.698, 0.189, -1.048};

};

#endif // CPG_CONTROLLER_H
