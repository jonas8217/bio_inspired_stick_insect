#include "cpg_controller.h"
using namespace std;
// initialize controller
cpg_controller::cpg_controller(ros::NodeHandle &node_handle_i): node_handle (node_handle_i)
{
    // set up ros advertisers (publishers)
    pub_cpg_values = node_handle.advertise<std_msgs::Float64MultiArray>("sim_ros_interface/target/legs_cpgs",1);
    pub_joint_positions = node_handle.advertise<std_msgs::Float64MultiArray>("sim_ros_interface/target/joint_positions",1);
    pub_joint_offsets = node_handle.advertise<std_msgs::Float64MultiArray>("sim_ros_interface/target/joint_offsets",1);

    get_sim_state = node_handle.subscribe("sim_ros_interface/simulation_state", 1, &cpg_controller::getSimState, this);

    // setup CPGS
    initialize_cpgs(alpha,frequency);
}

// publish cpgs method
void cpg_controller::publish_cpgs(std::vector<double> cpgs){
    // initialize ros message
    std_msgs::Float64MultiArray message;

    // push back all cpg values to message
    for(int i = 0; i < cpgs.size(); i++){
        message.data.push_back(cpgs[i]);
    }

    // publish message
    pub_cpg_values.publish(message);
}

// publish cpgs method
void cpg_controller::publish_joint_positions(std::vector<double> joint_positions){
    // initialize ros message
    std_msgs::Float64MultiArray message;

    // push back all cpg values to message
    for(int i = 0; i < joint_positions.size(); i++){
        message.data.push_back(joint_positions[i]);
    }
    // publish message
    pub_joint_positions.publish(message);
}

void cpg_controller::publish_joint_offsets(){
    // publish the offsets
    std_msgs::Float64MultiArray message;

    // set front leg offsets
    for(int i = 0; i < 3; i++){ 
        for(int j = 0; j < 2; j++){ // 2 because there are two legs
            message.data.push_back(F_offset[i]);
            message.data.push_back(M_offset[i]);
            message.data.push_back(H_offset[i]);
        }
    }

    // publish message
    pub_joint_offsets.publish(message);
}

// initialize cpg with frequency and alpha (synaptic plasticity)
void cpg_controller::initialize_cpgs(double alpha,double frequency){
    cpg.set_so2_weights(alpha,frequency);
}

// Provided simple premotor
std::vector<double> cpg_controller::SimplePreMotor(double C1, double C2, int leg_index){
    std::vector<double> output;
    output.resize(0);

    double fact = 1.0;

    switch (leg_index){

    case 0: // Right Front leg
        output.push_back(C1*fact+F_offset[0]);
        output.push_back( std::max(C2,0.0)*fact+F_offset[1]);
        output.push_back(0.0+F_offset[2]);
        break;

    case 1: // Right Middle leg
        output.push_back( C1*fact+M_offset[0]) ; 
        output.push_back( std::max(C2,0.0)*fact+M_offset[1]);
        output.push_back( 0.0+M_offset[2]);
        break;

    case 2: // Right Hind leg
        output.push_back( C1*fact+H_offset[0]);
        output.push_back( std::max(C2,0.0)*fact+H_offset[1]);
        output.push_back( 0.0+H_offset[2]);
        break;


    case 3: // Left Front leg
        output.push_back( C1*fact+F_offset[0]);
        output.push_back( std::max(C2,0.0)*fact+F_offset[1]);
        output.push_back(0.0+F_offset[2]);
        break;

    case 4: // Left Middle leg
        output.push_back( C1*fact+M_offset[0]) ; 
        output.push_back( std::max(C2,0.0)*fact+M_offset[1]);
        output.push_back( 0.0+M_offset[2]);
        break;

    case 5: // Left Hind leg
        output.push_back( C1*fact+H_offset[0]);
        output.push_back( std::max(C2,0.0)*fact+H_offset[1]);
        output.push_back( 0.0+H_offset[2]);
        break;
        
    default:
        output.push_back(0.0);
        output.push_back(0.0);
        output.push_back(0.0);
        break;

    }
    return output;
}

// take cpg steps
void cpg_controller::take_step(){
    // vector of all outputs
    std::vector<double> all_outputs;
    std::vector<double> cpg_1;
    std::vector<double> cpg_2;
    std::vector<double> output;
    std::vector<double> joint_positions;

    // run cpg step for each leg
    output = cpg.CPG_step();

    if(step >= max){
        step = 0;
    }
    // delay and output left first joint (cpg_1 is old cpg_2)
    cpg_1.push_back(l0[step%xl0][0]);
    cpg_1.push_back(l0[step%xl0][1]);
    l0[step%xl0][0]=output[0];
    l0[step%xl0][1]=output[1];

    cpg_2.push_back(output[0]);
    cpg_2.push_back(output[1]);

    // Update step
    step++;

    // Convert CPG to joint poses
    std::vector<std::vector<double>> all_joint_positions; 
    std::vector<double> output_move_1 = SimplePreMotor(cpg_1[0], cpg_1[1],0);
    std::vector<double> output_move_2 = SimplePreMotor(cpg_2[0], cpg_2[1],1);
    std::vector<double> output_move_3 = SimplePreMotor(cpg_1[0], cpg_1[1],2);
    std::vector<double> output_move_4 = SimplePreMotor(cpg_2[0], cpg_2[1],3);
    std::vector<double> output_move_5 = SimplePreMotor(cpg_1[0], cpg_1[1],4);
    std::vector<double> output_move_6 = SimplePreMotor(cpg_2[0], cpg_2[1],5);

    all_joint_positions.push_back(output_move_1);
    all_joint_positions.push_back(output_move_2);
    all_joint_positions.push_back(output_move_3);
    all_joint_positions.push_back(output_move_4);
    all_joint_positions.push_back(output_move_5);
    all_joint_positions.push_back(output_move_6);

    // Collect all cpg outputs
    // leg 1
    all_outputs.push_back(cpg_1[0]);
    all_outputs.push_back(cpg_1[1]);
    // leg 2
    all_outputs.push_back(cpg_2[0]);
    all_outputs.push_back(cpg_2[1]);
    // leg 3
    all_outputs.push_back(cpg_1[0]);
    all_outputs.push_back(cpg_1[1]);
    // leg 4
    all_outputs.push_back(cpg_2[0]);
    all_outputs.push_back(cpg_2[1]);
    // leg 5
    all_outputs.push_back(cpg_1[0]);
    all_outputs.push_back(cpg_1[1]);
    // leg 6
    all_outputs.push_back(cpg_2[0]);
    all_outputs.push_back(cpg_2[1]);


    // Set joint positions
    for(int j = 0; j < number_of_joints;j++){
        for(int i = 0; i < all_joint_positions.size();i++){
            joint_positions.push_back(all_joint_positions[i][j]);
        }
    }


    // publish the received values after seetled
    if(settle_step > settle_limit){
        publish_cpgs(all_outputs);
        publish_joint_positions(joint_positions);
    }
    else{
        settle_step++;
    }
    publish_joint_offsets();
}

