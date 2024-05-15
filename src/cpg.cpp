#include "cpg.h"

// Empty constructor
CPG::CPG()
{
    weights= {0,0,0,0};
}

// Activation function without given inputs
void CPG::activation(){
    // Calculate activations
    for(int i = 0; i < activations.size();i++){
        // Calculate output using activation and tanh
        neuron_outputs[i] = tanh(activations[i]);
        if(i == 0){
            activations[i] = weights[i] * neuron_outputs[i] + weights[i+1]*neuron_outputs[i+1] + bias[i];
        }
        else{
            activations[i] = weights[i+1] * neuron_outputs[i] + weights[i+2]*neuron_outputs[i-1] + bias[i];
        }
    }
}

// Activation function with given inputs
// Inputs : vector of one input to each neuron
void CPG::activation(std::vector<double> inputs){
    // needed values
    double input_1 = inputs[0];
    double input_2 = inputs[1];

    // Activation calculation
    //double a_1 = w_11*o_1+w_12*o_2+synaptic_plasticity*input_1+bias_1;
    //double a_2 = w_21*o_1+w_22*o_2+synaptic_plasticity*input_2+bias_2;
    //double a_1 = w_11*o_1+w_12*o_2+input_1+bias_1;
    //double a_2 = w_21*o_1+w_22*o_2+input_2+bias_2;
    //activations[0] = a_1;
    //activations[1] = a_2;

    // get neuron outputs
    //o_1 = tanh(a_1);
    //o_2 = tanh(a_2);
    //neuron_outputs[0] = o_1;
    //neuron_outputs[1] = o_2;

    /*CPG::neuron_outputs_old = CPG::neuron_outputs;
    // Calculate activations
    for(int i = 0; i < CPG::activations.size();i++){
        if(i == 0){
            CPG::activations[i] = CPG::weights[i] * CPG::neuron_outputs_old[i] + CPG::weights[i+1]*CPG::neuron_outputs_old[i+1] + inputs[i] + CPG::bias[i];
        }
        else{
            CPG::activations[i] = CPG::weights[i+1] * CPG::neuron_outputs_old[i] + CPG::weights[i+2]*CPG::neuron_outputs_old[i-1] + inputs[i]+ CPG::bias[i];
        }

        // Calculate output using activation and tanh
        CPG::neuron_outputs[i] = tanh(CPG::activations[i]);
    } */
}

// Set weights based on SO(2)-matrix
// frequency : frequency of output pattern -pi < > pi
// alpha : synaptic plasticity > 1
void CPG::set_so2_weights(double alpha,double theta){
    std::cout << "made it in" << std::endl;
    // Set weights
    weights[0] = alpha*cos(theta);
    weights[1] = alpha*sin(theta);
    std::cout << "halfway" << std::endl;
    weights[2] = alpha*cos(theta);
    weights[3] = alpha*(-sin(theta));
    std::cout << "calculations are done" << std::endl;
}

// print weight function
void CPG::print_weights(){
    std::cout << "w_11: " << weights[0] << std::endl;
    std::cout << "w_12: " << weights[1] << std::endl;
    std::cout << "w_22: " << weights[2] << std::endl;
    std::cout << "w_21: " << weights[3] << std::endl;
}

// Full CPG step without input
std::vector<double> CPG::CPG_step(){
    // run activation
    activation();
    // Return outputs
    return neuron_outputs;
}

// Full CPG step with input
std::vector<double> CPG::CPG_step(std::vector<double> inputs){
    // run activation
    activation(inputs);
    // Return outputs
    return neuron_outputs;
}

std::vector<double> CPG::get_activations(){
    return activations;
}

std::vector<double> CPG::get_cpg_outputs(){
    return neuron_outputs;
}
