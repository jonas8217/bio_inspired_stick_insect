#ifndef CPG_H
#define CPG_H

// Includes
#include <iostream>
#include <vector>
#include <fstream>
#include "cmath"

class CPG
{
public:
    // Constructors
    CPG();

    // Destructor
    ~CPG(){}

    // Activation functions
    void activation();
    void activation(std::vector<double> inputs);

    // Full CPG step
    std::vector<double> CPG_step();
    std::vector<double> CPG_step(std::vector<double> inputs);

    // Weight functions
    void set_so2_weights(double alpha,double theta);

    // Various print functions
    void print_weights();

    // Various get and set functions

    // get activations
    std::vector<double> get_activations();

    // get outputs
    std::vector<double> get_cpg_outputs();


private:
    std::vector<double> weights; // w_11, w_12, w_22, w_21
    std::vector<double> neuron_outputs={0,0}; //CPG neuron outputs initialised to 0 for 2 neurons
    std::vector<double> activations={0,0}; // initialiation of activation vector
    std::vector<double> bias = {0.1,0.1};
};

#endif // CPG_H
