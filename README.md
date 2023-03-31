# SnakeRaven_IBVS_simulation
This code is a MATLAB simulation of the SnakeRaven vision system. Image Based Visual Servoing (IBVS) involves taking image features for measurements and computing the necessary camera motion. This MATLAB simulation does this with a virtual four dot target and builds on from the SnakeRaven simulator in repository: https://github.com/Andrew-Raz-ACRV/SnakeRavenSimulation

The main simulator is run using the command: 
'''
SnakeRaven_VisualServo_simulator
'''
It utilises the functions in IBVS_demo, Math_functions, Plotting_functions and SnakeRaven_kinematics.
It can run either the IBVS orientation partitioned or Visual Predictive Control algorithms as mentioned in the chapter in my [thesis](https://eprints.qut.edu.au/235042/)
Noe: the Visual Predictive Controller requires the Optimization Toolbox

Example output:

![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven_IBVS_simulation/blob/main/ibvs3d.png)
![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven_IBVS_simulation/blob/main/ibvse.png)
![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven_IBVS_simulation/blob/main/ibvser.png)
![alt text](https://github.com/Andrew-Raz-ACRV/SnakeRaven_IBVS_simulation/blob/main/ibvsv.png)

There is also code for processing validation data results which is executed by [IBVS_result_processor](https://github.com/Andrew-Raz-ACRV/SnakeRaven_IBVS_simulation/tree/main/IBVS_result_processing) and requires the Robotics toolbox and Signal Processing toolbox.

It also includes a variety of additional code for testing hand-eye calibration which may need Peter Corke's Robotic toolbox for MATLAB: https://github.com/petercorke/robotics-toolbox-matlab
