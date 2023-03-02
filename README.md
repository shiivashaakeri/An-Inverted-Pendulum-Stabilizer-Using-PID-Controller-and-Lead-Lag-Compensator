# An Inverted Pendulum Stabilizer Using PID Controller and Lead-Lag Compensator
This project implements a controller for an inverted pendulum system using a PID controller and lead-lag compensator. The goal is to stabilize the inverted pendulum, which is a classic example of a nonlinear, unstable system, by designing a controller that can keep the pendulum upright.

The mechanical system was modeled using mathematical equations, and the state-space equations were obtained. The transfer function was derived from these equations, and then the controller was designed using Bode plot and Nyquist diagram in MATLAB. The root locus was used for stability analysis, and the compensator editor window in MATLAB was used to stabilize the system.

Finally, a PID controller was designed using the PID MATLAB tool.

## System Overview
The inverted pendulum is a classic control problem where a pendulum is mounted on a cart that can move along a horizontal track. The objective is to keep the pendulum balanced in an upright position by moving the cart back and forth. This is a challenging problem because the system is inherently unstable and nonlinear.

In this project, we will be using a simulation of an inverted pendulum system to design and implement a controller to stabilize the pendulum.

## Installation
To run this project, you will need to have MATLAB installed on your system. Once you have MATLAB installed, simply clone this repository to your local machine:
``` sh
git clone https://github.com/your_username/An-Inverted-Pendulum-Stabilizer-Using-PID-Controller-and-Lead-Lag-Compensator.git
```
## Usage

The project includes MATLAB files for modeling the mechanical system, deriving the state-space equations, designing the controller using Bode plot and Nyquist diagram, and implementing the PID controller and lead-lag compensator.

To use this project, simply open the MATLAB files and run the code in the MATLAB environment. You can modify the parameters of the system and the controller to experiment with different configurations and test the performance of the system.

## Acknowledgments 
We would like to express our gratitude to Prof. Aras Adhami for providing guidance and support throughout the course. We would also like to thank Farbod Mousavi for their valuable feedback and assistance during the development of this project.

Additionally, we would like to acknowledge the contributions of the open-source community and the various resources available online that helped us in understanding the concepts and tools used in this project. Some of the useful weblinks are:

- [Control System Toolbox documentation](https://www.mathworks.com/help/control/index.html)
- [Control System Designer documentation](https://www.mathworks.com/help/control/control-system-designer-app.html)
- [PID Controller Design in MATLAB](https://www.mathworks.com/help/control/ug/pid-controller-design.html)
- [Lead-Lag Compensator Design in MATLAB](https://www.mathworks.com/help/control/ug/lead-compensator-design.html)
- [Root Locus Design in MATLAB](https://www.mathworks.com/help/control/ug/root-locus-design.html)
- [Inverted Pendulum Modeling and Control - University of Michigan](https://ctms.engin.umich.edu/CTMS/index.phpexample=InvertedPendulum&section=SystemModeling)
- [PID Control of Inverted Pendulum - University of Michigan](https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlPID)
- [Inverted Pendulum Control - YouTube Video Tutorial](https://www.youtube.com/watch?v=qjhAAQexzLg)


