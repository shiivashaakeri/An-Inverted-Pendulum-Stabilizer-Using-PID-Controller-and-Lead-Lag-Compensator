# An-Inverted-Pendulum-Stabilizer-Using-PID-Controller-and-Lead-Lag-Compensator
The inverted pendulum has a unique trait; it is unpredictable, non-linear, and consists of multiple variables. Balancing by PID controller is a continuous process where it corrects the feedback system error from the difference between the measured value and the desired value. Balancing by Lead-Lag compensator is designing a lead compensator for the process to change the shape of the root locus and choose the gain so that the poles are in the desired position and designing a lag compensator to leave the dominant closed-loop poles of the lead-compensated process in approximately the position but provide extra low-frequency gain.

![](./Figures/Aspose.Words.a923060b-a05e-42fc-8d72-5b51313a9b74.001.png)

**The Inverted Pendulum System**

Shiva Shakeri

Department of ECE, University of Tehran Linear Control Systems

Dr. Aras Adhami

July 2021



1. **Introduction**

Balance an inverted pendulum on a moving cart that moves horizontally in one direction, is a classic problem in control systems.

In this project, different methods are presented to keep the inverted pendulum in equilibrium. In this system, an inverted pendulum is connected to a chariot that moves in the direction of the horizontal axis when a force is applied. This system has two intrinsic equilibrium points that One is stable and the other is unstable. The stable equilibrium point is where the pendulum is down, which naturally goes into this state without the need for any controllers. The unstable equilibrium point is related to the position where the pendulum is placed vertically on the cart, which requires a controller to balance. The main goal of this project is to design a controller to keep the pendulum at its unstable equilibrium point.

First, we model the transfer function of the system using mechanical equations and then we simulate the system with MATLAB to clarify the general situation of the problem. First, we design a PID controller for this system and the results. Then we design the lag-lead controller system using the locus root diagram and observe the results in the simulation, and finally, we design the controller using the bad diagram and the Nyquist diagram. And we simulate the results of this step.

From this simulation, we can observe the intrinsic response of the system and obtain the desired results by placing a controller and changing its coefficients.
