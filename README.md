# UAS-Modeling-and-Simulation
MATLAB and Simulink model of the Unmanned Aircraft System (UAS)
| No. | Program | File name | Description |
| :---         |     :---:      |          ---: | :---         |
| 1 | MATLAB | Missile_EoM.m | Function for the missile ordinary differential equations |
| 2 | MATLAB | Assignment3_work.m | The main script file to execute Missile_EoM.m |

![image](https://github.com/komxun/UAS-Modeling-and-Simulation/assets/133139057/7c0ebbfb-7638-4fcd-a63d-b58b25db1927)


# Introduction
A schematic diagram of the UAV system is illustrated in Fig.1. Overall, the main 6-degree of freedom equations of motions 
require inputs from the total forces and moments models which depend on the environment model. At the end of each time-step, 
the output states of the main 6-DOF equations of motion will be fed back into the environment and the actuator model. 
The actuator can be designed to control the aerodynamics and the propulsive force.

|![image](https://github.com/komxun/UAS-Modeling-and-Simulation/assets/133139057/0b5cfd19-57a0-47d0-b57e-6011d11a92d4)|
|:--:|
|Figure 1: A schematic diagram of the Unmanned Aircraft System (UAS)|

# UAS Modeling
In this work, the model of a missile has been created in both Simulink and MATLAB
## MATLAB model
The aerodynamics coefficients have been estimated using **interp1()** with linear interpolation and extrapolation, 
and **interp2()** with the Spline interpolation and Spline extrapolation method. The integrator for the equation of motion 
is set to be **ode45**. The missile equation of motion has been coded in function  **Missile_EoM.m** which can be executed 
in the main script: **main_Assignment3.m**

## Simulink model
The missile model has also been created in Simulink with file name **missile_model_2.slx** The overall system diagram is shown below:
|![image](https://github.com/komxun/UAS-Modeling-and-Simulation/assets/133139057/05c4944c-291e-494d-ad6a-38189735712c)|
|:--:|
|Figure 6: Overall system diagram|

|![image](https://github.com/komxun/UAS-Modeling-and-Simulation/assets/133139057/b2c3d35a-585d-4230-9732-270ba8b72040)|
|:--:|
|Figure 7: Actuator model|

|![image](https://github.com/komxun/UAS-Modeling-and-Simulation/assets/133139057/8f7bf76e-6b6c-493d-8731-7ee13ab285e7)|
|:--:|
|Figure 8: Missile Dynamics Model|

|![image](https://github.com/komxun/UAS-Modeling-and-Simulation/assets/133139057/b30cc66c-1499-424e-b518-a54a5a3df20d)|
|:--:|
|Figure 9: Aerodynamics model|

|![image](https://github.com/komxun/UAS-Modeling-and-Simulation/assets/133139057/ec06bc74-1801-47cb-87fa-b965c6404fbc)|
|:--:|
|Figure 10: Aerodynamics Force and Moment model|

|![image](https://github.com/komxun/UAS-Modeling-and-Simulation/assets/133139057/088b8bdd-b568-4f66-8713-c8c908837696)|
|:--:|
|Figure 11: Total Force and Moment model|

|![image](https://github.com/komxun/UAS-Modeling-and-Simulation/assets/133139057/5797602f-413c-480f-b144-dc655119ddd1)|
|:--:|
|Figure 12: Equation of Motion|

