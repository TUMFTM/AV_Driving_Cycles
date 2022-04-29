# AV_Driving_Cycles
This tool develops driving cycles for specific driving style properties (comfort, consumption, fastness, subjective safety) of AVs.

It is an ACC simulation of an AV that follows leading vehicle that drives one of the today known cycles. The parameters of the ACC controller vary depending on the driving style of the AV. Depending on the fastness of the AV, other vehicles can be overtaken by the AV or the AV can be overtaken by other vehicles.

  
## Running the Model/Code

To start the simulation, you have to run the GUI  
```
GUI_AVDC.mlapp
```
You can find it in the folder
```
01_Main
```

The GUI is shown in the following figure. The numbers 1-6 will explain its functionality.
  
![GUI AV Drving Cycles](https://user-images.githubusercontent.com/72914074/163174684-a651846f-a2d3-408a-bd62-d835054cb70f.png)
  

The following description is brief. For a detailed explanation, please refer to the paper.

### 1: Input Mode
This defines the mode of the simulation. Depending on this input the values of the driving style or the values of the Controller Parameters are the input of the simulation.

### 2: Drving Style
The driving style of the AV is defined. The values of the properties comfort, fastness and (subjective) safety can be chosen. The property consumption is optimized in the given input. The interval of the values ranges from 5 to 10. Unfeasible combinations are highlighted as invalid and the next valid combinations are recommended.

### 3: Controller Parameters
The controller parameters of the ACC controller are defined. If the Input mode is "Driving Style", this will be filled automatically. The parameters are set time gap (t_set), Coefficient for deceleration (K_brk), (P_a_acc), (P_v_acc), maximal acceleration (a_max), minimal jerk (j_min), (set_speed) und (ovt_tol).

### 4: Traffic Settings
The traffic settings define the other vehicles. The Traffic Cycle defines their driving behavior and the Traffic Interval the distance in seconds between the other vehicles.

### 5: Actions
The four buttons start the different actions. 
Generate Cycle simulated the Driving Cycle with the current inputs. Export Cycle saves the current Driving Cycle and Export all Data all Driving Cycles. Exit closes the GUI.


### 6: Generated Drving Cycle
In these six boxes, the simulated driving cycles are plotted. The blue line is the driving cycle of the AV, the orange one the leading vehicle.
The chosen one is marked with frame. With a double click, you can choose another cycle. 

  
## Deployment
  
* [Matlab](https://de.mathworks.com/products/matlab.html) R2020b
  
## Prerequisites

- Control System Toolbox,
- Statistics and Machine Learning Toolbox,
- Simulink,
- Simulink Real-Time,
- Stateflow
  
## Contributing and Support
  
We are strongly encouraged to improve quality and functionality of AV_Driving_Cycles. If you have any Feedback don't hesitate to contact the authors or the group leader of the vehicle concept research group at FTM of the TUM.

## Versioning
  
V1.0 initial public version of AV_Driving_Cycles
  
## Authors
- Xucheng Duan (Technical University of Munich): Code creating and detailing, Code documentation
- Ferdinand Schockenhoff (Institute for Automotive Technology, Technical University of Munich): Creation of research topic, Conceptualization, Supervison, Code documentation
  
## License
This project is licensed under the LGPL License - see the LICENSE.md file for details.
 
 
## Sources
We used the LDS of König et al.:
* Repository: https://github.com/TUMFTM/Modular-Quasi-Static-Longitudinal-Simulation-for-BEV
* Paper: Adrian König, Lorenzo Nicoletti et. al. „An Open-Source Modular Quasi-Static Longitudinal Simulation for Full Electric Vehicles,“ in 15th International Conference on Ecological Vehicles and Renewable Energies, Monte-Carlo, Monaco, 2020, pp. 1–9, DOI: 10.1109/EVER48776.2020.9242981.

The following paper describes the development of the tool:

XXXXXXXXXXXXXXXXXXXXXX

