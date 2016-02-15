Read Me
Author: Kerianne Gross, kerianne.gross@us.af.mil
Updated: 23 November 2015

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Quick Start: 

Run Simulation (you don't need Simulink Design Verifier for this)
1. Add the entire folder to your path, by right clicking on the folder, selecting "Add to 
Path" and then selecting "All folders and subfolders."
2. Run the Constants file, by opening the Constants.m and hitting run. (This initiates all 
your variables, including the initial and final time of the simulation).
3. Open Simulation_RTA.slx and run
4. If desired run "Plotter.m" after the simulation completes to plot the state over time. 
(Once you run this, you have to run "Constants.m" again before running the simulation 
again.

Proving Properties (requires Simulink Design Verifier)
1. Add the entire folder to your path, by right clicking on the folder, selecting "Add to 
Path" and then selecting "All folders and subfolders."
2. Run the Constants file, by opening the Constants.m and hitting run. (This initiates all 
your variables, including the initial and final time of the simulation).
3. Open one of the "Verification" files
4. Run the verification file to ensure everything loaded properly; fix any issues.
5. Analyze the system by going to the main menu at the top (where File and Edit are), and 
selection "Analysis" -> Design Verifier -> Prove Properties -> Model

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Notes on this file package:

1. This model was created in Matlab 2015a.

2. The Simulation_RTA uses the rate limited PID controller as the verified controller, the 
ramp function as the unverified controller, and the decision module with history for the 
decision module

2.1. The PID controller without rate limiting functions the same as the PID controller with 
rate limiting in this simulaiton; however, Simulink Design Verifier cannot prove that it
will never violate reaction wheel velocity or acceleration properties under some 
circumstances.

2.2. Decision Module (without history): If a decision module is implemented which switches 
back as soon as the saftey properties are no longer violated, then this example will never 
settle. In the example, a ramp function essentially commands a constant acceleration 
that is within limits. Eventually this constant acceleration causes the maximum velocity 
to be reached. Once the maximum velocity of the reaction wheels is reached, the velocity 
safety property is violated and the controller switches to the verified PID controller. 
Without a history that checks if the system has been previously violated, as soon as the 
PID controller brings the velocity back down below the maximum, the decision module 
switches control right back to the unverified controller which commands a constant 
acceleration that causes the maximum velocity to be reached again, causing the decision 
module to switch control back to the verified controller. If violation History is not 
included, the decision module will switch back and forth between the the two controllers 
and never settle on the final desired state.

3. PID Control Verification with Simulink Design Verifier: If you try to prove all the PID 
controller properies at once, you will get "undecided due to nonlinearies" for one or more 
of the properties. You can enable/disabe properties by double clicking on the property and 
selecting or deselecting the checkbox next to "enable." If you disable all but the 
properties that were previously "undecided due to nonlinearies," these properties will 
prove. It's a software bug that will probably be fixed soon.



