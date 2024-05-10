# RobotDynamicsAndControl Team 3
Scott Beck, Sungmo Park and Praful Sigdel

# Introduction and Project Structure
In this project, we are asked to design controllers for a number of tasks for the A1 quadruped robot in the MATLAB Simscape environment. To do so, we use a combined approach of Quasi-Linear MPC simulating Single Rigid Body Dynamics (SRBD) of the robot to determine the desired Ground Reaction Forces (GRFs) of the feet of the robot which are on the ground, combined with a swing controller which places the feet which are not on the ground in an opportune location to balance the robot during the next stance phase. The overall design of the controller block (for all tasks except for the obstacle course - which will be covered separately) is shown below.

<img width="732" alt="Screenshot 2024-05-09 at 11 14 06 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/d36edc41-5242-4568-9013-0f177c3f5b63">

Each of these components will be talked about in additional detail in their respective section below. The overall flow of the controller is that an initial state, a desired state trajectory for the robot center of mass, and a gait map are provided by the designer of the motion, depending on the task to be performed. These three values are used by the MPC controller to output ground reaction forces for the feet (set to be equal to zero for those feet not on the ground), and convert those forces into motor torques using the current joint state. The current state and the gait are simultaneously used to generate desired trajectories for the feet that are in the air, and these desired trajectories are subsequently compared to the current foot positions and used to implement a PD controller that tracks the feet along the designed trajectory. The commanded torques from the GRFs and the commanded torques from the swing controller are then summed, and the total torques are then passed through a saturation filter to ensure that no commanded torque exceeds the maximum allowed value of the robot motors.
These torques are fed into a MATLAB Simulink simulation of the full non-linear dynamics of the robot, and the simulation is used to generate new values for the current robot state. These new values then form the basis for the next calculated GRFs and swing controller torques, and the process is repeated until the task is completed.


# Simulink Simulation and State Sensing
The non-linear dynamics simulation is performed in the Simscape environment, derived from the URDF file for the A1 quadruped robot, which includes proper joint angle and motor torque limits for each joint. The torques were provided to each leg block using a 3x1 vector of torques representing the hip yaw, hip pitch, and knee torques, respectively.

## Contact:
For all tasks, spatial contact forces are included between the spherical feet of the robot and the ground plane and other terrain (stairs, obstacles, etc.). For the obstacle course task, spatial contact forces are included between the convex hull of the robot trunk and the terrain. However, in all cases, contact is not explicitly modeled between the legs of the robot and the terrain - in a nominal case, no contact is expected or desired. This choice was made to speed up the simulation, as checking proximity between all components of the robot with more complex terrain tended to reduce the required timestep of the simulation significantly. To ensure that no components were unrealistically clipping through terrain, the output of the simulation was carefully reviewed to ensure no component collided with the terrain other than the feet.

## Sensing: 
To obtain the current state of the system, we require 3 sets of information - first we required the full state vector (position, orientation, linear velocity, angular velocity) of the trunk COM, which is obtained from the 6-DOF joint connecting the world frame to the reference frame of the trunk. The orientation of the body is obtained in quaternion form, and converted into a rotation matrix before being passed to the controller. Second, we require the joint angles and velocities, which are obtained directly from the revolute joints. Finally we require the foot positions and velocities for each of the legs. This could be obtained by using the trunk COM and the joint angles using forward kinematics - and indeed in a real implementation on a physical robot this would be required. However, for simplicity in implementation we implemented a sensor that directly returns the foot position and velocity from the Simulink simulation - we understand that this is unrealistic, as individual legs would not have position sensors returning their state in the world frame, but this allowed us to focus on other tasks in the controller - a FK approach could be implemented as an extension to this project.

## Gait Generation:
The control of the robot starts by determining what the current and future gait of the motion must be - that is, which legs are in “stance” mode, and which are in “swing” mode. In our approach, this gait can be parameterized by 2 scalars – 𝑡𝑐𝑦𝑐𝑙𝑒 and 𝑡𝑠𝑤𝑖𝑛𝑔 – and a 4x1 array of offsets. 𝑡𝑐𝑦𝑐𝑙𝑒 determines the time of the full gait cycle before it repeats, while 𝑡𝑠𝑤𝑖𝑛𝑔 specifies how long the
swing feet spend in the air before returning to the ground. The array of offsets determines how long each foot “waits” into the cycle before beginning its swing phase. Using these 4 parameters, an arbitrary gait cycle can be specified. For example, a common trotting gait – and the one used for most tasks in this project – could be specified by the parameters : 𝑡𝑐𝑦𝑐𝑙𝑒 = 0. 3 𝑠
, 𝑡𝑠𝑤𝑖𝑛𝑔 = 0. 15 𝑠, 𝑜𝑓𝑓𝑠𝑒𝑡𝑠 = [0; 0. 15; 0. 15; 0]. Equivalently, the gait used for stair climbing in
task 3, which moves only one foot at a time, can be parameterized: 𝑡𝑐𝑦𝑐𝑙𝑒 = 0. 36 𝑠,
𝑡𝑠𝑤𝑖𝑛𝑔 = 0. 09 𝑠, 𝑜𝑓𝑓𝑠𝑒𝑡𝑠 = [0; 0. 18; 0. 09; 0. 27]. Flight gaits can also easily be achieved by
ensuring that combination of 𝑡𝑠𝑤𝑖𝑛𝑔 and the offset array results in a period where flight phases for
all 4 feet overlap. Note that results were found to be best when both the cycle time, the swing time, and the offsets were chosen as multiples of 𝑑𝑡𝑀𝑃𝐶, as this prevented the gait prediction
from “phasing” over subsequent runs of the MPC.
Once a gait has been established, a gait map for the current time and the ensuing MPC timesteps can be generated by iterating over each timestep and determining the “mode” of each foot at that time. A graphical representation of the process of generating a gait map, adapted from [1], can be seen in Fig. 2 below.

<img width="854" alt="Screenshot 2024-05-09 at 11 16 43 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/2a2f50b1-c08c-4f7b-a024-196a00d32204">

# Foot Trajectory
In order for the quadruped to move, each of its feet needs to leave the ground for a certain time. To design the foot trajectory while the foot is in mid air, first we need to determine the targeted foot placement for the robot to place in the next stance phase, then design a trajectory for the foot to follow.

Foot Placement Design: Depending on the targeted task the foot placement is generated using a different strategy.

## Walking and Running: 
For the walking and running tasks, each foot placement was determined following the linear inverted pendulum model(LIPM) . LIPM determines the foot placement based on the position of hip placement, feed forward and backward term. Feed forward term(Tstance/2*Vcom) is the half of the COM’s travel distance while the foot is in the air. By determining the foot placement to be placed half the travel distance further from its hip position, the feed forward term pace the foot position along with the Vcom. On the other hand, the feedback term(Kstep*(Vcom-Vd)) brings the foot backward when the Vcom exceeds Vd. In the case of running, an additional constant alpha was because at high velocity, the foot placement could not keep pace with the Vcom.

<img width="461" alt="Screenshot 2024-05-09 at 11 18 54 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/6590db26-5dbd-4c76-aed6-115c44bdaf53">

Rotation: In case of the rotation, Vcom is zero canceling out both feed forward and backward term resulting in Eqn 3.

<img width="425" alt="Screenshot 2024-05-09 at 11 19 39 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/908c4960-c1d1-4c5c-9b1c-98de185e571c">

In both of the above cases, after 𝑃𝑓𝑜𝑜𝑡 is determined using Eqns 1, 2, or 3, the z location of the foot is projected down to 0, in order to ensure that the foot will be placed on solid ground and be
able to start the next stance phase.

## Stair Climbing: 
In stair climbing, the prediction of the next desired foot position functions in exactly the same manner as described above, using Equation 1 for foot position, but differs from the walking and running motions in that rather than projecting the z location of the foot down to 0, instead the predicted X and Y locations are compared to the “known” locations of the steps of the stairs, and the corresponding z value of the stair (if any) that exists at that location is set as the z value of the foot. This approach relies on pre-knowledge of the stairs, which in a real robot would have to be either hard-coded into its memory, or else generated in situ using computer vision of some variety. Alternatively, a contact detection algorithm could be used to estimate when the foot is in contact with the ground and update the contact map accordingly. However, this method was beyond the scope of what we were able to achieve in this project.

Additionally, in the stair climbing task, a “deadband” was implemented in proximity to the edge of the stair. If the above equations resulted in a desired foot position that was “too close” to the edge of the stair (and would likely either result either in tripping - if too close to the bottom of the stair, or in sliding off the stair - if too close to the top of the stair) then the foot desired foot position was modified to a predetermined distance from the bottom or top of the stair, depending on the commanded x value of the desired location.

## Foot Trajectory Generation: 
For walking and running, the foot trajectory design aims to smoothly transition between ground contact and swing phases. The ‘footTrajectory’ function generates trajectories for each foot based on initial, final positions, and number of interpolation points between them. By utilizing De Castlejau’s algorithm to evaluate a polynomial in Bezier curve, the trajectory ensures natural motion patterns. When traversing stairs, in addition to the trajectory generation for walking, the foot trajectory adapts to varying step heights. By analyzing the difference in elevation between consecutive steps and adjusting the height of the control points accordingly, the function dynamically adjusts the trajectory's peak height to ensure the foot can reach over the upcoming step. This ensures smooth transitions between steps whether staying on the same step or moving up one step, minimizing the risk of tripping or loss of balance. In addition to generating the position trajectory, the velocity trajectory was derived by taking numerical differentiation to the position trajectory.

## Controller Design
Our approach to control for these tasks falls into two categories: For tasks 1 through 3, the same controller controls the robot throughout the whole motion - this is a “walking” controller that commands a constant desired velocity in a given direction to the MPC controller (with a ramp-up and slow-down period at the beginning and end of the motion to prevent slipping when commanding higher speeds), and converts the resultant GRFs to joint torques. There are slight differences when applying this controller to stair climbing, which will be discussed, but broadly the same control approach applies. This is in contrast to the obstacle course in task 4, which changes between this walking controller and a “jumping” controller, which itself employs switching between MPC control for the jumping motion and a pure PD joint controller for pre-flight and pre-landing posing. This jumping controller will be discussed in a separate section.
Walking Controller Design

## MPC Controller Design: 
The MPC controller takes in the desired state trajectory over the time horizon of the MPC controller (𝑑𝑡𝑀𝑃𝐶 = 0. 03 𝑠, 𝑁𝑀𝑃𝐶 = 10, for all MPC in this project), the MPC
weighting matrices Q and R (tuned by trial and error for each task), the robot mass and moment of inertia, 𝑚 and 𝐼𝑏, and the aforementioned state and foot location information output from the
Simulink simulation. These inputs are used to formulate the controller in the form of a QP controller that can be solved using MATLAB’s quadprog function. The flowchart in Fig. 3 depicts how these inputs map to each of the arguments of the quadprog function.

<img width="595" alt="Screenshot 2024-05-09 at 11 22 54 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/ffb74b71-9fe5-40f3-8913-23de53b58fe6">

The controller itself implements a SRBD formulation of the robot dynamics, assuming all mass and moment of inertia is concentrated at the COM of the trunk. This formulation hews very closely to that presented in [2], though the small angle approximation applied in Eq. 11 of that text is not assumed in this implementation – rather, the full transformation from angular velocity vector to Euler angle rates described in Eq. 10 is used. Further assumptions include:
1. The MPC controller is run at an interval of 0. 03 𝑠, equal to 𝑑𝑡𝑀𝑃𝐶
2. Within the MPC controller, the foot positions are assumed constant at their current position throughout the MPC horizon (though the gait, of course, varies throughout these timesteps depending on the commanded gait map). These positions are obtained from Simulink, and then projected to the appropriate z location corresponding to the “ground” where the (x,y) location places it. (i.e. the z value of the stairs/obstacle on which the leg is standing, or 0 if on the ground)
3. A constraint is placed on the magnitude of of the GRFs output from the MPC controller ( 𝐹𝑚𝑎𝑥 = 500 𝑁, 𝐹𝑚𝑖𝑛 = 10 𝑁), in order to limit slipping and discourage saturation of the
motor torques, but no explicit limit is placed on motor torques in this formulation. Rather, after forces have been converted to torques via the above described method, a separate block clips the commanded torques to be within the torque limit of ||𝑇𝑚𝑎𝑥|| ≤ 33. 5 𝑁𝑚
(given in HW3), to ensure that this constraint is met.
The MPC controller performs identically during the stair climbing task, with the exception as noted above in assumption 2 that rather than projecting the foot position down to 0, the foot position is projected down to the relevant stair height.
Force to Torque Mapping: Since the MPC controller generates GRFs in the world frame, these first need to be converted to body frame and then converted into joint torques by using the Jacobian. Finally, since the GRFs reported by MPC represent forces exerted by the ground on the robot, and we require the force exerted by the robot end effector on the ground, the entire term is multiplied by -1. Eqn 4 below summarizes this process.

<img width="402" alt="Screenshot 2024-05-09 at 11 24 37 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/c9e062b5-5188-458e-ae18-6e7ee0c89446">

## Swing Controller Design:
In contrast, when the foot is in the air, the foot simply needs to follow the trajectory created by the foot trajectory block. To achieve this, the swing controller design implements a PD controller based on the swing controller described in [2]. Given the desired foot trajectory discussed above, values for 𝑝𝑓𝑜𝑜𝑡, 𝑑𝑒𝑠𝑖𝑟𝑒𝑑 and 𝑣𝑓𝑜𝑜𝑡, 𝑑𝑒𝑠𝑖𝑟𝑒𝑑 are established by
interpolation using the grid points generated by the ‘footTrajectory’ function and the current time 𝑡𝑠𝑖𝑚. These positions are generated in the world frame, to match with the foot positions reported
from the simulation. Then, desired torques are generated using Eqn 5, below.
<img width="505" alt="Screenshot 2024-05-09 at 11 25 26 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/e511f4ed-51e4-4878-a8ef-be4545f34626">
Where 𝐽𝑖 is the Jacobian of the 𝑖𝑡h leg, evaluated at the current state, 𝑅 is the rotation matrix representing the transformation between world and body frames, and 𝐾𝑝 and 𝐾𝑑 are the PD gains, chosen to be 𝐾𝑝 = 300 and 𝐾𝑑 = 10 for this application. Note that compared to Eq 1 in
[2], this equation neglects a feed-forward torque term (which considers the mass moment inertia of the leg and effect of gravitational acceleration), as it is assumed to be small compared to the feedback term. Additionally, the rotation matrix is needed in our case because our feet positions are reported in the world frame, in contrast to the case in [2], wherein the foot positions are in the body frame
This equation is, in essence, a feedback controller that generates a desired force to apply at the foot (the end effector) in the world frame. The rotation matrix then converts that force to a force in the body frame, at which point the Jacobian is used to calculate the joint torques required to produce the desired force at the end effector.

## Jumping Controller Design
Given the constraints of limited time and task simplicity, a straightforward model of SRBD trajectory planning was utilized, acknowledging the efficacy of trajectory optimization in more complex scenarios. The SRBD trajectory planning treats the jumping of the robot as a SRBD leaving the ground with an initial velocity. The trajectory planning is composed of 4 phases which are crouching, acceleration, mid air, impact and stabilizing as shown in the figure below.

<img width="575" alt="Screenshot 2024-05-09 at 11 26 15 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/64f3b4b8-2502-41f0-b445-10c3dd438bfc">

### I. Crouching: 
The crouching phase is proposed to maximize the time for each joint to accelerate the COM and pitch the body about 10 degrees. The pitch of the body increases normal force applied by the rear legs allowing the rear legs to create higher acceleration without slipping. The configuration was found by trial and error, then applied to the model using PD control.
### II. Acceleration: 
The goal of the acceleration phase is to achieve the desired Vcom before all feet leave the ground. By inputting the desired Vcom to the MPC controller, the torque required at each time step can be calculated. To ensure the front legs leave the ground before the rear leg does, gait map is adjusted accordingly.
### III. Mid Air: 
While in the air and for a short period of time after landing the quadruped changes its configuration to prepare for the impact. The impact configuration was designed so that the robot will extend its legs to the direction of impact so that it has more time to absorb the impact. In addition, the configuration of the impact was designed so that the front feet touch the ground first so that the torque created by COM cancels out the pitch rate of the COM.
### IV.Landing + Standing: 
After the impact phase ends, the PD control adjusts the configuration of the robot to the standing position so that it can progress to the next obstacle course.

## Task Results and Corresponding Score Breakdown:
#### Task 1
- Walking: All requirements for Task 1 were successfully completed, with each specified motion being performed at ≥ 0.5 m/s or ≥ 0.5 rad/s, as specified. Plots of the relevant velocity (linear or angular) vs. time are shown in Figures 5 through 8 for each task. Video of each motion is available in the linked folder.
Score: All requirements achieved - yielding 35 points.

<img width="1238" alt="Screenshot 2024-05-09 at 11 28 34 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/b3d4244f-9f5e-46a8-a8ef-1a66acea6c74">

Forward Motion: https://youtu.be/m1ESYlX8qYk?si=Tl5YOwzOMehc8csa

Backward Motion: https://youtu.be/H7yayTMDM1Q?si=pj_iY75nefFhZa21

Walking Sideways: https://youtu.be/MezQx2boX4I?si=FHt29BcuHELJAGPA

Spinning in Place: https://youtu.be/dvJnvrmFhn8?si=YSv_Gh6Unfd6vPHH

#### Task 2 
- Running: The 10 m goal was reached in a time of 4.75s, as shown in Fig. 9, with an average velocity of 2.5, after ramp-up. This was achieved with a trotting gait with a short flight phase. A video of the motion, as well as a close-up slowed-down video showing the flight phase is included in the linked folder.
Score: Per rubric - 200 / travel time(s) = 200 / 4.75 (s) = 42.1 points
<img width="1390" alt="Screenshot 2024-05-09 at 11 29 16 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/c3c55b65-e748-4210-9bb3-e1a88c258dd6">

Running Quadrupedal: https://youtu.be/MSF3aIHd8XI?si=5etHWX9mfzWmShqL


#### Task 3 
- Stair Climbing: The stair climbing goal was accomplished in 4.61 s, as measured as the time at which the last rear leg touched the ground on the top stair (measured from video). A video of the motion is included in the linked folder.
Score: Per rubric - 20/travel time(s) = 200 / 4.61 (s) = 4.3 points
<img width="1141" alt="Screenshot 2024-05-09 at 11 30 18 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/b5eb9a0d-11d2-45c8-a10b-5fa845557a28">

Climbing Stairs: https://youtu.be/I3LpZ6ute5A?si=jfkSTEVhXDAUC27q

#### Task 4
- Obstacle Course: The obstacle course was completed in a time of 16.34s, as measured as the time at which the trunk COM passed 8.0 m. The final obstacle overcome was set at 0.5 m tall. A video of the motion is included in the linked folder.
Score: Per rubric - 100/ travel time(s) + 100 x (h_obs - 0.2)
= 100/ 16.34 (s) + 100 x (0.5 - 0.2) = 36.1 points\

Obstacle Course Completion: https://youtu.be/T_Iipw_Sg2I?si=g3w5cFUR_AQUpK3_

<img width="967" alt="Screenshot 2024-05-09 at 11 32 14 PM" src="https://github.com/Praful22/A1-unitree-Quadrupedal-robot/assets/65821250/d57a57f6-f098-40e1-bad0-e5acddb70ba7">

Sources:
[1] G. Bledt, P. M. Wensing, S. Ingersoll and S. Kim, "Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains," 2018 IEEE International Conference on Robotics and Automation (ICRA), Brisbane, QLD, Australia, 2018, pp. 4399-4406, doi: 10.1109/ICRA.2018.8460904.
[2] Carlo, Jared & Wensing, Patrick & Katz, Benjamin & Bledt, Gerardo & Kim, Sangbae. (2018). Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control. 1-9. 10.1109/IROS.2018.8594448.




