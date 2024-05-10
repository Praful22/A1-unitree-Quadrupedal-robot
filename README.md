# RobotDynamicsAndControl
A repository consisting of a 3D A1 unitree quadrupedal robot performing walking, trotting, running, jumping and a few other tasks using trajectory Optimization and various controls strategies like QP, PD, PID, and Model Predictive Control in Matlab, Simulink, and Simscape.

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


# Links to Videos for Walking (Forward, Backward, Sideways), Trotting, Tunning, Jumping and completing an Obstacle Course.

Obstacle Course Completion: https://youtu.be/T_Iipw_Sg2I?si=g3w5cFUR_AQUpK3_

Forward Motion: https://youtu.be/m1ESYlX8qYk?si=Tl5YOwzOMehc8csa

Backward Motion: https://youtu.be/H7yayTMDM1Q?si=pj_iY75nefFhZa21

Walking Sideways: https://youtu.be/MezQx2boX4I?si=FHt29BcuHELJAGPA

Spinning in Place: https://youtu.be/dvJnvrmFhn8?si=YSv_Gh6Unfd6vPHH

Running Quadrupedal: https://youtu.be/MSF3aIHd8XI?si=5etHWX9mfzWmShqL

Climbing Stairs: https://youtu.be/I3LpZ6ute5A?si=jfkSTEVhXDAUC27q

