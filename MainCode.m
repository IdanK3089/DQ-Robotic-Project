% Vrep_Following
clear all;
close all;
clc;

%% initial
include_namespace_dq;
% False if we don't want to show intermediate frames
simulation_parameters.show_frames = false;
% Total simulation time, in seconds.
simulation_parameters.total_time = 50;

vi = DQ_VrepInterface; % CLASS DQ_VrepInterface - Communicate with V-REP using dual quaternions
vi.disconnect_all(); % Stop any connected simulations
vi.connect('127.0.0.1',19997); % Connect to Coppeliasim (V-REP)
vi.start_simulation(); % Start simulation in Coppeliasim (V-REP)

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
% vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% LBR4p Arm
lwr4_vreprobot = LBR4pVrepRobot('LBR4p',vi);
lwr4  = lwr4_vreprobot.kinematics();


%% GeneralRobot is our rewrite of the robot defining class so it can work with any arbitary robot.
%% The "kinematics" function was also rewritten to the generic form.

HolderRobot = GeneralRobot('Robot',vi,2);

HolderKin = HolderRobot.kinematics(2,[0, 0],[0.2, 0],[0, 0],[pi/2, -pi/2]);

gripper (clientID,0);  pause(1.5);

%% Initialize controllers
solver = DQ_QuadprogSolver; % Interface to Quadratic Program Solvers

%% Holder Robot Controller

Holder_control = DQ_PseudoinverseController(HolderKin);
Holder_control.set_control_objective(ControlObjective.Pose);



Holder_control.set_gain(10);
Holder_control.set_damping(0.1);
Holder_control.set_stability_threshold(0.0001);



lwr4_controller = DQ_ClassicQPController(lwr4,solver);
lwr4_controller.set_control_objective(ControlObjective.Pose);


lwr4_controller.set_gain(20);
lwr4_controller.set_damping(0.1);
lwr4_controller.set_stability_threshold(0.00001);

solver = DQ_QuadprogSolver; % Interface to Quadratic Program Solvers

%% Gripper
[r,h_tip]=vrep.simxGetObjectHandle(clientID,'Tip',vrep.simx_opmode_blocking);
[r,h_Cuboid1]=vrep.simxGetObjectHandle(clientID,'Cuboid1',vrep.simx_opmode_blocking);

%% Movment

pose_cube1 = vi.get_object_pose('Cuboid1');
pose_Tip = vi.get_object_pose('LBR4p_connection');
lwr4_q = lwr4_vreprobot.get_q_from_vrep();

r = cos(-pi/2) + j_*sin(-pi/2);
p = translation(pose_cube1) + k_*0.25;
lwr4_xd = r+ E_*0.5*p*r;

T = 0.01;
lwr4_controller.set_control_objective(ControlObjective.Pose);

time = 0;

HRobotQ  = HolderRobot.get_q_from_vrep();

while ~lwr4_controller.system_reached_stable_region() && ~Holder_control.system_reached_stable_region()
   
  r = cos(pi/2) + j_*sin(pi/2);
    p = 0.1*i_ + 0.2*j_ + 0.3*k_;
    xd = (r + E_*0.5*p*r)*sin(time);
 
 u = Holder_control.compute_setpoint_control_signal(HRobotQ,vec8(xd));
   HRobotQ = HRobotQ + T*u;
 
  HolderRobot.send_q_to_vrep(HRobotQ);

  time = time + 0.001;

    r = cos(-pi/2) + j_*sin(-pi/2);
    pose_cube1 = vi.get_object_pose('Cuboid1');
    p = translation(pose_cube1) + k_*0.15;
    lwr4_xd = r+ E_*0.5*p*r;
    lwr4_u  = lwr4_controller.compute_setpoint_control_signal(lwr4_q,vec8(lwr4_xd));
    lwr4_q = lwr4_q + T*lwr4_u ;
    lwr4_vreprobot.send_q_to_vrep(lwr4_q);
end


lwr4_controller.set_gain(10);
lwr4_controller.set_damping(0.1);
lwr4_controller.set_stability_threshold(0.00001);



lwr4_controller.set_control_objective(ControlObjective.Pose);


while ~lwr4_controller.system_reached_stable_region() 
   
    r = cos(-pi/2) + j_*sin(-pi/2);
    pose_cube1 = vi.get_object_pose('Cuboid1');
    p = translation(pose_cube1) + k_*0.088;
    lwr4_xd = r+ E_*0.5*p*r;
    lwr4_u  = lwr4_controller.compute_setpoint_control_signal(lwr4_q,vec8(lwr4_xd));
    lwr4_q = lwr4_q + T*lwr4_u ;
    lwr4_vreprobot.send_q_to_vrep(lwr4_q);
end
gripper (clientID,1);  pause(1.5);

lwr4_controller.set_control_objective(ControlObjective.Pose);


lwr4_controller.set_gain(20);
lwr4_controller.set_damping(0.1);

q = lwr4.fkm(lwr4_q);
N = 1000;
theta = linspace(0,pi,N);
R = 0.5;
ii=1;
while ~lwr4_controller.system_reached_stable_region() 
    
    r = cos(-pi/2) + j_*sin(-pi/2);
    pose_cube1 = vi.get_object_pose('Cuboid1');
    p = -i_*R*sin(theta(ii)) + j_*R*cos(theta(ii)) + k_*0.675;
    ii=ii+1;
    if ii>=N
        ii=N;
        p = -i_*0.25 - j_*0.45 + k_*0.475;
    end
    lwr4_xd = r+ E_*0.5*p*r;

    lwr4_u  = lwr4_controller.compute_setpoint_control_signal(lwr4_q,vec8(lwr4_xd));
    lwr4_q = lwr4_q + T*0.1*lwr4_u ;
    lwr4_vreprobot.send_q_to_vrep(lwr4_q);
end

gripper (clientID,0);  pause(1.5);

%% End V-REP
vi.stop_simulation();
vi.disconnect();

%% Test function, was not used in simulation
function line = get_line_from_vrep(vrep_interface,object_name,primitive)
line_object_pose = vrep_interface.get_object_pose(object_name);
p = translation(line_object_pose);
r = rotation(line_object_pose);
l = Ad(r,primitive);
m = cross(p,l);
line = l + DQ.E*m;
end


%% Test function, was not used in simulation
function [Jconstraint, bconstraint] = compute_constraints(lwr4, lwr4_q, cylinder1)
% We define differential inequalities given by d_dot >= -eta*d,
% where d is the distance from the end-effector to the
% primitive, d_dot is its time derivative and eta determines the
% maximum rate for the approach velocity.

% This information should be inside the robot itself
robot_radius = 0.2;
radius_cylinder1 = 0.1;

include_namespace_dq

% Get base translation and translation Jacobian
q = lwr4.fkm(lwr4_q);
Jx = lwr4.pose_jacobian(lwr4_q);
t = translation(q);
Jt = [lwr4.translation_jacobian(Jx,q)];

% Now we compute the primitives for the two cylinders
Jdist_cylinder1 = lwr4.point_to_line_distance_jacobian(Jt, t, cylinder1);
dist_cylinder1 = DQ_Geometry.point_to_line_squared_distance(t,cylinder1) - ...
    (radius_cylinder1 + robot_radius)^2;

% Assemble all constraint matrices into a unique matrix
Jconstraint = Jdist_cylinder1;
% And do analagously to the constraint vectors
bconstraint = dist_cylinder1;
end
