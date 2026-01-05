clear; close all; clc;

% Initialize project paths
setup();

sim = remApi('remoteApi');      % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1);             % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
sim.simxSynchronous(clientID, true);
simClient    = struct('sim', sim, ...
                      'clientID', clientID);

robot_length = struct('base_length',    0.50,  ...
                      'base_width',     0.20,  ...
                      'L1',             0.05,  ...
                      'L2',             0.15,  ...
                      'L3',             0.10);
robot_config = struct('leg_type',       "  ",  ...
                      'joint_angle',    "  ",  ...
                      'length', robot_length);
robot_motion = struct('gait',           "  ",  ...
                      'step',           0.00);
robot        = struct('config', robot_config,  ...
                      'motion', robot_motion);

robot.motion.gait = "ZERO";
control_gait(robot, simClient);
if (clientID>-1)
    disp('Connected to remote API server');  
    while true
        % pause(3);
        % robot.motion.gait = "WALK";
        % robot.motion.step = 10;
        % control_gait(robot, simClient);  
        robot.motion.gait = "FORWARD";
        robot.motion.step = 10;
        control_gait(robot, simClient); 
        robot.motion.gait = "TURN_RIGHT";
        robot.motion.step = 2;
        control_gait(robot, simClient);  
        robot.motion.gait = "FORWARD";
        robot.motion.step = 10;
        control_gait(robot, simClient);  
        robot.motion.gait = "TURN_RIGHT";
        robot.motion.step = 5;
        control_gait(robot, simClient);
        robot.motion.gait = "FORWARD";
        robot.motion.step = 20;
        control_gait(robot, simClient);  
        robot.motion.gait = "TURN_LEFT";
        robot.motion.step = 3;
        control_gait(robot, simClient);
    end 
else
    disp('Failed connecting to remote API server');
end
    sim.delete(); % call the destructor!
    
    disp('Program ended');