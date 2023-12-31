clear all
close all
clc

%The communication with the ROS server is closed if active
rosshutdown

%A new connection is estabilished with the ROS master
IP_ROS_Master = 'http://stefano-Ubuntu16:11311/'; % Remember to change this link with the YOUR machine

% This initializes the connection with the ROS network
rosinit(IP_ROS_Master)

%Topics will be used during the simulation
sub = rossubscriber('/crazyflie2/odometry_sensor1/odometry'); % contains the drone state
sub2 = rossubscriber('/crazyflie2/motor_speed'); 

% In case you want to specify the type of message is going to be shared
%sub = rossubscriber('/crazyflie2/odometry_sensor1/odometry','nav_msgs/Odometry'); % contains the drone state
%sub2 = rossubscriber('/crazyflie2/motor_speed','mav_msgs/Actuators');  %
[pubCmd, msgCmd] = rospublisher('/crazyflie2/command/motor_speed','mav_msgs/Actuators');

%ROS service employed during the aircraft simulation. In particular, the
%service pause and unpause physic allow to pause and start the simulation,
%respectively
pauseROS = rossvcclient('/gazebo/pause_physics');
unpauseROS = rossvcclient('/gazebo/unpause_physics');
time = rossvcclient('/gazebo/get_world_properties');
physicProperties = rossvcclient('/gazebo/get_physics_properties');

%Information about the simulation
physicPropertiesObject = call(physicProperties);
TimeStepGazebo = physicPropertiesObject.TimeStep;
PauseGazebo = physicPropertiesObject.Pause;
MaxUpdateRateGazebo = physicPropertiesObject.MaxUpdateRate;

%The vechile parameters
maxRotorsVelocity = 3052; %[rad/s]
motorConstant = 1.71465181e-08;
momentConstant = 0.004459273;

% Sample time of the controller
T = 0.01;
P.Ts = 0.01;

% physical parameters of airframe. 
%%% WARNING: Before running the simulation, be sure the parameters used in
%%% Gazebo to run the simulation are the same used in MATLAB. The Crazyflie
%%% parameters are available in 
%%% 
%%% 1. rotors_description/urdf/crazyflie2_forster.xacro
%%% 2. rotors_description/urdf/crazyflie2.xacro
P.gravity = 9.81;   % [m/s/s]
P.mass    = 0.027;   % [kg]
P.Jxx     = 1.657171e-05; % [kg-m2]
P.Jyy     = 1.657171e-05; % [kg-m2]
P.Jzz     = 2.9261652e-05; % [kg-m2]

% The dist from CoM to the center of ea. rotor in the b1-b2 plane
P.d  = 0.046; % [m]

% time constant for dirty derivative filter
P.tau = 0.05;

% Control gains (taken from Lee2011, arXiv:1003.2005v4)
P.kx = 4*P.mass; %4*
P.kv = 4.2*P.mass; %4.2*
P.kR = 8.81/2000; %5000
P.kOmega = 2.54/2000;

%Matrix mapping angular velocities into forces
P.Mix = inv([motorConstant motorConstant motorConstant motorConstant; ...
            -motorConstant*P.d/sqrt(2) -motorConstant*P.d/sqrt(2) motorConstant*P.d/sqrt(2) motorConstant*P.d/sqrt(2); ...
            motorConstant*P.d/sqrt(2) -motorConstant*P.d/sqrt(2) -motorConstant*P.d/sqrt(2) motorConstant*P.d/sqrt(2); ...
            momentConstant -momentConstant momentConstant -momentConstant]);
        
%The step size of the simulation. In other words, the number of steps the
%simulation will run
simulationSteps = 2000;

% Initialization of some variables of interest. The dimension is fixed to
% make the code faster in the execution
positionError = zeros(3,2000); % position error (tracking error)
desiredTrajectory = zeros(3,2000); % desired trajectory (reference signal)
actualTrajectory = zeros(3,2000); % actual drone trajectory

% Cell initialization
messageNumber = 1;
GazeboMessage = cell(simulationSteps, messageNumber);

% The time before the simulation starts. This allows to syncrhonize the
% environments, i.e., MATLAB + Gazebo
serviceTime = call(time);
startTime= serviceTime.SimTime;

% This FOR loops runs for a fixed numbers of iterations
for i = 1 : simulationSteps
    
    %The message from Gazebo
    msgS  = receive(sub);
    %msgSS = receive(subMotor);
    
    % Gazebo has stopped, i.e., it's in a hidle state
    call(pauseROS);
    
    %The simulation time later the simulation stop
    serviceTime = call(time);
    simulationTimeMeasurement = serviceTime.SimTime;
    if i == 1
        timeOffset = simulationTimeMeasurement;
        errorPlot = zeros(2000,1);
    end
    
    %The start and stop time fixing
    start = simulationTimeMeasurement - timeOffset;
    %start = stop;
    stop = start + T;
    
    %Number of steps printed on the screen
    i % This can be commented if it is not need. Just to see the connection works
	    
    %The vector of messages
    GazeboMessage{i,1} = msgS;
    
    %The propellers angular velocities storing
    %This is in the case of the subscriber at line 101
    %motor_velocities_GAZEBO(i,:) = msgSS.AngularVelocities;
    
    %Desired Trajectory
    [xd, b1d] = trajectory2(i);
    
    %Position. Minus are due to the mismatch between the reference system
    %in Gazebo and the one used to design the control law
    x_GAZEBO = msgS.Pose.Pose.Position.X;
    y_GAZEBO = -msgS.Pose.Pose.Position.Y;
    z_GAZEBO = -msgS.Pose.Pose.Position.Z;
    
    x = [x_GAZEBO; y_GAZEBO; z_GAZEBO];
    
    %Linear velocity    
    if i==1
        v = [0;0;0]; % at the beginnig the drone is on the ground, so it can be initialized with zero values
    else
        vx_ABC = msgS.Twist.Twist.Linear.X; % The velocity is retrieved by ROS message but it is in the body frame
        vy_ABC = msgS.Twist.Twist.Linear.Y;
        vz_ABC = msgS.Twist.Twist.Linear.Z;
        
        %Quaternion (they define the aircraft attitude)
        qr_GAZEBO = msgS.Pose.Pose.Orientation.W;
        qi_GAZEBO = msgS.Pose.Pose.Orientation.X;
        qj_GAZEBO = msgS.Pose.Pose.Orientation.Y;
        qk_GAZEBO = msgS.Pose.Pose.Orientation.Z;

        %Compute Rotational Matrix
        R = QuaternionToRotation(qr_GAZEBO,qi_GAZEBO,-qj_GAZEBO,-qk_GAZEBO);
        Rlin = [R(:,1);R(:,2);R(:,3)];
        
        vx = Rlin * vx_ABC;
        vy = Rlin * vx_ABC;
        vz = Rlin * vx_ABC;
        
        v = [vx; vy; vz];
    end

    %The angular veocities of the drone in ABC frame
    p_GAZEBO = msgS.Twist.Twist.Angular.X;
    q_GAZEBO = -msgS.Twist.Twist.Angular.Y;
    r_GAZEBO = -msgS.Twist.Twist.Angular.Z;
    
    Omega = [p_GAZEBO; q_GAZEBO; r_GAZEBO];
    
    %The Controller Algorythm
    uController = [xd; b1d; x; v; Rlin; Omega; T*(i-1)];
    output = ControlloreLee(uController, P); % Forces

    %Forces computed are converted into angular velocities
    angular_velocities = ForcesToAngular(output, P.Mix);
    
    %The new omega values are sent to Gazebo before remove it to pause
    msgCmd.AngularVelocities = [angular_velocities(1),...
        angular_velocities(2), angular_velocities(3), angular_velocities(4)];
    send(pubCmd,msgCmd)
    
    %Until the number of steps is not equal to 500
    if(i == 2000)
        serviceTime = call(time);
        stopSimuationTime = serviceTime.SimTime;
    end
    
    %Gazebo exits from pause
    call(unpauseROS);
    
    %Grandezze monitorate
    desiredTrajectory(:,i) = xd; 
    positionError(:,i) = x - xd;
    actualTrajectory(:,i) = x;
    
    %Previous position is stored to compute velocity
    xPrev = x_GAZEBO; % this can be used in the case the linear velocity is computed as the difference between
    yPrev = y_GAZEBO; % the current drone position and previsou value over the time interval
    zPrev = z_GAZEBO;
    
end
