clear all
close all 
clc

sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
ID = sim.simxStart('127.0.0.1',19997,true,true,5000,5);

if ID < 0
    disp('Failed connecting to remote API server. Exiting.');
    sim.delete();
    return;
else
    disp('Connected to remote API server.');
end

% get target handle
[rTargetHandle, target]=sim.simxGetObjectHandle(ID,'Target', sim.simx_opmode_oneshot_wait);

% joint names and handles
jointNames = {'Sawyer_joint1','Sawyer_joint2','Sawyer_joint3','Sawyer_joint4',...
    'Sawyer_joint5','Sawyer_joint6','Sawyer_joint7'};
for i = 1:7
    [rJointHandle,joint(i)] = sim.simxGetObjectHandle(ID,jointNames{i}, sim.simx_opmode_blocking);
end

% Gripper handles
[rLGripper, leftGripper] = sim.simxGetObjectHandle(ID,'BaxterGripper_leftFingerPad_connection',...
    sim.simx_opmode_blocking);
[rRGripper, rightGripper] = sim.simxGetObjectHandle(ID,'BaxterGripper_rightFingerPad_connection',...
    sim.simx_opmode_blocking);
tic
if ID > -1 % while connected to the server
    
%     Option to change code to work for specified time 
%     t = clock;
%     startTime = t(6);
%     currentTime = t(6);
%     while (currentTime - startTime < 5)
    
    % Stop and start the simulation
    rStop = sim.simxStopSimulation(ID,sim.simx_opmode_oneshot);
    pause(0.5)
    rStart = sim.simxStartSimulation(ID,sim.simx_opmode_oneshot);
    pause(0.5)
    
    % Initialise streaming
    sim.simxReadForceSensor(ID,leftGripper,sim.simx_opmode_streaming); 
    sim.simxReadForceSensor(ID,rightGripper,sim.simx_opmode_streaming); 
    sim.simxGetObjectPosition(ID,target, -1, sim.simx_opmode_streaming);
    sim.simxGetObjectOrientation(ID,target, -1, sim.simx_opmode_streaming);
    
    % Force before movement 
    [rLeft,lstate,lforce,ltorque] = sim.simxReadForceSensor(ID,leftGripper,...
            sim.simx_opmode_buffer); % Retrieve streamed data
    
    % Set the position of the target for the gripper to follow  
    targetPosition(ID, sim, target, 0.88, 0.17, 0.1);

    % Detect button by checking force while arm moves
    detectButton(ID, sim, leftGripper);

    % Get position of target (expected gripper position)
    [rPos, targetPos] = sim.simxGetObjectPosition(ID,target, -1, sim.simx_opmode_oneshot);
    [rOrient, objectEuler] = sim.simxGetObjectOrientation(ID,target, -1, sim.simx_opmode_oneshot);
    
    % Get joint positions of robot
    for i = 1:7
        [rJointPos(i), jointPos(i)] = sim.simxGetJointPosition(ID,joint(i),sim.simx_opmode_oneshot_wait);
    end

%     t=clock;
%     currentTime=t(6);  
end

% Return robot to starting position and stop simulation
rStart = sim.simxStartSimulation(ID,sim.simx_opmode_oneshot);
pause(0.5)
rStop = sim.simxStopSimulation(ID,sim.simx_opmode_oneshot);
pause(0.5)
sim.simxRemoveObject(ID,target,sim.simx_opmode_oneshot);
sim.simxFinish(ID);
sim.delete();

% Obtain plot of time against force data
forceMat = cleanTable();

disp('Program ended');