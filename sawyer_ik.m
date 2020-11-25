clc
clear all
close all

sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
ID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);

    
if ID < 0,
    disp('Failed connecting to remote API server. Exiting.');
    sim.delete();
    return;
else
    disp('Connected to remote API server.');
end

% joint handles
[r,h(1)]=sim.simxGetObjectHandle(ID,'Sawyer_joint1',sim.simx_opmode_blocking);
[r,h(2)]=sim.simxGetObjectHandle(ID,'Sawyer_joint2',sim.simx_opmode_blocking);
[r,h(3)]=sim.simxGetObjectHandle(ID,'Sawyer_joint3',sim.simx_opmode_blocking);
[r,h(4)]=sim.simxGetObjectHandle(ID,'Sawyer_joint4',sim.simx_opmode_blocking);
[r,h(5)]=sim.simxGetObjectHandle(ID,'Sawyer_joint5',sim.simx_opmode_blocking);
[r,h(6)]=sim.simxGetObjectHandle(ID,'Sawyer_joint6',sim.simx_opmode_blocking);
[r,h(7)]=sim.simxGetObjectHandle(ID,'Sawyer_joint7',sim.simx_opmode_blocking);
            

r=sim.simxStopSimulation(ID,sim.simx_opmode_oneshot);
pause(1)
r=sim.simxStartSimulation(ID,sim.simx_opmode_oneshot);
pause(0.5)            

for i = 1:7
r = sim.simxGetJointPosition(ID,h(i),sim.simx_opmode_streaming);
end
     
tic % start simulation timer

%while (toc < 20)

% get target handle
[returnCode2,Target]=sim.simxGetObjectHandle(ID,'Target',sim.simx_opmode_oneshot_wait); 

%[returnCode]=vrep.simxSetObjectQuaternion(ID,hTarget,-1,[0,0,0,0],vrep.simx_opmode_oneshot_wait);
%[returnCode]=vrep.simxSetObjectPosition(ID,hTarget,-1,[0 0 1],vrep.simx_opmode_oneshot_wait);
%pause(2)
%[returnCode]=vrep.simxSetObjectQuaternion(ID,hTarget,-1,[0,0,0,0],vrep.simx_opmode_oneshot_wait);
%[returnCode]=vrep.simxSetObjectPosition(ID,hTarget,-1,[1 0.15 0.5],vrep.simx_opmode_oneshot_wait);
%pause(2)
%[returnCode]=vrep.simxSetObjectQuaternion(ID,hTarget,-1,[0,0,0,0],vrep.simx_opmode_oneshot_wait);
%[returnCode]=vrep.simxSetObjectPosition(ID,hTarget,-1,[0.02 0.2 0.8],vrep.simx_opmode_oneshot_wait);
%pause(2)
%[returnCode]=vrep.simxSetObjectQuaternion(ID,hTarget,-1,[0,0,0,0],vrep.simx_opmode_oneshot_wait);
%[returnCode]=vrep.simxSetObjectPosition(ID,hTarget,-1,[0.88 0.17 0.5],vrep.simx_opmode_oneshot_wait);
%pause(0.5)
[returnCode]=sim.simxSetObjectQuaternion(ID,Target,-1,[0,0,0,0],sim.simx_opmode_oneshot_wait);
[returnCode]=sim.simxSetObjectPosition(ID,Target,-1,[0.88 0.17 0.1],sim.simx_opmode_oneshot_wait);
pause(2)


leftGripperForce = sim.simxGetObjectHandle(ID,'BaxterGripper_leftFingerPad_connection',sim.simx_opmode_blocking);
[rForce,s,f,t] = sim.simxReadForceSensor(ID,leftGripperForce,sim.simx_opmode_buffer);



%while (rForce == 1) % restart simulation if force detected
%    disp('restarting');
%    r=sim.simxStopSimulation(ID,sim.simx_opmode_oneshot); 
%    pause(1)
%    r=sim.simxStartSimulation(ID,sim.simx_opmode_oneshot);
%    pause(1)   
    
%    [returnCode]=sim.simxSetObjectQuaternion(ID,Target,-1,[0,0,0,0],sim.simx_opmode_oneshot_wait);
%    [returnCode]=sim.simxSetObjectPosition(ID,Target,-1,[0 0 1],sim.simx_opmode_oneshot_wait);
%    pause(2)
%end

%end

if (toc >= 20)  %end simulation after 20 seconds
    sim.simxStopSimulation(ID,sim.simx_opmode_oneshot);
end

[r, objectPos] = sim.simxGetObjectPosition(ID,Target, -1, sim.simx_opmode_oneshot_wait);
[r, objectEuler] = sim.simxGetObjectOrientation(ID,Target, -1, sim.simx_opmode_oneshot_wait);
objectPos
objectEuler

for i=1:7
    [returnCode1(i),end_position(i)] = sim.simxGetJointPosition(ID,h(i),sim.simx_opmode_streaming);
end

toc

end_position 

%r=sim.simxStartSimulation(ID,sim.simx_opmode_oneshot);
%pause(1)

sim.simxRemoveObject(ID,Target,sim.simx_opmode_oneshot);
sim.simxFinish(ID);
sim.delete(); 
disp('Program ended');
