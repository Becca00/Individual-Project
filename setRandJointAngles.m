function setRandJointAngle = setRandJointAngles()
    [sim, ID] = simSetup();
    
    % Joint names and handles
    jointNames = {'Sawyer_joint1','Sawyer_joint2','Sawyer_joint3','Sawyer_joint4',...
        'Sawyer_joint5','Sawyer_joint6','Sawyer_joint7'};
    for i = 1:7
        [rJointHandle,joint(i)] = sim.simxGetObjectHandle(ID,jointNames{i}, sim.simx_opmode_blocking);
    end
    
    % Set random joint position for start around upright position
    jointPos = [0.1,-pi/2,0.1,0.1,0.1,0.1,0.1] .* (2*pi * (rand-0.5)/100 + 1);
    
	for i=1:7
        sim.simxSetJointTargetPosition(ID,joint(i),jointPos(i),sim.simx_opmode_streaming);              
    end 
    
end