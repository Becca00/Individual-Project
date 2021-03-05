function jointAngle = getJointAngles()
    [sim, ID] = simSetup();
    
    % Joint names and handles
    jointNames = {'Sawyer_joint1','Sawyer_joint2','Sawyer_joint3','Sawyer_joint4',...
        'Sawyer_joint5','Sawyer_joint6','Sawyer_joint7'};
    for i = 1:7
        [rJointHandle,joint(i)] = sim.simxGetObjectHandle(ID,jointNames{i}, sim.simx_opmode_blocking);
    end

    for i = 1:7
        [rJointPos(i), jointPos(i)] = sim.simxGetJointPosition(ID,joint(i),sim.simx_opmode_oneshot_wait);
        jointAngle(i) = jointPos(i);
    end
    
end