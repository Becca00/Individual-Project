% Set the position of the target for the robot gripper to follow
% Requires ID, sim and target inputs
% x, y, z are the coordinates of within the simulation
function targetPosition(ID, sim, target, x, y, z)
    % Oneshot (non-blocking) operation mode is used so that measuring the force can occur at same time as movement
    rQuaternion = sim.simxSetObjectQuaternion(ID, target, -1, [0,0,0,0], sim.simx_opmode_oneshot);
    rPosition = sim.simxSetObjectPosition(ID, target, -1, [x y z], sim.simx_opmode_oneshot);
end

