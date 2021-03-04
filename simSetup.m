function [sim, ID] = simSetup()
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
end