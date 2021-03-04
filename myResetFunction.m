function [InitialObservation, LoggedSignal] = myResetFunction()
% Reset function to place Sawyer robot into a random initial state

% Connect to CoppeliaSim
[sim, ID] = simSetup();

% Retrieve the 7 joint angles 
jointAngle = getJointAngles;

% Return initial environment state variables as logged signals
LoggedSignal.State = jointAngle;
InitialObservation = LoggedSignal.State;

end