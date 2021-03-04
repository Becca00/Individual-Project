function [NextObs,Reward,IsDone,LoggedSignals] = myStepFunction(Action,LoggedSignals)
% This function applies the given action to the environment and evaluates
% the system dynamics for one simulation step.

% Define the environment constants

% Sample time
Ts = 0.1;

% End effector height at which to fail the episode - 2cm from ground
EffectorHeight = 0.02;

% Reward for hitting button
ButtonReward = 50;

% Penalty each time step the robot has not hit the button
TimePenalty = -1;

Force = Action;

% Unpack the state vector from the logged signals.
State = LoggedSignals.State;
Joint1 = State(1);
Joint2 = State(2);
Joint3 = State(3);
Joint4 = State(4);
Joint5 = State(5);
Joint6 = State(6);
Joint7 = State(7);

% Cache to avoid recomputation.
CosTheta = cos(Theta);
SinTheta = sin(Theta);
SystemMass = CartMass + PoleMass;
temp = (Force + PoleMass*HalfPoleLength*ThetaDot*ThetaDot*SinTheta)/SystemMass;

% Apply motion equations.
ThetaDotDot = (Gravity*SinTheta - CosTheta*temp) / ...
    (HalfPoleLength*(4.0/3.0 - PoleMass*CosTheta*CosTheta/SystemMass));
XDotDot  = temp - PoleMass*HalfPoleLength*ThetaDotDot*CosTheta/SystemMass;

% Perform Euler integration.
LoggedSignals.State = State + Ts.*[XDot;XDotDot;ThetaDot;ThetaDotDot];

% Transform state to observation.
NextObs = LoggedSignals.State;

% Check terminal condition.
X = NextObs(1);
Theta = NextObs(3);
IsDone = abs(X) > DisplacementThreshold || abs(Theta) > AngleThreshold;

% Get reward.
if ~IsDone
    Reward = RewardForNotFalling;
else
    Reward = PenaltyForFalling;
    end

end