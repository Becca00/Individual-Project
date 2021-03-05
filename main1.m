clear all
close all
clc

Ts = 0.1; % Step time
Tf = 20; % Total run time

ObservationInfo = rlNumericSpec([7 1]); % 7 joint angles
ObservationInfo.Name = 'Joint angle states';
ObservationInfo.Description = 'Joint1, Joint2, Joint3, Joint4, Joint5, Joint6, Joint7';

ActionInfo = rlNumericSpec([7 1],'LowerLimit',-2*pi,'UpperLimit',2*pi); % Full rotation of joint angles
ActionInfo.Name = 'Joint Actions';

[InitialObservation,LoggedSignals] = myResetFunction()

env = rlFunctionEnv(ObservationInfo,ActionInfo,'myStepFunction','myResetFunction');

rng(0);
InitialObs = reset(env)
[NextObs,Reward,IsDone,LoggedSignals] = step(env,10);
NextObs


% create a DDPG
L = 48; % number of neurons
statePath = [
    featureInputLayer(4,'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(L,'Name','fc2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu2')
    fullyConnectedLayer(L,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(1,'Name','fc4')];

actionPath = [
    featureInputLayer(1,'Normalization','none','Name','action')
    fullyConnectedLayer(L, 'Name', 'fc5')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
    
criticNetwork = connectLayers(criticNetwork,'fc5','add/in2');

plot(criticNetwork)

criticOptions = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1,'L2RegularizationFactor',1e-4);

critic = rlQValueRepresentation(criticNetwork,ObservationInfo,ActionInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

actorNetwork = [
    featureInputLayer(4,'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(L,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(L,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(1,'Name','fc4')
    tanhLayer('Name','tanh1')
    scalingLayer('Name','ActorScaling1','Scale',20,'Bias',-10)];

actorOptions = rlRepresentationOptions('LearnRate',1e-4,'GradientThreshold',1,'L2RegularizationFactor',1e-4);
actor = rlDeterministicActorRepresentation(actorNetwork,ObservationInfo,ActionInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling1'},actorOptions);

agentOptions = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',64);
agentOptions.NoiseOptions.Variance = 0.6;
agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;

agent = rlDDPGAgent(actor,critic,agentOptions);

maxepisodes = 50;
maxsteps = ceil(Tf/Ts);
trainingOpts = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes,...
    'MaxStepsPerEpisode',maxsteps,...
    'Verbose',false,...
    'Plots','training-progress',...
    'StopTrainingCriteria','EpisodeReward',...
    'StopTrainingValue',1000);

doTraining = true;

if doTraining    
    % Train the agent.
    trainingStats = train(agent,env,trainingOpts);
    save ('Matlab.mat','agent')
else
    % Load a pretrained agent for the example.
    load('Matlab.mat','agent')       
end

experience = sim(env,agent);

data1=experience.Observation.CartPoleStates.Data;
time1=experience.Observation.CartPoleStates.Time;
plot(time1,data1(1,:))