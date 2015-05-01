%% OLCAR - Exercise 2a - Mountain Car
close all; clearvars; clc;
       
addpath(genpath(pwd)); % add folders and subfolders to path

%% Define the Task parameters
Task = struct;
Task.start_time     = 0;
Task.goal_time      = 500;      % Maximum task duration
Task.start_x        = [-pi/6;   % position p
                        0];     % velocity v

%% Step 1: Create a discretized model of the system

%Specify the parameters to be used for MDP modeling. These are the
%parameters required to run the solution code, set with values that give a
%reasonable solution. Please modify the parameters as you see fit.
MDP_Params = struct;
MDP_Params.pos_N = 20; %Number of bins to use for position discretization
MDP_Params.vel_N = 20; %Number of bins to use for velocity discretization
MDP_Params.u_N = 5; %Number of bins to use for action discretization
MDP_Params.modeling_iter = 50; %Number of modeling iterations for each 
                                %state & action
                                %Set = 1 to build a deterministic model

timer = tic;                                
[Task,Controller] = MDP_Design(Task,MDP_Params);
%[Task,Controller] = MDP_Design_Solution(Task,MDP_Params);
%Optionally save the model so it doesn't have to be recomputed every time
save('Discrete_Model.mat','Task','Controller');
%save('Discrete_Model_Solution.mat','Task','Controller');
toc(timer)

% Simulations made on 2015/04/24:
% Our Discretization:           Elapsed time is 332.632794 seconds.
% Solution's Discretizatoin:    Elapsed time is 836.499078 seconds.

%% Step 2: Generalized Policy Improvemnt

%Optionally load a previously saved model for GPI testing
%load('Discrete_Model.mat');
%load('Discrete_Model_Solution.mat');
% size(Task.P_s_sp_a)
% 
% ans =
% 
%    400   400     5

%Specify the parameters to be used for the GPI algorithm. These are the
%parameters required to run the solution code, set with values that give a
%reasonable solution. Please modify the parameters used as you see fit.
GPI_Params = struct;
GPI_Params.maxIter_PE = 100; %Maximum iterations for Policy Evaluation
GPI_Params.maxIter_PI = 100; %Maximum iterations for Policy Improvement
GPI_Params.minDelta_V = 0.01; %Minimum change in V before terminating PE
GPI_Params.minDelta_Policy = 0.1; %Minimum change in the policy before terminating GPI
GPI_Params.alpha = 0.96; %Cost decay factor


%Controller = GPI_Design(Task,Controller,GPI_Params);
%To run the provided solution use:
Controller = GPI_Design_Solution(Task,Controller,GPI_Params);

%% Run the simulation using the controller found and plot results

sim_out_mdp = Mountain_Car_Simulator(Task,Controller);

visualizeMountainCar(50, sim_out_mdp, true); %Set last argument to false 
                                             %to skip the animation
