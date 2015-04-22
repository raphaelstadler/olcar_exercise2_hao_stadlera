%% OLCAR - Exercise 2b - Cliff World
close all; clearvars; clc;

addpath(genpath(pwd)); % add folders and subfolders to path

%% Map the position vector states to natural number indices
[X1,X2] = meshgrid(4:-1:1,1:12);
X = [X1(:)'; X2(:)']; %Vector of x,y position pairs
S = 1:size(X,2); %Vector of natural number state indices
A = 1:4; %Indices of all possible actions

%Create functions to convert from natural number states indices to position
%vector states
stateX2S = @(x) knnsearch(X',x');
stateS2X = @(s) X(:,s);

%% Find the optimal policy
UseMonteCarlo = true; %Set to false to run Q-Learning

if (UseMonteCarlo)
    %% Step 1: On-Policy Monte Carlo Method
    
    %Specify the parameters to be used by the algorithm. These are the
    %parameters required to run the solution code, set with values that 
    %give a reasonable solution.
    MC_Params = struct;
    MC_Params.epsilon = 0.3;
    MC_Params.k_epsilon = 1; %0.995; %epsilon_n+1 = epsilon_n * k_epsilon
    MC_Params.training_iterations = 500; %Number of training episodes
    MC_Params.episode_length = 500; %Length of each training episode
    MC_Params.omega = 0.1; %Learning rate
    MC_Params.alpha = 1; %Cost decay factor
    
    Q = Monte_Carlo(S,A,stateS2X,stateX2S,MC_Params);
    
    %To run the provided solution use:
    %Q = Monte_Carlo_Solution(S,A,stateS2X,stateX2S,MC_Params);
    
else
    %% Step 2: Q-Learning
    
    %Specify the parameters to be used by the algorithm. These are the
    %parameters required to run the solution code, set with values that 
    %give a reasonable solution.
    QL_Params = struct;
    QL_Params.epsilon = 0.1;
    QL_Params.k_epsilon = 0.995; %epsilon_n+1 = epsilon_n * k_epsilon
    QL_Params.training_iterations = 500; %Number of training episodes
    QL_Params.episode_length = 500; %Length of each training episode
    QL_Params.omega = 0.1; %Learning rate
    QL_Params.alpha = 1; %Cost decay factor
    
    Q = Q_Learning(S,A,stateS2X,stateX2S,QL_Params);
    
    %To run the provided solution use:
    %Q = Q_Learning_Solution(S,A,stateS2X,stateX2S,QL_Params);
    
end

%% Plot the solution
VisualizeCliffWorld(Q,stateX2S,stateS2X)
