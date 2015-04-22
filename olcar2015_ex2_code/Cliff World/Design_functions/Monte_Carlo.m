function Q = Monte_Carlo(S,A,stateS2X,stateX2S,Parameters)
%MONTE_CARLO solves for an optimal control policy using the On-Policy Monte
%Carlo method (script section 2.9)
%
% Inputs:   S -- Vector of natural number state indicies
%           A -- Vector of natural number action indices
%           stateS2X -- Function to convert state indices to their physical
%               x,y value
%           stateX2S -- Function to convert x,y grid positions to their
%               corresponding state indices
%           Parameters -- Struct containing parameters for the learning
%               algorithm. Suggested parameters and a set of possible 
%               values provided in main_ex2a.m
%
% Outputs:  Q -- Action value function of size [length(S),length(A)]. Note
%               that the resulting policy is not returned. The policy
%               should be naturally encoded in Q.
%

%% Initialize Variables


%Create object for incremental plotting of reward after each episode
windowSize = 10; %Sets the width of the sliding window fitler used in plotting
plotter = RewardPlotter(windowSize);


%% On-Policy Monte Carlo Algorithm (see script section 2.9.3)

for TrainLoop = 1:Parameters.training_iterations
    %% Generate a training episode

    %while ... %Episode termination criteria
        % Execute the current epsilon-Greedy Policy


        % Interaction with environment
        %Note that this function takes and returns states expressed by
        %their x,y grid positions. Use the 'state*2*' functions if
        %necessary.
        %[xp,reward,isEpisodeEnd] = Cliff_World(stateS2X(currState),action);
        %nextState = stateX2S(xp);


        % Log data for the episode


    % end
    
    
    %% Monte Carlo Policy Improvement Step
    
    
    
    %Update the reward plot
    %EpisodeTotalReturn = ... %Sum of the reward obtained during the episode
    plotter = UpdatePlot(plotter,EpisodeTotalReturn);
    
    %% Decrease the exploration
    %Set k_epsilon = 1 to maintain constant exploration
	Parameters.epsilon = Parameters.epsilon * Parameters.k_epsilon;
    
    
end

end
