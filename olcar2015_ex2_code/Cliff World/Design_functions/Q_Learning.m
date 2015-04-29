function Q = Q_Learning(S,A,stateS2X,stateX2S,Parameters)
%Q_LEARNING solves for an optimal action value function using the
%Q-Learning algorithm (script section 2.10)
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
% Choose an arbitrary initial action value function
Q = zeros(length(S),length(A));

% Choose an arbitrary initial policy (index)
pi_ind = ones(length(S));

%Create object for incremental plotting of reward after each episode
windowSize = 10; %Sets the width of the sliding window fitler used in plotting
plotter = RewardPlotter(windowSize);


%% Q-Learning Algorthim (script section 2.9)
for TrainLoop = 1:Parameters.training_iterations
    %% Generate a training episode
    
    % Initialize x: The current state of the episode
    % Each episode has to begin at the starting point
    currState = stateX2S([4 1]);   
    
    episodicReward = 0;
    episodeSize = 1;
    while (episodeSize <= Parameters.episode_length) && ~isEpisodeEnd %Episode termination criteria
        % Execute the current epsilon-Greedy Policy
        [~,u_star] = max(Q(currState,:));
        action = choose_epsilon_greedy(A, u_star); % A:possible action indices, u_star: greedy action index   
        
        % Interaction with environment
        %Note that this function takes and returns states expressed by
        %their x,y grid positions. Use the state*2* functions if
        %necessary.
        [xp,reward,isEpisodeEnd] = Cliff_World(stateS2X(currState),action);
        nextState = stateX2S(xp);
        
        %% Update Q(s,a)
        Q(currState,action) = Q(currState,action) + Parameters.omega*(reward + Parameters.alpha*max(Q(nextState,:)) - Q(currState,action));
        
        % Log data for the episode
        episodicReward = episodicReward + reward;

        currState = nextState;
        episodeSize = episodeSize + 1;
        
    end

    %Update the reward plot
    EpisodeTotalReturn = episodicReward; %Sum of the reward obtained during the episode
    plotter = UpdatePlot(plotter,EpisodeTotalReturn);
    
    
    %% Decrease the exploration
    %Set k_epsilon = 1 to maintain constant exploration
	Parameters.epsilon = Parameters.epsilon * Parameters.k_epsilon; 
    
end
    function u_ind = choose_epsilon_greedy(A, greedy_ind)
        % function
    end
end
