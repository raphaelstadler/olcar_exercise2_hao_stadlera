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

% Choose an arbitrary initial action value function
Q = zeros(length(S),length(A));

% Choose an arbitrary initial policy (index)
pi_ind = ones(length(S));

%Create object for incremental plotting of reward after each episode
windowSize = 10; %Sets the width of the sliding window filter used in plotting
plotter = RewardPlotter(windowSize);

isStatePartOfEpisode = zeros(length(S));

%% On-Policy Monte Carlo Algorithm (see script section 2.9.3) - Algorithm 6, p.59
for TrainLoop = 1:Parameters.training_iterations
    %% Generate a training episode (a)
    
    % Initialize the current state of the episode
    % Each episode has to begin at the starting point
    currState = stateX2S([4 1]');   
    
    % Initialize indicator variable if a specific state is part of this episode
    isStatePartOfEpisode(:) = 0;
    
    episodicReward = 0;
    episodeSize = 1;
    isEpisodeEnd = false;
    while (episodeSize <= Parameters.episode_length) && ~isEpisodeEnd %Episode termination criteria
        isStatePartOfEpisode(currState) = 1;
        
        % Execute the current epsilon-Greedy Policy
        action = pi_ind(currState); % A:possible action indices, pi_ind(currState): greedy action index   
        
        % Interaction with environment
        %Note that this function takes and returns states expressed by
        %their x,y grid positions. Use the 'state*2*' functions if
        %necessary.
        [xp,reward,isEpisodeEnd] = Cliff_World(stateS2X(currState),action);
        nextState = stateX2S(xp);
    
        % Policy Evaluation (b)
        Q(currState,action) = Q(currState,action) + Parameters.omega*(reward - Q(currState,action));       
        
        % Log data for the episode
        episodicReward = episodicReward + reward;
        
        currState = nextState;
        episodeSize = episodeSize + 1;
    end
    fprintf('Iteration %i\t Episode length:\t %d\t epsilon = %6.6f\n', TrainLoop, (episodeSize-1), Parameters.epsilon);
    
    %% Monte Carlo Policy Improvement Step (c)
    % For each state of the episode
    for s=S(isStatePartOfEpisode==1)
        [~,u_star] = max(Q(s,:));
        
        pi_ind(s) = choose_epsilon_greedy(A, u_star); % A:possible action indices, u_star: greedy action index   
    end
    
    %Update the reward plot
    EpisodeTotalReturn = episodicReward; %Sum of the reward obtained during the episode
    plotter = UpdatePlot(plotter,EpisodeTotalReturn);
    
    %% Decrease the exploration (d)
    %Set k_epsilon = 1 to maintain constant exploration
	Parameters.epsilon = Parameters.epsilon * Parameters.k_epsilon;
end

    function u_ind = choose_epsilon_greedy(ind_array, greedy_ind)
        u_ind = -1;     % initialize action index with invalid number
        unif = rand;    % uniformly distributed RV: [0..1]
        
        l = length(ind_array);
        
        p_nonGreedy = Parameters.epsilon/l; % probability of non-greedy action
        
        nongreedy_ind_array = ind_array(ind_array~=greedy_ind);
        for k=1:length(nongreedy_ind_array)
            if (unif >= (k-1)*p_nonGreedy) && (unif < k*p_nonGreedy)
                u_ind = nongreedy_ind_array(k);  % non-greedy action is taken (with prob. p_nonGreedy)
            end
        end
        
        if u_ind == -1      % greedy action is taken (with prob p_Greedy = 1 - (l-1)*p_nonGreedy
            u_ind = greedy_ind;
        end
    end
end
