function [Task,Controller] = MDP_Design(Task,Parameters)
%MDP_Design -- Generate a Markov Decision Process model of the system using
%               discretized state and action spaces.
%
%Inputs:    Task -- specifies the control task (ie IC and end time)
%           Parameters -- parameters to be used for modeling. Suggested
%               parameters and a set of possible values provided in 
%               main_ex2a.m
%
%Outputs:   Task -- Structure including the following modeling information
%               .S -- List of state indices
%               .A -- List of action idices
%               .P_s_sp_a -- Transition probability distribution. Specifies 
%                   the likelihood of moving from state 's' to state 'sp' 
%                   after applying action 'a'
%               .R_s_a -- Reward function. Expected intermediate reward 
%                   received from applying action 'a' from state 's'
%           Controller -- Discrete controller structure including
%               .state_c2d_handle -- Handle to function which converts a 
%                   set of states in continuous space to their respective 
%                   discretized bins
%               .action_d2c_handle -- Handle to function which converts a 
%                   set of discretized action space bins to their 
%                   corresponding continuous space values
%

%% Step 1: Discretize the state and action spaces

%Important: Note that the provided state_c2d function assumes that the 'X'
%corresponding to each discretized state bin indicates the center point
%of that bin (i.e. the halfway point between the extremes of position and
%velocity in that bin). If you discretize the state space another way, you
%will also need to modify this function as well.

% Allowed range of positions x: [-1.2, +0.5]
% Allowed range of velocity  v: [-0.07, +0.07]
x_min = -1.2; x_max = 0.5; v_min = -0.07; v_max = 0.07;
delta_x = (x_max - x_min)/(Parameters.pos_N-1); % Note: For N bins, there are N-1 intervals
delta_v = (v_max - v_min)/(Parameters.vel_N-1);

[X1,X2] = meshgrid( x_min:delta_x:x_max,...
                    v_min:delta_v:v_max);

X = [X1(:)'; X2(:)'];   % [2 x (pos_N*vel_N)] matrix of all possible discretized states

Task.S = 1:size(X,2);   % [1 x (pos_N*vel_N)] list of indices for each corresponding 
                        % discretized state value

% Allowed range of control   u: [-1, +1]
% (Actuation via car acceleration a)
u_min = -1; u_max = 1;
delta_u = (u_max - u_min)/(Parameters.u_N-1);

U = u_min:delta_u:u_max;    % [1 x u_N] array of all possible discretized actions

Task.A = 1:length(U);       % [1 x u_N] array of indices for each corresponding 
                            % discretized action
                      
% Initialize the MDP model of the discretized system
Task.P_s_sp_a = zeros(length(Task.S),length(Task.S),length(Task.A));    % [length(S) x length(S) x length(A)]
Task.R_s_a    = zeros(length(Task.S),length(Task.A));                   % [length(S) x length(A)]

%% Step 2: Generate the discrete state/action space MDP model 
for a = Task.A   % loop over the actions
    fprintf('Discrete system model for action a = %6.4f \n', U(a));
    
    for s = Task.S  % loop over states
        
        for i = 1:Parameters.modeling_iter % loop over modeling iterations
            unifp = rand()*delta_x;
            unifv = rand()*delta_v;
            p0 = X(1,s) + unifp - 0.5*delta_x; % position
            v0 = X(2,s) + unifv - 0.5*delta_v; % velocity  
            action = U(:,a); % inputs

            %Simulate for one time step. This function inputs and returns
            %states expressed by their physical continuous values. You may
            %want to use the included state_*2* functions provided to do
            %this conversion.
            [p1,v1,r,isTerminalState] = Mountain_Car_Single_Step(p0,v0,action); % Note: isTerminalState is nowhere needed in this scope

            % Convert to index of successor state (p1, v1)
            sp = state_c2d([p1; v1]);
            
            % Update the model with the iteration's simulation results
            % Count how many times sp is reached from s
            Task.P_s_sp_a(s,sp,a)  = Task.P_s_sp_a(s,sp,a) + 1;
            
            Task.R_s_a(s,a) = Task.R_s_a(s,a) + r;
        end % modeling_iter
    end        
end

% Normalize P_s_sp_a to come from counts to probabilities
Task.P_s_sp_a = (1/Parameters.modeling_iter)*Task.P_s_sp_a;
% Calculate average reward
Task.R_s_a = (1/Parameters.modeling_iter)*Task.R_s_a;

%% Create the discrete space controller
Controller = struct;
Controller.isDiscrete = true;

%Store handles to the functions used to convert between the discrete and
%continous state and action spaces
Controller.stateC2D = @state_c2d;
Controller.actionD2C = @(A) U(:,A);


    %% Assigning continuous states to the corresponding discrete bin indices
    %***This assumes that X indicates the CENTERPOINT of the bins. If you
    %discretize the states in another way, this function should be changed
    %as well.
    function Sp = state_c2d(Xp)
        % Finding the nearest discrete state bin
        [Sp,~] = knnsearch(X',Xp');
        Sp = Sp';        
    end

end