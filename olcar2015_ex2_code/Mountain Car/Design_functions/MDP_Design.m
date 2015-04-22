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
%               .action_d2c_handle -- Handle to function which convers a 
%                   set of discretized action space bins to their 
%                   corresponding continuous space values
%

%% Step 1: Discretize the state and action spaces

%Important: Note that the provided state_c2d function assumes that the 'X'
%corresponding to each discretized state bin indicates the center point
%of that bin (ie the halfway point between the extremes of position and
%velocity in that bin). If you discretize the state space another way, you
%will also need to modify ths function as well.




%X = ... % [2 x (pos_N*vel_N)] matrix of all possible discretized states
%Task.S = ... % [1 x (pos_N*vel_N)] list of indices for each corresponding 
              % discretized state value

%U = ... % [1 x u_N] array of all possible discretized actions
%Task.A = ... % [1 x u_N] array of indices for each corresponding 
                      % discretized action


%Initialize the MDP model of the discretized system
%Task.P_s_sp_a = ... % [length(S) x length(S) x length(A)]
%Task.R_s_a = ... % [length(S) x length(A)]


%% Step 2: Generate the discrete state/action space MDP model 

for a = Task.A   % loop over the actions

    for s = Task.S  % loop over states
        
        for i = 1:Parameters.modeling_iter % loop over modeling iterations
            
            %p0 = ...
            %v0 = ...
            %action = ...

            
            %Simulate for one time step. This function inputs and returns
            %states expressed by their physical continuous values. You may
            %want to use the included state_*2* functions provided to do
            %this conversion.
            %[p1,v1,r,isTerminalState] = Mountain_Car_Single_Step(p0,v0,action);

            
            %Update the model with the iteration's simulation results
            %Task.P_s_sp_a(  ,  ,  ) = ...
            %Task.R_s_a(  ,  ) = ...
        end
    end        
end


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
        Sp =Sp';        
    end

end