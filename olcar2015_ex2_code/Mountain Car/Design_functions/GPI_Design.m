function Controller = GPI_Design(Task,Controller,Parameters)
%GPI_DESIGN finds a value function and contol policy for the specified MDP
%using Generalized Policy Iteration (OLCAR script section 2.8)
%
% Inputs:   Task -- Structure containing MDP model. See MDP_Design for
%               description of internal parameters.
%           Controller -- Struct containing function to convert between
%               continuous and discrete parameters. See MDP_Design for a
%               description of internal parameters.
%           Parameters -- Struct containing parameters for the GPI. See
%               main_ex2a.m for a description of the parameters. This can 
%               be modified as desired.
%
% Outputs:  Controller -- Sctruct containing the following
%               .V -- Value function [1 x (N_pos*N_vel)]
%               .Policy -- Optimal control policy found


%% Initialization

%Initialize the policy
%Policy = ... %[length(Task.S) x length(Task.A)]

% Initialize the value function
%V = ... % [length(Task.S) x 1]



while true
    %% Policy Evaluation (PE) (see script section 2.6)
    
    
    %% Policy Improvment (PI) (see script section 2.7)
    
    
    %% Check algorithm convergence
    %if ...
    %    break
    %end
    
end

end