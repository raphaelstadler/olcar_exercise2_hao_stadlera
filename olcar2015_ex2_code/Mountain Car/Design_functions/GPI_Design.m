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
Policy = zeros(Task.S, Task.A); %[length(Task.S) x length(Task.A)]

% Initialize the value function
V = zeros(Task.S) % [length(Task.S) x 1]


while true
    %% Policy Evaluation (PE) (see script section 2.6)
    while true
        delta_V = 0;       
        v = V;
        V = A*V + B;    % Policy update step (see script Algorithm 1)
                        % Compare with eq 2.11 (matrix form)
        delta_V = max(delta_V, norm(v - V));
        if delta_V < Parameters.minDelta_V
            break
        end
    end    
    
    %% Policy Improvment (PI) (see script section 2.7)
    policyIsStable = ones(size(Task.S));
    b = Policy;
    
    % Initialize pi_x and calculate argmax_u(..)
    % Policy update rule
    argmax_term = 0;
    for a = Task.A
        argmax_term_temp = Task.P_s_sp_a + Task.R_s_a; % TODO: argmax_u(sum_sp'(P_s_sp_a*(R_s_a + alpha * V(sp))))
        
        if argmax_term_temp > argmax_term
            Policy = Controller.actionD2C(a);
        end
    end   
    
    if b ~= Policy
        policyIsStable = false;
    end
    
    %% Check algorithm convergence
    if policyIsStable
        break
    end
end

end