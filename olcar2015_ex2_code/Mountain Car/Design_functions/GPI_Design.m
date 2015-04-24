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
Policy = zeros(size(Task.S,2), Task.A); %[length(Task.S) x length(Task.A)]

% Initialize the value function
V = zeros(Task.S) % [length(Task.S) x 1]

while true
    %% Policy Evaluation (PE) (see script section 2.6)
    PE_iter = 1;
    while PE_iter <= GPI_Params.maxIter_PE
        delta_V = 0;       
        v = V;
        V = A*V + B;    % Policy update step (see script Algorithm 1)
                        % Compare with eq 2.11 (matrix form)
        delta_V = max(delta_V, norm(v - V));
        
        fprintf('Iteration %d of PE.\tab Maximum change in V(x): %6.4f\n', PE_iter, delta_V);
        if delta_V < Parameters.minDelta_V
            break
        end
        PE_iter = PE_iter + 1;
    end    
    
    %% Policy Improvment (PI) (see script section 2.7)
    PI_iter = 1;
    while (PI_iter <= GPI_Params.maxIter_PI) && ~policyIsStable
        delta_Policy = 0;
        policyIsStable = true;
        b = Policy;

        % Initialize pi_x and calculate argmax_u(..)
        % Policy update rule
        argmax_term = 0;
        for a = Task.A
            % Integrate cost decay factor GPI_Params.alpha
            argmax_term_temp = Task.P_s_sp_a + Task.R_s_a; % TODO: argmax_u(sum_sp'(P_s_sp_a*(R_s_a + alpha * V(sp))))

            if argmax_term_temp > argmax_term
                Policy = Controller.actionD2C(a);
            end
        end

        delta_Policy = norm(b - Policy);

        fprintf('Iteration %d of PI.\tab Maximum change in Policy: %6.4f\n', PI_iter, delta_Policy);
        
        %% Check algorithm convergence
        if delta_Policy > GPI_Params.minDelta_Policy
            policyIsStable = false;
        end
        PI_iter = PI_iter + 1;
    end
end

% Store optimal control policy and value function in controller
Controller.Policy = Policy;
Controller.V = V;

end