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
%  Initialize the policy
%  At the beginning for all statesx every action is equally likely
Policy = ones(length(Task.S), length(Task.A)); %[length(Task.S) x length(Task.A)]
Policy = (1/size(Policy,2))*Policy;

% Initialize the value function
V = zeros(length(Task.S),1); % [length(Task.S) x 1]

while true
    %% Policy Evaluation (PE) (see script section 2.6)
    PE_iter = 1;
    while PE_iter <= Parameters.maxIter_PE
        delta_V = zeros(size(V));
        
        % Naive implementation
        for s = Task.S
           v = V(s);
           
           sum_outer = 0;
           for a = Task.A
                sum_P_V = Parameters.alpha * dot(Task.P_s_sp_a(s,:,a),V(:));
                sum_outer = sum_outer + Policy(s,a) * ( Task.R_s_a(s,a) + sum_P_V );
           end % a
           V(s) = sum_outer;
           
           delta_V(s) = max(delta_V(s), norm(v - V(s)));
        end % s
        
%         TODO: Replace naive implementation by matrix/vector operations
%         Someting as follows:
%         v = V;
%         V = A*V + B;    % Policy update step (see script Algorithm 1)
%                         % Compare with eq 2.11 (matrix form)
%         delta_V = max(delta_V, norm(v - V));
        
        %fprintf('Iteration %i of PE.\t Maximum change in V(x): %6.4f\n', PE_iter, delta_V);
        if max(delta_V) < Parameters.minDelta_V
            break
        end
        
        PE_iter = PE_iter + 1;
    end    
    
    %% Policy Improvment (PI) (see script section 2.7)
    PI_iter = 1;
    while (PI_iter <= Parameters.maxIter_PI)
        delta_Policy = zeros(size(Policy));
        policyIsStable = true;

        % Initialize pi_x and calculate argmax_u(..)
        % Policy update rule
        argmax_term = 0;

        % Naive implementation
        for s = Task.S
            b = Policy(s);
            
            max_term = -1E6;
            for a = Task.A
                term = Task.R_s_a(s,a) + Parameters.alpha * dot(Task.P_s_sp_a(s,:,a), V);
                if term > max_term
                    max_term = term;
                    argmax_term = a;
                end
            end
            
            Policy(s) = Controller.actionD2C(argmax_term);
            
            delta_Policy(s) = norm(b - Policy(s));
            if delta_Policy(s) > Parameters.minDelta_Policy
                policyIsStable = false;
            end
        end
        
%         TODO: Replace naive implementation by matrix/vector operations
%         Someting as follows:        
%         term = Task.R_s_a(s,:) + Parameters.alpha * Task.P_s_sp_a(s,:,:) * V;
        
        %fprintf('Iteration %i of PI.\t Maximum change in Policy: %6.4f\n', PI_iter, delta_Policy);
        PI_iter = PI_iter + 1;

        %% Check algorithm convergence
        if policyIsStable
            % Store optimal control policy and value function in controller
            % and return
            Controller.Policy = Policy;
            Controller.V = V;
            return;
        end
    end
end

end