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
pi_x_u = ones(length(Task.S), length(Task.A)); %[length(Task.S) x length(Task.A)]
pi_x_u = (1/size(pi_x_u,2))*pi_x_u;

% Initially, always use action 1
pi = ones(length(Task.S),1);

% Initialize the value function
V = zeros(length(Task.S),1); % [length(Task.S) x 1]

while true
    %% Policy Evaluation (PE) (see script section 2.6)
    PE_iter = 1;
    while PE_iter <= Parameters.maxIter_PE
        delta_V = 0;
        
        % Naive implementation
        for s = Task.S  % for each state (sweep)
            v = V(s);
            sum_outer = 0; % summation over u
            for a = Task.A
                sum_P_V = Parameters.alpha * dot(Task.P_s_sp_a(s,:,pi(s)),V(:));
                sum_outer = sum_outer + pi_x_u(s,a) * ( Task.R_s_a(s,a) + sum_P_V );
            end % a
            V(s) = sum_outer;
            delta_V = max(delta_V, norm(v - V(s)));
        end % s
        
        fprintf('Iteration %i of PE.\t Maximum change in V(x): %6.4f\n', PE_iter, delta_V);
        if delta_V < Parameters.minDelta_V
            break
        end
        
        PE_iter = PE_iter + 1;
    end % while (PE)  
    
    %% Policy Improvment (PI) (see script section 2.7)
    PI_iter = 1;
    
    policyIsStable = true;
    
    while (PI_iter <= Parameters.maxIter_PI)
        % Naive implementation
        for s = Task.S
            b = pi(s);
            
            argmax_term = 0;
            max_term = -1E6;
            for a = Task.A
                term = Task.R_s_a(s,a) + Parameters.alpha * dot(Task.P_s_sp_a(s,:,a), V(:));
                if term > max_term
                    max_term = term;
                    argmax_term = a;
                end
            end
            
            pi(s) = argmax_term;
            
            if b ~= pi(s)
                policyIsStable = false;
            end
        end
         
        PI_iter = PI_iter + 1;
        
        %% Check algorithm convergence
        if policyIsStable
            % Store optimal control policy and value function in controller
            % and return
            
            % Change policy dimensions from vector to matrix format
            Policy = zeros(length(Task.S),length(Task.A));
            for s = Task.S
               Policy(s,pi(s)) = 1; 
            end
            Controller.Policy = Policy;
            Controller.V = V;
            fprintf('Final Iteration of PI: %i.\n',PI_iter-1);
            return;
        end
    end
    fprintf('Final Iteration of PI: %i.\n',PI_iter-1);    
end

end