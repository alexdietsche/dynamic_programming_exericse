function [ J_opt, u_opt_ind ] = ValueIteration2(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER TERMINAL_STATE_INDEX

% initialization
terminationThreshold = 1e-8;

J_opt = zeros(K, 1);
J_opt_next = ones(K, 1);
u_opt_ind = zeros(K, 1);


while (true)
    
    for i = 1:K
        cost_to_go_candidates = zeros(5, 1);
        for l = 1:5
            cost_to_go_last = 0;
            for j = 1:K
                if i == TERMINAL_STATE_INDEX
                    cost_to_go_last = 0;
                else
                    cost_to_go_last = cost_to_go_last + P(i, j, l) * J_opt_next(j);
                end
                
            end
            cost_to_go_candidates(l) = G(i, l) + cost_to_go_last;
        end
        
        % gauss-seidel update
        [J_opt_next(i), u_opt_ind(i)] = min(cost_to_go_candidates);
    end
    
    if (norm(J_opt - J_opt_next) < terminationThreshold)
        break
    end
    
    J_opt = J_opt_next;
    
end

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ? Probably assign J_opt zero cost there...

% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

J_opt(TERMINAL_STATE_INDEX) = 0;

end