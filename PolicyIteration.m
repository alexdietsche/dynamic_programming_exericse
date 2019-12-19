function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

J_opt = zeros(K, 1);

% initialize with proper policy
% using optimal policy from value iteration
% [~, u_opt_ind] = ValueIteration(P, G);
% commanding HOVER at all states
u_opt_ind = 5 * ones(K, 1);

while true
    
    J_opt_prev = J_opt;
    u_opt_prev = u_opt_ind;
    
    % policy evaluation: solve system of linear equations
    % P_opt and G_opt correspond to P and G with the current optimal policy
    % without the terminal state
    P_opt = zeros(K, K);
    % TODO: find better way to do this
    for i = 1 : K
        if i == TERMINAL_STATE_INDEX
            P_opt(i, :) = zeros(1, K);
        else
            P_opt(i, :) = P(i, :, u_opt_ind(i));
        end
    end
    G_opt = diag(G(:,u_opt_ind));

    % solve for J: (I - P) must be invertible!
    J_opt = (eye(size(P_opt)) - P_opt)\G_opt;
    
    % exit condition: iterate until cost-to-go is optimal
    if abs(J_opt_prev - J_opt) < 1e-8
        break;
    end

    % policy improvement: obtain new stationary policy (same as VI)
    for i = 1 : K
        [~, u_opt_ind(i)] = min(G(i, :)' + squeeze(P(i, :, :))' * J_opt);
    end
    
%     % exit condition: iterate until policy is optimal
%     if u_opt_prev == u_opt_ind
%         break;
%     end
end

J_opt(TERMINAL_STATE_INDEX) = 0;

end
