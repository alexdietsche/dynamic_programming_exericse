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

% remove terminal state for systems of equations to be solvable
global TERMINAL_STATE_INDEX
P_without_terminal_state = P;
G_without_terminal_state = G;
P_without_terminal_state(TERMINAL_STATE_INDEX,:,:) = zeros(K,5);

% initialie cost
J_opt = zeros(K, 1);
% initialize policy with hover to be admissible
u_opt_ind = HOVER*ones(K, 1);


while true
    
    % get indices of G matrix depending on control input
    idx = sub2ind(size(G_without_terminal_state), [1:K]', [u_opt_ind]);
    G_mu_h = G_without_terminal_state(idx);
    
    % get transition probability matrix for given policy
    P_mu_h = [];
    for i=1:size(P_without_terminal_state,1)
        for j=1:size(P_without_terminal_state,2)
            P_mu_h(i,j) = P_without_terminal_state(i,j,u_opt_ind(i));
        end
    end
       
    % solve linear system of equations to find optimal cost
    J_mu_h = (eye(size(G_mu_h,1)) - P_mu_h)\G_mu_h;
    
    % policy improvment
    J_mu_h_next = zeros(size(J_mu_h));
    for i=1:K
        for l=1:5
            J_i(l) = G_without_terminal_state(i,l) + P_without_terminal_state(i,:,l)*J_mu_h;
        end
        [val, idx] = min(J_i);
        u_opt_ind(i) = idx;
        J_mu_h_next(i) = val;
    end

    sum(J_mu_h - J_mu_h_next);
    
    % check if cost change is low enough to exit pi
    if abs(J_mu_h - J_mu_h_next) < 0.0001
        break
    end
end

J_opt = J_mu_h_next;
u_opt_ind = u_opt_ind;

end
