function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

% initialize outputs
u_opt_ind = zeros(K, 1);

% calculate optimal cost-to-go
% objective is to minimize negative sum of cost-to-go for all states
f = -1 * ones(K, 1);
% inequality constraints for optimal cost-to-go expressed for every input
A = [];
b = [];
% Aeq = [];
% beq = [];
% lb = [];
% ub = [];
% options = optimoptions('linprog','Algorithm','interior-point');

% filter out terminal state
P_opt = P;
P_opt(TERMINAL_STATE_INDEX, :, :) = 0;

for l = 1 : 5
    b = [b; G(:, l)];
    A = [A; eye(size(P, [1,2])) - P_opt(:, :, l)];
end

% get rid of inequalities containing inf
delete_mask = (b == inf);
b(delete_mask) = [];
A(delete_mask, :) = [];

% J_opt = linprog(f,A,b,Aeq,beq,lb,ub,options);
J_opt = linprog(f,A,b);

% obtain stationary policy given optimal cost-to-go
    for i = 1 : K
        [~, u_opt_ind(i)] = min(G(i, :)' + squeeze(P(i, :, :))' * J_opt);
    end

J_opt(TERMINAL_STATE_INDEX) = 0;
    
end

