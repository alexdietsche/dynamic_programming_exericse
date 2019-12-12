function G = ComputeStageCosts( stateSpace, map )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map)
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the ***expected*** stage cost if we are in state i and
%           apply control input l.

global GAMMA R P_WIND Nc
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K
global TERMINAL_STATE_INDEX

% array with probabilities of being shot down (P_R)
P_R = zeros(size(map));
[m_s, n_s, ~] = find(map==SHOOTER);
num_shooters = length(m_s);
% assign probabilities around every shooter
for s = 1:num_shooters
    % iterate over square Region of size RxR
    for m_i = (m_s(s) - R):(m_s(s) + R)
        for n_i = (n_s(s) - R):(n_s(s) + R)
            % if cell is inside of map
            if all(([0, 0] < [m_i, n_i]) .* ([m_i, n_i] <= size(P_R)))
                % if cell is in range R of shooter
                dist = abs(m_i - m_s(s)) + abs(n_i - n_s(s));
                if dist <= R
                    P_R(m_i, n_i) = P_R(m_i, n_i) + GAMMA / (dist + 1);
                end
            end
        end
    end
end

% initialize G matrix with zeros
G = zeros(K, 5);

% iterate over all K initial states (i.e., cells):
for i = 1:K
    
    % terminal state incurrs no cost
    if i == TERMINAL_STATE_INDEX
        G(i, :) = 0;
        continue
    end
    
    % coords of current state
    current_state = stateSpace(i, :);
    m_i = current_state(1);
    n_i = current_state(2);
    has_package = current_state(3);
    
    % iterate over possible inputs u (north, south, east, west, hover):
    for l = 1:5
        % coords after applying input
        switch l
            case NORTH
                m_j = m_i;
                n_j = n_i + 1;
            case SOUTH
                m_j = m_i;
                n_j = n_i - 1;
            case EAST
                m_j = m_i + 1;
                n_j = n_i;
            case WEST
                m_j = m_i - 1;
                n_j = n_i;
            case HOVER
                m_j = m_i;
                n_j = n_i;
        end
        
        % if input is possible
        if ismember([m_j, n_j, has_package], stateSpace, 'rows')
            
            % compute expected cost depending on wind and shooter:
            cost = 0;
            % not moved and not shot:
            P_R_j = P_R(m_j, n_j);
            cost = cost + (1 - P_WIND) * (1 - P_R_j);
            % not moved and shot
            cost = cost + (1 - P_WIND) * P_R_j * Nc;
            
            % iterate over 4 adjacent cells (north, south, east, west):
            for w = 1:4
                % coords after wind
                switch w
                    case NORTH
                        m_w = m_j;
                        n_w = n_j + 1;
                    case SOUTH
                        m_w = m_j;
                        n_w = n_j - 1;
                    case EAST
                        m_w = m_j + 1;
                        n_w = n_j;
                    case WEST
                        m_w = m_j - 1;
                        n_w = n_j;
                end
                % if cell is in stateSpace:
                if ismember([m_w, n_w, has_package], stateSpace, 'rows')
                    % moved and not shot
                    P_R_w = P_R(m_w, n_w);
                    cost = cost + 0.25 * P_WIND * (1 - P_R_w);
                    % moved and shot
                    cost = cost + 0.25 * P_WIND * P_R_w * Nc;
                    
                    % moved and crashed
                else
                    cost = cost + 0.25 * P_WIND * Nc;
                end
            end
            % store expected cost of input l
            G(i, l) = cost;
        else
            % input is not possible and incurrs inf cost
            G(i, l) = inf;
        end
    end
    
end
end

