function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

% TODO: 
% - figure out how to avoid small rounding errors of base in probabilities near shooters

% coords of pick-up
[m_p, n_p] = find(map==PICK_UP);

% index of base
[m_base, n_base] = find(map==BASE);
[~, BASE_INDEX] = ismember([m_base, n_base, 0], stateSpace, 'rows');

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
           if all(([0, 0] <= [m_i, n_i]) .* ([m_i, n_i] <= size(P_R)))
               % if cell is in range R of shooter
               dist = abs(m_i - m_s(s)) + abs(n_i - n_s(s));
               if dist <= R
                   P_R(m_i, n_i) = P_R(m_i, n_i) + GAMMA / (dist + 1);
               end
           end
       end
   end
end

% initialize P matrix with zeros (most entries will be zero)
P = zeros(K, K, 5);

% iterate over all K initial states (i.e., cells):
for i = 1:K
    
    % terminal state maps to terminal state only
    if i == TERMINAL_STATE_INDEX
        P(i, TERMINAL_STATE_INDEX, :) = 1;
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

            % distribute propabilites of wind and shooter:
            % add to this cell: (1-p_wind)*(1-p_r) (not moved and not shot)
            % if at pick-up and without package: change state to package
            has_package_j = has_package;
            if all([m_j, n_j] == [m_p, n_p] * ~has_package)
                has_package_j = 1;
            end
            [~, j] = ismember([m_j, n_j, has_package_j], stateSpace, 'rows');
            P_R_j = P_R(m_j, n_j);
            P(i, j, l) = P(i, j, l) + (1 - P_WIND) * (1 - P_R_j);
            % add to base cell: (1-p_wind)*p_r (not moved and shot)
            P(i, BASE_INDEX, l) = P(i, BASE_INDEX, l) + (1 - P_WIND) * P_R_j;

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
                    % add to cell: p_wind*0.25*(1-p_r) (moved and not shot)
                    % if at pick-up and without package: change state to package
                    has_package_w = has_package;
                    if all([m_w, n_w] == [m_p, n_p] * ~has_package)
                        has_package_w = 1;
                    end
                    [~, j] = ismember([m_w, n_w, has_package_w], stateSpace, 'rows');
                    P_R_w = P_R(m_w, n_w);
                    P(i, j, l) = P(i, j, l) + 0.25 * P_WIND * (1 - P_R_w);
                    % add to base cell: p_wind*0.25*p_r (moved and shot)
                    P(i, BASE_INDEX, l) = P(i, BASE_INDEX, l) + 0.25 * P_WIND * P_R_w;

                % else: add to base cell: p_wind*0.25  (crashed)
                else
                    P(i, BASE_INDEX, l) = P(i, BASE_INDEX, l) + 0.25 * P_WIND;
                end
            end
        end
    end
            
end

% Floor P to get rid of very small deviations
% P = floor(P * 10^7) / 10^7;

end
