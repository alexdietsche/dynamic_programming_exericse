function P = ComputeTransitionProbabilities( stateSpace, map)
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


P = zeros(K, K, 5);

[basePositionX, basePositionY] = find(map == BASE);
[~, baseIdx] = ismember([basePositionX, basePositionY, 0], stateSpace, 'row');

[pickUpPositionX, pickUpPositionY] = find(map == PICK_UP);
[~, pickUpIdx] = ismember([pickUpPositionX, pickUpPositionY, 0], stateSpace, 'row');
[~, pickedUpIdx] = ismember([pickUpPositionX, pickUpPositionY, 1], stateSpace, 'row');

[shooterPositionsX, shooterPositionsY] = find(map == SHOOTER);
numberOfShooters = size(shooterPositionsX, 1);
shootersPresent = true;

if (numberOfShooters == 0)
    shootersPresent = false;
end

windMovements = [1, 0;
    -1, 0
    0, 1
    0, -1];

for i = 1 : K
    
    % check for terminal state
    if (i == TERMINAL_STATE_INDEX)
        continue
    end
    
    currentState = stateSpace(i, :);
    psi = currentState(3);
    
    for u = 1 : 5
        
        m = currentState(1);
        n = currentState(2);
        
        switch u
            case NORTH
                n = n +1;
            case SOUTH
                n = n - 1;
            case EAST
                m = m + 1;
            case WEST
                m = m - 1;
            case HOVER
                % no movement
        end
        
        [isInMap, j] = ismember([m, n, psi], stateSpace, 'row');
        
        % check if move valid
        if (~isInMap)
            continue;
        end
        
        % calculate shoot down probability
        shootDownProbability = 0;
        if (shootersPresent)
            for s = 1 : numberOfShooters
                distanceToShooter = abs(m - shooterPositionsX(s)) + abs(n - shooterPositionsY(s));
                if (distanceToShooter <= R)
                    shootDownProbability = shootDownProbability + GAMMA / (1 + distanceToShooter);
                end
            end
        end
        
        % probability of no wind and not being shot down
        probabilityNoWindNoShootDown = (1 - P_WIND) * (1 - shootDownProbability);
        probabilityNoWindShootDown = (1 - P_WIND) * shootDownProbability;
        
        if (j == pickUpIdx)
            P(i,pickedUpIdx, u) = P(i,pickedUpIdx, u) + probabilityNoWindNoShootDown;
            
        else
            P(i, j, u) = P(i, j, u) + probabilityNoWindNoShootDown;
        end
        
        % shoot down
        P(i, baseIdx, u) = P(i, baseIdx, u) + probabilityNoWindShootDown;
        
        
        % wind
        for k = 1 : 4
            nWind = n + windMovements(k, 2);
            mWind = m + windMovements(k, 1);
            [isInMap, j] = ismember([mWind , nWind, psi], stateSpace, 'row');
            
            % calculate shoot down probability
            shootDownProbability = 0;
            if (shootersPresent)
                for s = 1 : numberOfShooters
                    distanceToShooter = abs(mWind - shooterPositionsX(s)) + abs(nWind - shooterPositionsY(s));
                    if (distanceToShooter <= R)
                        shootDownProbability = shootDownProbability + GAMMA / (1 + distanceToShooter);
                    end
                end
            end
            
            if(isInMap)
                if (j == pickUpIdx)
                    P(i,pickedUpIdx, u) = P(i,pickedUpIdx, u) + (P_WIND / 4) * (1 - shootDownProbability);
                else
                    P(i, j, u) = P(i, j, u) + (P_WIND / 4) * (1 - shootDownProbability);
                end
                
                P(i, baseIdx, u) = P(i, baseIdx, u) + (P_WIND / 4) * shootDownProbability;
            else
                P(i, baseIdx, u) = P(i, baseIdx, u) + P_WIND / 4;
            end
        end
    end
    
end

% terminal state 
for i = 1 : 5
    P(TERMINAL_STATE_INDEX, TERMINAL_STATE_INDEX, i) = 1;
end

end
