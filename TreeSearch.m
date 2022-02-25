close all

searchDepth = 2;
state_Ego0 = randi([0 20]); % State = liveness value
states_Other0 = [randi([0 20]), randi([0 20])]; % 2 other vehicles

% TODO: Stop deepening if for depth 1 (next decision) only one decision remains 
safety_limit = -Inf*ones(1, searchDepth); % Maximum safety level for each depth
for depthBound = 1:searchDepth % Iterative deepening
    % Initialise digraph
    nodeID_global = 0;
    initNode_Ego = DigraphTree.getNodeName(nodeID_global, state_Ego0, safety_limit(1));
    dG_initial = DigraphTree.initialise(initNode_Ego, [0, 1, 1]);
    [dG_initial, ~, nodeID_global] = DigraphTree.expand(dG_initial, nodeID_global, initNode_Ego, states_Other0, 'InitialStates', [1, 0, 1], [1, 0, 1], safety_limit(1));
    
    alpha_0.safety = -Inf;
    alpha_0.liveness = -Inf;
    beta_0.safety = Inf;
    beta_0.liveness = Inf;

    % Best decisions iteratively
    [bestDecision_iteration, value_iteration, dG_iteration, ~] = planMaxManeuver(state_Ego0, states_Other0, alpha_0, beta_0, depthBound, depthBound, safety_limit, dG_initial, nodeID_global, nodeID_global);
    safety_limit(depthBound) = value_iteration.safety;
    if ~isempty(bestDecision_iteration)
        % Minimum Violation Planning
        bestDecision_Ego = bestDecision_iteration;
        figure('Name', ['Iteration: ', num2str(depthBound)])
        plot(dG_iteration, 'EdgeLabel', dG_iteration.Edges.Power, 'EdgeColor', cell2mat(dG_iteration.Edges.Color), 'NodeColor', cell2mat(dG_iteration.Nodes.Color), 'Layout', 'layered')
    end
end

function [decisionsNext_Ego, value_max, graph, nodeID_global] = planMaxManeuver(state_Ego, states_Other, alpha, beta, depth2go, depthBound, safety_limit, graph, nodeID_global, parentID)
% Plan and decide for a safe maneuver according to specified searching depth
    
    decisionMax_Ego = []; % Decision of current depth, that maximises future value
    decisionsNext_Ego = []; % Next planned decisions (starting with most recent one)
    value_max.safety = -Inf;
    value_max.liveness = -Inf;

    depth2go = depth2go - 1;
    depthCurrent = depthBound - depth2go;
    depthPrev = depthCurrent - 1;
            
    if depthPrev == 0 % Necessary to connect digraph depth 0
        depthPrev = 1;
    end
    parentNode = DigraphTree.getNodeName(parentID, states_Other, safety_limit(depthPrev));
    maxNode = [];

    % Decisions Ego
    decisions_Ego = Decisions.calculateDecisions_Ego();
    
    % Decisions other vehicles
    decisions_Other = Decisions.calculateDecisions_Other();
    % Future state depends on current state and decision
    futureStates_Other  = [1; 2]*states_Other; % [minV1, maxV2; minV2, maxV2]
    
    % Safety check
    decisionsSafe_Ego = decisions_Ego;
    for id_decision = length(decisions_Ego):-1:1 % Reverse to remove unsafe decisions without confusing idx
        safetyLevel = 0;
        for id_other = 1:length(decisions_Other)
            rng(state_Ego*depthCurrent*id_decision) % Keep randomness deterministic
            safetyLevel = min(safetyLevel, -getNumberUnSafeTransitions());

            if safetyLevel < safety_limit(depthCurrent)
                % Remove unsafer decisions
                decisionsSafe_Ego(id_decision) = [];
                break
            end
        end

        if safetyLevel >= safety_limit(depthCurrent) % Only consider safer/as safe decisions 
            futureState_Ego = randi([0 20])*state_Ego; % Future state depends on current state and decision
             % Add safe decision to digraph
            [graph, childNode, nodeID_global] = DigraphTree.expand(graph, nodeID_global, parentNode, ...
                futureState_Ego, decisionsSafe_Ego{id_decision}, ...
                [0, 1, 0], [0, 1, 0], safetyLevel);
            
            if depth2go == 0
                % No future decision for this depth
                decisionsFuture_Ego = [];
                % Combined value for liveness and safety
                value = evaluate(safetyLevel, futureState_Ego);  
            else
                futureStatesCombinations_Other = getStateCombinations(futureStates_Other);
                [decisionsFuture_Ego, value, graph, nodeID_global] = planMinManeuver(futureState_Ego, futureStatesCombinations_Other, alpha, beta, depth2go, depthBound, safety_limit, graph, nodeID_global, nodeID_global);
            end

            % Max behaviour: Ego vehicle
            if Values.isGreater(value, value_max)
                value_max = value;
                decisionsNext_Ego = decisionsFuture_Ego;
                decisionMax_Ego = decisionsSafe_Ego(id_decision);
                maxNode = childNode;

                alpha = Values.Max(alpha, value_max);
                if Values.isLessEqual(beta, alpha) 
                    break
                end
            end    
        end
    end
    
    % Higlight best node and edge in digraph
    if ~isempty(maxNode) 
        graph = DigraphTree.changeNodeColor(graph, maxNode, [0, 1, 1]);
        graph = DigraphTree.changeEdgeColor(graph, parentNode, maxNode, [0, 1, 1]);
    end
    
    decisionsNext_Ego = [decisionMax_Ego; decisionsNext_Ego];
end

function [decisionsNext_Ego, value_min, graph, nodeID_global] = planMinManeuver(futureState_Ego, futureStatesCombinations_Other, alpha, beta, depth2go, depthBound, safety_limit, graph, nodeID_global, parentID)
% For other vehicles choose the action, which is considered the most unsafe

    decisionsNext_Ego = [];
    value_min.safety = Inf;
    value_min.liveness = Inf;
    
    depthCurrent = depthBound - depth2go;
        
    parentNode = DigraphTree.getNodeName(parentID, futureState_Ego, safety_limit(depthCurrent));
    minNode = [];

    for id_statesOther = 1:size(futureStatesCombinations_Other, 1)
        futureStates_Other = futureStatesCombinations_Other(id_statesOther, :);
        
        % Add other vehicles' possible decisions to digraph
        [graph, childNode, nodeID_global] = DigraphTree.expand(graph, nodeID_global, parentNode, ...
                futureStates_Other, ['Other_{Combi', num2str(id_statesOther), '}'], ...
                [1, 0, 0], [1, 0, 0], safety_limit(depthCurrent));

        [decisionsFuture_Ego, value, graph, nodeID_global] = planMaxManeuver(futureState_Ego, futureStates_Other, alpha, beta, depth2go, depthBound, safety_limit, graph, nodeID_global, nodeID_global);

        % Min behaviour: Other vehicles
        if isempty(decisionsFuture_Ego)
            decisionsNext_Ego = [];
            minNode = [];
            value_min.safety = -Inf;
            value_min.liveness = -Inf;
            break % These decisions are unsafe if at least one combination of the
                  % other vehicles' possible future states might be unsafe
        elseif value.safety <= value_min.safety
            if Values.isLess(value, value_min)
                value_min = value;
                decisionsNext_Ego = decisionsFuture_Ego;
                minNode = childNode;

                beta = Values.Min(beta, value_min);
                if Values.isLessEqual(beta, alpha)
                    break
                end
            end   
        end
    end
    
    % Higlight worst node and edge in digraph
    if ~isempty(minNode)
        graph = DigraphTree.changeNodeColor(graph, minNode, [1, 0, 1]);
        graph = DigraphTree.changeEdgeColor(graph, parentNode, minNode, [1, 0, 1]);
    end
end

function stateCombinations = getStateCombinations(states)
%Possible state combinations
    stateCombinations = combvec(states(:, 1)', states(:, 2)')';
end

function value = evaluate(safety, state)
% Evaluate safety and state

    value.safety = safety;
    liveness = state; % Evaluate according to state
    value.liveness = liveness;
end

function number_unsafeStates = getNumberUnSafeTransitions()
    number_unsafeStates = randi([0 3]); % Mock-up
end