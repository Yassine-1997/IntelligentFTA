%% Full Implementation: Mobile Agent Traversal with Energy Consumption Model
clear;
clc;
close all;

% Step 1: WSN Deployment and K-means Clustering
numNodes = 100; % Number of sensor nodes
areaSize = 100; % Area size (100x100 units)
nodes = rand(numNodes, 2) * areaSize; % Random (x, y) coordinates for nodes

numClusters = 5; % Number of clusters
[idx, clusterCenters] = kmeans(nodes, numClusters);

% Visualize Clusters
figure;
hold on;
colors = lines(numClusters);
for i = 1:numClusters
    clusterNodes = nodes(idx == i, :);
    scatter(clusterNodes(:, 1), clusterNodes(:, 2), 50, colors(i, :), 'filled'); % Cluster nodes
    scatter(clusterCenters(i, 1), clusterCenters(i, 2), 200, colors(i, :), 'p', 'LineWidth', 2); % Cluster centers
end
scatter(areaSize/2, areaSize/2, 250, 'k', 'x', 'LineWidth', 3); % Sink node at center
title('WSN Deployment and K-means Clustering');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

% Step 2: Cluster Head (CH) Selection using Tabu Search
maxIterations = 100; % Maximum iterations for Tabu Search
tabuListLength = 10; % Length of the Tabu List
CHs = zeros(numClusters, 1); % To store the selected CH indices

for cluster = 1:numClusters
    clusterNodes = find(idx == cluster); % Extract nodes for this cluster
    if isempty(clusterNodes)
        error(['No nodes found for cluster ', num2str(cluster)]);
    end

    % Initialize Tabu Search
    tabuList = [];
    bestNode = clusterNodes(1);
    bestScore = inf;

    for iter = 1:maxIterations
        % Evaluate scores for all nodes
        scores = zeros(length(clusterNodes), 1);
        for i = 1:length(clusterNodes)
            node = clusterNodes(i);
            distance = norm(nodes(node, :) - clusterCenters(cluster, :));
            energy = rand(); % Simulated energy level
            scores(i) = distance * 0.7 + energy * 0.3; % Weighted score
        end

        % Find the best candidate not in Tabu List
        [~, sortedIdx] = sort(scores);
        for i = 1:length(clusterNodes)
            candidate = clusterNodes(sortedIdx(i));
            if ~ismember(candidate, tabuList)
                candidateScore = scores(sortedIdx(i));
                break;
            end
        end

        % Update Tabu List
        tabuList = [tabuList, candidate];
        if length(tabuList) > tabuListLength
            tabuList(1) = []; % Remove oldest entry
        end

        % Update best node
        if candidateScore < bestScore
            bestNode = candidate;
            bestScore = candidateScore;
        end
    end

    % Assign the best node as the CH
    CHs(cluster) = bestNode;
end

% Visualize CH Selection
figure;
hold on;
for i = 1:numClusters
    clusterNodes = nodes(idx == i, :);
    scatter(clusterNodes(:, 1), clusterNodes(:, 2), 50, colors(i, :), 'filled');
    scatter(nodes(CHs(i), 1), nodes(CHs(i), 2), 200, colors(i, :), 's', 'LineWidth', 2); % CHs
end
scatter(areaSize/2, areaSize/2, 250, 'k', 'x', 'LineWidth', 3); % Sink node
title('Cluster Head Selection with Tabu Search');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

% Step 3: Construct MST using Kruskal's Algorithm
sink_node = [areaSize/2, areaSize/2]; % Sink node coordinates
CH_positions = nodes(CHs, :); % CH positions

% Prepare input for Kruskal's MST function
best_solutions = num2cell(CH_positions, 2); % Convert CH positions to cell array
MST_edges = kruskal_mst(best_solutions, sink_node); % Call the MST function

% Visualize the Optimized MST
figure;
hold on;

% Plot the Cluster Heads (CHs) and Sink Node
scatter(CH_positions(:, 1), CH_positions(:, 2), 200, 'r', 'filled'); % CHs
scatter(sink_node(1), sink_node(2), 300, 'b', 'filled'); % Sink node

% Plot the MST edges
for i = 1:size(MST_edges, 1)
    plot([MST_edges(i, 1), MST_edges(i, 3)], ...
         [MST_edges(i, 2), MST_edges(i, 4)], 'g-', 'LineWidth', 1.5);
end

title('Optimized MST Starting from Sink Node');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

% Step 4: Energy Consumption Model
numNodes = size(nodes, 1); % Total number of nodes
initialEnergy = 0.5; % Initial energy in Joules
E_elec = 50e-9; % Energy for transmitter/receiver circuitry (J/bit)
E_amp = 100e-12; % Energy for amplifier (J/bit/m^2)
dataSize = 4000; % Data size in bits (per transmission)

% Initialize energy levels for all nodes
nodeEnergy = ones(numNodes, 1) * initialEnergy;

% Initialize dead nodes tracker
deadNodes = zeros(1, 1); % Track the number of dead nodes over rounds

% Run Simulation for Rounds
numRounds = 600; % Number of rounds to simulate
for round = 1:numRounds
    fprintf('Round %d:\n', round);

    % Loop through Cluster Heads (CHs) for transmission
    for i = 1:size(CH_positions, 1)
        % Transmission from CH to Sink
        distToSink = norm(CH_positions(i, :) - sink_node);
        E_tx = E_elec * dataSize + E_amp * dataSize * distToSink^2;

        % Check if the CH has enough energy
        if nodeEnergy(CHs(i)) > 0
            nodeEnergy(CHs(i)) = nodeEnergy(CHs(i)) - E_tx; % Decrease energy
        end

        % Reception at Sink (assume no energy loss at sink node)
        E_rx = E_elec * dataSize;

        % Check if node is dead
        if nodeEnergy(CHs(i)) <= 0 && ~ismember(CHs(i), deadNodes)
            fprintf('Node %d died in round %d\n', CHs(i), round);
            deadNodes = [deadNodes, CHs(i)];
        end
    end

    % Count the number of dead nodes
    numDeadNodes = sum(nodeEnergy <= 0);
    fprintf('Dead nodes after round %d: %d\n', round, numDeadNodes);

    % Stop simulation if all nodes are dead
    if numDeadNodes == numNodes
        fprintf('All nodes are dead. Simulation stopped.\n');
        break;
    end
end



% MST Function (Kruskal's Algorithm)
function MST_edges = kruskal_mst(best_solutions, sink_node)
    % Convert cell array to matrix for easier handling
    CH_XY = [];
    for i = 1:length(best_solutions)
        if iscell(best_solutions{i})
            CH_XY = [CH_XY; cell2mat(best_solutions{i})];
        else
            CH_XY = [CH_XY; best_solutions{i}];
        end
    end
    CH_XY = [CH_XY; sink_node]; % Add sink node to the list of CHs

    % Number of CHs including the sink node
    numCH = size(CH_XY, 1);

    % Create edge list with distances
    edgeList = [];
    for i = 1:numCH
        for j = i+1:numCH
            distance = norm(CH_XY(i, :) - CH_XY(j, :));
            edgeList = [edgeList; i, j, distance];
        end
    end

    % Sort edge list by distance
    edgeList = sortrows(edgeList, 3);

    % Initialize MST edges and disjoint set
    MST_edges = [];
    parent = 1:numCH;
    rank = zeros(1, numCH);

    % Find function for disjoint set
    function root = find(x)
        if parent(x) ~= x
            parent(x) = find(parent(x)); % Path compression
        end
        root = parent(x);
    end

    % Union function for disjoint set
    function union(x, y)
        rootX = find(x);
        rootY = find(y);
        if rootX ~= rootY
            if rank(rootX) > rank(rootY)
                parent(rootY) = rootX;
            elseif rank(rootX) < rank(rootY)
                parent(rootX) = rootY;
            else
                parent(rootY) = rootX;
                rank(rootX) = rank(rootX) + 1;
            end
        end
    end

    % Kruskal's algorithm to construct MST
    numEdges = 0;
    i = 1;
    while numEdges < numCH - 1
        node1 = edgeList(i, 1);
        node2 = edgeList(i, 2);
        if find(node1) ~= find(node2)
            MST_edges = [MST_edges; CH_XY(node1, :), CH_XY(node2, :)];
            union(node1, node2);
            numEdges = numEdges + 1;
        end
        i = i + 1;
    end
end