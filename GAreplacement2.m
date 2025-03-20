clc; clear; close all;

%% Initialize WSN Parameters
initial_energy = 1; % Joules (sensor nodes initial energy)
E_tx = 50e-9; % Transmission energy cost (50 nJ/bit)
E_rx = 50e-9; % Reception energy cost (50 nJ/bit)
E_amp = 10e-9; % Amplification energy cost (10 nJ/bit/m^2)
packet_size = 1000; % Packet size (bits)
control_packet_size = 50; % Control packet size (bits)
num_iterations = 100; % Number of iterations for optimization
numRounds = 3000; % Simulation rounds

network_size = 200; % Network size (m)
sink_node = [0,0]; % Sink node position
numNodes = 100; % Number of nodes
nodes = [rand(numNodes, 1) * network_size, rand(numNodes, 1) * network_size]; % Random node positions
energy_levels = initial_energy * ones(numNodes, 1); % Initial energy levels for all nodes
death_threshold = 0.2 * initial_energy; % Node dies when energy falls below 20%

%% Metrics
FND = 0; HND = 0; alive_nodes_time = zeros(numRounds, 1);
total_energy_consumption = zeros(numRounds, 1);
dead_nodes_per_round = zeros(numRounds, 1);
ch_replacement_count = zeros(numRounds, 1);
ch_replacement_log = [];

%% Clustering & CH Selection (K-means)
optimal_k = 10; % Number of clusters (CHs)
[idx, C] = kmeans(nodes, optimal_k); % K-means for clustering
dead_clusters = zeros(1, optimal_k); % Track clusters with all dead nodes

best_solutions = cell(1, optimal_k);
best_costs = inf * ones(1, optimal_k);

for i = 1:optimal_k
    cluster_nodes = nodes(idx == i, :);
    best_solutions{i} = cluster_nodes(randi(size(cluster_nodes, 1)), :);
end

%% Simulation Loop
for round = 1:numRounds
    % Energy Depletion and Communication
    for i = 1:numNodes
        if energy_levels(i) > 0
            % Calculate energy for transmission, reception, and amplification
            dist_to_sink = norm(nodes(i, :) - sink_node);
            energy_levels(i) = energy_levels(i) - ...
                               (E_tx * packet_size + E_rx * packet_size + E_amp * dist_to_sink^2);

            % Check if node energy falls below death threshold
            if energy_levels(i) < death_threshold
                energy_levels(i) = 0; % Node dies
            end
        end
    end
    
    % Record Metrics
    dead_nodes_per_round(round) = sum(energy_levels == 0);
    total_energy_consumption(round) = sum(energy_levels);
    alive_nodes_time(round) = sum(energy_levels > 0);
    
    if FND == 0 && dead_nodes_per_round(round) > 0
        FND = round;
    end
    if HND == 0 && dead_nodes_per_round(round) >= numNodes / 2
        HND = round;
    end
    
    % CH Replacement using IMA (Intelligent Mobile Agent)
    for i = 1:optimal_k
        % Skip if the entire cluster is down
        if dead_clusters(i) == 1
            continue;
        end
        
        % Check if any node in the cluster is still alive
        cluster_nodes = nodes(idx == i, :);
        cluster_energy = energy_levels(idx == i);
        
        if any(cluster_energy > 0) % If there's at least one alive node
            ch_index = find(ismember(nodes, best_solutions{i}, 'rows'), 1);
            if ~isempty(ch_index) && energy_levels(ch_index) < death_threshold
                % Select a new CH using GA
                new_ch = ga_select_new_ch(cluster_nodes, cluster_energy);
                ch_replacement_count(round) = ch_replacement_count(round) + 1;
                ch_replacement_log = [ch_replacement_log; round, best_solutions{i}, new_ch];
                best_solutions{i} = new_ch;
                
                % Check if all nodes in the cluster are down
                if all(energy_levels(idx == i) == 0)
                    dead_clusters(i) = 1; % Mark the cluster as downed
                end
            end
        else
            dead_clusters(i) = 1; % If all nodes are dead, mark the cluster as downed
        end
    end
end

%% Compute MST (Kruskal's Algorithm) for CHs and Sink Node
CHs_with_sink = [cell2mat(best_solutions'); sink_node];  % Include Sink
MST_edges = compute_mst(CHs_with_sink);

%% Visualization: Initial WSN with MST on CHs and Clusters in Different Colors
figure;
hold on;

% Generate distinct colors for each cluster
colors = lines(optimal_k); % Generates 'optimal_k' unique colors

% Plot all nodes, coloring them by cluster
for i = 1:optimal_k
    cluster_nodes = nodes(idx == i, :);
    plot(cluster_nodes(:,1), cluster_nodes(:,2), 'o', 'Color', colors(i,:), ...
         'MarkerSize', 4, 'DisplayName', sprintf('Cluster %d', i));
end

% Plot CHs (larger circles, same color as their cluster)
for i = 1:optimal_k
    plot(best_solutions{i}(1), best_solutions{i}(2), 'o', ...
         'MarkerSize', 8, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k', ...
         'LineWidth', 1.5, 'DisplayName', sprintf('CH Cluster %d', i));
end

% Plot Sink node (black square)
plot(sink_node(1), sink_node(2), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k', ...
     'DisplayName', 'Sink Node');

% Plot MST edges (connecting only CHs and the Sink)
for i = 1:size(MST_edges, 1)
    node1 = CHs_with_sink(MST_edges(i, 1), :);
    node2 = CHs_with_sink(MST_edges(i, 2), :);
    plot([node1(1), node2(1)], [node1(2), node2(2)], 'k-', 'LineWidth', 1.5, ...
         'DisplayName', 'MST Edges');
end

title('Final WSN with MST on CHs (Clusters in Different Colors)');
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');
legend();
grid on;
hold off;

%% Plot Metrics

% Plot Dead Nodes per Round
figure;
plot(1:numRounds, dead_nodes_per_round, 'r-', 'LineWidth', 2);
title('Dead Nodes per Round');
xlabel('Rounds');
ylabel('Dead Nodes');
grid on;

% Plot Alive Nodes per Round
figure;
plot(1:numRounds, alive_nodes_time, 'g-', 'LineWidth', 2);
title('Alive Nodes per Round');
xlabel('Rounds');
ylabel('Alive Nodes');
grid on;

% Plot Total Energy Consumption
figure;
plot(1:numRounds, total_energy_consumption, 'b-', 'LineWidth', 2);
title('Total Energy Consumption');
xlabel('Rounds');
ylabel('Total Energy (J)');
grid on;

% Plot FND and HND
figure;
hold on;
plot(1:numRounds, dead_nodes_per_round, 'r-', 'LineWidth', 2);
if FND > 0
    plot(FND, dead_nodes_per_round(FND), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
end
if HND > 0
    plot(HND, dead_nodes_per_round(HND), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
end
title('FND and HND');
xlabel('Rounds');
ylabel('Dead Nodes');
legend('Dead Nodes per Round', 'First Node Dead (FND)', 'Half Node Dead (HND)');
grid on;
hold off;

%% Function: GA-Based CH Selection
function new_ch = ga_select_new_ch(cluster_nodes, cluster_energy)
    population_size = 20;
    generations = 50;
    population = randi([1, size(cluster_nodes, 1)], 1, population_size);
    for gen = 1:generations
        fitness = cluster_energy(population);
        [~, best_idx] = max(fitness);
        new_ch = cluster_nodes(population(best_idx), :);
    end
end

%% Function: Compute MST (Kruskal's Algorithm)
function MST_edges = compute_mst(nodes)
    N = size(nodes, 1);
    edges = [];
    for i = 1:N
        for j = i+1:N
            dist = norm(nodes(i,:) - nodes(j,:));
            edges = [edges; i, j, dist];
        end
    end
    edges = sortrows(edges, 3);
    parent = 1:N;
    MST_edges = [];
    function root = find(x)
        while parent(x) ~= x, x = parent(x); end
        root = x;
    end
    for k = 1:size(edges,1)
        a = find(edges(k,1)); b = find(edges(k,2));
        if a ~= b
            MST_edges = [MST_edges; edges(k,1:2)];
            parent(a) = b;
        end
    end
end

