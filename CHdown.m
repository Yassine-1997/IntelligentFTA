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
dead_chs_per_round = zeros(numRounds, 1); % Track dead CHs per round
dead_clusters_per_round = zeros(numRounds, 1); % Track number of dead clusters per round
dead_nodes_per_round = zeros(numRounds, 1);

%% Clustering & CH Selection (K-means)
optimal_k = 10; % Number of clusters (CHs)
[idx, C] = kmeans(nodes, optimal_k); % K-means for clustering
best_solutions = cell(1, optimal_k);
for i = 1:optimal_k
    cluster_nodes = nodes(idx == i, :);
    best_solutions{i} = cluster_nodes(randi(size(cluster_nodes, 1)), :);
end

% Initial WSN Visualization (Before Simulation)
figure;
hold on;

% Plot all nodes, coloring them by cluster
colors = lines(optimal_k); % Generates 'optimal_k' unique colors
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

% Compute MST (Kruskal's Algorithm) for CHs and Sink Node
CHs_with_sink = [cell2mat(best_solutions'); sink_node];  % Include Sink
MST_edges = compute_mst(CHs_with_sink);

% Plot MST edges (connecting only CHs and the Sink)
for i = 1:size(MST_edges, 1)
    node1 = CHs_with_sink(MST_edges(i, 1), :);
    node2 = CHs_with_sink(MST_edges(i, 2), :);
    plot([node1(1), node2(1)], [node1(2), node2(2)], 'k-', 'LineWidth', 1.5, ...
         'DisplayName', 'MST Edges');
end

title('Initial WSN Configuration with MST');
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');
legend();
grid on;
hold off;
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
    
    % Track Dead CHs
    dead_chs = zeros(1, optimal_k); % Initialize CH death status
    for i = 1:optimal_k
        ch_index = find(ismember(nodes, best_solutions{i}, 'rows'), 1);
        if energy_levels(ch_index) < death_threshold
            dead_chs(i) = 1; % CH is dead
        end
    end
    dead_chs_per_round(round) = sum(dead_chs); % Count dead CHs in this round
    
    % Track Dead Nodes per Round (for other metrics)
    dead_nodes_per_round(round) = sum(energy_levels == 0);
    total_energy_consumption(round) = sum(energy_levels);
    alive_nodes_time(round) = sum(energy_levels > 0);
    
    % Track FND and HND
    if FND == 0 && dead_nodes_per_round(round) > 0
        FND = round;
    end
    if HND == 0 && dead_nodes_per_round(round) >= numNodes / 2
        HND = round;
    end
    
    % Track Dead Clusters
    dead_clusters = zeros(1, optimal_k); % Initialize dead cluster status
    for i = 1:optimal_k
        cluster_nodes = nodes(idx == i, :);
        cluster_energy = energy_levels(idx == i);
        
        if all(cluster_energy == 0) % If all nodes in cluster are dead
            dead_clusters(i) = 1; % Mark the cluster as dead
        end
    end
    dead_clusters_per_round(round) = sum(dead_clusters); % Count dead clusters in this round

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
    
    % Periodic Plot (e.g., every 1000 rounds)
    if mod(round, 1000) == 0
        % Compute MST (Kruskal's Algorithm) for CHs and Sink Node
        CHs_with_sink = [cell2mat(best_solutions'); sink_node];  % Include Sink
        MST_edges = compute_mst(CHs_with_sink);

        % Visualization: WSN with MST & Dead Clusters
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
            if dead_chs(i) == 0  % If CH is not dead
                plot(best_solutions{i}(1), best_solutions{i}(2), 'o', ...
                     'MarkerSize', 8, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k', ...
                     'LineWidth', 1.5, 'DisplayName', sprintf('CH Cluster %d', i));
            else  % If CH is dead, mark with red "X"
                plot(best_solutions{i}(1), best_solutions{i}(2), 'rx', 'MarkerSize', 10, ...
                     'DisplayName', sprintf('Dead CH Cluster %d', i));
            end
        end

        % Plot Dead Clusters (clusters with all dead nodes)
        for i = 1:optimal_k
            if dead_clusters(i) == 1
                plot(mean(nodes(idx == i, 1)), mean(nodes(idx == i, 2)), 'kx', 'MarkerSize', 12, 'LineWidth', 2, ...
                     'DisplayName', sprintf('Dead Cluster %d', i));
            end
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

        title(sprintf('WSN with MST and Dead Clusters at Round %d', round));
        xlabel('X Coordinate (m)');
        ylabel('Y Coordinate (m)');
        legend();
        grid on;
        hold off;
    end
end

%% Plot Dead Clusters per Round
figure;
plot(1:numRounds, dead_clusters_per_round, 'LineWidth', 2);
xlabel('Round');
ylabel('Dead Clusters');
title('Dead Clusters per Round');
grid on;

%% Compute MST (Kruskal's Algorithm) for CHs and Sink Node
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
