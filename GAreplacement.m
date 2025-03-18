clc; clear; close all;

%% Initialize WSN Parameters
initial_energy = 3; % Joules
E_tx = 0.2; % Increased energy transmission cost (J/bit)
E_rx = 0.05; % Increased energy reception cost (J/bit)
num_iterations = 100; % Tabu Search iterations
numRounds = 200; % Simulation rounds

total_energy_consumption = zeros(numRounds, 1);
dead_nodes_per_round = zeros(numRounds, 1);
ch_replacement_count = zeros(numRounds, 1);

%% Network Deployment
network_size = 300;
sink_node = [250, 250];
numNodes = 100; % Example node count
nodes = [rand(numNodes, 1) * network_size, rand(numNodes, 1) * network_size];
energy_levels = initial_energy * ones(numNodes, 1);
death_threshold = 0.2 * initial_energy;

%% Clustering & CH Selection (K-means + Tabu Search)
optimal_k = 7;
[idx, C] = kmeans(nodes, optimal_k);

best_solutions = cell(1, optimal_k);
best_costs = inf * ones(1, optimal_k);

for i = 1:optimal_k
    cluster_nodes = nodes(idx == i, :);
    num_cluster_nodes = size(cluster_nodes, 1);
    tabu_list = zeros(num_cluster_nodes, num_iterations);
    
    for iter = 1:num_iterations
        current_solution = randperm(num_cluster_nodes);
        current_cost = sum(vecnorm(cluster_nodes(current_solution, :) - sink_node, 2, 2));
        
        for j = 1:num_cluster_nodes
            neighbor = current_solution;
            neighbor(j) = current_solution(mod(j+1, num_cluster_nodes) + 1);
            neighbor_cost = sum(vecnorm(cluster_nodes(neighbor, :) - sink_node, 2, 2));
            
            if neighbor_cost < current_cost
                current_solution = neighbor;
                current_cost = neighbor_cost;
            end
        end
        
        if current_cost < best_costs(i)
            best_solutions{i} = cluster_nodes(current_solution(1), :);
            best_costs(i) = current_cost;
        end
    end
end

%% Simulation Loop (CH Monitoring + GA-Based CH Replacement)
for round = 1:numRounds
    % Simulate realistic energy consumption per node
    for i = 1:numNodes
        if energy_levels(i) > 0
            energy_levels(i) = energy_levels(i) - (E_tx + E_rx + rand()*0.05); % Randomized depletion
            if energy_levels(i) < death_threshold
                energy_levels(i) = 0;
            end
        end
    end
    
    % Detect Dead Nodes & CH Monitoring
    dead_nodes_per_round(round) = sum(energy_levels == 0);
    total_energy_consumption(round) = sum(energy_levels);
    
    % CH Replacement using GA
    for i = 1:optimal_k
        ch_index = find(ismember(nodes, best_solutions{i}, 'rows'), 1);
        if ~isempty(ch_index) && energy_levels(ch_index) < death_threshold
            best_solutions{i} = ga_select_new_ch(nodes(idx == i, :), energy_levels(idx == i));
            ch_replacement_count(round) = ch_replacement_count(round) + 1;
            fprintf('Round %d: CH replaced in cluster %d\n', round, i);
        end
    end
end

%% Visualization
figure;
subplot(3,1,1);
plot(1:numRounds, dead_nodes_per_round, 'r', 'LineWidth', 2);
title('Dead Nodes per Round'); xlabel('Rounds'); ylabel('Dead Nodes'); grid on;

subplot(3,1,2);
plot(1:numRounds, total_energy_consumption, 'b', 'LineWidth', 2);
title('Total Energy Consumption'); xlabel('Rounds'); ylabel('Remaining Energy'); grid on;

subplot(3,1,3);
plot(1:numRounds, ch_replacement_count, 'g', 'LineWidth', 2);
title('CH Replacements per Round'); xlabel('Rounds'); ylabel('CH Changes'); grid on;

%% GA-Based CH Replacement Function
function new_ch = ga_select_new_ch(cluster_nodes, cluster_energy)
    population_size = 20;
    generations = 50;
    
    % Initialize population with random node indices
    population = randi([1, size(cluster_nodes, 1)], 1, population_size);
    
    for gen = 1:generations
        fitness = cluster_energy(population); % Higher energy = better fitness
        [~, best_idx] = max(fitness);
        new_ch = cluster_nodes(population(best_idx), :);
    end
    fprintf('GA selected new CH at (%.2f, %.2f)\n', new_ch(1), new_ch(2));
end
