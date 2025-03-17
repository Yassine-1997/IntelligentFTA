%% Full Implementation: Simulate and Plot CH Energy Dynamics
clear;
clc;
close all;

% Step 1: WSN Deployment and K-means Clustering
numNodes = 100; % Number of sensor nodes
areaSize = 1000; % Larger area size (1000x1000 units for realistic distances)
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
sink_node = [areaSize/2, areaSize/2]; % Sink node at center
scatter(sink_node(1), sink_node(2), 250, 'k', 'x', 'LineWidth', 3); % Sink node
title('WSN Deployment and K-means Clustering');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

% Step 2: Cluster Head (CH) Selection
CHs = zeros(numClusters, 1); % To store the selected CH indices
for cluster = 1:numClusters
    clusterNodes = find(idx == cluster); % Nodes in the current cluster
    % Select the node closest to the cluster center as the CH
    distances = vecnorm(nodes(clusterNodes, :) - clusterCenters(cluster, :), 2, 2);
    [~, closestNodeIdx] = min(distances);
    CHs(cluster) = clusterNodes(closestNodeIdx);
end

% Step 3: Initialize Energy Model
initialEnergy = 2; % Initial energy in Joules
E_elec = 50e-9; % Energy for transmitter/receiver circuitry (J/bit)
E_amp = 100e-12; % Energy for amplifier (J/bit/m^2)
dataSize = 4000; % Data size in bits per transmission
nodeEnergy = ones(numNodes, 1) * initialEnergy; % Initial energy for all nodes

% Energy levels of CHs (to track only CH energy)
CH_energy = nodeEnergy(CHs);

% Step 4: Simulation of Rounds
numRounds = 100; % Maximum number of rounds
deadNodes = zeros(numRounds, 1); % Track dead nodes per round

figure; % Create a figure for dynamic plotting
hold on;
for round = 1:numRounds
    fprintf('Round %d:\n', round);

    % Intra-cluster communication (Sensor nodes to CHs)
    for cluster = 1:numClusters
        clusterNodes = find(idx == cluster);
        for node = clusterNodes'
            if nodeEnergy(node) > 0 && node ~= CHs(cluster) % Skip the CH itself
                % Transmission energy to CH
                distToCH = norm(nodes(node, :) - nodes(CHs(cluster), :));
                E_tx = E_elec * dataSize + E_amp * dataSize * distToCH^2;
                nodeEnergy(node) = nodeEnergy(node) - E_tx;

                % Check if the node is dead
                if nodeEnergy(node) <= 0
                    fprintf('Node %d died in round %d\n', node, round);
                end
            end
        end
    end

    % Inter-cluster communication (CHs to Sink)
    for cluster = 1:numClusters
        if nodeEnergy(CHs(cluster)) > 0
            % Transmission energy to Sink
            distToSink = norm(nodes(CHs(cluster), :) - sink_node);
            E_tx = E_elec * dataSize + E_amp * dataSize * distToSink^2;
            nodeEnergy(CHs(cluster)) = nodeEnergy(CHs(cluster)) - E_tx;

            % Check if the CH is dead
            if nodeEnergy(CHs(cluster)) <= 0
                fprintf('Cluster Head %d died in round %d\n', CHs(cluster), round);

                % Stop simulation when the first CH dies
                fprintf('Simulation stopped: First CH died.\n');
                break;
            end
        end
    end

    % Update CH energy levels in the tracking matrix
    CH_energy_over_time(round, :) = nodeEnergy(CHs); % Record energy levels

    % Dynamic Plot: Energy levels of CHs
    clf; % Clear the current figure
    for cluster = 1:numClusters
        plot(1:round, CH_energy_over_time(1:round, cluster), '-o', 'LineWidth', 2);
        hold on;
    end
    title(sprintf('Round %d: Cluster Head Energy Levels', round));
    xlabel('Rounds');
    ylabel('Energy (J)');
    ylim([0, initialEnergy]); % Set y-axis limits
    grid on;
    drawnow; % Update the plot dynamically

    % Stop simulation if a CH has died
    if any(nodeEnergy(CHs) <= 0)
        break; % Exit the simulation
    end
end

