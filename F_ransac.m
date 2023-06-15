function [correspondences_robust] = F_ransac(correspondences, varargin)
    % Diese Funktion implementiert den RANSAC-Algorithmus zur Bestimmung von
    % robusten Korrespondenzpunktpaaren
    
    %% Input parser
    x1_pixel = [correspondences(1:2, :); ones(1, size(correspondences, 2))];
    x2_pixel = [correspondences(3:4, :); ones(1, size(correspondences, 2))];
    
    % Set default parameter values
    defaultEpsilon = 0.5;
    defaultP = 0.5;
    defaultTolerance = 0.01;
    
    % Create input parser
    parser = inputParser;

    parser.addOptional('epsilon', defaultEpsilon, @(x) validateattributes(x, {'numeric'}, {'scalar', 'nonempty', '>', 0, '<', 1}));
    parser.addOptional('p', defaultP, @(x) validateattributes(x, {'numeric'}, {'scalar', 'nonempty', '>', 0, '<', 1}));
    parser.addOptional('tolerance', defaultTolerance, @(x) validateattributes(x, {'numeric'}, {'scalar', 'nonempty', 'nonnegative'}));
    
    % Parse input arguments
    parse(parser, varargin{:});
    epsilon = parser.Results.epsilon;
    p = parser.Results.p;
    tolerance = parser.Results.tolerance;
    
    %% RANSAC Algorithmus Vorbereitung
    k = 8;
    s = ceil(log(1-p)/log(1-(1-epsilon)^k));
    largest_set_size = 0;
    largest_set_dist = inf;
    largest_set_F = zeros(3,3);
  
    
    %% RANSAC Algorithmus
    for i = 1:s
        % Select k random points
        rand_indices = randperm(size(correspondences, 2), k);
        x1_rand = x1_pixel(:, rand_indices);
        x2_rand = x2_pixel(:, rand_indices);
        
        % Estimate the fundamental matrix using the eight-point algorithm
        F = epa([x1_rand; x2_rand]);
        
        % Calculate the Sampson distance for all points
        sd = sampson_dist(F, x1_pixel, x2_pixel);
        
        % Define the consensus set
        consensus_set = sd < tolerance;
        
        % Calculate the number of points in the consensus set and the total Sampson distance
        consensus_set_size = sum(consensus_set);
        consensus_set_dist = sum(sd(consensus_set));
        
        % Update the largest consensus set if necessary
        if consensus_set_size > largest_set_size || (consensus_set_size == largest_set_size && consensus_set_dist < largest_set_dist)
            largest_set_size = consensus_set_size;
            largest_set_dist = consensus_set_dist;
            largest_set_F = F;
        end
    end
    
    % Return the robust correspondences
    correspondences_robust = correspondences(:, sd < tolerance);
end