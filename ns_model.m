clc;
clear;
clear java;

% add JIDT path -> Gives warning error if ENTIRE filepath is not specified
javaaddpath('/home/austyn/Documents/MATLAB/infodynamics-dist-1.3/infodynamics.jar')

% add paths to JIDT CA octave & matlab code
% addpath('/home/austyn/Documents/MATLAB/infodynamics-dist-1.3/demos/octave/CellularAutomata');
addpath('/home/austyn/Documents/MATLAB/infodynamics-dist-1.3/demos/octave');

%% === START Variable System Parameters === %%
% These 'variable' initial system parameters can be edited to change the outcome
% of the simulation

% Debug flags - set to true to print out debugging messages
verbose = false;

%% INITIALISATION
% Initialise cell grid of size n by m
timesteps = 200; %number of time steps
roadlength = 50; %length of 'road'
c = cell(timesteps, roadlength);
max_roadlength = roadlength;


% Maximum Velocity:
% the max number of cells a vehicle can move per time step
v_max = 5;

% Vehicle generation method
% Options = 'random', 'naive', 'static'
initialisation_method = 'random';

%% Simulation mode
% Choose mode: 'single' or 'multiple'
simulationMode = 'single';

% Number of simulation rounds
num_sims = 10000;

%% Plot graph of multiple simulations
plotGraph = true;

%% Calculate or Plot Transfer Entropy (single simulation only)
calcTE = true;
plotTE = false;


%% === END Variable System Parameters === %%

% Initialise variable to track which row we are up to
row_counter = 1;

% disp('Initial grid (t = 0)');
% disp(c);

roadLengthTenth = max_roadlength/10;

% Pre-allocate arrays of size 'num_sims'
% Arrays store averaged values that will be analyzed later
time_average_flow_array = zeros(num_sims, roadLengthTenth);
time_average_density_array = zeros(num_sims, roadLengthTenth);

num_cars_array = zeros(num_sims, 1);
missing_cars_array = zeros(num_sims, 1);

system_size_counter = 0; % probs not needed

average_TE_array = zeros(num_sims, roadLengthTenth);
average_MI_array = zeros(num_sims, roadLengthTenth);


%% TE and MI calculation and plotting variables
options.plotOptions.plotRows = timesteps; % number of timesteps
options.plotOptions.plotCols = roadlength; % length of road
options.plotOptions.plotStartRow = 1; % plot from row # onwards
options.plotOptions.plotStartCol = 1; % plot from column # onwards

base = v_max + 2; % This allows for the multiple states
%             measureId = 'active';
%             measureId = 'transfer';
%             measureId = 'mutual';
measureId = 'transferAndMutual';

measureParams.k = 1; % History length of 16 for info dynamics measures

%not sure if measureParams.j is right??
measureParams.j = 1;

if not(isfield(options, 'plotOptions'))
    options.plotOptions = {}; % Create it ready for plotRawCa etc
end
if not(isfield(options, 'saveImages'))
    options.saveImages = false;
end
if not(isfield(options, 'saveImagesFormat'))
    options.saveImagesFormat = 'eps';
end
if not(isfield(options, 'plotRawCa'))
    options.plotRawCa = true;
end

if (strcmp(options.saveImagesFormat, 'eps'))
    printDriver = 'epsc'; % to force colour
    fontSize = 32;
else
    printDriver = options.saveImagesFormat;
    fontSize = 13;
end
figNum = 2;


%% START SIMULATION
fprintf('Simulation started, Time %s\n', datestr(now));
for h = roadLengthTenth:roadLengthTenth:max_roadlength
    roadlength = h; % set the current system road length in each loop
    fprintf('Sim round %d of 10\n', (roadlength*10)/max_roadlength);
    fprintf('System size %d, Time %s\n', roadlength, datestr(now));
    system_size_counter = system_size_counter + 1; % increment on each loop
    
    for k = 1:num_sims
        % Initialise/clean grid
        for i = 1:timesteps %down y
            for j = 1:roadlength %right x
                c{i, j} = ' ';
            end
        end

        %% Initialise vehicles (locations and speeds)
        num_cars = 0;

        if strcmp(initialisation_method, 'random')
            %% Random vehicle initialisation
            % Randomly generate a capped value for number of cars for this round
            % only. The aim is to ensure an even spread of simulations for a wide
            % range of vehicle numbers.
            max_num_cars = round(rand * roadlength);

            for j = 1:roadlength
                % Check each step that we do not exceed our predetermined value
                if num_cars >= max_num_cars
                    break;
                end
                % generate vehicle with random speed rounded to nearest int
                speed = round(v_max * rand, 0);
                c{1, j} = speed;
                num_cars = num_cars + 1;
            end
        elseif strcmp(initialisation_method, 'naive')
            %% Naive vehicle initialisation
            naive_num_cars = 10; % set number of cars to be created using this naive method

            for j = 1:roadlength
                % Check each step that we do not exceed our predetermined value
                if num_cars >= naive_num_cars
                    break;
                end
                % generate vehicle with random speed rounded to nearest int
                speed = round(v_max * rand, 0);
                c{1, j} = speed;
                num_cars = num_cars + 1;
            end
        elseif strcmp(initialisation_method, 'static')
        %% Previous static method of seeding cars
        % Good for replicating exact base/start conditions, not good for 
        % large sets
            num_cars = 10;
            c{1, 1} = 3;
            c{1, 2} = 3;
            c{1, 3} = 3;
            c{1, 4} = 3;
            c{1, 5} = 3;
            c{1, 6} = 1;
            c{1, 7} = 3;
            c{1, 8} = 3;
            c{1, 9} = 1;
            c{1, 10} = 3;
        end

        num_cars_array(k) = num_cars;

        for j = 1:timesteps-1
            row_counter = j;
            for i = 1:roadlength
                if (c{row_counter, i} >= 0 && c{row_counter, i} ~= ' ')
                    % Get current velocity
                    velocity = c{row_counter, i};

                    % set new velo = curr velo
                    % new_velocity = curr_velocity;

                    % print position out
                    if verbose fprintf('Position: %d\n', i); end

                    %%% CALCULATE GAP %%%
                    % Calculate gap to next vehicle via recursive function
                    gap = recursegap(c, row_counter, i, roadlength);
                    if verbose fprintf('Row: %d Gap: %d\n', row_counter, gap); end

                    %%% 1. ACCELERATION %%%
                    if gap > velocity + 1 && velocity < v_max
                        % If gap greater than current velocity and under v_max,
                        % car can accelerate
                        % Increase velocity
                        velocity = velocity + 1;
                        % testing ending if statement so each step is independent
                    end

                    %%% 2. SLOWING DOWN (Due to other cars) %%%
                    % If a vehicle at site i sees the next vehicle at
                    % site i + j (with j =< v), it reduces its speed to
                    % j - 1 [v -> j - 1]

                    if gap <= velocity && velocity ~= 0
                        % Otherwise if gap is LESS than the velocity value
                        % incremented by one and the velocity is NOT zero,
                        % decelerate the car

                        % Site i is the current site. j is the number of steps
                        % to be taken and MUST be EQUAL TO OR LESS than
                        % velocity v

                        % Most likely need to re-evaluate this code

                        % set new velocity of vehicle as the smaller of two
                        % numbers
                        if gap == velocity
                            % do nothing? or assign like below???
                        elseif gap + 1 == velocity
                            velocity = velocity - 1;
                        else
                            velocity = min(velocity - 1, gap);
                        end
                        if verbose fprintf('new_velocity: %d\n', velocity); end
                    end
                    
                    %%% 3. RANDOMISATION %%%
                    %% Make randomisation step independent from accel/deceleration
                    % i.e. it can accelerate and brake immediately
                    if velocity > 0 && gap ~= 1
                        % if rand number less than p threshold, then reduce velocity
                        % probability p
                        p = 0.25;

                        if rand < p && velocity ~= 0
                            velocity = velocity - 1;
                        end
                    end

                    %%% CAR MOTION %%%
                    % Check modulo and if road has wrapped properly (road is of
                    % length 'm')
                    newPosition = mod(i + velocity, roadlength);
                    if newPosition == 0
                        newPosition = roadlength;
                    end
                    %[TODO: Write catch case so cars don't drive on top of each other???]
                    % Probably not needed if random seeding is fixed

                    if verbose fprintf('new velocity = %d\n', velocity); end

                    %% Update car movement in the next row
                    c{row_counter + 1, newPosition} = velocity;
                end
            end
        end

        %% Print grid after t timesteps
    %     fprintf('After %d timesteps (t = %d)\n', n, n);
    %     disp(c);

        %% Detect missing cars:
        num_cars_at_end = 0;
        for j = 1:roadlength %right x
            if c{timesteps, j} ~= ' '
                num_cars_at_end = num_cars_at_end + 1;
            end
        end
        missing_cars_array(k) = num_cars - num_cars_at_end;
        if missing_cars_array(k) > 0
            fprintf('Missing cars: %d\n', num_cars - num_cars_at_end);
        end

        %%% Calculating time averaged density and flow
        % Define + Reset counter variables on each loop
        tafsum = 0;
        tadsum = 0;
        
        %% Calculate time averaged density
        % Calculates density for the centre column in the roadway
        % This should help avoid skewing the average when run over enough
        % timesteps?

        tadTargetColumn = ceil(roadlength/2);
        
        for i = 1:timesteps
            if verbose fprintf('c{%d, 1}: %d', i, c{i, 1}); end
            % if cell is occupied (i.e is not empty), add one to the total sum
            if c{i, 1} ~= ' '
                tadsum = tadsum + 1;
            end

            if verbose disp(tadsum); end
        end

        tad = tadsum/timesteps; 
        % fprintf('Time Averaged Density: %.2f\n', tad);

        %% Calculate time averaged flow
        % Calculates flow between column 1 and 2

        % cellgrid{time steps, road length}
        % for each row
        for i = 1:timesteps-1
            if verbose fprintf('c{%d, 1}: %d', i, c{i, 1}); end
        %     for each value up to max velocity
            for j = 1:v_max
                % From the perpective of the vehicle moving across (right), 
                % check each cell moving right up to v_max
                % temp value equals cell under target cell minus j value
                % Modulo to check for road wrap-around
                % i.e in this case mod(1 - j, roadlength) where '1' is the
                % first column
                temp1 = mod(1 - j, roadlength);
%                 fprintf('mod: %d, temp1: %d, roadlength: %d\n', (1-j), temp1, roadlength);
                if temp1 == 0
                    temp1 = roadlength;
                end
                % if cell is not empty and is greater than j (i.e. moving)
                if c{i+1, temp1} ~= ' ' && c{i+1, temp1} >= j %% TOD0: CHECK IF = SIGN should be here
                    tafsum = tafsum + 1;
                    if verbose fprintf('Flow at timestep %d: %d %f\n', i, c{i+1, temp1}, c{i+1, temp1}/timesteps); end
                end
            end
            if verbose fprintf('Time Averaged Flow cumulative sum: ', tafsum); end
        end

        % WHAT HAPPENS IF TIME AVE FLOW IS ZERO??? (need catching case)
        taf = tafsum/timesteps;
        % fprintf('Time Averaged Flow: %.2f\n', taf);

        % Save averages to an array
        time_average_density_array(k, system_size_counter) = tad;
        time_average_flow_array(k, system_size_counter) = taf; 

        
        %% Transfer Entropy Toolbox (JIDT)
        %% Functions for calculating & plotting TE based on JIDT by Joseph Lizier
        if calcTE == true


            %% Convert NS model data to feed into here
            % Call function that converts the cell data from the above NS model
            % simulation into a matrix
            % NStoTEMatrix.m arguments: NStoTEMatrix(cellgrid, timesteps, roadLength, max_velocity)
            caStates = NStoTEMatrix(c, timesteps, roadlength, v_max);
            %% Outputs caStates matrix for TE calculation
            % caStates


            % convert the states to a format usable by java:
            caStatesJInts = octaveToJavaIntMatrix(caStates);


            plottedOne = false;
            
            
            %============================
            % Mutual Information
            % Adapted from JIDT Cellular Automata Demos to calculate Mutual
            % Information
            if ((ischar(measureId) && (strcmpi('mutual', measureId) || strcmpi('transferAndMutual', measureId) || strcmpi('all', measureId) || strcmpi('mutualinfo', measureId))) || ...
                (not(ischar(measureId)) && ((measureId == 1) || (measureId == -1))))
                % Compute apparent transfer entropy
                if (measureParams.j == 0)
                    error('Cannot compute mutual info from a cell to itself (setting measureParams.j == 0)');
                end
                mutualInfoCalc = javaObject('infodynamics.measures.discrete.MutualInformationCalculatorDiscrete', base, measureParams.k);
                %^ this section takes slightly diff args to TE equivalent
                
                mutualInfoCalc.initialise();
%                 mutualInfoCalc.addObservations(caStatesJInts, measureParams.j);
%                 mutualInfoCalc.addObservations(caStatesJInts, 1, 2);
                for i = 0:(roadlength-1)
                    for j = 1:v_max
    %                     fprintf('%d\n', i);
                        % Adjusting for road wrapping when selecting target
                        % column
                        if i + j < roadlength
                            targetColumn = i + j;
                        elseif i + j >= roadlength
                            targetColumn = mod(i+j, roadlength);
                        end
                        mutualInfoCalc.addObservations(caStatesJInts, i, targetColumn);
                    end
                end
                %^ .addObservations within
                % 'MutualInformationCalculatorDiscrete' doesn't work in a
                % similar way to 'TransferEntropyCalculatorDiscrete'
                
                avMutualInfo = mutualInfoCalc.computeAverageLocalOfObservations();
                % fprintf('Average apparent transfer entropy (j=%d) = %.4f\n', measureParams.j, avMutualInfo);

                % Store average MI
                average_MI_array(k, system_size_counter) = avMutualInfo;

                if plotTE == true
                    javaLocalValues = mutualInfoCalc.computeLocalFromPreviousObservations(caStatesJInts, measureParams.j);
                    localValues = javaMatrixToOctave(javaLocalValues);
                    if (isfield(options, 'movingFrameSpeed'))
                        % User has requested us to evaluate information dynamics with a moving frame of reference
                        % (see Lizier and Mahoney paper).
                        % Need to shift the computed info dynamics back (to compensate for earlier shift to CA states:
                        localValues = accumulateShift(localValues, options.movingFrameSpeed);
                    end

                    figure(figNum)
                    figNum = figNum + 1;
                    plotLocalInfoValues(localValues, options.plotOptions);
                    if (options.saveImages)
                        set(gca, 'fontsize', fontSize);
                        colorbar('fontsize', fontSize);
                        print(sprintf('figures/%s-transfer-k%d-j%d.%s', ruleString, measureParams.k, measureParams.j, options.saveImagesFormat), sprintf('-d%s', printDriver));
                    end
                    plottedOne = true;
                end
            end

            
            
            % Make the local information dynamics measurement(s)

            %============================
            % Active information storage
            if ((ischar(measureId) && (strcmpi('active', measureId) || strcmpi('all', measureId))) || ...
                (not(ischar(measureId)) && ((measureId == 0) || (measureId == -1))))
                % Compute active information storage
                activeCalc = javaObject('infodynamics.measures.discrete.ActiveInformationCalculatorDiscrete', base, measureParams.k);
                activeCalc.initialise();
                activeCalc.addObservations(caStatesJInts);
                avActive = activeCalc.computeAverageLocalOfObservations();
                fprintf('Average active information storage = %.4f\n', avActive);
                
                if plotTE == true
                    javaLocalValues = activeCalc.computeLocalFromPreviousObservations(caStatesJInts);
                    localValues = javaMatrixToOctave(javaLocalValues);
            % 		if (isfield(options, 'movingFrameSpeed'))
            % 			% User has requested us to evaluate information dynamics with a moving frame of reference
            % 			% (see Lizier and Mahoney paper).
            % 			% Need to shift the computed info dynamics back (to compensate for earlier shift to CA states:
            % 			localValues = accumulateShift(localValues, options.movingFrameSpeed);
            % 		end

                    figure(figNum)
                    figNum = figNum + 1;
                    plotLocalInfoValues(localValues, options.plotOptions);
                    if (options.saveImages)
                        set(gca, 'fontsize', fontSize);
                        colorbar('fontsize', fontSize);
                        print(sprintf('figures/%s-active-k%d.%s', ruleString, measureParams.k, options.saveImagesFormat), sprintf('-d%s', printDriver));
                    end
                    plottedOne = true;
                end
            end

            %============================
            % Apparent transfer entropy
            if ((ischar(measureId) && (strcmpi('transfer', measureId) || strcmpi('transferAndMutual', measureId) || strcmpi('all', measureId) || strcmpi('apparenttransfer', measureId))) || ...
                (not(ischar(measureId)) && ((measureId == 1) || (measureId == -1))))
                % Compute apparent transfer entropy
                if (measureParams.j == 0)
                    error('Cannot compute transfer entropy from a cell to itself (setting measureParams.j == 0)');
                end
                transferCalc = javaObject('infodynamics.measures.discrete.TransferEntropyCalculatorDiscrete', base, measureParams.k);
                transferCalc.initialise();
                transferCalc.addObservations(caStatesJInts, measureParams.j);
                avTransfer = transferCalc.computeAverageLocalOfObservations();
                % fprintf('Average apparent transfer entropy (j=%d) = %.4f\n', measureParams.j, avTransfer);

                % Store average apparent TE
                average_TE_array(k, system_size_counter) = avTransfer;

                if plotTE == true
                    javaLocalValues = transferCalc.computeLocalFromPreviousObservations(caStatesJInts, measureParams.j);
                    localValues = javaMatrixToOctave(javaLocalValues);
                    if (isfield(options, 'movingFrameSpeed'))
                        % User has requested us to evaluate information dynamics with a moving frame of reference
                        % (see Lizier and Mahoney paper).
                        % Need to shift the computed info dynamics back (to compensate for earlier shift to CA states:
                        localValues = accumulateShift(localValues, options.movingFrameSpeed);
                    end

                    figure(figNum)
                    figNum = figNum + 1;
                    plotLocalInfoValues(localValues, options.plotOptions);
                    if (options.saveImages)
                        set(gca, 'fontsize', fontSize);
                        colorbar('fontsize', fontSize);
                        print(sprintf('figures/%s-transfer-k%d-j%d.%s', ruleString, measureParams.k, measureParams.j, options.saveImagesFormat), sprintf('-d%s', printDriver));
                    end
                    plottedOne = true;
                end
            end

            %============================
            % Complete transfer entropy, conditioning on all other sources
            if ((ischar(measureId) && (strcmpi('transfercomplete', measureId) || strcmpi('completetransfer', measureId) || strcmpi('all', measureId))) || ...
                (not(ischar(measureId)) && ((measureId == 2) || (measureId == -1))))
                % Compute complete transfer entropy
                if (measureParams.j == 0)
                    error('Cannot compute transfer entropy from a cell to itself (setting measureParams.j == 0)');
                end
                transferCalc = javaObject('infodynamics.measures.discrete.ConditionalTransferEntropyCalculatorDiscrete', ...
                    base, measureParams.k, neighbourhood - 2);
                transferCalc.initialise();
                % Offsets of all parents can be included here - even 0 and j, these will be eliminated internally:
                transferCalc.addObservations(caStatesJInts, measureParams.j, octaveToJavaIntArray(fullSetOfParents));
                avTransfer = transferCalc.computeAverageLocalOfObservations();
                fprintf('Average complete transfer entropy (j=%d) = %.4f\n', measureParams.j, avTransfer);
                
                if plotTE == true
                    javaLocalValues = transferCalc.computeLocalFromPreviousObservations(caStatesJInts, ...
                                measureParams.j, octaveToJavaIntArray(fullSetOfParents));
                    localValues = javaMatrixToOctave(javaLocalValues);
                    if (isfield(options, 'movingFrameSpeed'))
                        % User has requested us to evaluate information dynamics with a moving frame of reference
                        % (see Lizier and Mahoney paper).
                        % Need to shift the computed info dynamics back (to compensate for earlier shift to CA states:
                        localValues = accumulateShift(localValues, options.movingFrameSpeed);
                    end

                    figure(figNum)
                    figNum = figNum + 1;
                    plotLocalInfoValues(localValues, options.plotOptions);
                    if (options.saveImages)
                        set(gca, 'fontsize', fontSize);
                        colorbar('fontsize', fontSize);
                        print(sprintf('figures/%s-transferComp-k%d-j%d.%s', ruleString, measureParams.k, measureParams.j, options.saveImagesFormat), sprintf('-d%s', printDriver));
                    end
                    plottedOne = true;
                end
            end

            %============================
            % Separable information
            if ((ischar(measureId) && (strcmpi('separable', measureId) || strcmpi('all', measureId))) || ...
                (not(ischar(measureId)) && ((measureId == 3) || (measureId == -1))))
                % Compute separable information
                separableCalc = javaObject('infodynamics.measures.discrete.SeparableInfoCalculatorDiscrete', ...
                    base, measureParams.k, neighbourhood - 1);
                separableCalc.initialise();
                % Offsets of all parents can be included here - even 0 and j, these will be eliminated internally:
                separableCalc.addObservations(caStatesJInts, octaveToJavaIntArray(fullSetOfParents));
                avSeparable = separableCalc.computeAverageLocalOfObservations();
                fprintf('Average separable information = %.4f\n', avSeparable);
                
                if plotTE == true
                    javaLocalValues = separableCalc.computeLocalFromPreviousObservations(caStatesJInts, ...
                                octaveToJavaIntArray(fullSetOfParents));
                    localValues = javaMatrixToOctave(javaLocalValues);
                    if (isfield(options, 'movingFrameSpeed'))
                        % User has requested us to evaluate information dynamics with a moving frame of reference
                        % (see Lizier and Mahoney paper).
                        % Need to shift the computed info dynamics back (to compensate for earlier shift to CA states:
                        localValues = accumulateShift(localValues, options.movingFrameSpeed);
                    end

                    figure(figNum)
                    figNum = figNum + 1;
                    plotLocalInfoValues(localValues, options.plotOptions);
                    if (options.saveImages)
                        set(gca, 'fontsize', fontSize);
                        colorbar('fontsize', fontSize);
                        print(sprintf('figures/%s-separable-k%d.%s', ruleString, measureParams.k, options.saveImagesFormat), sprintf('-d%s', printDriver));
                    end
                    plottedOne = true;
                end
            end

            %============================
            % Entropy
            if ((ischar(measureId) && (strcmpi('entropy', measureId) || strcmpi('all', measureId))) || ...
                (not(ischar(measureId)) && ((measureId == 4) || (measureId == -1))))
                % Compute entropy
                entropyCalc = javaObject('infodynamics.measures.discrete.EntropyCalculatorDiscrete', ...
                    base);
                entropyCalc.initialise();
                entropyCalc.addObservations(caStatesJInts);
                avEntropy = entropyCalc.computeAverageLocalOfObservations();
                fprintf('Average entropy = %.4f\n', avEntropy);
                
                if plotTE == true
                    javaLocalValues = entropyCalc.computeLocalFromPreviousObservations(caStatesJInts);
                    localValues = javaMatrixToOctave(javaLocalValues);
                    if (isfield(options, 'movingFrameSpeed'))
                        % User has requested us to evaluate information dynamics with a moving frame of reference
                        % (see Lizier and Mahoney paper).
                        % Need to shift the computed info dynamics back (to compensate for earlier shift to CA states):
                        % (Note for entropy, the shifts to and back don't make any difference)
                        localValues = accumulateShift(localValues, options.movingFrameSpeed);
                    end

                    figure(figNum)
                    figNum = figNum + 1;
                    plotLocalInfoValues(localValues, options.plotOptions);
                    if (options.saveImages)
                        set(gca, 'fontsize', fontSize);
                        colorbar('fontsize', fontSize);
                        print(sprintf('figures/%s-entropy.%s', ruleString, options.saveImagesFormat), sprintf('-d%s', printDriver));
                    end
                    plottedOne = true;
                end
            end

            %============================
            % Entropy rate
            if ((ischar(measureId) && (strcmpi('entropyrate', measureId) || strcmpi('all', measureId))) || ...
                (not(ischar(measureId)) && ((measureId == 5) || (measureId == -1))))
                % Compute entropy rate
                entRateCalc = javaObject('infodynamics.measures.discrete.EntropyRateCalculatorDiscrete', base, measureParams.k);
                entRateCalc.initialise();
                entRateCalc.addObservations(caStatesJInts);
                avEntRate = entRateCalc.computeAverageLocalOfObservations();
                fprintf('Average entropy rate = %.4f\n', avEntRate);
                
                if plotTE == true
                    javaLocalValues = entRateCalc.computeLocalFromPreviousObservations(caStatesJInts);
                    localValues = javaMatrixToOctave(javaLocalValues);
                    if (isfield(options, 'movingFrameSpeed'))
                        % User has requested us to evaluate information dynamics with a moving frame of reference
                        % (see Lizier and Mahoney paper).
                        % Need to shift the computed info dynamics back (to compensate for earlier shift to CA states:
                        localValues = accumulateShift(localValues, options.movingFrameSpeed);
                    end

                    figure(figNum)
                    figNum = figNum + 1;
                    plotLocalInfoValues(localValues, options.plotOptions);
                    if (options.saveImages)
                        set(gca, 'fontsize', fontSize);
                        colorbar('fontsize', fontSize);
                        print(sprintf('figures/%s-entrate-k%d.%s', ruleString, measureParams.k, options.saveImagesFormat), sprintf('-d%s', printDriver));
                    end
                    plottedOne = true;
                end
            end

            %============================
            % Excess entropy
            if ((ischar(measureId) && (strcmpi('excess', measureId) || strcmpi('all', measureId))) || ...
                (not(ischar(measureId)) && ((measureId == 6) || (measureId == -1))))
                % Compute excess entropy
                excessEntropyCalc = javaObject('infodynamics.measures.discrete.PredictiveInformationCalculatorDiscrete', base, measureParams.k);
                excessEntropyCalc.initialise();
                excessEntropyCalc.addObservations(caStatesJInts);
                avExcessEnt = excessEntropyCalc.computeAverageLocalOfObservations();
                fprintf('Average excess entropy = %.4f\n', avExcessEnt);
                
                if plotTE == true
                    javaLocalValues = excessEntropyCalc.computeLocalFromPreviousObservations(caStatesJInts);
                    localValues = javaMatrixToOctave(javaLocalValues);
                    if (isfield(options, 'movingFrameSpeed'))
                        % User has requested us to evaluate information dynamics with a moving frame of reference
                        % (see Lizier and Mahoney paper).
                        % Need to shift the computed info dynamics back (to compensate for earlier shift to CA states:
                        localValues = accumulateShift(localValues, options.movingFrameSpeed);
                    end

                    figure(figNum)
                    figNum = figNum + 1;
                    plotLocalInfoValues(localValues, options.plotOptions);
                    if (options.saveImages)
                        set(gca, 'fontsize', fontSize);
                        colorbar('fontsize', fontSize);
                        print(sprintf('figures/%s-excessentropy-k%d.%s', ruleString, measureParams.k, options.saveImagesFormat), sprintf('-d%s', printDriver));
                    end
                    plottedOne = true;
                end
            end

%             if (not(plottedOne))
%                 error(sprintf('Supplied measureId %s did not match any measurement types', measureId));
%             end
            
            %% Save ASCII text file of matrix
            % Uncomment below line to generate text file
            % dlmwrite('TEmatrix.txt', caStates, 'delimiter', ' ', 'precision', 1);
        end
    end
end





%% Plotter
fprintf('Plotting started, Time %s\n', datestr(now));
if plotGraph == true
    % initialise array
    timeAveragedData = zeros(num_sims, 4*max_roadlength/10);

    % Sort flow & density for each simulation across multiple system sizes
    % and store them in an array
    for i = 1:system_size_counter
        colNum = i*4;
        timeAveragedData(1:num_sims, colNum-3:colNum) = sortrows([time_average_density_array(:, i) time_average_flow_array(:, i) average_TE_array(:, i) average_MI_array(:, i)], 1);
    end
    
    %% Scatter graph of single
    figure
%     y = smooth(time_average_density_array(:,5), time_average_flow_array(:,5), 0.1, 'rlowess');
%     plot(time_average_density_array(:,5),y,'r');

    scatter(time_average_density_array(:,5), time_average_flow_array(:,5), 'filled');
    hold on; % needs to be here or scatter plot wins race condition...
    y = smooth(timeAveragedData(:, 17), timeAveragedData(:, 18), 0.1, 'lowess');
    plot(timeAveragedData(:, 17),y,'r');
    legend( ...
        'dots', ...
        'y5' ...
    );
    
    title('Averaged Density vs Flow')
    xlabel('Time Averaged Density')
    ylabel('Time Averaged Flow') 
    
    %% Smoothed Flow vs Density data (Multiple System Sizes)
    figure
    % subplot(3,1,2)
    % scatter(time_average_density_array, time_average_flow_array, 'filled');
    % title('Averaged Density vs Flow')
    % xlabel('Time Averaged Density')
    % ylabel('Time Averaged Flow')
        
    % sort number of cars for easy graph reading
    num_cars_array_sorted = sort(num_cars_array);

    % == Plotting cars per 'simulation round' is now rather pointless given
    % that there are 10 rounds and we only output the graph of one
%     subplot(3,2,1)
%     bar(num_cars_array_sorted);
%     title('Cars per simulation - Sorted (Ascending)')
%     xlabel('Number of simulations run')
%     ylabel('Number of cars')
%     xlim([0 num_sims])
% 
% 
%     
% 
%     subplot(3,2,[3 6])
    % Filter line
    % windowSize = num_sims/5;
    % yy = filter(ones(1,windowSize)/windowSize,1, timeAveragedData(:, 2));

    y1 = smooth(timeAveragedData(:, 1), timeAveragedData(:, 2), 0.1, 'lowess');
    y2 = smooth(timeAveragedData(:, 5), timeAveragedData(:, 6), 0.1, 'lowess');
    y3 = smooth(timeAveragedData(:, 9), timeAveragedData(:, 10), 0.1, 'lowess');
    y4 = smooth(timeAveragedData(:, 13), timeAveragedData(:, 14), 0.1, 'lowess');
    y5 = smooth(timeAveragedData(:, 17), timeAveragedData(:, 18), 0.1, 'lowess');
    y6 = smooth(timeAveragedData(:, 21), timeAveragedData(:, 22), 0.1, 'lowess');
    y7 = smooth(timeAveragedData(:, 25), timeAveragedData(:, 26), 0.1, 'lowess');
    y8 = smooth(timeAveragedData(:, 29), timeAveragedData(:, 30), 0.1, 'lowess');
    y9 = smooth(timeAveragedData(:, 33), timeAveragedData(:, 34), 0.1, 'lowess');
    y10 = smooth(timeAveragedData(:, 37), timeAveragedData(:, 38), 0.1, 'lowess');
    
    plot( ...
        timeAveragedData(:, 1),y1,'b-', ...
        timeAveragedData(:, 5),y2,'m-', ...
        timeAveragedData(:, 9),y3,'c-', ...
        timeAveragedData(:, 13),y4,'g-', ...
        timeAveragedData(:, 17),y5,'r-', ...
        timeAveragedData(:, 21),y6,'b--', ...
        timeAveragedData(:, 25),y7,'m--', ...
        timeAveragedData(:, 29),y8,'c--', ...
        timeAveragedData(:, 33),y9,'g--', ...
        timeAveragedData(:, 37),y10,'r--' ...
    );
    % set(gca);
    legend( ...
        'y1', ...
        'y2', ...
        'y3', ...
        'y4', ...
        'y5', ...
        'y6', ...
        'y7', ...
        'y8', ...
        'y9', ...
        'y10', ...
        'Location','eastoutside' ...
    );
    title('Averaged Density vs Flow')
    xlabel('Time Averaged Density')
    ylabel('Time Averaged Flow')

    %% Plot Averaged Density vs TE only
    figure

    te1 = smooth(timeAveragedData(:, 1), timeAveragedData(:, 3), 0.1, 'lowess');
    te2 = smooth(timeAveragedData(:, 5), timeAveragedData(:, 7), 0.1, 'lowess');
    te3 = smooth(timeAveragedData(:, 9), timeAveragedData(:, 11), 0.1, 'lowess');
    te4 = smooth(timeAveragedData(:, 13), timeAveragedData(:, 15), 0.1, 'lowess');
    te5 = smooth(timeAveragedData(:, 17), timeAveragedData(:, 19), 0.1, 'lowess');
    te6 = smooth(timeAveragedData(:, 21), timeAveragedData(:, 23), 0.1, 'lowess');
    te7 = smooth(timeAveragedData(:, 25), timeAveragedData(:, 27), 0.1, 'lowess');
    te8 = smooth(timeAveragedData(:, 29), timeAveragedData(:, 31), 0.1, 'lowess');
    te9 = smooth(timeAveragedData(:, 33), timeAveragedData(:, 35), 0.1, 'lowess');
    te10 = smooth(timeAveragedData(:, 37), timeAveragedData(:, 39), 0.1, 'lowess');

    plot( ... 
        timeAveragedData(:, 1), te1, 'b-', ...
        timeAveragedData(:, 5), te2, 'm-', ...
        timeAveragedData(:, 9), te3, 'c-', ...
        timeAveragedData(:, 13), te4, 'g-', ...
        timeAveragedData(:, 17), te5, 'r-', ...
        timeAveragedData(:, 21), te6, 'b--', ...
        timeAveragedData(:, 25), te7, 'm--', ...
        timeAveragedData(:, 29), te8, 'c--', ...
        timeAveragedData(:, 33), te9, 'g--', ...
        timeAveragedData(:, 37), te10, 'r--' ...
    );

    axis([0 1 0 inf])

    if strcmpi('transfer', measureId) || strcmpi('transferAndMutual', measureId)
        legend( ...
        'te1', ...
        'te2', ...
        'te3', ...
        'te4', ...
        'te5', ...
        'te6', ...
        'te7', ...
        'te8', ...
        'te9', ...
        'te10', ...
        'Location','eastoutside' ...
        );

        title('Averaged Density vs TE')
        xlabel('Time Averaged Density')
        ylabel('Averaged TE')
    end

    %% Plot Averaged Density vs MI only
    figure

    mi1 = smooth(timeAveragedData(:, 1), timeAveragedData(:, 4), 0.1, 'lowess');
    mi2 = smooth(timeAveragedData(:, 5), timeAveragedData(:, 8), 0.1, 'lowess');
    mi3 = smooth(timeAveragedData(:, 9), timeAveragedData(:, 12), 0.1, 'lowess');
    mi4 = smooth(timeAveragedData(:, 13), timeAveragedData(:,16), 0.1, 'lowess');
    mi5 = smooth(timeAveragedData(:, 17), timeAveragedData(:, 20), 0.1, 'lowess');
    mi6 = smooth(timeAveragedData(:, 21), timeAveragedData(:, 24), 0.1, 'lowess');
    mi7 = smooth(timeAveragedData(:, 25), timeAveragedData(:, 28), 0.1, 'lowess');
    mi8 = smooth(timeAveragedData(:, 29), timeAveragedData(:, 32), 0.1, 'lowess');
    mi9 = smooth(timeAveragedData(:, 33), timeAveragedData(:, 36), 0.1, 'lowess');
    mi10 = smooth(timeAveragedData(:, 37), timeAveragedData(:, 40), 0.1, 'lowess');

    plot( ... 
        timeAveragedData(:, 1), mi1, 'b-', ...
        timeAveragedData(:, 5), mi2, 'm-', ...
        timeAveragedData(:, 9), mi3, 'c-', ...
        timeAveragedData(:, 13), mi4, 'g-', ...
        timeAveragedData(:, 17), mi5, 'r-', ...
        timeAveragedData(:, 21), mi6, 'b--', ...
        timeAveragedData(:, 25), mi7, 'm--', ...
        timeAveragedData(:, 29), mi8, 'c--', ...
        timeAveragedData(:, 33), mi9, 'g--', ...
        timeAveragedData(:, 37), mi10, 'r--' ...
    );

    axis([0 1 0 inf])

    if strcmpi('mutual', measureId) || strcmpi('transferAndMutual', measureId)
        legend( ...
        'mi1', ...
        'mi2', ...
        'mi3', ...
        'mi4', ...
        'mi5', ...
        'mi6', ...
        'mi7', ...
        'mi8', ...
        'mi9', ...
        'mi10', ...
        'Location','eastoutside' ...
        );

        title('Averaged Density vs MI')
        xlabel('Time Averaged Density')
        ylabel('Averaged MI')
    end

    %% Plot single "Averaged Flow & TE relationship" graph
    figure

    % Dual y-axis plot
    yyaxis left
    plot( ...
        timeAveragedData(:, 17), y5 ...
    );
    ylabel('Time Averaged Flow');

    yyaxis right
    plot( ...
        timeAveragedData(:, 17), te5, ...
        timeAveragedData(:, 17), mi5 ...
    );
    % ylabel('Average Transfer Entropy');
    xlabel('Time Averaged Density');

    % Single y-axis plot
    % plot( ...
    %     timeAveragedData(:, 28),y10,'r-', ...
    %     timeAveragedData(:, 28), te10, 'b-' ...
    % );

    if strcmpi('transfer', measureId)
        ylabel('Average Transfer Entropy');
        legend( ...
            'y5', ...
            'te5' ...
        );

        title('Averaged Flow & TE relationship');
    elseif strcmpi('mutual', measureId)
        ylabel('Average Mutual Information');
        legend( ...
            'y5', ...
            'mi5' ...
        );

        title('Averaged Flow & MI relationship');
    else
        ylabel('Average TE & MI');
        legend( ...
            'y5', ...
            'transfer entropy 5', ...
            'mutual information 5' ...
        );

        title('Averaged Flow & TE/MI relationship');
    end
end



%% Export final cell array to csv format
cell2csv('test.csv', c, ', ', 2013, '.');
% cell2csv('tadseries.csv', tadseries, ', ', 2013, '.');
% cell2csv('tafseries.csv', tafseries, ', ', 2013, '.');

fprintf('Plotting finished, Time %s\n', datestr(now));
fprintf('Simulation finished, Time %s\n', datestr(now));