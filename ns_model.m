clc;
clear;
clear java;

% add JIDT path -> Gives warning error if ENTIRE filepath is not specified
javaaddpath('/home/austyn/Documents/MATLAB/infodynamics-dist-1.3/infodynamics.jar')

% add TRENTOOL path
addpath('~/Documents/MATLAB/TRENTOOL3-master');

% add Fieldtrip toolbox path
addpath('~/Documents/MATLAB/fieldtrip-20160727');

%% === START Variable System Parameters === %%
% These 'variable' initial system parameters can be edited to change the outcome
% of the simulation

% Debug flags - set to true to print out debugging messages
verbose = false;

%% INITIALISATION
% Initialise size n by m grid of cells
n = 100; %number of time steps
m = 50; %length of 'road'
c = cell(n, m);

% Maximum Velocity
v_max = 1;

% Number of simulation rounds
num_sims = 1;

% Vehicle generation method
% Options = 'random', 'naive', 'static'
initialisation_method = 'random';

%% Simulation mode
% Choose mode: 'single' or 'multiple'
simulationMode = 'single';

% Number of simulation rounds
num_sims = 1;

%% Plot graph of multiple simulations
plotGraph = false;

%% Plot Transfer Entropy (single simulation only)
plotTE = true;


%% === END Variable System Parameters === %%

% Initialise variable to track which row we are up to
row_counter = 1;

% disp('Initial grid (t = 0)');
% disp(c);

% Flow & Density Value storage
fnd_storage = zeros(num_sims, 3);

% Pre-allocate array of size 'num_sims'
% Arrays store averaged values that will be analyzed later
time_average_flow_array = zeros(num_sims, 1);
time_average_density_array = zeros(num_sims, 1);

num_cars_array = zeros(num_sims, 1);
missing_cars_array = zeros(num_sims, 1);


%% START SIMULATION
for k = 1:num_sims
    % Might be a better way to initialise/clean grid
    % might only need to initialise first row and then dynamically add rows
    % (dynamic allocation computationally expensive)
    for i = 1:n %down y
        fnd_storage(i, 1) = i;
        for j = 1:m %right x
            c{i, j} = ' ';
        end
    end
    
    generation_gap = 0;

    % [TODO: Combine above and below for loops]
    
    %% Initialise vehicles (locations and speeds)
    % Method of random vehicle seeding
    num_cars = 0;
    
    if strcmp(initialisation_method, 'random')
        %% Random vehicle initialisation
        % Randomly generate a capped value for number of cars for this round
        % only. The aim is to ensure an even spread of simulations for a wide
        % range of vehicle numbers.
        max_num_cars = rand * m;

        for j = 1:m
            % Check each step that we do not exceed our predetermined value
            if num_cars >= max_num_cars
                break;
            end
            if generation_gap ~= 0
                generation_gap = generation_gap - 1;
                continue
            end
            % generate vehicle with random speed rounded to nearest int
            speed = round(v_max * rand, 0);
            generation_gap = speed;
            c{1, j} = speed;
            num_cars = num_cars + 1;
        end
    elseif strcmp(initialisation_method, 'naive')
        %% Naive vehicle initialisation
        naive_num_cars = 10; % set number of cars to be created using this naive method

        for j = 1:m
            % Check each step that we do not exceed our predetermined value
            if num_cars >= naive_num_cars
                break;
            end
            if generation_gap ~= 0
                generation_gap = generation_gap - 1;
                continue
            end
            % generate vehicle with random speed rounded to nearest int
            speed = round(v_max * rand, 0);
            generation_gap = speed;
            c{1, j} = speed;
            num_cars = num_cars + 1;
        end
    elseif strcmp(initialisation_method, 'static')
    %% Previous static method of seeding cars
    % Good for replicating exact base/start conditions, not good for large
    % sets
        num_cars = 3;
        c{1, 2} = 3;
        c{1, 6} = 1;
        c{1, 9} = 1;
    end
    
    num_cars_array(k) = num_cars;

    for j = 1:n-1
        row_counter = j;
        for i = 1:m
            if (c{row_counter, i} >= 0 && c{row_counter, i} ~= ' ')
                % Get current velocity
                velocity = c{row_counter, i};
                
                % set new velo = curr velo
                % new_velocity = curr_velocity;

                % print position out
                if verbose fprintf('Position: %d\n', i); end

                %%% CALCULATE GAP %%%
                % pseudocode:
                % if next cell is a ' ' (space)
                %   gap =  recursegap()
                %     then increment gap variable by 1
                % else return 1
                % Calculate gap to next vehicle thru recursive function
                gap = recursegap(c, row_counter, i, m);
                if verbose fprintf('Row: %d Gap: %d\n', row_counter, gap); end

                %%% 1. ACCELERATION %%%
                if gap > velocity + 1 && velocity < v_max
                    % If gap greater than current velocity and under v_max,
                    % car can accelerate
                    % Increase velocity
                    velocity = velocity + 1;
                    % testing ending if statement so each step is independent
                end
                
                
                %%% What time dependent data can be encoded?
                %%% recode acceleration as binary or trinary interaction??
                
                
                    
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

                %%% 3. RANDOMISATION %%%
                
                %% CHECK THAT THIS SECTION IS ACTUALLY REACHED!
                end
%                 elseif velocity > 0
                %% Make randomisation step independent from accel/deceleration
                % i.e. it can accelerate and brake immediately
                if velocity > 0 && velocity > 0
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
                newPosition = mod(i + velocity, m);
                if newPosition == 0
                    newPosition = m;
                end
                %[TODO: Write catch case so cars don't drive on top of each other???]
                % Probably not needed if random seeding is fixed
                % loljks still need to write this
                
                if verbose fprintf('new velocity = %d\n', velocity); end
                
                %% Update car movement in the next row
                c{row_counter + 1, newPosition} = velocity;
            end
        end
    end

    %% Print grid after t timesteps
    fprintf('After %d timesteps (t = 10)\n', n);
    disp(c);

    %% Detect missing cars:
    num_cars_at_end = 0;
    for j = 1:m %right x
        if c{n, j} ~= ' '
            num_cars_at_end = num_cars_at_end + 1;
        end
    end
    missing_cars_array(k) = num_cars - num_cars_at_end;
    fprintf('Missing cars: %d\n', num_cars - num_cars_at_end);
    
    
    %% Calc time averaged density
    % only calcs for first column atm
    tadsum = 0;

    for i = 1:n
        if verbose fprintf('c{%d, 1}: %d', i, c{i, 1}); end
        % if cell is empty, add one to the total sum
        if c{i, 1} ~= ' '
            tadsum = tadsum + 1;
            
            %% Experimental data
            fnd_storage(i, 2) = 1;
        end
        
        if verbose disp(tadsum); end
    end

    tad = tadsum/n; 
    fprintf('Time Averaged Density: %.2f\n', tad);

    %% Calc time averaged flow
    % Need to figure this out better
    % calc for column 1 and 2

    tafsum = 0;
    
    % c{n time steps, m road length}
    % for each row
    for i = 1:n-1
    %     print message out
        if verbose fprintf('c{%d, 1}: %d', i, c{i, 1}); end
    %     for each value up to max velocity
        for j = 1:v_max
    %         check each of the cells up to v_max
    %         temp value equals cell under target cell minus j value
            temp1 = mod(1 - j, m);
            if temp1 == 0
                temp1 = m;
            end

            if c{i+1, temp1} ~= ' ' && c{i+1, temp1} >= j
                tafsum = tafsum + 1;
                fprintf('Flow at timestep %d: %d %f\n', i, c{i+1, temp1}, c{i+1, temp1}/n);
                
                %% Experimental data
                fnd_storage(i, 3) = c{i+1, temp1};
            end
        end
        if verbose fprintf('Time Averaged Flow cumulative sum: ', tafsum); end
    end

    % WHAT HAPPENS IF TIME AVE FLOW IS ZERO??? (need catching case)
    taf = tafsum/n;
    fprintf('Time Averaged Flow: %.2f\n', taf);
    
    % Save averages to an array
    time_average_density_array(k) = tad;
    time_average_flow_array(k) = taf; 
    
    % Reset counter variables
    taf = 0;
    tafsum = 0;
    tad = 0;
    tadsum = 0;
end

% sort number of cars for easy graph reading
num_cars_array_sorted = sort(num_cars_array);


%% Export final cell array to csv format
cell2csv('test.csv', c, ', ', 2013, '.');
% cell2csv('tadseries.csv', tadseries, ', ', 2013, '.');
% cell2csv('tafseries.csv', tafseries, ', ', 2013, '.');



%% Plotter
if plotGraph == true
    figure
    % subplot(3,1,2)
    % scatter(time_average_density_array, time_average_flow_array, 'filled');
    % title('Averaged Density vs Flow')
    % xlabel('Time Averaged Density')
    % ylabel('Time Averaged Flow')

    subplot(3,2,1)
    bar(num_cars_array_sorted);
    title('Cars per simulation - Sorted (Ascending)')
    xlabel('Number of simulations run')
    ylabel('Number of cars')
    xlim([0 num_sims])


    timeAveragedData = sortrows([time_average_density_array time_average_flow_array], 1);

    subplot(3,2,[3 6])
    % Filter line
    % windowSize = num_sims/5;
    % yy = filter(ones(1,windowSize)/windowSize,1, timeAveragedData(:, 2));

    yy = smooth(timeAveragedData(:, 1), timeAveragedData(:, 2), 0.1, 'rlowess');
    plot(timeAveragedData(:, 1),timeAveragedData(:, 2),'b.',timeAveragedData(:, 1),yy,'r-');
    % set(gca);
    legend('Original Data','Smoothed Data Using ''rloess''', 'Location','NW');
    title('Averaged Density vs Flow')
    xlabel('Time Averaged Density')
    ylabel('Time Averaged Flow')
end



%% Transfer Entropy Toolbox (JIDT)
% % Generate some random binary data.
% % Note that we need the *1 to make this a number not a Boolean,
% %  otherwise this will not work (as it cannot match the method signature)
% sourceArray=fnd_storage(:,2);
% destArray = [0; sourceArray(1:99)];
% sourceArray2=fnd_storage(:,3);
% % Create a TE calculator and run it:
% teCalc=javaObject('infodynamics.measures.discrete.TransferEntropyCalculatorDiscrete', 2, 1);
% teCalc.initialise();
% % Since we have simple arrays of doubles, we can directly pass these in:
% teCalc.addObservations(sourceArray, destArray);
% fprintf('\nFor copied source, result should be close to 1 bit : ');
% result = teCalc.computeAverageLocalOfObservations()
% teCalc.initialise();
% teCalc.addObservations(sourceArray2, destArray);
% fprintf('\nFor random source, result should be close to 0 bits: ');
% result2 = teCalc.computeAverageLocalOfObservations()

%% CA Simulation
if plotTE == true
    % add paths to JIDT CA octave & matlab code
    addpath('/home/austyn/Documents/MATLAB/infodynamics-dist-1.3/demos/octave/CellularAutomata');
    addpath('/home/austyn/Documents/MATLAB/infodynamics-dist-1.3/demos/octave');


    options.plotOptions.plotRows = n; % number of timesteps
    options.plotOptions.plotCols = m; % length of road
    options.plotOptions.plotStartRow = 1; % plot from row # onwards
    options.plotOptions.plotStartCol = 1; % plot from column # onwards
    % input final cell state as calculated by prev code
    % NOT CORRECT -> not being parsed correctly
    % options.initialState = c; % <- legit will take anything and "work" (not throw errors) =/

    %% Values commented out below don't seem to affect JIDT code 

    % base = 3; % this is as we have 3 states in the most basic model: an unoccupied space, velocity = 0 or 1
    base = v_max + 2; % This allows for the multiple states
    % rule = 30; %54?
    % timesteps = 10; % this is the number of rows. 
    measureId = 'transfer';
    measureParams.k = 1; % History length of 16 for info dynamics measures

    %not sure if measureParams.j is right??
    measureParams.j = 1;

    % cells = m*n; % number of cells

    % plotLocalInfoMeasureForCA(neighbourhood, base, rule, cells, timesteps, measureId, measureParams, options);

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

        %%====== Create here ======%%
        %% Convert NS model data to feed into here
        % Call function that converts the cell data from the above NS model
        % simulation into a matrix
        caStates = NStoTEMatrix(c, 100, 50, v_max);
        %% Outputs caStates matrix for TE calculation
        caStates


        % convert the states to a format usable by java:
        caStatesJInts = octaveToJavaIntMatrix(caStates);


        plottedOne = false;

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

        %============================
        % Apparent transfer entropy
        if ((ischar(measureId) && (strcmpi('transfer', measureId) || strcmpi('all', measureId) || strcmpi('apparenttransfer', measureId))) || ...
            (not(ischar(measureId)) && ((measureId == 1) || (measureId == -1))))
            % Compute apparent transfer entropy
            if (measureParams.j == 0)
                error('Cannot compute transfer entropy from a cell to itself (setting measureParams.j == 0)');
            end
            transferCalc = javaObject('infodynamics.measures.discrete.TransferEntropyCalculatorDiscrete', base, measureParams.k);
            transferCalc.initialise();
            transferCalc.addObservations(caStatesJInts, measureParams.j);
            avTransfer = transferCalc.computeAverageLocalOfObservations();
            fprintf('Average apparent transfer entropy (j=%d) = %.4f\n', measureParams.j, avTransfer);
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

        if (not(plottedOne))
            error(sprintf('Supplied measureId %s did not match any measurement types', measureId));
        end
    %% Save ASCII text file of matrix
    % save('TEmatrix.txt', 'a', '-ASCII');
    dlmwrite('TEmatrix.txt', caStates, 'delimiter', ' ', 'precision', 1);
end
