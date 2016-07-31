clc;
clear;

% add JIDT path -> Gives warning error
% addpath('~/Documents/MATLAB/infodynamics-dist-1.3/infodynamics.jar')

% add TRENTOOL path
addpath('~/Documents/MATLAB/TRENTOOL3-master');

% add Fieldtrip toolbox path
addpath('~/Documents/MATLAB/fieldtrip-20160727');


% debug flags
verbose = false;

%% INITIALISATION
% Initialise n by m grid of cells
n = 100; %number of time steps
m = 50; %length of 'road'
c = cell(n, m); 


% Initialise variable to track which row we are up to
row_counter = 1;

% Maximum Velocity
v_max = 3;

% Number of simulation rounds
num_sims = 1;

% Pre-allocate array of size 'num_sims'
% Arrays store averaged values that will be analyzed later

time_average_flow_array = zeros(1, num_sims);
time_average_density_array = zeros(1, num_sims);

num_cars_array = zeros(1, num_sims);
missing_cars_array = zeros(1, num_sims);

% Vehicle generation method
% Options = 'random', 'naive', 'static'
initialisation_method = 'random';



% disp('Initial grid (t = 0)');
% disp(c);

%% START SIMULATION
for k = 1:num_sims
    % Might be a better way to initialise/clean grid
    % might only need to initialise first row and then dynamically add rows
    % (dynamic allocation computationally expensive)
    for i = 1:n %down y
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
        max_num_cars = floor(rand * m);

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
                fprintf('Row: %d Gap: %d\n', row_counter, gap);

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
                
                fprintf('new velocity = %d\n', velocity);
                
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
        % if cell isn't empty and cell speed is greater than 0
        % why does it have to be greater than zero? if speed is zero it
        % still takes up roadspace... 
        if c{i, 1} ~= ' ' %&& c{i, 1} > 0
            tadsum = tadsum + c{i, 1};
            % should be:   
            % sum = sum + 1;
    %         disp(sum);
            fprintf('Density at timestep %d: %d %f\n', i, c{i, 1}, c{i, 1}/n);
            
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

            if c{i+1, temp1} ~= ' ' && c{i+1, temp1} > j 
                tafsum = tafsum + 1;
                fprintf('Flow at timestep %d: %d %f\n', i, c{i+1, temp1}, c{i+1, temp1}/n);
                
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
