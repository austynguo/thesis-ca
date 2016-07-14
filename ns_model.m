clc;
clear;

% debug flags
verbose = false;

%% INITIALISATION
% Initialise n by m grid of cells
n = 100; %number of time steps
m = 80; %length of 'road'
c = cell(n, m); 


% Initialise variable to track which row we are up to
row_counter = 1;

% Maximum Velocity
v_max = 5;

% Number of simulation rounds
num_sims = 1;

% Pre-allocate array of size 'num_sims'
% Arrays store averaged values that will be analyzed later
time_average_flow_array = zeros(1, num_sims);
time_average_density_array = zeros(1, num_sims);
num_cars_array = zeros(1, num_sims);
missing_cars_array = zeros(1, num_sims);


%% Previous static method of seeding cars 
% num_cars = 3;
% c{1, 2} = 3;
% c{1, 6} = 1;
% c{1, 9} = 1;

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
    
    % Initialise some vehicles
    % Method of randomly seeding vehicles
    num_cars = 0;
    for j = 1:m
        if generation_gap ~= 0
            generation_gap = generation_gap - 1;
            continue
        end
        % 50% chance of new vehicle being created
        if 0.5 > rand
            % generate vehicle with random speed rounded to nearest int
            speed = round(v_max * rand, 0);
            generation_gap = speed;
            c{1, j} = speed;
            num_cars = num_cars + 1;
        end
    end
    num_cars_array(k) = num_cars;

    for j = 1:n-1
        row_counter = j;
        for i = 1:m
            if (c{row_counter, i} >= 0 && c{row_counter, i} ~= ' ')
                % Get velocity
                velocity = c{row_counter, i};

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
                    % If gap greater than velocity and under v_max, car can accelerate
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
                        % do nothing
                    elseif gap + 1 == velocity
                        velocity = velocity - 1;
                    else
                        velocity = min(velocity - 1, gap);
                    end
                    if verbose
                        fprintf('velocity: %d\n', velocity);
                    end

                %%% 3. RANDOMISATION %%%
                
                %% CHECK THAT THIS SECTION IS ACTUALLY REACHED!
                end
%                 elseif velocity > 0
                %% Make randomisation step independent from accel/deceleration
                % i.e. it can accelerate and brake immediately
                if velocity > 0
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
    tadseries = zeros(1, n); %preallocate number of timesteps in currenet round

    for i = 1:n
        if verbose fprintf('c{%d, 1}: %d', i, c{i, 1}); end
        if c{i, 1} ~= ' ' && c{i, 1} > 0
            tadsum = tadsum + c{i, 1};
            % should be:
            % sum = sum + 1;
    %         disp(sum);
            fprintf('Density at timestep %d: %d %f\n', i, c{i, 1}, c{i, 1}/n);
            
        end
        
        tadseries(i) = c{i, 1}/n;
        
        if verbose disp(tadsum); end
    end

    tad = tadsum/n; 
    fprintf('Time Averaged Density: %.2f\n', tad);

    %% Calc time averaged flow
    % Need to figure this out better
    % calc for column 1 and 2

    tafsum = 0;
    tafseries = zeros(1, n); %preallocate number of timesteps in currenet round
    
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
            
            tafseries(i) = c{i+1, temp1}/n;
            
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





