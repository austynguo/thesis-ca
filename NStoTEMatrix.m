%% Convert Nagel-Schreckenberg formatted data to TE-friendly format
% This function converts data given by ns_model.m which is in 'cell' format
% to a matrix (array) format that is required by the JIDT Toolkit for the 
% calculation of Transfer Entropy and related meaures
function data = NStoTEMatrix(cellgrid, timesteps, roadLength, v_max)
    % Format: array(row, col)
    data = zeros(timesteps, roadLength);
    % Parse through every cell in the grid
    for i = 1:timesteps % num timesteps
        for j = 1:roadLength % length of 'road'
            % Store each cell in a temporary variable before writing it to
            % the new matrix
            tempvar = cellgrid{i, j};
            fprintf('%d ',tempvar);
            % If cell is an empty/unoccupied cell, assigned it a value of 2
            % This is as JIDT doesn't allow for empty cells/matrix cells
            %% [TODO: Confirm with Joe that this does not invalidate results]
            % Met Joe 24.08.16, his opinion is that it does not invalidate
            if tempvar == ' '
                % Use v_max + 1 to replace empty ' ' spaces
                % This is as the JIDT code does not allow empty or negative
                % valued matrix entries
                tempvar = v_max + 1;
            end
            data(i, j) = tempvar;
        end
        fprintf('\n');
    end
%     cell2csv('converted_NS_data.csv', data, ', ', 2013, '.');
%     data