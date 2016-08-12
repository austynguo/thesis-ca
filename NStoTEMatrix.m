%% Convert Nagel-Schreckenberg formatted data to TE-friendly format
% This function converts data given by ns_model.m which is in 'cell' format
% to a matrix (array) format that is required by the JIDT Toolkit for the 
% calculation of Transfer Entropy and related meaures
function data = NStoTEMatrix(cellgrid, timesteps, roadLength)
    % Format: array(row, col)
    data = zeros(timesteps, roadLength);
    for i = 1:timesteps % num timesteps
        for j = 1:roadLength % length of 'road'
            tempvar = cellgrid{i, j};
            fprintf('%d ',tempvar);
            if tempvar == ' '
                tempvar = 2;
            end
            data(i, j) = tempvar;
        end
        fprintf('\n');
    end
%     cell2csv('converted_NS_data.csv', data, ', ', 2013, '.');
%     data