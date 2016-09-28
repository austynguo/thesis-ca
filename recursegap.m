%% Calculate vehicle gap recursively
% pseudocode:
% if next cell is a ' ' (space)
%   gap = recursegap()
%     then increment gap variable by 1
% else return 1
function gap = recursegap(cellgrid, row, position, arraylength)
    % Grid is a toroid (essentially a loop connected from finish to start)
    % Take modulo of position variable and the array length
    % to obtain position of vehicle on 'wrapped around' road
    wrappedposition = mod(position + 1, arraylength);
    % Cannot have position '0' so change to last position on array instead
    if wrappedposition == 0
        wrappedposition = arraylength;
    end
    % Check if next cell is empty
    if (cellgrid{row, wrappedposition} == ' ')
        gap = 1 + recursegap(cellgrid, row, wrappedposition, arraylength);
%         fprintf('gap: %d, position: %d\n', gap, position);
    else
    % If next cell not empty, recurse back up
        gap = 0;
    end
    