function idx = timeRangeIndices(timeVec, tStart, tEnd)
%TIMERANGEINDICES Returns indices of a time vector within a time range.
%
%   idx = TIMERANGEINDICES(timeVec, tStart, tEnd) returns the indices of
%   the datetime vector timeVec that are between tStart and tEnd seconds
%   from the first element of timeVec.
%
%   Inputs:
%       timeVec - datetime vector (e.g., tt.Date)
%       tStart  - start time in seconds (relative to first timestamp)
%       tEnd    - end time in seconds (relative to first timestamp)
%
%   Output:
%       idx - indices of timeVec within [tStart, tEnd]

    % Validate inputs
    if ~isdatetime(timeVec)
        error('timeVec must be a datetime vector');
    end
    if tStart < 0 || tEnd < 0 || tEnd < tStart
        error('tStart and tEnd must be non-negative and tEnd >= tStart');
    end

    % Compute elapsed time from the first timestamp in seconds
    elapsedSec = seconds(timeVec - timeVec(1));

    % Find indices within the range
    idx = find(elapsedSec >= tStart & elapsedSec <= tEnd);
end