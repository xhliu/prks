%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/19/2014
%   Function: cope with time wrap around
%               only for special format, i.e., ID_IDX & TIMESTAMP_IDX
%%
function y = unwrap(t, wrap_period)

MIN_GAP = wrap_period / 2;

ID_IDX = 2;
TIMESTAMP_IDX = 10;

nodes = unique(t(:, ID_IDX));
for j = 1 : length(nodes)
    node_ix = (t(:, ID_IDX) == nodes(j));
    s = t(node_ix, TIMESTAMP_IDX);
    % sanity check if time is increasing
%     plot(s);
    
    % wrap around points; add MIN_GAP bcoz sometimes a entry is larger than
    % its next entry somehow
    %IX = find(s(1:end-1) > s(2:end));
    IX = find(s(1:end-1) > (s(2:end) + MIN_GAP));
    len = length(IX);
    for i = 1 : len
        if i < len
            s(IX(i) + 1 : IX(i + 1)) = s(IX(i) + 1 : IX(i + 1)) + i * wrap_period;
        else
            s(IX(i) + 1 : end) = s(IX(i) + 1 : end) + i * wrap_period;
        end
    end
%     plot(s);
    t(node_ix, TIMESTAMP_IDX) = s;
end
y = t;
end