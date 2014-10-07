%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   7/10/2014
%   Function: compute ctrl plane behavior: e.g., % to stay in ctrl; % to be
%   receiver bcoz of being conservative; use job 23160
%% 
%% always keep first
load debugs;
t = debugs;
% type = DBG_CONTROLLER_FLAG;
% line = 716; %686; %575;
type = DBG_TDMA_FLAG;
line = 605; %543; %1024;
t = t(t(:, 3) == type, :);
t = t(t(:, 4) == line, :);

%% filter out links whose pdr is below even req even without interference
load link_pdrs;
s = link_pdrs;
cdfplot(s(:, end));
MIN_PDR = 0.8;
receivers = s(s(:, end) < MIN_PDR, 2);

%% frequency to stay in ctrl 
STATUS_IDX = 7;
ctrl_ratio = [];
% only active nodes, either sender or receiver
s = t(t(:, STATUS_IDX) < 2, :);
nodes = unique(s(:, 2));
nodes = setdiff(nodes, receivers);
for i = 1 : length(nodes)
    node = nodes(i);
    s = t(t(:, 2) == node, :);
    ctrl_ratio = [ctrl_ratio; node size(s, 1) sum(s(:, STATUS_IDX) == 2) / size(s, 1)];
end
cdfplot(ctrl_ratio(:, end));
str = ['ctrl_plane_ratio'];

%% conservative ratio for receivers
conservative_ratio = [];
nodes = unique(t(:, 2));
nodes = setdiff(nodes, receivers);
fprintf('filter segment before convergence\n');
for i = 1 : length(nodes)
    node = nodes(i);
    s = t(t(:, 2) == node, :);
    % filter segment before convergence: based on log seq#
    s = s(s(:, 11) > quantile(s(:, 11), 0), :);
    conservative_ratio = [conservative_ratio; node sum(s(:, 10) == 1) / size(s, 1)];
end
cdfplot(conservative_ratio(:, end));
str = ['conservative_ratio'];
