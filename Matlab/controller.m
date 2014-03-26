%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute convergence time, both for link and network
%% 
load debugs;
t = debugs;
type = DBG_CONTROLLER_FLAG;
line = 1051;
t = t(t(:, 3) == type, :);
t = t(t(:, 4) == line, :);

%% line # if changed
% s = t;
% s = s(:, 4);
% cdfplot(s);
% unique(s)

%% preprocess time
% ms
SLOT_LEN = 512;
% time in us
SLOT_WRAP_LEN = 2 ^ 32;
TIMESTAMP_IDX = 10;
% time wrap around
t = unwrap(t, SLOT_WRAP_LEN);
% convert into slots
t(:, TIMESTAMP_IDX) = floor(t(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));

%% %% controller
% (DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, le->nb, link_pdr, -le->rx_nI.abs
% / 128, le->rx_er_border_idx + 1, link_pdr_version, (SUCCESS == call
% GlobalTime.getGlobalTime(&g_now)) ? g_now : INVALID_TIME)
POI_IDX = 8;

%% 1) status
STEP_SIZE = 30;
slots = unique(t(:, TIMESTAMP_IDX));
len = length(slots) / STEP_SIZE;
data = cell(len, 1);
for i = 1 : len
    slot = slots(i * STEP_SIZE);
    s = t(t(:, TIMESTAMP_IDX) == slot, POI_IDX);
    data{i} = s;
end

%% 2) status change over every period, i.e., every STEP_SIZE step (16 slots)
STEP_SIZE = 30;
slots = unique(t(:, TIMESTAMP_IDX));
len = length(slots) / STEP_SIZE;
data = cell(len, 1);
% each period
for i = 2 : len
    slot = slots(i * STEP_SIZE);
    last_slot = slots((i - 1) * STEP_SIZE);
    fprintf('slot: %d\n', slot);
    
    % [sender, receiver]
    s = t(t(:, TIMESTAMP_IDX) == slot, :);
    links = unique(s(:, [5 2]), 'rows');
    % [links(j, :), slot, current_value, last_value, current_value - last_value]
    tmp = [];
    % each link
    for j = 1 : size(links, 1)
        r = s(s(:, 5) == links(j, 1) & s(:, 2) == links(j, 2), POI_IDX);
        if length(r) ~= 1
            fprintf('something fishy: %d\n', length(r));
            return;
        end
        current_value = r;
        
        r = t(t(:, 5) == links(j, 1) & t(:, 2) == links(j, 2) & t(:, TIMESTAMP_IDX) == last_slot, POI_IDX);
        if length(r) ~= 1
            fprintf('something missing: %d\n', length(r));
            continue;
        end
        last_value = r;
        
        tmp = [tmp; links(j, :), slot, current_value, last_value, current_value - last_value];
    end
    
    data{i} = tmp(:, end);
end
% remove first empty cell
data(1) = [];

%% categorize into inc, dec, and unchanged
len = size(data, 1);
r = zeros(len, 1);
hold on;
for i = 1 : len
    s = data{i};
    r(i) = sum(s > 0) / length(s) * 100;
end
plot(r);
for i = 1 : len
    s = data{i};
    r(i) = sum(s < 0) / length(s) * 100;
end
plot(r);
for i = 1 : len
    s = data{i};
    r(i) = sum(s == 0) / length(s) * 100;
end
plot(r);
legend({'ER size increase', 'ER size decrease', 'ER size unchanged'});
%%
group = zeros(0);
dataDisp = zeros(0);
for i = 1 : size(data, 1)
    group = [group ; ones(size(data{i})) + i - 1];
    dataDisp = [dataDisp ; data{i}];
end
% figure;
boxplot(dataDisp, group, 'notch', 'on');

%% display
bw_ylabel = 'Exclusion region size change';
set(gca, 'FontSize', 50);
% set(gca, 'yscale', 'log');
ylabel(bw_ylabel);
xlabel('Time (240 seconds)');

%% make x-axis label sparser
TICK_LABEL_STEP_SIZE = 5;
LEN = size(data, 1);
str = cell(LEN, 1);
set(gca, 'xtick', 1:LEN);
for i = 1 : LEN
    if mod(i, TICK_LABEL_STEP_SIZE)
        str{i} = '';
    else
        str{i} = num2str(i);
    end
end
set(gca, 'xticklabel', str);
%%
maximize;
set(gcf, 'Color', 'white');
% cd(FIGURE_DIR);
cd('~/Dropbox/iMAC/Xiaohui/figure/');
%
str = ['controller_er_size_change_timeseries'];
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
