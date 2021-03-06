%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   4/13/2014
%   Function: compute concurrency based on unique concurrent sets, not simply in pkts / slot
%% 
SLOT_LEN = 32; %32; %512;
jobs = [23048]; %20849 %21103

fprintf('slot length %d\n', SLOT_LEN);
% MAIN_DIR = '~/Projects/tOR/RawData/';
MAIN_DIR = '~/Downloads/Jobs/';

for job_id = 1 : length(jobs)
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    cd(job_dir);
    if exist('schedule_unique_concurrent_set.mat', 'file')
        fprintf('skip processed %d-th job %d\n', job_id, jobs(job_id));
        continue;
    else
        fprintf('processing %d-th job job %d\n', job_id, jobs(job_id));
    end
% if 0
%%
load debugs;
t = debugs;
type = DBG_TDMA_FLAG;
line = 709; %575; %686; %642 %543;
t = t(t(:, 3) == type, :);
t = t(t(:, 4) == line, :);

%% line # if changed
% s = t;
% s = s(:, 4);
% cdfplot(s);
% unique(s)

%% concurrency
% filter slots w/o data by default
% fprintf('filter slots w/o data\n');
% t = t(t(:, 5) == 1, :);
SLOT_IDX = 10;
% to cope w/ slot/time wrap around
SLOT_WRAP_LEN = 2 ^ 32 / (SLOT_LEN * 2 ^ 10);
t = unwrap(t, SLOT_WRAP_LEN);

% [x ix] = sort(t(:, SLOT_IDX));
% t = t(ix, :);

slots = unique(t(:, SLOT_IDX));
len = length(slots);
schedule = cell(len, 1);
% each slot
for i = 1 : len
    fprintf('slot %d\n', i);
    % concurrent set, denoted by sender of a link
    cs = t(t(:, SLOT_IDX) == slots(i), 2);
    schedule{i} = sort(cs);
end

% end
%% unique over schedule
% [unique_concurrent_set ucs_slots ucs_slots_size]
ucs = cell(len, 3);
idx = 1;
for i = 1 : len
    duplicate = false;
    cs = schedule{i};
    fprintf('unique slot %d\n', i);
    
    % compare w/ each existing cs
    if idx > 1
        for j = 1 : (idx - 1)
            cs_ = ucs{j, 1};

            if length(cs) ~= length(cs_)
                continue;
            end
            % existing ucs found
            if sum(cs ~= cs_) == 0
                duplicate = true;
                ucs{j, 2} = [ucs{j, 2}; slots(i)];
                ucs{j, 3} = ucs{j, 3} + 1;
                break;
            end
        end
    end
    
    % new ucs found
    if ~duplicate
        ucs{idx, 1} = cs;
        ucs{idx, 2} = [ucs{idx, 2}; slots(i)];
        ucs{idx, 3} = 1;
        idx = idx + 1;
    end
end
ucs(idx : end, :) = [];
save('schedule_unique_concurrent_set.mat', 'schedule', 'ucs');
%% figure;
load txrxs;
% time in us
TIME_WRAP_LEN = 2 ^ 32;
TIMESTAMP_IDX = 10;

% time wrap around
t = unwrap(rxs, TIME_WRAP_LEN);
t = floor(t(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));

fprintf('filter incomplete ucs in SCREAM below!\n');
% ucs = ucs(1 : 100, :);

len = size(ucs, 1);
% rx thruput based on ucs
rx_concurrency_ucs = zeros(len, 1);
% each ucs
for i = 1 : len
    fprintf('ucs %d\n', i);
    ucs_slots = ucs{i, 2};
    ucs_rx_cnt = [];
    for j = 1 : length(ucs_slots)
        rx_cnt = sum(t == ucs_slots(j));
        ucs_rx_cnt = [ucs_rx_cnt; rx_cnt];
    end
    rx_concurrency_ucs(i) = mean(ucs_rx_cnt);
end
%% save('concurrency.mat', 'concurrency');
t = rx_concurrency_ucs;
cdfplot(t);
fprintf('concurrency median %f, mean %f\n', median(t), mean(t));
% corrcoef(concurrency, ucs_freq)
end