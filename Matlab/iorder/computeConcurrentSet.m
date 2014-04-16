%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   4/13/2013
%   Function: compute concurrency based on unique concurrent sets, not simply in pkts / slot
%% 
jobs = [];
SLOT_LEN = 512; %32; %512;
jobs = [21103];

fprintf('slot length %d\n', SLOT_LEN);
MAIN_DIR = '~/Projects/tOR/RawData/';

for job_id = 1 : length(jobs)
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    cd(job_dir);
    if false %exist('schedule_unique_concurrent_set.mat', 'file')
        fprintf('skip processed %d-th job %d\n', job_id, jobs(job_id));
        continue;
    else
        fprintf('processing %d-th job job %d\n', job_id, jobs(job_id));
    end
%%
load debugs;
t = debugs;
type = DBG_TDMA_FLAG;
line = 543; %642 %543;
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

%% unique over schedule
% unique concurrent set
ucs = cell(len, 1);
% frequency of a ucs
ucs_freq = zeros(len, 1);
idx = 1;
for i = 1 : len
    duplicate = false;
    cs = schedule{i};
    fprintf('unique slot %d\n', i);
    
    % compare w/ each existing cs
    if idx > 1
        for j = 1 : (idx - 1)
            cs_ = ucs{j};

            if length(cs) ~= length(cs_)
                continue;
            end
            % existing ucs found
            if sum(cs ~= cs_) == 0
                duplicate = true;
                ucs_freq(j) = ucs_freq(j) + 1;
                break;
            end
        end
    end
    
    % new ucs found
    if ~duplicate
        ucs{idx} = cs;
        ucs_freq(idx) = 1;
        idx = idx + 1;
    end
end
ucs(idx : end) = [];
ucs_freq(idx : end) = [];
save('schedule_unique_concurrent_set.mat', 'schedule', 'ucs', 'ucs_freq');
%% figure;
load schedule_unique_concurrent_set;
t = ucs;
len = size(t, 1);
concurrency = zeros(len, 1);
% each slot
for i = 1 : len
    fprintf('ucs %d\n', i);
    concurrency(i) = length(ucs{i});
end
t = concurrency;
% concurrency(concurrency == 0) = [];
% concurrency = ce(:, end);
% load link_pdrs;
% % sanity check concurrency
% fprintf('total concurrency %d, total packets sent %d, ratio: %f\n', sum(concurrency), sum(link_pdrs(:, 3)), sum(concurrency) / sum(link_pdrs(:, 3)));
%% save('concurrency.mat', 'concurrency');
cdfplot(t);
fprintf('concurrency median %f, mean %f\n', median(t), mean(t));
%%
t = concurrency .* ucs_freq;
sum(ucs_freq)
sum(t) / sum(ucs_freq)

%%
corrcoef(concurrency, ucs_freq)
end