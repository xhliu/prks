%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/15/14
%   Function: display latency vs pdr req for various protocols
%% 
idx = 0;
data = cell(PDR_REQ_CNT, PROTOCOL_CNT);

%% PRKS
% us
SCALE = 5 / PRKS_SLOT_LEN / 1000;

fprintf('processing PRKS\n');
job = prks_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : length(pdr_job)
        fprintf('processing job %d\n', pdr_job(j));
        job_dir = [MAIN_DIR num2str(pdr_job(j))];
        cd(job_dir);
        load link_seq_latency;
        tmp{i} = [tmp{i}; link_seq_latency(:, end) * SCALE];
    end
end
idx = idx + 1;
data(:, idx) = tmp;


%% SCREAM
% us
SCALE = 5 / SCREAM_SLOT_LEN / 1000;

fprintf('processing SCREAM\n');
pdr_job = scream_job;

len = length(job);
tmp = [];
% each job
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load link_seq_latency;
    tmp = [tmp; link_seq_latency(:, end) * SCALE];
end
idx = idx + 1;
for i = 1 : PDR_REQ_CNT
    data{i, idx} = tmp;
end

%% process data for display
ALPHA = 0.05;
M = size(data, 1);
N = size(data, 2);
barvalue = nan(size(data));
error = nan(size(data));
for i = 1 : M
    for j = 1 : N
        barvalue(i, j) = mean(data{i, j});
        s = data{i, j};
        error(i, j) = norminv(1 - ALPHA / 2, 0, 1) * std(s) / sqrt(length(s));
    end
end

%% display
bw_ylabel = 'Latency (ms)';
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
h = barweb(barvalue, error, [], groupnames, [], bw_xlabel, bw_ylabel);
h.legend = legend(bw_legend, 'orientation', 'horizontal');
