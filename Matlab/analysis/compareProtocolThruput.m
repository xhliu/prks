%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/14/14
%   Function: display throughput vs pdr req for various protocols
%% 
idx = 0;
data = cell(PDR_REQ_CNT, PROTOCOL_CNT);

%% PRKS
fprintf('processing PRKS\n');
job = prks_job;

BOOTSTRAP_SECONDS = 500;
SCALE = 512 / 5;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_pdrs;
        thruput = sum(link_pdrs(:, 4)) / (pdr_job(j, 2) * 60 - BOOTSTRAP_SECONDS) * SCALE;
        fprintf('throughput %f\n', thruput);
        tmp{i} = [tmp{i}; thruput];
    end
end
idx = idx + 1;
data(:, idx) = tmp;


%% CSMA
fprintf('processing CSMA\n');
pdr_job = csma_job;

BOOTSTRAP_SECONDS = 30;
SCALE = 1;

tmp = [];
% each pdr req
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load link_pdrs;
    thruput = sum(link_pdrs(:, 4)) / (pdr_job(j, 2) * 60 - BOOTSTRAP_SECONDS) * SCALE;
    fprintf('throughput %f\n', thruput);
    tmp = [tmp; thruput];
end
idx = idx + 1;
for i = 1 : PDR_REQ_CNT
    data{i, idx} = tmp;
end


%% RTSCTS
fprintf('processing RTSCTS\n');
pdr_job = rtscts_job;

BOOTSTRAP_SECONDS = 30;
SCALE = 1;

%len = length(job);
tmp = [];
% each pdr req
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load link_pdrs;
    thruput = sum(link_pdrs(:, 4)) / (pdr_job(j, 2) * 60 - BOOTSTRAP_SECONDS) * SCALE;
    fprintf('throughput %f\n', thruput);
    tmp = [tmp; thruput];
end
idx = idx + 1;
for i = 1 : PDR_REQ_CNT
    data{i, idx} = tmp;
end



%% RIDB
fprintf('processing RIDB\n');
job = ridb_job;

% not 900 
BOOTSTRAP_SECONDS = 500;
SCALE =  1.28 * 32 / 5;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_pdrs;
        thruput = sum(link_pdrs(:, 4)) / (pdr_job(j, 2) * 60 - BOOTSTRAP_SECONDS) * SCALE;
        fprintf('throughput %f\n', thruput);
        tmp{i} = [tmp{i}; thruput];
    end
end
idx = idx + 1;
data(:, idx) = tmp;


%% RIDB_OLAMA
%{
fprintf('processing RIDB_OLAMA\n');
job = ridbolama_job;

BOOTSTRAP_SECONDS = 500;
SCALE = 512 / 5;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_pdrs;
        thruput = sum(link_pdrs(:, 4)) / (pdr_job(j, 2) * 60 - BOOTSTRAP_SECONDS) * SCALE;
        fprintf('throughput %f\n', thruput);
        tmp{i} = [tmp{i}; thruput];
    end
end
idx = idx + 1;
data(:, idx) = tmp;
%}


%% CMAC
fprintf('processing CMAC\n');
job = cmac_job;

BOOTSTRAP_SECONDS = 420;
SCALE = 1;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_pdrs;
        thruput = sum(link_pdrs(:, 4)) / (pdr_job(j, 2) * 60 - BOOTSTRAP_SECONDS) * SCALE;
        fprintf('throughput %f\n', thruput);
        tmp{i} = [tmp{i}; thruput];
    end
end
idx = idx + 1;
data(:, idx) = tmp;


%% SCREAM
fprintf('processing SCREAM\n');
pdr_job = scream_job;

ACTIVE_LINK_SIZE = 100; %59; %100;
INTERFERENCE_DIAMETER = 3; %8; % 3;
ROUND_LEN = 1 + INTERFERENCE_DIAMETER + 1;
FRAME_LEN = ROUND_LEN * ACTIVE_LINK_SIZE;
BOOTSTRAP_SECONDS = (FRAME_LEN * 4 * 2 + FRAME_LEN * ACTIVE_LINK_SIZE) * SCREAM_SLOT_LEN / 1024;

SCALE = 128 / 100 * 32 / 5;

len = length(job);
tmp = [];
% each job
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load link_pdrs;
    thruput = sum(link_pdrs(:, 4)) / (pdr_job(j, 2) * 60 - BOOTSTRAP_SECONDS) * SCALE;
    fprintf('throughput %f\n', thruput);
    tmp = [tmp; thruput];
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
bw_ylabel = 'Throughput (pps)';
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
h = barweb(barvalue, error, [], groupnames, [], bw_xlabel, bw_ylabel);
h.legend = legend(bw_legend, 'orientation', 'horizontal');

%%
set(gca, 'FontSize', 30);
ylabel(bw_ylabel);
xlabel(bw_xlabel);

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
%
if NetEye
str = ['peer_throughput_bar'];
else
str = ['indriya_peer_throughput_bar'];
end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
