%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/20/14
%   Function: display per slot throughput vs pdr req for various protocols
%%
idx = 0;
data = cell(PDR_REQ_CNT, PROTOCOL_CNT);

%% PRKS
fprintf('processing PRKS\n');
job = prks_job;

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
        load e2e_link_pdrs;
        tmp{i} = [tmp{i}; e2e_link_pdrs(:, end)];
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
% each job
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load e2e_link_pdrs;
    tmp = [tmp; e2e_link_pdrs(:, end)];
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

tmp = [];
% each job
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load e2e_link_pdrs;
    tmp = [tmp; e2e_link_pdrs(:, end)];
end
idx = idx + 1;
for i = 1 : PDR_REQ_CNT
    data{i, idx} = tmp;
end


%% RIDB
fprintf('processing RIDB\n');
job = ridb_job;

SCALE = 1024 / 32;

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
        load e2e_link_pdrs;
        tmp{i} = [tmp{i}; e2e_link_pdrs(:, end)];
    end
end
idx = idx + 1;
data(:, idx) = tmp;


%% RIDB_OLAMA
%{
fprintf('processing RIDB_OLAMA\n');
job = ridbolama_job;

SCALE = 1024 / 5;

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
        load rx_concurrency;
        tmp{i} = [tmp{i}; rx_concurrency * SCALE];
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
        load e2e_link_pdrs;
        tmp{i} = [tmp{i}; e2e_link_pdrs(:, end)];
    end
end
idx = idx + 1;
data(:, idx) = tmp;

%% SCREAM
fprintf('processing SCREAM\n');
pdr_job = scream_job;

SCALE = 1024 / 32;
% ftsp slot
SCALE = SCALE * 128 / 100;

tmp = [];
% each job
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load e2e_link_pdrs;
    tmp = [tmp; e2e_link_pdrs(:, end)];
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
        s = data{i, j} * 100;
        barvalue(i, j) = mean(s);
        error(i, j) = norminv(1 - ALPHA / 2, 0, 1) * std(s) / sqrt(length(s));
    end
end

%% display bar
bw_ylabel = 'PDR (%)';
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
% h = barweb(barvalue, error, [], groupnames, [], bw_xlabel, bw_ylabel);
fprintf('skip the error and continue execution, barerrorbar breaks down for row vector\n');
h = barerrorbar({1:6, barvalue}, {1:6, barvalue, error, error, 'rx'});
%h.legend = legend(bw_legend, 'orientation', 'horizontal');
% h.legend = legend(bw_legend);
%%
set(gca, 'FontSize', 30);
ylabel(bw_ylabel);
% xlabel(bw_xlabel);
xlabel('');
set(gca, 'xticklabel', bw_legend);
str = ['peer_e2e_pdr_bar_multihop'];

%% process data for display
hold on;
M = size(data, 1);
N = size(data, 2);
for i = 1 : M
    for j = 1 : N
        s = data{i, j} * 100;
        s = s(s <= 100);
        cdfplot(s);
    end
end

%% display cdf
bw_xlabel = 'PDR (%)';
bw_ylabel = 'CDF';
legend(bw_legend);
% 
set(gca, 'FontSize', 30);
ylabel(bw_ylabel);
xlabel(bw_xlabel);
% xlabel('');
str = ['peer_e2e_pdr_cdf_multihop'];

%% maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
%
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
