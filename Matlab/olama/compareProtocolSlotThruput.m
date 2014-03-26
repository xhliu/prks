%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/20/14
%   Function: display per slot throughput vs pdr req for various protocols
%%
idx = 0;
data = cell(PDR_REQ_CNT, PROTOCOL_CNT);

%% PRKS
fprintf('processing PRKS NAMA\n');
job = prks_nama_job;

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


%% PRKS
fprintf('processing PRKS ONAMA\n');
job = prks_onama_job;

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
bw_ylabel = 'Throughput (packets/sec)';
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
h = barweb(barvalue, error, [], groupnames, [], bw_xlabel, bw_ylabel);
h.legend = legend(bw_legend);

%%
set(gca, 'FontSize', 40);
ylabel(bw_ylabel);
xlabel(bw_xlabel);
grid on;

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');

str = ['peer_slot_throughput_bar'];
if ~is_neteye
    str = [str '_indriya'];
end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);

% [358 1083;    340 978;    331 883;    303 750]