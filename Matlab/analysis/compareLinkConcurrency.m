%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/15/14
%   Function: display concurrency vs pdr req for various protocols
%%
% SCALE = 1;
% SCALE = 1024 / 5;
SCALE = 1024 / 5 * 1140 / 305.9;
PDR_REQ_CNT = 1;
PROTOCOL_CNT = 2;
MAIN_DIR = '~/Projects/tOR/RawData/';
idx = 0;
data = cell(PDR_REQ_CNT, PROTOCOL_CNT);

%% PRKS
fprintf('processing PRKS\n');
% SCALE = 1024 / 5;
% pdr_job = cell(0);
pdr_job = [21143]; % 21101

%len = length(job);
tmp = [];
% each job
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load rx_concurrency_sample;
    tmp = [tmp; rx_concurrency * SCALE];
end
idx = idx + 1;
for i = 1 : PDR_REQ_CNT
    data{i, idx} = tmp;
end


%% SCREAM
fprintf('processing SCREAM\n');
% SCALE = 1024 / 5;
% pdr_job = cell(0);
pdr_job = [20849];

tmp = [];
% each job
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load rx_concurrency_sample;
    tmp = [tmp; rx_concurrency * SCALE];
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
groupnames = [];
bw_legend = {'PRKS', 'SCREAM'};
bw_xlabel = [];
bw_ylabel = 'Packets / second';
% bw_ylabel = 'Packets / slot';
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
h = barweb(barvalue, error, [], groupnames, [], bw_xlabel, bw_ylabel);
h.legend = legend(bw_legend, 'orientation', 'vertical');

%%
set(gca, 'FontSize', 30);
ylabel(bw_ylabel);
xlabel(bw_xlabel);

maximize;
set(gcf, 'Color', 'white');
% cd(FIGURE_DIR);
cd('~/Dropbox/iMAC/Xiaohui/figure/');
%
% if is_neteye
% str = ['prks_vs_scream_rx_concurrency']; %
str = ['prks_vs_scream_link_normalized_throughput'];
% else
% str = ['indriya_peer_concurrency_bar'];
% end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
