%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/15/14
%   Function: display concurrency vs pdr req for various protocols
%%
idx = 0;
% if is_neteye
    data = cell(PDR_REQ_CNT, PROTOCOL_CNT);
% else
%     % no iOrder
%     data = cell(PDR_REQ_CNT, 2);
% end

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
        load concurrency;
        tmp{i} = [tmp{i}; concurrency];
    end
end
idx = idx + 1;
data(:, idx) = tmp;

%% PRKS-R
fprintf('processing PRKS-R\n');
job = prksr_job;

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
        load concurrency;
        tmp{i} = [tmp{i}; concurrency];
    end
end
idx = idx + 1;
data(:, idx) = tmp;

% %% PRKS-L
% fprintf('processing PRKS-L\n');
% job = prksl_job;
% 
% len = length(job);
% tmp = cell(len, 1);
% % each pdr req
% for i = 1 : len
%     pdr_job = job{i};
%     % each job
%     for j = 1 : size(pdr_job, 1)
%         fprintf('processing job %d\n', pdr_job(j, 1));
%         job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
%         cd(job_dir);
%         load concurrency;
%         tmp{i} = [tmp{i}; concurrency];
%     end
% end
% idx = idx + 1;
% data(:, idx) = tmp;


% if is_neteye
% %% iOrder
% idx = idx + 1;
% % iorder is from testiOrder
% data(:, idx) = iorder;
% end
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
bw_ylabel = 'Concurrency (packets per slot)';
% if is_neteye
%     bw_legend = {'PRKS-NAMA', 'PRKS-ONAMA', 'iOrder'};
% else
%     bw_legend = {'PRKS-NAMA', 'PRKS-ONAMA'};
% end
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
h = barweb(barvalue, error, [], groupnames, [], bw_xlabel, bw_ylabel);
%h.legend = legend(bw_legend, 'orientation', 'horizontal');
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

str = 'prks_variant_concurrency';
if ~is_neteye
    str = '_indriya';
end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
