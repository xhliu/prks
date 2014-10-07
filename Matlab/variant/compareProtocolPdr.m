%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/14/14
%   Function: display pdr vs pdr req for various protocols
%% 
idx = 0;
data = cell(PDR_REQ_CNT, PROTOCOL_CNT);

%% PRKS
fprintf('processing PRKS\n');
job = prks_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
%{
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j));
        job_dir = [MAIN_DIR num2str(pdr_job(j))];
        cd(job_dir);
        load link_pdrs;
        s = link_pdrs(:, end) * 100;
        tmp{i} = [tmp{i}; s];
    end
end
%}

%
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        % load link_pdrs;
        load trimmed_link_pdrs
        s = trimmed_link_pdrs(:, end) * 100;
        tmp{i} = [tmp{i}; s];
    end
end
%}
idx = idx + 1;
data(:, idx) = tmp;


%% PRKS
fprintf('processing PRKS\n');
job = prksr_job;

len = length(job);
tmp = cell(len, 1);

%
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        % load link_pdrs;
        load trimmed_link_pdrs
        s = trimmed_link_pdrs(:, end) * 100;
        tmp{i} = [tmp{i}; s];
    end
end
%}
idx = idx + 1;
data(:, idx) = tmp;


%% PDR bargraph for diff. prococols
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
bw_ylabel = 'PDR (%)';
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
h = barweb(barvalue, error, [], groupnames);
h.legend = legend(bw_legend, 'orientation', 'horizontal');

%% 
set(gca, 'FontSize', 30);
ylabel(bw_ylabel);
xlabel(bw_xlabel);
set(gca, 'ytick', 0:10:100);

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
%
str = ['prks_variant_pdr'];
if ~is_neteye
    str = [str '_indriya'];
end
export_fig(gcf, str, '-eps');
export_fig(gcf, str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);