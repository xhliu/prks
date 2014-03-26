%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/15/14
%   Function: display latency vs pdr req for various protocols
%% 
% CI of mean or median
is_median = false;
% is_median = true;

idx = 0;
data = cell(PDR_REQ_CNT, PROTOCOL_CNT);
% SAMPLE_SIZE = 200;

%% PRKS NAMA
% us
SCALE = 5 / PRKS_NAMA_SLOT_LEN / 1000;

fprintf('processing PRKS NAMA\n');
job = prks_nama_job;

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
        % [sender receiver seqno latency]
        load link_seq_latency2;
        t = link_seq_latency;
%         senders = unique(t(:, 1));
%         % equal samples from various links
%         for k = 1 : length(senders)
%             s = t(t(:, 1) == senders(k), end);
% %             if length(s) > SAMPLE_SIZE
% %                 s = s(end - SAMPLE_SIZE, end);
% %             else
% %                 disp('less');
% %             end
%             s = s(s > 0, :);
%             if isempty(s)
%                 continue;
%             end
%             tmp{i} = [tmp{i}; median(s) * SCALE];
%         end
        t = t(:, end);
        t = t(t > 0, :);
        tmp{i} = [tmp{i}; t * SCALE];
    end
end
idx = idx + 1;
data(:, idx) = tmp;


%% PRKS ONAMA
% us
SCALE = 5 / PRKS_ONAMA_SLOT_LEN / 1000;

fprintf('processing PRKS ONAMA\n');
job = prks_onama_job;

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
        % [sender receiver seqno latency]
        load link_seq_latency2;
        t = link_seq_latency;
%         senders = unique(t(:, 1));
%         % equal samples from various links
%         for k = 1 : length(senders)
%             s = t(t(:, 1) == senders(k), end);
% %             if length(s) > SAMPLE_SIZE
% %                 s = s(end - SAMPLE_SIZE, end);
% %             else
% %                 disp('less');
% %             end
%             s = s(s > 0, :);
%             if isempty(s)
%                 continue;
%             end
%             tmp{i} = [tmp{i}; median(s) * SCALE];
%         end
        t = t(:, end);
        t = t(t > 0, :);
        tmp{i} = [tmp{i}; t * SCALE];
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
lo_err = nan(size(data));
hi_err = nan(size(data));
for i = 1 : M
    for j = 1 : N
        if ~is_median
            % mean and its CI
            barvalue(i, j) = mean(data{i, j});
            s = data{i, j};
            error(i, j) = norminv(1 - ALPHA / 2, 0, 1) * std(s) / sqrt(length(s));
        else
            % median and its CI
            r = data{i, j};
            r = sort(r);
            r(isnan(r)) = [];
            % err bound
            lo_idx = ceil(0.5 * size(r, 1) - 0.5 * norminv(1 - ALPHA / 2, 0, 1) * sqrt(size(r, 1)));
            lo = r(lo_idx);
            lo = median(r) - lo;
            hi_idx = ceil(0.5 * size(r, 1) + 0.5 * norminv(1 - ALPHA / 2, 0, 1) * sqrt(size(r, 1)));
            hi = r(hi_idx);
            hi = hi - median(r);

            barvalue(i, j) = median(r);
            lo_err(i, j) = lo;
            hi_err(i, j) = hi;
        end
    end
end

%% display
bw_ylabel = 'Latency (ms)';
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
if ~is_median
    h = barweb(barvalue, error, [], groupnames, [], bw_xlabel, bw_ylabel);
else
    h = barerrorbar({1:size(barvalue, 1), barvalue}, {repmat((1:size(barvalue, 1))', 1, PROTOCOL_CNT), barvalue, lo_err, hi_err, 'rx'});
end
%h.legend = legend(bw_legend, 'orientation', 'horizontal');
h.legend = legend(bw_legend);
set(gca, 'xticklabel', groupnames);
%%
set(gca, 'FontSize', 40);
% set(gca, 'yscale', 'log');
ylabel(bw_ylabel);
xlabel(bw_xlabel);
grid on;

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
%%
str = ['peer_latency_bar'];
if ~is_neteye
    str = [str '_indriya'];
end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
