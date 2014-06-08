%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/22/14
%   Function: display retx latency vs pdr req for various protocols
%% use mean or median
is_median = false;

%% 
idx = 0;
data = cell(PDR_REQ_CNT, PROTOCOL_CNT);

%% PRKS
CONVERGE_TIME_IN_HOUR = 1;
BOOTSTRAP_TIME = CONVERGE_TIME_IN_HOUR * 3600 * 2 ^ 20;
fprintf('retx delay of PRKS after %f hours\n', CONVERGE_TIME_IN_HOUR);

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
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_seq_tx_cnt_latency_2;
        % [sender receiver seqno tx_attempt_cnt tx_timestamp rx_timestamp latency]
        t = link_seq_tx_cnt_latency_2;
        t = t(t(:, end) > 0, :);
        % after convergence
        t = t(t(:, 5) >= BOOTSTRAP_TIME, :);
%         % max # of transmissions, including retx
%         max_tx_cnt = quantile(t(:, 4), pdr_reqs(i) / 100);
%         t = t(t(:, 4) <= max_tx_cnt, end);
%         tmp{i} = [tmp{i}; t * SCALE];
        % each link, i.e., each sender
        senders = unique(t(:, 1));
        for k = 1 : length(senders)
            sender = senders(k);
            s = t(t(:, 1) == sender, :);
            
            max_tx_cnt = quantile(s(:, 4), pdr_reqs(i) / 100);
            s = s(s(:, 4) <= max_tx_cnt, end);
            tmp{i} = [tmp{i}; s * SCALE];
        end
    end
end
idx = idx + 1;
data(:, idx) = tmp;



%% CSMA
SCALE = 1;

fprintf('processing CSMA\n');
pdr_job = csma_job;

len = PDR_REQ_CNT; %length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    %pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_seq_tx_cnt_latency_2;
        t = link_seq_tx_cnt_latency_2;
        t = t(t(:, end) > 0, :);
        % each link, i.e., each sender
        senders = unique(t(:, 1));
        for k = 1 : length(senders)
            sender = senders(k);
            s = t(t(:, 1) == sender, :);
            
            max_tx_cnt = quantile(s(:, 4), pdr_reqs(i) / 100);
            s = s(s(:, 4) <= max_tx_cnt, end);
            tmp{i} = [tmp{i}; s * SCALE];
        end
    end
end
idx = idx + 1;
data(:, idx) = tmp;




%% RTSCTS
SCALE = 1;

fprintf('processing RTSCTS\n');
pdr_job = rtscts_job;

len = PDR_REQ_CNT; %length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    %pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_seq_tx_cnt_latency_2;
        t = link_seq_tx_cnt_latency_2;
        t = t(t(:, end) > 0, :);
        % each link, i.e., each sender
        senders = unique(t(:, 1));
        for k = 1 : length(senders)
            sender = senders(k);
            s = t(t(:, 1) == sender, :);

            max_tx_cnt = quantile(s(:, 4), pdr_reqs(i) / 100);
            s = s(s(:, 4) <= max_tx_cnt, end);
            tmp{i} = [tmp{i}; s * SCALE];
        end
    end
end
idx = idx + 1;
data(:, idx) = tmp;

%% RIDB
% us
SCALE = 5 / RIDB_SLOT_LEN / 1000;

fprintf('processing RIDB\n');
job = ridb_job;

len = PDR_REQ_CNT; %length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_seq_tx_cnt_latency_2;
        t = link_seq_tx_cnt_latency_2;
        t = t(t(:, end) > 0, :);
        % each link, i.e., each sender
        senders = unique(t(:, 1));
        for k = 1 : length(senders)
            sender = senders(k);
            s = t(t(:, 1) == sender, :);

            max_tx_cnt = quantile(s(:, 4), pdr_reqs(i) / 100);
            s = s(s(:, 4) <= max_tx_cnt, end);
            tmp{i} = [tmp{i}; s * SCALE];
        end
    end
end
idx = idx + 1;
data(:, idx) = tmp;

%% RIDB_OLAMA
%{
% us
SCALE = 5 / RIDBOLAMA_SLOT_LEN / 1000;

fprintf('processing RIDB_OLAMA\n');
job = ridbolama_job;

len = PDR_REQ_CNT; %length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_seq_tx_cnt_latency_2;
        t = link_seq_tx_cnt_latency_2;
        % each link, i.e., each sender
        senders = unique(t(:, 1));
        for k = 1 : length(senders)
            sender = senders(k);
            s = t(t(:, 1) == sender, :);

            max_tx_cnt = quantile(s(:, 4), pdr_reqs(i) / 100);
            s = s(s(:, 4) <= max_tx_cnt, end);
            tmp{i} = [tmp{i}; s * SCALE];
        end
    end
end
idx = idx + 1;
data(:, idx) = tmp;
%}


%% CMAC
SCALE = 1;

fprintf('processing CMAC\n');
job = cmac_job;

len = PDR_REQ_CNT; %length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    test_median=[];
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_seq_tx_cnt_latency_2;
        t = link_seq_tx_cnt_latency_2;
        t = t(t(:, end) > 0, :);
        % each link, i.e., each sender
        senders = unique(t(:, 1));
        for k = 1 : length(senders)
            sender = senders(k);
            s = t(t(:, 1) == sender, :);

            max_tx_cnt = quantile(s(:, 4), pdr_reqs(i) / 100);
            s = s(s(:, 4) <= max_tx_cnt, end);
            test_median=[test_median; s];
            tmp{i} = [tmp{i}; s * SCALE];
        end
    end
    fprintf ('median is: %d\n',median(test_median));
end
idx = idx + 1;
data(:, idx) = tmp;


%% SCREAM
% us
SCALE = 5 / SCREAM_SLOT_LEN / 1000;

fprintf('processing SCREAM\n');
% pdr_job = scream_job;

% len = PDR_REQ_CNT; %length(job);
% tmp = cell(len, 1);
% % each pdr req
% for i = 1 : len
%     %pdr_job = job{i};
%     % each job
%     for j = 1 : size(pdr_job, 1)
%         fprintf('processing job %d\n', pdr_job(j, 1));
%         job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
%         cd(job_dir);
%         load link_seq_tx_cnt_latency_2;
%         t = link_seq_tx_cnt_latency_2;
%         t = t(t(:, end) > 0, :);
%         % each link, i.e., each sender
%         senders = unique(t(:, 1));
%         for k = 1 : length(senders)
%             sender = senders(k);
%             s = t(t(:, 1) == sender, :);
% 
%             max_tx_cnt = quantile(s(:, 4), pdr_reqs(i) / 100);
%             s = s(s(:, 4) <= max_tx_cnt, end);
%             tmp{i} = [tmp{i}; s * SCALE];
%         end
%     end
% end
% idx = idx + 1;
% data(:, idx) = tmp;

job = scream_job;

len = PDR_REQ_CNT; %length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_seq_tx_cnt_latency_2;
        t = link_seq_tx_cnt_latency_2;
        t = t(t(:, end) > 0, :);
        % each link, i.e., each sender
        senders = unique(t(:, 1));
        for k = 1 : length(senders)
            sender = senders(k);
            s = t(t(:, 1) == sender, :);

            max_tx_cnt = quantile(s(:, 4), pdr_reqs(i) / 100);
            s = s(s(:, 4) <= max_tx_cnt, end);
            tmp{i} = [tmp{i}; s * SCALE];
        end
    end
end
idx = idx + 1;
data(:, idx) = tmp;

%% process data for display
ALPHA = 0.05;
M = size(data, 1);
N = size(data, 2);
barvalue = nan(size(data));
error = zeros(size(data));

lo_err = nan(size(data));
hi_err = nan(size(data));
for i = 1 : M
    for j = 1 : N
        if is_median
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
        
        else
            % mean and its CI
            barvalue(i, j) = mean(data{i, j});
            s = data{i, j};
            error(i, j) = norminv(1 - ALPHA / 2, 0, 1) * std(s) / sqrt(length(s));
        end
    end
end

%% display
bw_ylabel = 'Latency (ms)';
% % bw: short for barweb
% %barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
% %bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
if is_median
    h = barerrorbar({1:size(barvalue, 1), barvalue}, {repmat((1:size(barvalue, 1))', 1, PROTOCOL_CNT), barvalue, lo_err, hi_err, 'rx'});
else
    h = barweb(barvalue, error, [], groupnames, [], bw_xlabel, bw_ylabel);
end
h.legend = legend(bw_legend, 'orientation', 'horizontal');
%
set(gca, 'FontSize', 30);
set(gca, 'yscale', 'log');
ylabel(bw_ylabel);
xlabel(bw_xlabel);
set(gca, 'xticklabel', groupnames);
% ylim([1 10^5]);
%%
maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
%
if is_median
    str = ['peer_retx_latency_median_bar_with_ci_2'];
else
    str = ['peer_retx_latency_mean_bar'];
end
if ~is_neteye
    str = [str '_indriya'];
end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
