%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   7/10/2014
%   Function: compute ctrl overhead: # of ctrl pkts / # of DATA pkts; # of
%   ctrl pkts approximate w/ # of slots since 1 ctrl wins contention each
%   slot
%% 
% MAIN_DIR = '~/Projects/tOR/RawData/';
% MAIN_DIR = '~/Downloads/Jobs/';
% jobs = [20865];
% 1) SCREAM or RIDB without OLAMA
% SLOT_LEN = 32; %32; %512;

SLOT_LEN = 512;
jobs = [];
for i = 1 : length(prks_job)
    jobs = [jobs; prks_job{i}(:, 1)];
end

% SLOT_LEN = 32; %32; %512;
% for i = 1 : length(ridb_job)
%     jobs = [jobs; ridb_job{i}(:, 1)];
% end
ctrl_overhead_ratio = [];

fprintf('slot length %d\n', SLOT_LEN);

for job_id = 1 : length(jobs)
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    cd(job_dir);
%     if exist('rx_concurrency.mat', 'file')
%         fprintf('skip processed %d-th job %d\n', job_id, jobs(job_id));
%         continue;
%     else
        fprintf('processing %d-th job job %d\n', job_id, jobs(job_id));
%     end
    load link_pdrs;
    %%
    load txrxs;
    % time in us
    SLOT_WRAP_LEN = 2 ^ 32;
    TIMESTAMP_IDX = 10;

    %% time wrap around
%     t = unwrap(rxs, SLOT_WRAP_LEN);
%     s = floor(t(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));
%     %% 
%     [c e] = hist(s, unique(s));
%     ce = [e c'];
%     %% figure;
%     rx_concurrency = ce(:, end);

    rxs = unwrap(rxs, SLOT_WRAP_LEN);
    s = floor(rxs(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));
    total = max(s) - min(s) + 1;
    
    % # of data packet
    fprintf('%d, %d, %d, %d, %f\n', min(s), max(s), total, sum(link_pdrs(:, 3)), total/sum(link_pdrs(:, 3)));
    ctrl_overhead_ratio = [ctrl_overhead_ratio; jobs(job_id) total/sum(link_pdrs(:, 3))];
end

cdfplot(ctrl_overhead_ratio(:, end) * 100);

set(gca, 'FontSize', 30);
xlabel('Control overhead (%)');
ylabel('CDF');
title('');
xlim([10 26]);

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
%
str = ['ctrl_overhead'];

export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);