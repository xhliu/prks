%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute concurrency in pkts / s; only for sync protocols
%% 
% MAIN_DIR = '~/Projects/tOR/RawData/';
% MAIN_DIR = '~/Downloads/Jobs/';
% jobs = [20857];
% 1) SCREAM or RIDB without OLAMA
% SLOT_LEN = 32; %32; %512;

% SLOT_LEN = 512;
% jobs = [];
% for i = 1 : length(prks_job)
%     jobs = [jobs; prks_job{i}(:, 1)];
% end

SLOT_LEN = 32;
jobs = [];
for i = 1 : length(scream_job)
    jobs = [jobs; scream_job{i}(:, 1)];
end


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

% %% sanity check jobs
% load debugs;
% t = debugs;
% % type = DBG_CONTROLLER_FLAG;
% % line = 1018; %686; %575;
% type = DBG_TDMA_FLAG;
% line = 543;
% t = t(t(:, 3) == type, :);
% t = t(t(:, 4) == line, :);
% s = t;
% % sum(s(:, 9) > s(:, 10))
% % s = s(s(:, 2) == 15, :);
% s = s(:, 10);
% 
% load link_pdrs;
% fprintf('job %d: %f, %d\n', job_id, max(s) / 32, size(link_pdrs, 1));

if 1
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
    % 
    [c e] = hist(s, unique(s));
    ce = [e c'];
    plot(e);
    % add slots w/o any rx
    total = max(s) - min(s) + 1;
    rx_concurrency = [c'; zeros(total - length(c), 1)];
    fprintf('<%d, %d>\n', min(s), max(s));
    
%     ce(1:10, :)

    % sanity check rx concurrency
    load link_pdrs;
    fprintf('total concurrency %d, total packets received %d, ratio: %f\n', sum(rx_concurrency), sum(link_pdrs(:, 4)), sum(rx_concurrency) / sum(link_pdrs(:, 4)));
    save('rx_concurrency.mat', 'rx_concurrency');
    cdfplot(rx_concurrency);
    fprintf('rx_concurrency median %f, mean %f, scaled %f\n', median(rx_concurrency), mean(rx_concurrency), 128 / 59 * mean(rx_concurrency));
end
end