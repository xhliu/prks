%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute concurrency in pkts / s; only for sync protocols
%% 
% MAIN_DIR = '~/Projects/tOR/RawData/';
% MAIN_DIR = '~/Downloads/Jobs/';
jobs = [];

SLOT_LEN = 32;
for i = 1 : length(scream_job)
    jobs = [jobs; scream_job{i}(:, 1)];
end

% SLOT_LEN = 512; %32; %512;
% for i = 1 : length(prks_onama_job)
%     jobs = [jobs; prks_onama_job{i}(:, 1)];
% end


fprintf('slot length %d\n', SLOT_LEN);

for job_id = 1 : length(jobs)
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    cd(job_dir);
    if exist('rx_concurrency.mat', 'file')
        fprintf('skip processed %d-th job %d\n', job_id, jobs(job_id));
        continue;
    else
        fprintf('processing %d-th job job %d\n', job_id, jobs(job_id));
    end

    %%
    load txrxs;
    % time in us
    SLOT_WRAP_LEN = 2 ^ 32;
    TIMESTAMP_IDX = 10;

%     %% time wrap around
%     t = unwrap(rxs, SLOT_WRAP_LEN);
%     s = floor(t(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));
%     %% 
%     [c e] = hist(s, unique(s));
%     ce = [e c'];
% 
%     %% figure;
%     rx_concurrency = ce(:, end);

    rxs = unwrap(rxs, SLOT_WRAP_LEN);
    s = floor(rxs(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));
    % 
    [c e] = hist(s, unique(s));
    plot(e);
    % add slots w/o any rx
    total = max(s) - min(s) + 1;
    rx_concurrency = [c'; zeros(total - length(c), 1)];

    % sanity check rx concurrency
    load link_pdrs;
    fprintf('total concurrency %d, total packets received %d, ratio: %f\n', sum(rx_concurrency), sum(link_pdrs(:, 4)), sum(rx_concurrency) / sum(link_pdrs(:, 4)));
    save('rx_concurrency.mat', 'rx_concurrency');
    cdfplot(rx_concurrency);
    fprintf('rx_concurrency median %f, mean %f, 1.28 scale %f\n', median(rx_concurrency), mean(rx_concurrency), 1.28 * mean(rx_concurrency));

end