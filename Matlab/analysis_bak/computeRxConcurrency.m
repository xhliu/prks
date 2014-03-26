%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute concurrency in pkts / s; only for sync protocols
%% 
jobs = [];
% 1) SCREAM or RIDB w/o OLAMA
% SLOT_LEN = 32;
% jobs = [jobs; scream_job(:, 1)];
% for i = 1 : length(ridb_job)
%     jobs = [jobs; ridb_job{i}(:, 1)];
% end

% 2) PRKS or RIDB w/ OLAMA
SLOT_LEN = 512;
for i = 1 : length(prks_job)
    jobs = [jobs; prks_job{i}(:, 1)];
end
for i = 1 : length(ridbolama_job)
    jobs = [jobs; ridbolama_job{i}(:, 1)];
end

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

    %% time wrap around
    t = unwrap(rxs, SLOT_WRAP_LEN);
    s = floor(t(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));
    %% 
    [c e] = hist(s, unique(s));
    ce = [e c'];

    %% figure;
    rx_concurrency = ce(:, end);
    % sanity check rx concurrency
    load link_pdrs;
    fprintf('total concurrency %d, total packets received %d, ratio: %f\n', sum(rx_concurrency), sum(link_pdrs(:, 4)), sum(rx_concurrency) / sum(link_pdrs(:, 4)));
    save('rx_concurrency.mat', 'rx_concurrency');
    cdfplot(rx_concurrency);
    fprintf('rx_concurrency median %f, mean %f\n', median(rx_concurrency), mean(rx_concurrency));

end