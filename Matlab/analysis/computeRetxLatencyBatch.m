%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/21/2014
%   Function: compute retx latency for multiple jobs
%%
%% whether sync protocol or not
% CSMA, RTS-CTS, CMAC: async
% is_sync_protocol = false;
% jobs = async_jobs;

% % PRKS, SCREAM, and RIDB (w/ or w/o OLAMA): sync
is_sync_protocol = false;
jobs = sync_jobs;
% 
% for job_id = 1 : length(jobs)
%     job_dir = [MAIN_DIR num2str(jobs(job_id))];
%     computeRetxLatency(job_dir, is_sync_protocol);
% end


% is_sync_protocol = true;
% jobs = sync_jobs;
% jobs = [49229];
% jobs = [];
% for i = 1 : length(scream_job)
%     jobs = [jobs; scream_job{i}(:, 1)];
% end

for job_id = 1 : length(jobs)
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    computeRetxLatency(job_dir, is_sync_protocol);
end

%% link_seq_tx_cnt_latency [sender receiver seqno tx_attempt_cnt latency]

