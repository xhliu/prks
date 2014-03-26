%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/21/2014
%   Function: compute retx latency for multiple jobs
%%
%% whether sync protocol or not
% CSMA, RTS-CTS, CMAC: async
is_sync_protocol = false;
jobs = async_jobs;

% PRKS, SCREAM, and RIDB (w/ or w/o OLAMA): sync
% is_sync_protocol = true;
% jobs = sync_jobs;

for job_id = 1 : length(jobs)
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    computeRetxLatency(job_dir, is_sync_protocol);
end

is_sync_protocol = true;
jobs = sync_jobs;

for job_id = 1 : length(jobs)
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    computeRetxLatency(job_dir, is_sync_protocol);
end