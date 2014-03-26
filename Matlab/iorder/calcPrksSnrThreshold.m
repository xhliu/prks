%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/26/14
%   Function: find the SNR threshold PRKS can ensure at various pdr req
%   
%% PRKS
fprintf('processing PRKS\n');
job = prks_job;

len = length(job);

%
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        % [slot, # of concurrent links, signal, noise+interference, snr]
        load link_snrs;
    end
end


