%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   11/1/2013
%   Function: compute trimmed_link_pdrs (i.e., meet pdr requirement) and display pdr vs pdr req for prks
%% 
% us; 1 hour seems a good choice for neteye
% 0.5 for Indriya
CONVERGE_TIME_IN_HOUR = 1;

job = prks_job;
BOOTSTRAP_TIME = CONVERGE_TIME_IN_HOUR * 3600 * 2 ^ 20;
fprintf('pdr of PRKS after %f hours\n', CONVERGE_TIME_IN_HOUR);

pdr_req = [70];
% pdr_req_str = {'70', '80', '90', '95'};

TIMESTAMP_IDX = 10;
SLOT_WRAP_LEN = 2 ^ 32;

SRC_IDX = 3;
DST_IDX = 3;
% SEQ_IDX = 4;

len = length(job);
data = cell(len, 1);
% each pdr req
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        if exist('trimmed_link_pdrs.mat', 'file')
            fprintf('skip processed job %d\n', pdr_job(j, 1));
            load trimmed_link_pdrs;
            data{i} = [data{i}; trimmed_link_pdrs(:, end) * 100];
            continue;
        else
            fprintf('processing job %d\n', pdr_job(j, 1));
        end
        
        %load link_pdrs;
        % compute link_pdrs after convergence, unlike raw pdr
        load txrxs;
        
        % cope w/ time wrap around
        txs = unwrap(txs, SLOT_WRAP_LEN);
        rxs = unwrap(rxs, SLOT_WRAP_LEN);
        
        % trim using global time
        txs = txs(txs(:, TIMESTAMP_IDX) >= BOOTSTRAP_TIME, :);
        rxs = rxs(rxs(:, TIMESTAMP_IDX) >= BOOTSTRAP_TIME, :);

        link_pdrs = [];
        srcs = unique(txs(:, 2));

        for k = 1 : size(srcs, 1)
           src = srcs(k);

           tx = txs(txs(:, 2) == src, :);
           dst = tx(1, DST_IDX);

           rx = rxs(rxs(:, 2) == dst & rxs(:, SRC_IDX) == src, :);

           link_pdrs = [link_pdrs; src dst size(tx, 1) size(rx, 1) size(rx, 1) / size(tx, 1)];
        end
        s = link_pdrs;
        trimmed_link_pdrs = s(s(:, end) * 100 >= pdr_req(i) & s(:, end) <= 1, :);
        save('trimmed_link_pdrs.mat', 'trimmed_link_pdrs');
        
        s = link_pdrs(:, end);
        % outlier due to missing log
        s(s > 1) = [];
        s = s * 100;
        % filter
        s(s < pdr_req(i)) = [];
        data{i} = [data{i}; s];
    end
end
% fprintf('pdr filter enabled\n');

% %%
% group = zeros(0);
% dataDisp = zeros(0);
% for i = 1 : size(data, 1)
%     group = [group ; ones(size(data{i})) + i - 1];
%     dataDisp = [dataDisp ; data{i}];
% end
% % figure;
% boxplot(dataDisp, group, 'notch', 'on');
% %set(gca, 'xticklabel', groupnames);
% set(gca, 'xtick', [1 2 3 4], 'xticklabel', groupnames);
% 
% %% display
% bw_ylabel = 'PDR (%)';
% set(gca, 'FontSize', 50);
% % set(gca, 'yscale', 'log');
% ylabel(bw_ylabel);
% xlabel(bw_xlabel);
% 
% maximize;
% set(gcf, 'Color', 'white');
% cd(FIGURE_DIR);
% % cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
% %
% str = ['prks_mix_txpower_pdr_vs_pdr_req'];
% if ~is_neteye
%     str = [str '_indriya'];
% end
% export_fig(str, '-eps');
% export_fig(str, '-jpg', '-zbuffer');
% saveas(gcf, [str '.fig']);
