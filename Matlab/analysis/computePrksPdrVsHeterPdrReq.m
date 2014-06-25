%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   11/1/2013
%   Function: display pdr vs pdr req, both homo and heterogeneous
%% 
pdr_req = [70 80 90 95];
pdr_req_str = {'70', '80', '90', '95'};
if 0
%% homogeneous link pdr req
% job{1} = [43302];
% job{2} = [43311];
% job{3} = [43301];
% job{4} = [43300];
% job{1} = [19809 19831];
% job{2} = [19811 19832];
% job{3} = [19802 19815 19834];
% job{4} = [19813 19835];

% % pdr req 70
% job{1} = [19920 19923];
% % 80
% job{2} = [19922 19924];
% % 90
% job{3} = [19925 19919];
% % 95
% job{4} = [19921 19926];

% main_dir = '~/Projects/tOR/RawData/';
% main_dir = '~/Downloads/Indriya/';

% us; 1 hour seems a good choice
BOOTSTRAP_TIME = 1 * 3600 * 2 ^ 20;
fprintf('processing PRKS\n');
job = prks_job;


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
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        
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
        
        s = link_pdrs(:, end);
        % outlier due to log issue
        s(s > 1) = [];
        s = s * 100;
        % filter
        s(s < pdr_req(i)) = [];
        data{i} = [data{i}; s];
    end
end
fprintf('pdr filter enabled\n');

end
%% heterogeneous link pdr req
% job 19928
PDR_REQ_IDX = 9;

load link_pdrs;
load debugs;
pdr = [];
%
fprintf('merge job 21616 & 21617 \n')
data = cell(length(pdr_req), 1);

t = debugs;
type = DBG_CONTROLLER_FLAG;
line = 1023;
t = t(t(:, 3) == type, :);
% do not consider line # bcoz sender and receiver id check later remove
% such line anyway
t = t(t(:, 4) == line, :);
for link_id = 1 : size(link_pdrs, 1)
    %link_id = 20;
    s = t;
    % receiver
    s = s(s(:, 2) == link_pdrs(link_id, 2), :);
    % sender
    s = s(s(:, 5) == link_pdrs(link_id, 1), :);
    
    % various pdr req
    if isempty(s)
        continue;
    end
    link_pdr_req = s(1, PDR_REQ_IDX);
    i = ceil((link_pdr_req - 70) / 10) + 1;
    
    s = link_pdrs(link_id, end);
    % outlier due to log issue
    s(s > 1) = [];
    s = s * 100;
    % filter
    s(s < pdr_req(i)) = [];
    data{i} = [data{i}; s];
end
fprintf('filter enabled\n');

%%
group = zeros(0);
dataDisp = zeros(0);
for i = 1 : size(data, 1)
    group = [group ; ones(size(data{i})) + i - 1];
    dataDisp = [dataDisp ; data{i}];
end
% figure;
boxplot(dataDisp, group, 'notch', 'on');
%set(gca, 'xticklabel', pdr_req_str);
set(gca, 'xtick', [1 2 3 4], 'xticklabel', groupnames);

%% display
bw_ylabel = 'PDR (%)';
set(gca, 'FontSize', 50);
% set(gca, 'yscale', 'log');
ylabel(bw_ylabel);
xlabel(bw_xlabel);

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
%
str = ['prks_pdr_vs_heter_pdr_req'];
if ~is_neteye
    str = [str '_indriya'];
end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
%% %% PRKS after convergence
% load txrxs;
% 
% TIMESTAMP_IDX = 10;
% SLOT_WRAP_LEN = 2 ^ 32;
% 
% SRC_IDX = 3;
% DST_IDX = 3;
% SEQ_IDX = 4;
% 
% % cope w/ time wrap around
% txs = unwrap(txs, SLOT_WRAP_LEN);
% rxs = unwrap(rxs, SLOT_WRAP_LEN);
% 
% % trim using global time
% % us; 1 hour seems a good choice
% BOOTSTRAP_TIME = 1 * 3600 * 2 ^ 20;
% txs = txs(txs(:, TIMESTAMP_IDX) >= BOOTSTRAP_TIME, :);
% rxs = rxs(rxs(:, TIMESTAMP_IDX) >= BOOTSTRAP_TIME, :);
% 
% link_pdrs = [];
% srcs = unique(txs(:, 2));
% 
% for i = 1 : size(srcs, 1)
%    src = srcs(i);
%    
%    tx = txs(txs(:, 2) == src, :);
%    dst = tx(1, DST_IDX);
%    
%    rx = rxs(rxs(:, 2) == dst & rxs(:, SRC_IDX) == src, :);
%    
%    link_pdrs = [link_pdrs; src dst size(tx, 1) size(rx, 1) size(rx, 1) / size(tx, 1)];
% end

%% MIN_SAMPLE_SIZE does not help to inc pdr
% MIN_SAMPLE_SIZE = 20 * 100;
% hold on;
% t = link_pdrs;
% s = t(t(:, 3) >= MIN_SAMPLE_SIZE, end);
% sum(s >= 0.9) / length(s)
% cdfplot(s);