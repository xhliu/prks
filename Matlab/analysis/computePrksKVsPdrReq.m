%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   11/1/2013
%   Function: compute trimmed_link_pdrs (i.e., meet pdr requirement) and display pdr vs pdr req for prks
%% 
% us; 1 hour seems a good choice for neteye
% 0.5 for Indriya
CONVERGE_TIME_IN_HOUR = 1;

% vary traffic load
IN_GAIN_IDX = 8;
ER_BORDER_GAIN_IDX = 9;
% vary interval
% IN_GAIN_IDX = 9;
% ER_BORDER_GAIN_IDX = 10;
% TX_POWER = -25;

job = prks_mixed_traffic_job;
BOOTSTRAP_TIME = CONVERGE_TIME_IN_HOUR * 3600 * 2 ^ 20;
% fprintf('pdr of PRKS after %f hours\n', CONVERGE_TIME_IN_HOUR);

pdr_req = [70 80 90 95];
pdr_req_str = {'70', '80', '90', '95'};

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
        
        % calculate K
        load debugs;
        t = debugs;
        type = DBG_CONTROLLER_FLAG;
        line = 1044;    %1042
        % type = DBG_HEARTBEAT_FLAG;
        % line = 172;
        t = t(t(:, 3) == type, :);
%         t = t(t(:, 4) == line, :);
        t = t(t(:, 4) == 1042 | t(:, 4) == 1044, :);
        % K = sender power / er border interferer power
        s = t(:, ER_BORDER_GAIN_IDX) - t(:, IN_GAIN_IDX);

        data{i} = [data{i}; s];
    end
end
fprintf('simple K calculation, not special case\n');

%%
group = zeros(0);
dataDisp = zeros(0);
for i = 1 : size(data, 1)
    group = [group ; ones(size(data{i})) + i - 1];
    dataDisp = [dataDisp ; data{i}];
end
% figure;
boxplot(dataDisp, group, 'notch', 'on');
%set(gca, 'xticklabel', groupnames);
set(gca, 'xtick', [1 2 3 4], 'xticklabel', groupnames);

%% display
bw_ylabel = 'PRK parameter (dB)';
set(gca, 'FontSize', 50);
% set(gca, 'yscale', 'log');
ylabel(bw_ylabel);
xlabel(bw_xlabel);

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
%%
str = ['prks_mixed_traffic_K_vs_pdr_req_boxplot'];
if ~is_neteye
    str = [str '_indriya'];
end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
