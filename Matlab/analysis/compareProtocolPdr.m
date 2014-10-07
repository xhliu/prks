%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/14/14
%   Function: display pdr vs pdr req for various protocols
%% 
idx = 0;
data = cell(PDR_REQ_CNT, PROTOCOL_CNT);

%% PRKS
fprintf('processing PRKS\n');
job = prks_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
%{
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j));
        job_dir = [MAIN_DIR num2str(pdr_job(j))];
        cd(job_dir);
        load link_pdrs;
        s = link_pdrs(:, end) * 100;
        tmp{i} = [tmp{i}; s];
    end
end
%}

%
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        % load link_pdrs;
        load trimmed_link_pdrs
        s = trimmed_link_pdrs(:, end) * 100;
        tmp{i} = [tmp{i}; s];
    end
end
%}
idx = idx + 1;
data(:, idx) = tmp;


%% CSMA
fprintf('processing CSMA\n');
pdr_job = csma_job;

len = length(job);
tmp = [];
% each job
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load link_pdrs;
    s = link_pdrs(:, end) * 100;
    tmp = [tmp; s];
end
idx = idx + 1;
for i = 1 : PDR_REQ_CNT
    data{i, idx} = tmp;
end


%% RTSCTS
fprintf('processing RTSCTS\n');
pdr_job = rtscts_job;

len = length(job);
tmp = [];
% each job
for j = 1 : size(pdr_job, 1)
    fprintf('processing job %d\n', pdr_job(j, 1));
    job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
    cd(job_dir);
    load link_pdrs;
    s = link_pdrs(:, end) * 100;
    tmp = [tmp; s];
end
idx = idx + 1;
for i = 1 : PDR_REQ_CNT
    data{i, idx} = tmp;
end


%% RIDB
fprintf('processing RIDB\n');
job = ridb_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_pdrs;
        s = link_pdrs(:, end) * 100;
        tmp{i} = [tmp{i}; s];
    end
end
idx = idx + 1;
data(:, idx) = tmp;


%% RIDB_OLAMA
%{
fprintf('processing RIDB_OLAMA\n');
job = ridbolama_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_pdrs;
        s = link_pdrs(:, end) * 100;
        tmp{i} = [tmp{i}; s];
    end
end
idx = idx + 1;
data(:, idx) = tmp;
%}

%% CMAC
fprintf('processing CMAC\n');
job = cmac_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_pdrs;
        s = link_pdrs(:, end) * 100;
        tmp{i} = [tmp{i}; s];
    end
end
idx = idx + 1;
data(:, idx) = tmp;



%% SCREAM
fprintf('processing SCREAM\n');
% pdr_job = scream_job;
% 
% len = length(job);
% tmp = [];
% % each job
% for j = 1 : size(pdr_job, 1)
%     fprintf('processing job %d\n', pdr_job(j, 1));
%     job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
%     cd(job_dir);
%     load link_pdrs;
%     s = link_pdrs(:, end) * 100;
%     tmp = [tmp; s];
% end
% idx = idx + 1;
% for i = 1 : PDR_REQ_CNT
%     data{i, idx} = tmp;
% end
job = scream_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        fprintf('processing job %d\n', pdr_job(j, 1));
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
        load link_pdrs;
        s = link_pdrs(:, end) * 100;
        tmp{i} = [tmp{i}; s];
    end
end
idx = idx + 1;
data(:, idx) = tmp;

%% PDR bargraph for diff. prococols
%% process data for display
ALPHA = 0.05;
M = size(data, 1);
N = size(data, 2);
barvalue = nan(size(data));
error = nan(size(data));
for i = 1 : M
    for j = 1 : N
        barvalue(i, j) = mean(data{i, j});
        s = data{i, j};
        error(i, j) = norminv(1 - ALPHA / 2, 0, 1) * std(s) / sqrt(length(s));
    end
end

%% display
bw_ylabel = 'PDR (%)';
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
h = barweb(barvalue, error, [], groupnames);
h.legend = legend(bw_legend, 'orientation', 'horizontal');

%% 
set(gca, 'FontSize', 30);
ylabel(bw_ylabel);
xlabel(bw_xlabel);
set(gca, 'ytick', 0:10:100);

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');
%
str = ['peer_pdr_bar'];
if ~is_neteye
    str = [str '_indriya'];
end
export_fig(gcf, str, '-eps');
export_fig(gcf, str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);

if 0
%% CDF of PDR for all protocols in a single figure
% legend_str = {'PRKS70', 'PRKS80', 'PRKS90', 'PRKS95', ...
%              'CSMA',   'RTS-CTS', ...
%              'RIDB70', 'RIDB80', 'RIDB90', 'RIDB95', ...
%              'RIDB-OLAMA70', 'RIDB-OLAMA80',  'RIDB-OLAMA90', 'RIDB-OLAMA95', ...
%              'CMAC70', 'CMAC80', 'CMAC90', 'CMAC95', 'SCREAM'};
legend_str = {'PRKS70', 'PRKS80', 'PRKS90', 'PRKS95', ...
             'CSMA',   'RTS-CTS', ...
             'RIDB70', 'RIDB80', 'RIDB90', 'RIDB95', ...
             'CMAC70', 'CMAC80', 'CMAC90', 'CMAC95', 'SCREAM'};      
%%
% PRKS
color = ['r' 'g' 'm' 'b' 'c' 'k' 'y'];
protocol_ind = 1;
figure
for i = 1: M
    cdfvalue = data{i, protocol_ind};
    h = cdfplot(cdfvalue);
    set(h, 'Color', color(i), 'LineStyle', '-', 'LineWidth', 2)
    hold on
end
protocol_ind = protocol_ind + 1;

% CSMA
cdfvalue = data{1, protocol_ind};
h = cdfplot(cdfvalue);
set(h, 'Color', 'y', 'LineStyle', '-', 'LineWidth', 2)
hold on
protocol_ind = protocol_ind + 1;

% RTSCTS
cdfvalue = data{1, protocol_ind};
h = cdfplot(cdfvalue);
set(h, 'Color', 'k', 'LineStyle', '--', 'LineWidth', 2)
hold on
protocol_ind = protocol_ind + 1;


% RIDB
for i = 1: M
    cdfvalue = data{i, protocol_ind};
    h = cdfplot(cdfvalue);
    set(h, 'Color', color(i), 'LineStyle', '-', 'Marker', 'x')
    hold on
end
protocol_ind = protocol_ind + 1;

% RIDB-OLAMA
%{
for i = 1: M
    cdfvalue = data{i, protocol_ind};
    h = cdfplot(cdfvalue);
    set(h, 'Color', color(i), 'LineStyle', '-', 'Marker', '.')
    hold on
end
protocol_ind = protocol_ind + 1;
%}

% CMAC
for i = 1: M
    cdfvalue = data{i, protocol_ind};
    h = cdfplot(cdfvalue);
    set(h, 'Color', color(i), 'LineStyle', '-')
    hold on
end
protocol_ind = protocol_ind + 1;

% SCREAM
cdfvalue = data{i, protocol_ind};
h = cdfplot(cdfvalue);
set(h, 'Color', 'c', 'LineStyle', '-', 'LineWidth', 2)
hold on
protocol_ind = protocol_ind + 1;

%h.legend = legend(bw_legend, 'orientation', 'horizontal');   
h.legend = columnlegend(1, legend_str);

xlabel('PDR (%)');
ylabel('CDF');
set(gca, 'FontSize', 30);

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
%
if is_neteye
str = ['pdr_cdf_all_protocols'];
else
str = ['indriya_pdr_cdf_all_protocols'];
end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);


%% CDF of PDR of diff. protocols for each PDR requirment
% legend_str = {'PRKS', 'CSMA', 'RTS-CTS', 'RIDB', 'RIDB-OLAMA', 'CMAC', 'SCREAM'};
legend_str = {'PRKS', 'CSMA', 'RTS-CTS', 'RIDB', 'CMAC', 'SCREAM'};   
pdr_req = [70 80 90 95];
marker = ['o' '*' '<' '.' 'v' '>'];
DOWN_SCALE = 5;
for i = 1: M
figure
protocol_ind = 1;
% PRKS
cdfvalue = data{i, protocol_ind};
%     h = cdfplot(cdfvalue);
[f, x] = ecdf(cdfvalue);
h = plot(x(1:DOWN_SCALE:end), f(1:DOWN_SCALE:end));
set(h, 'Color', 'r', 'LineStyle', '-', 'LineWidth', 2, 'Marker', marker(protocol_ind));
hold on
protocol_ind = protocol_ind + 1;

% CSMA
cdfvalue = data{1, protocol_ind};
%     h = cdfplot(cdfvalue);
[f, x] = ecdf(cdfvalue);
h = plot(x(1:DOWN_SCALE:end), f(1:DOWN_SCALE:end));
set(h, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2, 'Marker', marker(protocol_ind));
hold on
protocol_ind = protocol_ind + 1;

% RTSCTS
cdfvalue = data{1, protocol_ind};
%     h = cdfplot(cdfvalue);
[f, x] = ecdf(cdfvalue);
h = plot(x(1:DOWN_SCALE:end), f(1:DOWN_SCALE:end));
set(h, 'Color', 'm', 'LineStyle', '-', 'LineWidth', 2, 'Marker', marker(protocol_ind));
hold on
protocol_ind = protocol_ind + 1;

% RIDB
cdfvalue = data{i, protocol_ind};
%     h = cdfplot(cdfvalue);
[f, x] = ecdf(cdfvalue);
h = plot(x(1:DOWN_SCALE:end), f(1:DOWN_SCALE:end));
set(h, 'Color', 'k', 'LineStyle', '-', 'LineWidth', 2, 'Marker', marker(protocol_ind));
hold on
protocol_ind = protocol_ind + 1;

% RIDB-OLAMA
%{
cdfvalue = data{i, protocol_ind};
h = cdfplot(cdfvalue);
set(h, 'Color', color(i), 'LineStyle', '-', 'Marker', '.')
hold on
protocol_ind = protocol_ind + 1;
%}

% CMAC
cdfvalue = data{i, protocol_ind};
%     h = cdfplot(cdfvalue);
[f, x] = ecdf(cdfvalue);
h = plot(x(1:DOWN_SCALE:end), f(1:DOWN_SCALE:end));
set(h, 'Color', 'c', 'LineStyle', '-', 'LineWidth', 2, 'Marker', marker(protocol_ind));
hold on
protocol_ind = protocol_ind + 1;

% SCREAM
cdfvalue = data{1, protocol_ind};
%     h = cdfplot(cdfvalue);
[f, x] = ecdf(cdfvalue);
h = plot(x(1:DOWN_SCALE:end), f(1:DOWN_SCALE:end));
set(h, 'Color', 'y', 'LineStyle', '-', 'LineWidth', 2, 'Marker', marker(protocol_ind));
hold on

%h.legend = legend(bw_legend, 'orientation', 'horizontal');   
h.legend = columnlegend(1, legend_str);

xlabel('PDR (%)');
ylabel('CDF');
set(gca, 'FontSize', 30);

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
%
str = ['pdr_cdf_' num2str(pdr_req(i))];

if ~is_neteye
    str = [str '_indriya'];
end

export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);


end

end