%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   3/12/14
%   Function: display link settling time vs pdr req for variants; also pdr
%   variance after convergence
%%
% compute settling time or pdr variance
is_settling_time = true;
% is_settling_time = false;

idx = 0;
data = cell(PDR_REQ_CNT, 3);

%%
type = DBG_CONTROLLER_FLAG;
line = 1022; % based on PRKS-L & PRKS-R
LINE_OFFSET = 10;
% controller
% (DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, nb, link_pdr, link_pdr_sample,
% le->rx_er_border_idx + 1, reference_pdr, delta_i_dB)
LINK_PDR_IDX = 6;

%% PRKS
fprintf('processing PRKS\n');
job = prks_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_req = pdr_reqs(i);
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
%         if exist('link_settling_time.mat', 'file')
%             fprintf('skip processed %d-th job %d\n', j, pdr_job(j, 1));
%             tmp{i} = [tmp{i}; link_settling_time];
%             continue;
%         else
%             fprintf('processing %d-th job job %d\n', j, pdr_job(j, 1));
%         end
        
        load debugs;
        t = debugs;
%         type = DBG_CONTROLLER_FLAG;
%         line = 1022; %1019
        t = t(t(:, 3) == type, :);
%         t = t(t(:, 4) == line, :);
        t = t((t(:, 4) >= line - LINE_OFFSET) & (t(:, 4) <= line + LINE_OFFSET), :);
        load link_pdrs;
        % fprintf('warning: varying link pdr\n');
%         pdr_req = 90;
        link_settling_time = [];
        for link_id = 1 : size(link_pdrs, 1)
            fprintf('link %d\n', link_id);
        %     fprintf('warning: fixed link id\n');
        %     link_id = 75;

            s = t;
            % receiver
            s = s(s(:, 2) == link_pdrs(link_id, 2), :);
            % sender
            s = s(s(:, 5) == link_pdrs(link_id, 1), :);

            % various pdr req
            if isempty(s)
                fprintf('empty\n');
                continue;
            end

        %     % find pdr_req
        %     m = r(r(:, 2) == link_pdrs(link_id, 1), :);
        %     pdr_req = m(1, 10);

            %if ~all(pdr_req == s(:, 9))
%             if length(unique(s(:, 9))) ~= 1
%                 fprintf('error\n');
%                 return;
%             end

            % rising time; approximate settling time since pdr rarely drops after
            % rises to requirement
            ix = find(s(:, LINK_PDR_IDX) >= pdr_req, 1);            
            if is_settling_time
                link_settling_time = [link_settling_time; ix];
            else
                if ~isempty(ix)
                    % after convergence
                    samples = s(ix : end, LINK_PDR_IDX);
                    % deviation
%                     err = abs(samples - pdr_req);
%                     tmp{i} = [tmp{i}; err]; % mean(err)
                    % cov
                    tmp{i} = [tmp{i}; std(samples) / mean(samples)];
                end
            end
        end
        if is_settling_time
            tmp{i} = [tmp{i}; link_settling_time];
            save('link_settling_time.mat', 'link_settling_time');
        end
    end
end
idx = idx + 1;
data(:, idx) = tmp;

%% PRKS-R
fprintf('processing PRKS-R\n');
job = prksr_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_req = pdr_reqs(i);
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
%         if exist('link_settling_time.mat', 'file')
%             fprintf('skip processed %d-th job %d\n', j, pdr_job(j, 1));
%             tmp{i} = [tmp{i}; link_settling_time];
%             continue;
%         else
%             fprintf('processing %d-th job job %d\n', j, pdr_job(j, 1));
%         end
        
        load debugs;
        t = debugs;
%         type = DBG_CONTROLLER_FLAG;
%         line = 1022; %1019
        t = t(t(:, 3) == type, :);
        t = t(t(:, 4) == line, :);
%         t = t((t(:, 4) >= line - LINE_OFFSET) & (t(:, 4) <= line + LINE_OFFSET), :);
        load link_pdrs;
        % fprintf('warning: varying link pdr\n');
%         pdr_req = 90;
        link_settling_time = [];
        for link_id = 1 : size(link_pdrs, 1)
            fprintf('link %d\n', link_id);
        %     fprintf('warning: fixed link id\n');
        %     link_id = 75;

            s = t;
            % receiver
            s = s(s(:, 2) == link_pdrs(link_id, 2), :);
            % sender
            s = s(s(:, 5) == link_pdrs(link_id, 1), :);

            % various pdr req
            if isempty(s)
                fprintf('empty\n');
                continue;
            end

        %     % find pdr_req
        %     m = r(r(:, 2) == link_pdrs(link_id, 1), :);
        %     pdr_req = m(1, 10);

            %if ~all(pdr_req == s(:, 9))
%             if length(unique(s(:, 9))) ~= 1
%                 fprintf('error\n');
%                 return;
%             end

            % rising time; approximate settling time since pdr rarely drops after
            % rises to requirement
            ix = find(s(:, LINK_PDR_IDX) >= pdr_req, 1);            
            if is_settling_time
                link_settling_time = [link_settling_time; ix];
            else
                if ~isempty(ix)
                    % after convergence
                    samples = s(ix : end, LINK_PDR_IDX);
                    % deviation
%                     err = abs(samples - pdr_req);
%                     tmp{i} = [tmp{i}; err]; % mean(err)
                    % cov
                    tmp{i} = [tmp{i}; std(samples) / mean(samples)];
                end
            end
        end
        if is_settling_time
            tmp{i} = [tmp{i}; link_settling_time];
            save('link_settling_time.mat', 'link_settling_time');
        end
    end
end
idx = idx + 1;
data(:, idx) = tmp;



%% PRKS-L
fprintf('processing PRKS-L\n');
job = prksl_job;

len = length(job);
tmp = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_req = pdr_reqs(i);
    pdr_job = job{i};
    % each job
    for j = 1 : size(pdr_job, 1)
        job_dir = [MAIN_DIR num2str(pdr_job(j, 1))];
        cd(job_dir);
%         if exist('link_settling_time.mat', 'file')
%             fprintf('skip processed %d-th job %d\n', j, pdr_job(j, 1));
%             tmp{i} = [tmp{i}; link_settling_time];
%             continue;
%         else
%             fprintf('processing %d-th job job %d\n', j, pdr_job(j, 1));
%         end
        
        load debugs;
        t = debugs;
%         type = DBG_CONTROLLER_FLAG;
%         line = 1022; %1019
        t = t(t(:, 3) == type, :);
        t = t(t(:, 4) == line, :);
%         t = t((t(:, 4) >= line - LINE_OFFSET) & (t(:, 4) <= line + LINE_OFFSET), :);
        load link_pdrs;
        % fprintf('warning: varying link pdr\n');
%         pdr_req = 90;
        link_settling_time = [];
        for link_id = 1 : size(link_pdrs, 1)
            fprintf('link %d\n', link_id);
        %     fprintf('warning: fixed link id\n');
        %     link_id = 75;

            s = t;
            % receiver
            s = s(s(:, 2) == link_pdrs(link_id, 2), :);
            % sender
            s = s(s(:, 5) == link_pdrs(link_id, 1), :);

            % various pdr req
            if isempty(s)
                fprintf('empty\n');
                continue;
            end

        %     % find pdr_req
        %     m = r(r(:, 2) == link_pdrs(link_id, 1), :);
        %     pdr_req = m(1, 10);

            %if ~all(pdr_req == s(:, 9))
%             if length(unique(s(:, 9))) ~= 1
%                 fprintf('error\n');
%                 return;
%             end

            % rising time; approximate settling time since pdr rarely drops after
            % rises to requirement
            ix = find(s(:, LINK_PDR_IDX) >= pdr_req, 1);            
            if is_settling_time
                link_settling_time = [link_settling_time; ix];
            else
                if ~isempty(ix)
                    % after convergence
                    samples = s(ix : end, LINK_PDR_IDX);
%                     % deviation
%                     err = abs(samples - pdr_req);
%                     tmp{i} = [tmp{i}; err]; % mean(err)
                    % cov
                    tmp{i} = [tmp{i}; std(samples) / mean(samples)];
                end
            end
        end
        if is_settling_time
            tmp{i} = [tmp{i}; link_settling_time];
            save('link_settling_time.mat', 'link_settling_time');
        end
    end
end
idx = idx + 1;
data(:, idx) = tmp;


% if is_neteye
% %% iOrder
% idx = idx + 1;
% % iorder is from testiOrder
% data(:, idx) = iorder;
% end
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
if is_settling_time
    bw_ylabel = 'Number of control steps';
else
    bw_ylabel = 'Coefficient of variation of PDR';
%     bw_ylabel = 'PDR deviation from requirement (%)';
end
% bw: short for barweb
%barweb(barvalues, errors, width, groupnames, bw_title, bw_xlabel,
%bw_ylabel, bw_colormap, gridstatus, bw_legend, error_sides, legend_type)
h = barweb(barvalue, error, [], groupnames, [], bw_xlabel, bw_ylabel);
%h.legend = legend(bw_legend, 'orientation', 'horizontal');
h.legend = legend(bw_legend);

%%
set(gca, 'FontSize', 40);
ylabel(bw_ylabel);
xlabel(bw_xlabel);
grid on;

maximize;
set(gcf, 'Color', 'white');
cd(FIGURE_DIR);
% cd('~/Dropbox/iMAC/Xiaohui/signalMap/figures/');

if is_settling_time
    str = 'prks_variant_settling_time';
else
%     str = 'prks_variant_pdr_abs_deviation';
    str = 'prks_variant_pdr_cov';
end
export_fig(str, '-eps');
export_fig(str, '-jpg', '-zbuffer');
saveas(gcf, [str '.fig']);
