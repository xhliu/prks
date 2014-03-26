%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   11/11/2013
%   Function: schedule using LAMA
%% %% 1) conflict matrix given
%% generate conflict_matrix
load debugs;
t = debugs;
type = DBG_TDMA_FLAG;
line = 193;
t = t(t(:, 3) == type, :);
t = t(t(:, 4) == line, :);
% (DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, i, my_ll_addr, call
% Util.getReceiver(), le->sender, le->receiver, 0)

load link_pdrs;
l = link_pdrs;
len = size(l, 1);
conflict_matrix = repmat(false, len);
% each link
for i = 1 : len
    for j = 1 : len
        % two links conflict
        if sum(t(:, 6) == l(i, 1) & t(:, 7) == l(i, 2) & t(:, 8) == l(j, 1) & t(:, 9) == l(j, 2)) > 0
            conflict_matrix(i, j) = true;
        end
    end
end

%% sanity check: symmetric conflict for static diffused ER
if sum(sum(conflict_matrix ~= conflict_matrix')) > 0
    fprintf('error: asymmetric ratio %f \n', sum(sum(conflict_matrix ~= conflict_matrix')) / sum(sum(conflict_matrix)));
else
    fprintf('symmetric\n');
end

%%
for j = 1 : 7
IS_OPT = true;
IS_RANDOM = false;
SLOT_LEN = 128;
MAX_ROUND = inf;

[schedule rounds] = LAMA(conflict_matrix, SLOT_LEN, MAX_ROUND, IS_RANDOM, IS_OPT);

%%
% figure;
% cdfplot(rounds);

%% %% 2) topology given, but conflict_matrix unknown
% IS_OPT = false;
% IS_RANDOM = false;
% ROW_CNT = 4; %7;
% COLUMN_CNT = 4; %15;
% INTERFERENCE_RANGE = 2;
% SLOT_LEN = 1000;
% MAX_ROUND = inf;
% [schedule conflict_matrix] = simpleLAMA(SLOT_LEN, MAX_ROUND, IS_RANDOM, IS_OPT, ROW_CNT, COLUMN_CNT, INTERFERENCE_RANGE);
% %% sanity check optimal LAMA
% i = 1;
% total_node_cnt = ROW_CNT * COLUMN_CNT;
% prios = mod((1 : total_node_cnt)' + i - 1, total_node_cnt);
% nodes = [];
% for i = 1 : ROW_CNT
%     nodes = [nodes; prios((i - 1) * COLUMN_CNT + 1 : i * COLUMN_CNT)'];
% end
% statistics
% conflict set size
% cdfplot(sum(conflict_matrix));

%% display
concurrency = size(length(schedule), 1);
scheduled_link = zeros(1000000, 1);
idx = 1;
for i = 1 : length(schedule)
    len = length(schedule{i});
    concurrency(i) = len;
    scheduled_link(idx : idx + len - 1) = schedule{i};
    idx = idx + len;
end
% sanity check: ahould all be equal to QUEUE_LEN for iOrder()
scheduled_link(idx : end) = [];
% hist(scheduled_link, ROW_CNT * COLUMN_CNT);
% figure;
hold on;
cdfplot(concurrency);
fprintf('concurrency %f, %f\n', median(concurrency), mean(concurrency));
end