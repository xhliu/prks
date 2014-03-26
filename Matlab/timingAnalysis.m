%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   5/23/2013
%   Function: timing related analysis, especially for ftsp
%%
% if 0
%% %% FTSP sync accuracy
load debugs;
% (DBG_FLAG, 255, call PacketTimeStamp.isValid(msg), 0, is_synced, 0, 0, 
% hdr->seqno, global_rx_timestamp)
t = debugs;
t = t(t(:, 3) == 255, :);
% a commander in each channel; seperate bcoz they may tx at different times
t = t(t(:, 5) == 98, :);
fprintf('%d nodes rx ftsp command\n', length(unique(t(:, 2))));

%% sync ratio
tmp = t;
% length(find(tmp(:, 4) == 0)) / length(tmp(:, 4))
% tmp = [tmp(:, 2:4) tmp(:, 15)];
% only synchronous
% [node_id, seqno, timestamp, validity, sync]
% tmp = tmp(tmp(:, 6) == 0, [2, 5, 10, 4]);
tmp = tmp(:, [2, 9, 10, 4, 6]);
[res, IX] = sort(tmp(:, 2));
tmp = tmp(IX, :);
% tmp = tmp(tmp(:, 1) ~= 71 & tmp(:, 1) ~= 72 & tmp(:, 1) ~= 29, :);
%
err = [];
sync_ratio = [];
diffs = cell(max(tmp(:, 2)), 1);
diff_idx = 1;
for i = min(tmp(:, 2)) : max(tmp(:, 2))
    sync_ratio = [sync_ratio; length(find(tmp(:, 2) == i & tmp(:, 5) == 0)) / length(find(tmp(:, 2) == i))];
    
    % sync error
    % valid timestamp & synchronous only
    res = tmp(tmp(:, 2) == i & tmp(:, 4) ~= 0 & tmp(:, 5) == 0, :);
    if isempty(res)
        continue;
    end
    % pairwise diff
    t = res(:, 3);
    s = [];
    len = length(t);
    for j = 1 : len - 1
        for k = j + 1 : len
            s = [s; abs(t(j) - t(k))];
        end
    end
    diffs{diff_idx} = s;
    diff_idx = diff_idx + 1;
    diff = max(res(:, 3)) - min(res(:, 3));
    % err = [err; sync_ratio, diff];
    err = [err; diff];
end
diffs(diff_idx : end, :) = [];
save('ftsp_err.mat', 'err', 'sync_ratio', 'diffs');
figure;

h = plot(sync_ratio(1:end) * 100);
title('sync ratio vs time');
set(h,'Color','red');
saveas(h, 'sync ratio.fig');
% converge for the 1st time
sync_idx = find(sync_ratio >= 1, 1);

%% sync err
% ignore time when root may be erased
MARGIN = 2;
%sync_idx = 2;
data = diffs(sync_idx + MARGIN: end - MARGIN, 1);
% data([227 : 232]) = [];
% data(155) = [];
group = zeros(0);
dataDisp = zeros(0);
for i = 1 : size(data, 1)
    group = [group ; ones(size(data{i})) + i - 1];
    dataDisp = [dataDisp ; data{i}];
end
figure;
boxplot(dataDisp, group, 'notch', 'on');
title('sync error vs time');

%%
set(gca, 'yscale', 'log');

%{
%% regression spikes
load debugs;
t = debugs;
t = t(t(:, 3) == 16, :);
nodes = unique(t(:, 2));
for i = 1 : length(nodes)
    % 30 & 57
    s = t(t(:, 2) == nodes(i), :);
    % offset
    plot(s(1000:end, 8));
    title(num2str(nodes(i)));
%     set(gca, 'yscale', 'log');
end


%% regression correctness check
%DBG_FLAG, DBG_FTSP_FLAG, __LINE__, skew > 0, offsetAverage > 0, localAverage, 
%offsetAverage_abs >> 16, offsetAverage_abs, skew_abs * 1000000UL
%DBG_FLAG, DBG_FTSP_FLAG, __LINE__, 0, 0, te->timeOffset > 0,
%timeOffset_abs >> 16, timeOffset_abs, te->localTime
clc;
% [localtime offset]
FTSP_TABLE_SIZE = 8;
LINE_IDX = 4;
THRESHOLD = 1000;
PPM = 1000;
diffs = [];
load debugs;
s = debugs;
MIN_LINE = 271;
MAX_LINE = 277;
syms alpha beta x;
f = sym('alpha * x / 1000000 + beta');

% one entry followed by the complete table
for i = 1 : (size(s, 1) - FTSP_TABLE_SIZE - 1)
    if s(i, LINE_IDX) == MIN_LINE
        if all(s(i + 1 : i + FTSP_TABLE_SIZE, LINE_IDX) == MAX_LINE)
            if s(i + FTSP_TABLE_SIZE + 1, LINE_IDX) == MIN_LINE
                % found
                idx = i;
                table = [];
                for j = idx + 1 : idx + FTSP_TABLE_SIZE
                    table = [table; s(j, 10), (-1) ^ (1 + s(j, 7)) * (s(j, 8) * 2 ^ 16 + s(j, 9))];
                end
                % offset = a * localtime + b;
                p = polyfit(table(:, 1), table(:, 2), 1);
                % in ppm
                a_ = p(1) * 1000000;
                b_ = p(2);
                fprintf('theory: %f, %f\n', a_, b_);
                % offset = offsetAverage + skew * (localTime - localAverage)
                a = (-1) ^ (1 + s(idx, 5)) * s(idx, 10);
                b = (-1) ^ (1 + s(idx, 6)) * (s(idx, 8) * 2 ^ 16 + s(idx, 9)) - a_ / 1000000 * s(idx, 7);
                fprintf('practice: %f, %f\n', a, b);
%                 diffs = [diffs; a_ a b_ b];
%                 if abs(b - b_) > THRESHOLD
%                     scatter(table(:, 1), table(:, 2));
%                     if abs(a) < PPM
%                         tmp = debugs(idx : idx + FTSP_TABLE_SIZE, :);                        
%                     end
%                 end
                for k = 1 : size(table, 1)
                    localtime = table(k, 1);
                    t_value = subs(f, [alpha beta x], [a_ b_ localtime]);
                    p_value = subs(f, [alpha beta x], [a b localtime]);
                    diff = [i k t_value p_value t_value - p_value];
                    diffs = [diffs; diff];
                    if abs(t_value - p_value) > THRESHOLD
                        scatter(table(:, 1), table(:, 2));
                        diff
                    end
                end
            end
        end
    end
end
save('diffs.mat', 'diffs');
% offset
%% err of offset estimation
syms alpha beta x;
f = sym('alpha * x / 1000000 + beta');
% localtime = table(end, 1);
% t_value = subs(f, [a b x], [49.214052, 78066.894275 localtime])
% p_value = subs(f, [a b x], [49.000000, 78285.464177 localtime])
% t_value - p_value
%% pkt-level sync & corruption
load debugs;
t = debugs;
t = t(t(:, 3) == 16, :);
% tx/rx
t = t(t(:, 4) == 0, :);
% is forwarder enabled
t = t(t(:, 5) ~= 0, :);
% t = t(t(:, 2) == 11, :);
figure;
hist(t(:, 2), 100)


%% link offset at receiver only
load debugs;
t = debugs;
t = t(t(:, 3) == 16, :);
t = t(t(:, 4) > 0, :);
%(DBG_FLAG, DBG_FTSP_FLAG, call SubAMPacket.source(msg), call SlotInfo.isForwarderEnabled(), 
% call TimeSyncPacket32khz.isValid(msg), call PacketTimeStamp32khz.isValid(msg), 
% rx_timestamp >> 16, rx_timestamp, getFooter(msg)->timesync.timestamp);
MIN_SAMPLE_SIZE = 10;
TIMESTAMP_OVERRIDEN = 255;
SENDER_IDX = 4;
ENABLED_IDX = 5;
VALID_IDX = 6;
OVERRIDE_IDX = 7;
% whether forwarder is enabled
t = t(t(:, ENABLED_IDX) == 0, :);
rx = t;
receivers = unique(rx(:, 2));
% each receiver
for i = 1 : length(receivers)
    receiver = receivers(i);
    r = rx(rx(:, 2) == receiver, :);
%     r = r(r(:, OVERRIDE_IDX) == TIMESTAMP_OVERRIDEN & r(:, VALID_IDX) == 1, :);
    r = r(r(:, VALID_IDX) == 1, :);
    
    senders = unique(r(:, SENDER_IDX));
    % each sender
    for j = 1 : length(senders)
        sender = senders(j);
%         link_t = tx(tx(:, 2) == sender, :);
        % reception on this link
        link_r = r(r(:, SENDER_IDX) == sender, :);
        rx_timestamp = link_r(:, 8) * (2 ^ 16) + link_r(:, 9);
        tx_timestamp = link_r(:, 10);
        link_offset = rx_timestamp - tx_timestamp;
        if length(link_offset) > MIN_SAMPLE_SIZE
            scatter(tx_timestamp, link_offset);
            str = sprintf('link <%d, %d> offset vs time %d', sender, receiver, mode(link_offset));
            title(str);
        end
    end
end
disp();


%% sanity check via local timestamping
% DBG_FLAG, DBG_FTSP_FLAG, __LINE__, call TimeSyncPacket32khz.isValid(msg), call PacketTimeStamp32khz.isValid(msg), 
% call SubAMPacket.source(msg), getFooter(msg)->seqno, getFooter(msg)->timesync.timestamp - rx_timestamp, rx_timestamp
% DBG_FLAG, DBG_FTSP_FLAG, __LINE__, 0, call PacketTimeStamp32khz.isValid(msg), call AMPacket.source(msg), 
% rcm->counter, 0, call PacketTimeStamp32khz.timestamp(msg)
SENDER_IDX = 7;
SEQ_IDX = 8;
VALID_IDX = 6;
TIMESTAMP_IDX = 10;
MIN_SAMPLE_SIZE = 100;
PPM = 0;

load debugs;
t = debugs;
t = t(t(:, 3) == 16, :);
%% valid timestamp only
t = t(t(:, VALID_IDX) ~= 0, :);
% iMAC
% tx = t(t(:, 4) == 124, :);
% rx = t(t(:, 4) == 114, :);
% raw
tx = t(t(:, 4) == 120, :);
rx = t(t(:, 4) == 110, :);
senders = unique(tx(:, 2));
receivers = unique(rx(:, 2));
skews = [];
for i = 1 : length(senders)
    sender = senders(i);
    s = tx(tx(:, 2) == sender, :);
    for j = 1 : length(receivers)
        receiver = receivers(j);
        if sender == receiver
            continue;
        end
        r = rx(rx(:, 2) == receiver & rx(:, SENDER_IDX) == sender, :);
        [C IA IB] = intersect(s(:, SEQ_IDX), r(:, SEQ_IDX));
        if length(C) < MIN_SAMPLE_SIZE
            continue;
        end
        tx_timestamp = s(IA, TIMESTAMP_IDX);
        rx_timestamp = r(IB, TIMESTAMP_IDX);
        scatter(tx_timestamp, tx_timestamp - rx_timestamp);
        p = polyfit(tx_timestamp, tx_timestamp - rx_timestamp, 1);
        p = p * 1000000;
        if abs(p(1)) > PPM
            fprintf('%f\n', p(1));
        end
        skews = [skews; p(1)];
    end
end
cdfplot(skews);

%%
t = rx;
t = t(t(:, 5) == 11, :);
t = t(t(:, 6) == 11, :);
cdfplot(t(:, 2));
t = t(t(:, 2) == 29, :);
t = t(:, 8) * 2 ^ 16 + t(:, 9) - t(:, 10);
plot(t(1:end));

%% tx vs rx
load debugs;
t = debugs;
sender = 11;
receiver = 27;
t = t(t(:, 3) == 16, :);
tx = t(t(:, 4) == 0 & t(:, 2) == sender, :);
rx = t(t(:, 4) > 0 & t(:, 2) == receiver, :);

diffs = [];
for idx = 1 : size(rx, 1)
%     idx = 6;
    rx_ = rx(idx, :);
    tx_ = tx(tx(:, 2) == rx_(4) & tx(:, 9) == rx_(9), :);
    if isempty(tx_)
        continue;
    end
%     diff = tx_(1, [5:6 9:10]) - rx_(1, [5:6 9:10]);
    if tx_(1, 5) && rx_(1, 5)
        diffs = [diffs; tx_(1, 7) * 2 ^ 16 + tx_(1, 8) - (rx_(1, 7) * 2 ^ 16 + rx_(1, 8))];
    end
%     if size(tx_, 1) > 1
%         fprintf('%d\n', idx);
%     end
%     if sum(diff ~= 0) > 0
%         fprintf('%d\n', idx);
%     end
end
plot(diffs);


%% link offset by seq #
% SENDER_IDX = 5;
% NUM_IDX = 6;
% SEQ_IDX = 7;
% LOCALTIME_IDX = 10;
% corrupt_cnt = 0;
% total = 0;
% receivers = unique(rx(:, 2));
% % each receiver
% for i = 1 : length(receivers)
%     receiver = receivers(i);
%     r = rx(rx(:, 2) == receiver, :);
%     
%     senders = unique(r(:, SENDER_IDX));
%     % each sender
%     for j = 1 : length(senders)
%         sender = senders(j);
%         link_t = tx(tx(:, 2) == sender, :);
%         % reception on this link
%         link_r = r(r(:, SENDER_IDX) == sender, :);
%         
%         link_offset = [];
%         % each seq
%         for k = 1 : size(link_r, 1)
%             seq = link_r(k, SEQ_IDX);
% %             IX = find(link_t(:, SEQ_IDX) == seq & link_t(:, NUM_IDX) == link_r(k, NUM_IDX), 1);
%             IX = find(link_t(:, SEQ_IDX) == seq, 1);
%             if ~isempty(IX)
%                 link_offset = [link_offset; link_t(IX, LOCALTIME_IDX) - link_r(k, LOCALTIME_IDX)];
%                 % rx == tx? NUM_IDX sometimes inconsistent, ignore for now
%                 % since it does not affect sync
%                 if ~all(link_r(k, [5 7:9]) == link_t(IX, [5 7:9]))
%                     corrupt_cnt = corrupt_cnt + 1;
%                     clc;
%                     link_r(k, 1:9)
%                     link_t(IX, 1:9)
%                 end
%                 total = total + 1;
%             end
%         end
%     if length(link_offset) > 200
%         plot(link_offset);
%         str = sprintf('link <%d, %d> offset %d', sender, receiver, mode(link_offset));
%         title(str);
%     end
%     end
% end
% fprintf('corruption ratio: %f, %d out of %d\n', corrupt_cnt / total,
% corrupt_cnt, total);

%% sender side bug
load debugs;
t = debugs;
t = t(t(:, 3) == 16, :);
t = t(t(:, 4) > 1, :);
% t = t(t(:, 4) ~= 62, :);
sum(t(:, 10) == 8 * 16 ^ 7) / size(t, 1)
s = t(t(:, 10) ~= 8 * 16 ^ 7, :);
s = 2 ^ 32 - s(:, [10]);
cdfplot(s)
set(gca, 'xscale', 'log')
sum(s > 10000) / size(t, 1)
sum(t(:, 4) ~= 62) / size(t, 1)

%% receiver side sanity check: packet-level timesync
% load debugs;
% t = debugs;
% t = t(t(:, 3) == 16, :);
% % t = t(t(:, 4) == 0, :);
% fprintf('rx timestamp invalid ratio: %f\n', sum(t(:, 7) == 0) / size(t, 1));
% % filter invalid ones
% t = t(t(:, 7) ~= 0, :);
% % receiver sender rx_timestamp log_timestamp
% t = [t(:, [2 6]), t(:, 8) * 65536 + t(:, 9), t(:, 10)];
% %
% nodes = unique(t(:, 1));
% for i = 1 : length(nodes)
%     s = t(t(:, 1) == nodes(i), :);
%     if sum(s(2:end, 3) <= s(1:end-1, 3)) > 0
%         disp('err1');
%     end
%     if sum(s(:, 3) >= s(:, 4)) > 0
%         disp('err2');
%     end
%     plot(s(:, 3));
%     plot(s(:, 4) - s(:, 3));
% end

%% link offset by seq #
% SENDER_IDX = 5;
% TYPE_IDX = 6;
% SEQ_IDX = 9;
% LOCALTIME_IDX = 10;
% INVALID_AGE = hex2dec('80000000');
% invalid_cnt = 0;
% err_cnt = 0;
% corrupt_cnt = 0;
% total = 0;
% 
% receivers = unique(rx(:, 2));
% % each receiver
% for i = 1 : length(receivers)
%     receiver = receivers(i);
%     r = rx(rx(:, 2) == receiver, :);
%     
%     senders = unique(r(:, SENDER_IDX));
%     % each sender
%     for j = 1 : length(senders)
%         sender = senders(j);
%         link_t = tx(tx(:, 2) == sender, :);
%         % reception on this link
%         link_r = r(r(:, SENDER_IDX) == sender, :);
%         
%         link_offset = [];
%         % each seq
%         for k = 1 : size(link_r, 1)
%             seq = link_r(k, SEQ_IDX);
% %             IX = find(link_t(:, SEQ_IDX) == seq & link_t(:, NUM_IDX) == link_r(k, NUM_IDX), 1);
%             IX = find(link_t(:, SEQ_IDX) == seq, 1);
%             if ~isempty(IX)
%                 event_age = link_r(k, LOCALTIME_IDX);
%                 if event_age ~= INVALID_AGE
%                     if link_t(IX, LOCALTIME_IDX) ~= event_age
%                         err_cnt = err_cnt + 1;
%                     else
%                         % tx_timestamp trustable
%                         rx_timestamp = link_r(k, 7) * 65536 + link_r(k, 8);
%                         if link_r(k, 6) >= 0  && rx_timestamp ~= INVALID_AGE
%                             link_offset = [link_offset; rx_timestamp - link_r(k, LOCALTIME_IDX)];
%                         end
%                     end
%                 else
%                     invalid_cnt = invalid_cnt + 1;
%                 end
%                 total = total + 1;
%             end
%         end
%     if length(link_offset) > 0
%         plot(link_offset);
%         str = sprintf('link <%d, %d> offset %d', sender, receiver, mode(link_offset));
%         title(str);
%     end
%     end
% end
% plot(link_offset);
% % error ratio exclude invalid entries
% fprintf('tx error ratio %f %d, invalid %f %d, out of %d\n', err_cnt / (total - invalid_cnt), err_cnt, invalid_cnt / total, invalid_cnt, total);
% % fprintf('corruption ratio %f\n', sum(rx(:, TYPE_IDX) ~= 62) / size(rx,

%% link offset by seq #
SENDER_IDX = 5;
TYPE_IDX = 6;
SEQ_IDX = 9;
LOCALTIME_IDX = 10;
INVALID_AGE = hex2dec('80000000');
invalid_cnt = 0;
err_cnt = 0;
corrupt_cnt = 0;
total = 0;

receivers = unique(rx(:, 2));
% each receiver
for i = 1 : length(receivers)
    receiver = receivers(i);
    r = rx(rx(:, 2) == receiver, :);
    
    senders = unique(r(:, SENDER_IDX));
    % each sender
    for j = 1 : length(senders)
        sender = senders(j);
        link_t = tx(tx(:, 2) == sender, :);
        % reception on this link
        link_r = r(r(:, SENDER_IDX) == sender, :);
        
        link_offset = [];
        % each seq
        for k = 1 : size(link_r, 1)
            seq = link_r(k, SEQ_IDX);
            
%             
%             IX = find(link_t(:, SEQ_IDX) == seq & link_t(:, NUM_IDX) == link_r(k, NUM_IDX), 1);
            IX = find(link_t(:, SEQ_IDX) == seq, 1);
            if ~isempty(IX)
                if link_r(k, TYPE_IDX) == 255
                    event_age = link_r(k, LOCALTIME_IDX);
                    if event_age ~= INVALID_AGE
                        if link_t(IX, LOCALTIME_IDX) ~= event_age
                            err_cnt = err_cnt + 1;
                        else
                            % tx_timestamp trustable
                            rx_timestamp = link_r(k, 7) * 65536 + link_r(k, 8);
                            if link_r(k, 6) >= 0  && rx_timestamp ~= INVALID_AGE
                                % rx_timestamp trustable
                                link_offset = [link_offset; rx_timestamp - link_r(k, LOCALTIME_IDX)];
                            end
                        end
                    else
                        invalid_cnt = invalid_cnt + 1;
                    end
                    total = total + 1;
                end
            end
%             end
        end
    if length(link_offset) > 80
        plot(link_offset);
        str = sprintf('link <%d, %d> offset %d', sender, receiver, mode(link_offset));
        title(str);
    end
    end
end
plot(link_offset);
% error ratio exclude invalid entries
fprintf('tx error ratio %f %d, invalid %f %d, out of %d\n', err_cnt / (total - invalid_cnt), err_cnt, invalid_cnt / total, invalid_cnt, total);
fprintf('fp ratio %f\n', sum(rx(:, TYPE_IDX) == 255 & rx(:, LOCALTIME_IDX) == INVALID_AGE) / size(rx, 1));
fprintf('fn ratio %f\n', sum(rx(:, TYPE_IDX) ~= 255 & rx(:, LOCALTIME_IDX) ~= INVALID_AGE) / size(rx, 1));

%% -----------------------------------------------------------------
% almost no tx
% -----------------------------------------------------------------
load debugs;
t = debugs;
t = t(t(:, 3) == 6, :);
% t = t(t(:, 2) == 30, :);
%
s = t((t(:, 6) == 0 & t(:, 7) == 0), :);
t = t(~(t(:, 6) == 0 & t(:, 7) == 0), :);
a = unique(t(:, 6:7), 'rows');
% cdfplot(t(:, 8));
%%
s = [bitshift(t(:, 8), 16) + t(:, 9) t(:, 10)];
t(:, 8:end) = [];
t = [t s];
%% 
SLOT_LEN = 1120;
nodes = unique(t(:, 2));
for i = 1 : length(nodes)
    node = nodes(i);
    s = t(t(:, 2) == node, :);
%     cdfplot(s(:, 7));
    s = s(:, 6);
%     s = floor(s / SLOT_LEN);
    hist(s(2:end) - s(1:end-1));
    str = sprintf('node %d', node);
    title(str);
end

%%
lo = 90;
hi = lo + 32 - 1;
p = polyfit(tx_timestamp(lo:hi), tx_timestamp(lo:hi) - rx_timestamp(lo:hi), 1);
p(1) * 1000000

%% 
load debugs;
t = debugs;
t = t(t(:, 2) == 58, :);
t = t(t(:, 10) == 133, :);

%%
load debugs;
t = debugs;
% timer
t = t(t(:, 10) == 135, :);
% alarm
% t = t(t(:, 10) == 195, :);
% t = t(t(:, 3) == 15, :);
s = t(:, 2);
[c e] = hist(s, unique(s));
ce = [e c'];
% ce = ce(ce(:, 2) < 1500, :);
% unique(t(:, 4))

%% delay
load debugs;
t = debugs;
line = 503;
t = t(t(:, 3) == 16, :);
t = t(t(:, 4) == line, :);
t = t(t(:, 10) < 15000, :);
% figure;
hold on;
cdfplot(t(:, 10));
% legend({'DATA', 'CTRL'})
% title(num2str(line));


%% %% property per node
s = t;
% is forwarder enabled?
s = s(s(:, 5) == 0, :);
%% hist(s(:, 6));
% fprintf('forward enable ratio: %f, added ratio: %f\n', size(s, 1) / size(t, 1), sum(s(:, 6)) / size(s, 1));
% [node localSeq seq new sender]
% s = s(:, [2 10 9 6 8]);
%
% in DATA channel?
% s = s(s(:, 10) == 0, :);
s = s(:, 2);
[c e] = hist(s, unique(s));
ce = [e c'];
% ce = [ce c'];
figure;
bar(ce(:, end));
% title('bootstrap');
%% not in STATE_SENDING and enough entries
% node 109 seems fishy, regard itself as root
ENTRY_SEND_LIMIT = 64;
nodes = unique(s(:, 2));
% [node normalState% notEnoughEntries%]
node_state_num = [];
for i = 1 : length(nodes)
    node = nodes(i);
    r = s(s(:, 2) == node, :);
    node_state_num = [node_state_num; node, 100 * sum(r(:, 9) == 1 | r(:, 9) == 4) / size(r, 1), 100 * sum(r(:, 8) < ENTRY_SEND_LIMIT) / size(r, 1)];
end
%%
figure;
cdfplot(node_state_num(:, 2));
%%
ce = [ce ce(:, 3) ./ ce(:, 2)]; % ce(:, 4) ./ ce(:, 3)];
figure;
cdfplot(ce(:, end));
%%
s = t;
% legitimate
s = s(s(:, 5) == 1, :);
next = s(:, 6) * 2 ^ 16 + s(:, 7);
% sync = next - 2 ^ 15 + 4000;
sync = s(:, 8) * 2 ^ 16 + s(:, 9);
now = s(:, 10);
interval = sync - now;
s = [next, now, sync, interval];
hist(s(:, end));
IX = find(s(:, end) > 100000);
%%
ix = IX(1);
r = [t(ix - 5 : ix + 5, 2) s(ix - 5 : ix + 5, :)];
plot(r(:, 4));
%%
s = t;
next = s(:, 6) * 2 ^ 16 + s(:, 7);
now = s(:, 8) * 2 ^ 16 + s(:, 9);
s = [s(:, 2) next now s(:, 10)];
IX = find(s(:, 1) == 75 & s(:, 2) == 145424384, 1);
%%
ix = IX(1);
r = s(ix - 4 : ix + 4, :);
plot(r(:, 3));
%%
s = t(:, 10);
s = s(s < 20000);
cdfplot(s);

%% ftsp state_sending
% hist(t(:, 5));
nodes = unique(t(t(:, 5) >= 6, 2));
for i = 1 : length(nodes)
    s = t(t(:, 2) == nodes(i), :);
    plot(s(:, 5));
    title(num2str(nodes(i)));
end
%% 
clc;
s = t(t(:, 2) == 9, :);
s = s(s(:, 4) == 151 | s(:, 4) == 158, :);
%
ix = find(s(:, 3) == 61, 1, 'last');
s = s(ix - 10 : ix, :);
% s = s(:, 10);
% s = s(2:end) - s(1:end-1);
% sum(s < 2^14) / size(s, 1)
% cdfplot(s)

% end
%% %% TDMA global time progress per slot
SLOT_LEN = 32 * 1024;
offsets = [];
nodes = unique(t(:, 2));
for i = 1 : length(nodes)
    node = nodes(i);
    s = t(t(:, 2) == node, :);
%     next_time = s(:, 6) * (2 ^ 16) + s(:, 7);
    interval = s(:, 6) * (2 ^ 16) + s(:, 7);
    global_time = s(:, 8) * (2 ^ 16) + s(:, 9);
    local_time = s(:, 10);
    offset = global_time - local_time;
    % plot(global_time);
%     scatter(local_time, offset);
%     p = polyfit(local_time, offset, 1);
    % s = global_time(2:end) - next_time(1:end-1);
    s = mod(global_time, SLOT_LEN);
%     s = next_time - global_time;
%     s = s(s < 25000, :);
    cdfplot(s);
%     if quantile(s, 0.98) > 10000
%         fprintf('node %d\n', node);
%     end
    offsets = [offsets; quantile(s, 0.98)];
%     fprintf('node %d: ppm %f, fire offset %f\n', node, p(1) * 10 ^ 6, quantile(s, 0.99));
end
cdfplot(offsets);

%%
s = t;
interval = s(:, 10);
slack = s(:, 8) * (2 ^ 16) + s(:, 9);
is = [interval slack];
err1 = is(is(:, 1) < is(:, 2), :);
err2 = interval(interval > 1000);
% s = s(s(:, 9) == 0, :);
% s = s(s(:, 8) == 1, 10);
% s = mod(s, SLOT_LEN);
% s = s(s > 10000, :);
% s = SLOT_LEN - s;
cdfplot(interval - slack);

%% always keep as last part
load debugs;
t = debugs;
type = 17;
t = t(t(:, 3) == type, :);
line = 212;
t = t(t(:, 4) == line, :);

%%
s = t;
s = s(s(:, 2) ~= 11, :);
%%
interval = s(:, 6) * (2 ^ 16) + s(:, 7);
global_time = s(:, 8) * (2 ^ 16) + s(:, 9);
% ix = (mod(global_time, SLOT_LEN) == 0);
% s = interval(ix, :);
cdfplot(mod(global_time, SLOT_LEN));

%}