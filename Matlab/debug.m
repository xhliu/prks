if 0
%% DBG constants
DBG_LOSS_FLAG = 0;
DBG_TX_FLAG = DBG_LOSS_FLAG + 1;
DBG_RX_FLAG = DBG_TX_FLAG + 1;
DBG_BACKOFF_FLAG = DBG_RX_FLAG + 1;
DBG_ER_FLAG = DBG_BACKOFF_FLAG + 1;
% 5
DBG_SM_FLAG = DBG_ER_FLAG + 1;
DBG_TX_FAIL_FLAG = DBG_SM_FLAG + 1;
DBG_TIMEOUT_FLAG = DBG_TX_FAIL_FLAG + 1;
DBG_BI_ER_FLAG = DBG_TIMEOUT_FLAG + 1;
DBG_EXEC_TIME_FLAG = DBG_BI_ER_FLAG + 1;
% 10
DBG_DELAY_FLAG = DBG_EXEC_TIME_FLAG + 1;
DBG_CANCEL_FLAG = DBG_DELAY_FLAG + 1;
DBG_CONTROLLER_FLAG = DBG_CANCEL_FLAG + 1;
DBG_COUNTER_NAV_FLAG = DBG_CONTROLLER_FLAG + 1;
DBG_CALC_FLAG = DBG_COUNTER_NAV_FLAG + 1;
% 15
DBG_HEARTBEAT_FLAG = DBG_CALC_FLAG + 1;
DBG_FTSP_FLAG = DBG_HEARTBEAT_FLAG + 1;
DBG_TDMA_FLAG = DBG_FTSP_FLAG + 1;
DBG_SPI_FLAG = DBG_TDMA_FLAG + 1;
DBG_DRIVER_FLAG = DBG_SPI_FLAG + 1;
% 20
DBG_ERR_FLAG = DBG_DRIVER_FLAG + 1;
DBG_OVERFLOW_FLAG = DBG_ERR_FLAG + 1;

%% always keep first
load debugs;
t = debugs;
type = DBG_CONTROLLER_FLAG;
line = 1042;
% type = DBG_HEARTBEAT_FLAG;
% line = 172;
t = t(t(:, 3) == type, :);
t = t(t(:, 4) == line, :);

%% line # if changed
s = t;
s = s(:, 4);
cdfplot(s);
unique(s)
%%
s = t;
% sum(s(:, 9) > s(:, 10))
% s = s(s(:, 2) == 15, :);
s = s(:, 10);
% max(s) / 32
% s = mod(s, 2 ^ 17);
% s = s(s(:, 6) == s(:, 7) - 1, :);
% s = mod(s, 128);
% s = unique(s);
% cnt = sum(s == 178);
% length(s) - cnt - cnt
% s = s(:, [2 10]);
% [x ix] = sort(s(:, 10));
% s = s(ix, :);
% s(:, 10) = s(:, 10) - min(s(:, 10));
cdfplot(s);
% ix = (x >= 2 ^ 15);
% x(ix) = x(ix) - 2 ^ 16;
% plot(s);
% 16384 slots to wrap around
% s = s(s(:, 9) == 775 + 16 * 9, :);
% s = s(2 : end) - s(1 : end - 1);
% ix = find(s > 1);
% s(ix - 10 : ix + 10)
% hold on;
% length(s)
%%
load link_pdrs;
cdfplot(link_pdrs(:, end));
%%
% load ~/Dropbox/Projects/PRK/Matlab/matdata/rx_concurrency.mat
load rx_concurrency;
t = rx_concurrency;
median(t)
mean(t)
cdfplot(t);
%%
load txrxs;
t = txs;
[x ia] = unique(t(:, 2));
% s = t(ia, :);
sum(t(ia, 6))

end

%% controller
r = [];
% (DBG_FLAG, DBG_CONTROLLER_FLAG, __LINE__, nb, link_pdr, link_pdr_sample,
% le->rx_er_border_idx + 1, reference_pdr, delta_i_dB)
load link_pdrs;
% fprintf('warning: varying link pdr\n');
pdr_req = 95;
LINK_PDR_IDX = 6;
link_settling_time = [];
for link_id = 1 : size(link_pdrs, 1)
    fprintf('link %d\n', link_id);
%     fprintf('warning: fixed link id\n');
%     link_id = 44;
    
    s = t;
    % receiver
    s = s(s(:, 2) == link_pdrs(link_id, 2), :);
    % sender
    s = s(s(:, 5) == link_pdrs(link_id, 1), :);
    
    % various pdr req
    if isempty(s)
        continue;
    end
    
%     % find pdr_req
%     m = r(r(:, 2) == link_pdrs(link_id, 1), :);
%     pdr_req = m(1, 9);
    
    %if ~all(pdr_req == s(:, 9))
%     if length(unique(s(:, 9))) ~= 1
%         fprintf('error\n');
%         return;
%     end
    
%     % convert signed integer
%     x = s(:, 10);
%     ix = (x >= 2 ^ 31);
%     x(ix) = x(ix) - 2 ^ 32;
%     s(:, 10) = x / 128;

%     sinr = -25 - s(:, 9) + s(:, 10);
%     ewma = 0.9 * s(:, 6) + 0.1 * s(:, 7);
%     if s(1, 9) < 95
%         continue;
%     end
    % rising time; approximate settling time since pdr rarely drops after
    % rises to requirement
    ix = find(s(:, LINK_PDR_IDX) >= pdr_req, 1);            
    link_settling_time = [link_settling_time; ix];
    if ~isempty(ix)
        r = [r; link_id s(1, LINK_PDR_IDX) ix];
    end
    
%     plot(s(:, [6 8 9]));
    plot([s(:, 6) repmat(pdr_req, size(s, 1), 1)]);
%     plot([s(:, [6 7 8 9 10]) ewma repmat(pdr_req, size(s, 1), 1) repmat(0, size(s, 1), 1)]);
%     plot([s(:, [6 7 8]) sinr repmat(pdr_req, size(s, 1), 1)]);
%     title(['link ' num2str(link_id) ' , pdr req ' num2str(pdr_req)]);
    title(['link ' num2str(link_id)]);
    legend({'pdr'}, 'Location', 'Best');
%     legend({'pdr', 'pdr sample', 'ER size', 'pdr req', 'deltaI', 'ewma'});
end
cdfplot(link_settling_time);
% save('link_settling_time.mat', 'link_settling_time');
%% mix power level
t = [
1, 2, 3;
2, 1, 3;
3, 5, 4;
4, 7, 4;
5, 3, 4;
6, 7, 3;
8, 9, 3;
9, 8, 3;
10, 27, 3;
11, 10, 3;
13, 11, 3;
14, 13, 3;
15, 14, 3;
16, 17, 3;
17, 6, 4;
18, 16, 3;
19, 18, 3;
20, 19, 4;
21, 4, 8;
22, 20, 3;
23, 22, 4;
26, 23, 6;
27, 40, 4;
28, 26, 4;
29, 28, 3;
30, 15, 3;
31, 32, 3;
32, 31, 3;
33, 34, 3;
34, 21, 3;
35, 49, 4;
38, 39, 3;
39, 38, 3;
40, 41, 3;
41, 29, 3;
42, 43, 3;
43, 30, 3;
44, 45, 3;
45, 44, 3;
46, 47, 3;
47, 33, 3;
48, 46, 3;
49, 35, 4;
50, 66, 3;
51, 50, 3;
52, 51, 3;
53, 48, 4;
54, 53, 3;
55, 54, 3;
56, 60, 3;
57, 42, 3;
58, 57, 3;
60, 58, 3;
61, 62, 3;
62, 52, 4;
63, 77, 3;
65, 66, 3;
67, 65, 3;
68, 67, 3;
69, 55, 3;
70, 69, 6;
71, 56, 4;
72, 71, 3;
73, 72, 4;
75, 73, 5;
76, 61, 3;
77, 63, 3;
78, 79, 5;
79, 78, 4;
80, 81, 3;
81, 80, 3;
83, 68, 3;
91, 93, 3;
92, 76, 3;
93, 92, 3;
94, 95, 3;
95, 96, 3;
96, 97, 3;
99, 97, 3;
100, 99, 3;
101, 86, 3;
103, 88, 3;
104, 88, 3;
105, 85, 3;
108, 91, 3;
110, 117, 3;
111, 116, 3;
113, 107, 3;
114, 108, 3;
115, 117, 4;
116, 110, 4;
118, 119, 3;
119, 118, 3;
122, 103, 3;
123, 84, 3;
124, 70, 4;
125, 124, 3;
126, 127, 3;
127, 84, 3;
129, 126, 3;
];
len = size(t, 1);
link_tx_power_level = zeros(len, 4);
for i = 1 : len
    power_level = t(i, 3);
    power = level_power_table(power_level + 1);
    link_tx_power_level(i, :) = [t(i, 1) t(i, 2) power power_level];
end
save('~/Dropbox/iMAC/Xiaohui/link_tx_power_level.mat', 'link_tx_power_level');

%%
load txrxs;
SLOT_LEN = 32;
t = tx_successes;
s = floor(t(:, 10) / (SLOT_LEN * 1024));
[c e] = hist(s, unique(s));
ce = [e c'];
%%
% [slot, # of concurrent links, signal, noise+interference, snr]
load ~/Projects/tOR/RawData/21103/link_pdrs;
t = link_pdrs;
% load ~/Projects/tOR/RawData/20856/link_snrs;
% for i = 1 : size(link_snrs, 1)
%     t = link_snrs{i};
%     s = t(:, end);
%     plot(s);
%     fprintf('nan ratio %f\n', sum(isnan(s)) / length(s));
% end
corrcoef(t(:, 3), t(:, 5))

%%
t = link_pdrs;
len = size(t, 1);
ix = t(:, end) <= 0;
nodes = t(ix, 1);

s = [];
while length(s) < length(nodes)
    s = nodes;
    for i = 1 : len
        % parent
        if sum(s == t(i, 2)) > 0
            nodes = [nodes; t(i, 1)];
        end
    end
    nodes = unique(nodes);
end
[x ix] = setdiff(t(:, 1), nodes);
s = t(ix, :);

%% compute PRK model paramter K
% variable t stores every log
INVALID_GAIN = 255;
LINK_PATHLOSS_IDX = 9;
ER_BORDER_PATHLOSS_IDX = 10;
% ignore invalid gain
t(t(:, LINK_PATHLOSS_IDX) == INVALID_GAIN, :) = [];
t(t(:, ER_BORDER_PATHLOSS_IDX) == INVALID_GAIN, :) = [];

% K = sender signal - border signal = (TX_POWER - link_pathloss) -
% (TX_POWER - er_border_pathloss) = er_border_pathloss - link_pathloss
K = t(:, ER_BORDER_PATHLOSS_IDX) - t(:, LINK_PATHLOSS_IDX);
cdfplot(K);
hold on;


%% signalling interval
interval = [];
nodes = unique(t(:, 2));
for i = 1 : length(nodes)
    s = t;
    % only successful tx
    s = s(s(:, 9) == 0, :);
    s = s(s(:, 2) == nodes(i), :); % 1045
    s = s(:, 10);
    s = s(2 : end) - s(1 : end - 1);
    interval = [interval; s];
end
cdfplot(interval);

%% concurrency set for a given slot
s = t;
idx = 3;
s = s(s(:, 5) == idx, :);
link_pdrs(schedule{idx}, 1:2)
s = s(s(:, 7) == 0, :);
% [x ix] = sort(s(:, 9));
% s = s(ix, :);
s = s(:, 9);
[c e] = hist(s, unique(s));
ce = [e c'];
cdfplot(ce(:, end));
%% 
s = link_pdrs;
% s = t;
% s = s(s(:, 8) == 0, :);
% IX = find(s(:, 4) == 782);
% offset = [];
% for i = 1 : length(IX)
%     ix = IX(i);
% %     s = s(ix - 10 : ix + 10, :);
%     offset = [offset; s(ix + 1, 4) s(ix + 1, 10) - s(ix, 10) s(ix + 1, 11) - s(ix, 11)];
% end
s = s(s(:, 3) > 1200, :);
s = s(:, 5);
% s = mod(s, 32);
% s = (s - 357) / 5  + 11;
% bits = 32;
% r = s;
% neg_ix = r >= 2 ^(bits - 1);
% all_ = repmat(2 ^ bits, size(r, 1), 1);
% s = r - all_ .* neg_ix;
% s = s(s < -100000);
cdfplot(s);
mean(s)
% hist(s, 100);

%% %% contention inconsistency: sender receiver out of sync
% (DBG_FLAG, DBG_TDMA_FLAG, __LINE__, getLinkSetSize(), call
% RadioState.getChannel(), status, m_data_addr, current_slot, next_slot_by_tx - current_slot)
% [node, nb, status, channel, slot]
t = [t(:, [2 8 7 6 9])];
% timewrap around
SLOT_LEN = 32;
fprintf('slot length %d\n', SLOT_LEN);
SLOT_WRAP_LEN = 2 ^ 32 / (SLOT_LEN * 2 ^ 10);
MIN_GAP = SLOT_WRAP_LEN / 2;
nodes = unique(t(:, 1));
for j = 1 : length(nodes)
    ix = t(:, 1) == nodes(j);
    s = t(ix, 5);
    % sanity check if time is increasing
%     plot(s);
    
    % wrap around points; add MIN_GAP bcoz sometimes a entry is larger than
    % its next entry somehow
    %IX = find(s(1:end-1) > s(2:end));
    IX = find(s(1:end-1) > (s(2:end) + MIN_GAP));
    len = length(IX);
    for i = 1 : len
        if i < len
            s(IX(i) + 1 : IX(i + 1)) = s(IX(i) + 1 : IX(i + 1)) + i * SLOT_WRAP_LEN;
        else
            s(IX(i) + 1 : end) = s(IX(i) + 1 : end) + i * SLOT_WRAP_LEN;
        end
    end
%     plot(s);
    t(ix, 5) = s; 
end

% per link
s = t;
nodes = unique(s(s(:, 3) == 0, 1));
%[sender receiver async_cnt/total rx_tx_cnt/async_cnt miss_cnt/total total]
results = zeros(length(nodes), 6);
idx = 1;
miss_slots = cell(length(nodes), 1);
tmp = [];
% each sender
for i = 1 : length(nodes)
    node = nodes(i);
    
    % assert
    node_s = s(s(:, 1) == node & s(:, 3) == 0, :);
    receivers = unique(node_s(:, 2));
    receivers = receivers(receivers > 0);
    if size(receivers, 1) ~= 1
        disp('err');
        break;
    end
    % only 1 receiver for each sender
    nb = receivers(1);
    nb_s = s(s(:, 1) == nb, :);   
    
    miss_cnt = 0;
    sync_cnt = 0;
    async_cnt = 0;
    rx_tx_cnt = 0;
    wrong_channel_cnt = 0;
    miss_slot = [];
    len = size(node_s, 1);
    % each tx
    for j = 1 : len
        slot = node_s(j, 5);
        IX = find(nb_s(:, 5) == slot, 1);
        if isempty(IX)
            miss_cnt = miss_cnt + 1;
            miss_slot = [miss_slot; slot];
            continue;
        end
        if nb_s(IX, 3) == 1 % && nb_s(IX, 2) == node
            sync_cnt = sync_cnt + 1;
        else
            async_cnt = async_cnt + 1;
            % receiver also decides to tx
            if nb_s(IX, 3) == 0
                fprintf('link <%d, %d> and <%d, %d> both tx @ slot %d\n', node_s(j, 1), node_s(j, 2), nb_s(IX, 1), nb_s(IX, 2), nb_s(IX, 5));
                rx_tx_cnt = rx_tx_cnt + 1;
            else
                fprintf('link <%d, %d> active but %d in ctrl @ slot %d\n', node_s(j, 1), node_s(j, 2), nb_s(IX, 1), nb_s(IX, 5));
                % receiver in ctrl channel b_s(IX, 3) == 2
                wrong_channel_cnt = wrong_channel_cnt + 1;
            end
        end
    end
    miss_slots{i} = miss_slot;
    results(idx, :) = [node nb async_cnt/len wrong_channel_cnt/async_cnt miss_cnt/len len];
    idx = idx + 1;
end
results(idx : end, :) = [];
%% [node nb async_cnt/len wrong_channel_cnt/async_cnt miss_cnt/len len]
cdfplot(results(:, 3));
sum(results(:, 3) > 0)
% s = results(:, 4);
% s = s(~isnan(s));
% figure;
% cdfplot(s);
title('async ratio');
save('results.mat', 'results');


%% controller
load link_pdrs;
pdr_req = 95;
pdr = [];
for link_id = 1 : size(link_pdrs, 1)
%     fprintf('warning\n');
%     link_id = 6;
    s = t;
    % receiver
    s = s(s(:, 2) == link_pdrs(link_id, 2), :);
    % sender
    s = s(s(:, 5) == link_pdrs(link_id, 1), :);
    
    % various pdr req
    if isempty(s)
        continue;
    end
%     pdr_req = s(1, 10);
    
    % convert signed integer
    x = s(:, 9);
    ix = (x >= 2 ^ 15);
    x(ix) = x(ix) - 2 ^ 16;
    s(:, 9) = x;
    
    % [link_pdr_ewma, link_pdr_sample, er_zie, ER boundary path loss]
    %s = [s(:, [6:7]) er_size s(:, [9 10])];
    plot([s(:, [6 8 10]) repmat(pdr_req, size(s, 1), 1)]);
    title(['link ' num2str(link_id)]);
%     legend({'pdr'});
    legend({'pdr', 'ER size', 'NI'}, 'Location', 'Best');
    %legend({'pdr EWMA', 'pdr sample'}, 'Location', 'Best');
end
%%
% figure;
hold on;
cdfplot(snrs);
%%
%legend({'pdr req 30', 'pdr req 90', 'pdr req 90 aggressive'});
legend({'pdr req 30', 'pdr req 90'});
%% for other protocols
% START_DATA_TIME
% NetEye: prks 900, csma/rtscts 30, cmac 400
% Kansei: prks 1000, csma/rtscts 30, cmac 400
% Indriya: prks 1000, csma/rtscts 30, cmac 400
START_DATA_TIME = 30;
load link_pdrs;
job_duration_in_mins = 120;
bootstrap_in_mins = START_DATA_TIME / 60;
MINS = job_duration_in_mins - bootstrap_in_mins;
SLOT_LEN = 32;
sum(link_pdrs(:, 4)) / (MINS * 60 * 1024 / SLOT_LEN)
%%
sum(link_pdrs(:, 4)) / (3600 - 1627) * 32 / 5

%% concurrency for each link
s = t(t(:, 8) == 0, :);
% each sender has at most 1 receiver
nodes = unique(s(:, 2));
len = length(nodes);
link_concurrency = zeros(len, 1);
for i = 1 : len
    r = s(s(:, 2) == nodes(i), 10);
    [C ia ib] = intersect(r, ce(:, 1));
    link_concurrency(i) = mean(ce(ib, 2));
    figure; cdfplot(ce(ib, 2));
end

%% display
% NUM_OF_PDRS = 3;
% concurrencys = cell(NUM_OF_PDRS, 1);

% concurrencys{1} = ce(:, end);

data = concurrencys;
% figure;
% hold on;
% for i = 1 : size(data)
%     cdfplot(data{i});
% end
% legend({'50', '70', '90'});


%% hypothesis test
ALPHA = 0.05;
samples = s(:, 2) / 100;
MU = 0.9;
% z = (mean(samples) - MU) / (std(samples) / sqrt(length(samples)));
% p = normcdf(z, 0, 1)
t = samples;
dev =  norminv(1 - ALPHA, 0, 1) * std(t) / sqrt(size(t, 1));
fprintf('[%.4f, %.4f]\n', mean(t), mean(t) + dev);


%% delta_i calculation precision loss
pdr_slope_table = [8405, 193, 114, 85, 70, 61, 54, 49, 45, 42, 40, 38, 36, 35, 34, 33, 31, 31, 30, 29, 29, 28, 28, 28, 27, 27, 27, 26, 26, 26, 26, 26, 26, 26, 26, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 28, 28, 28, 29, 29, 29, 30, 30, 31, 31, 32, 33, 33, 34, 35, 35, 36, 37, 38, 39, 40, 41, 43, 44, 46, 47, 49, 51, 53, 56, 58, 61, 65, 69, 73, 78, 84, 92, 100, 111, 125, 141, 164, 196, 243, 323, 6197, 12332, 34296];
pdr_inv_table = [0, 36, 51, 61, 69, 75, 81, 86, 91, 95, 99, 103, 107, 111, 114, 117, 121, 124, 127, 130, 133, 136, 138, 141, 144, 147, 149, 152, 155, 157, 160, 162, 165, 167, 170, 173, 175, 178, 180, 183, 185, 188, 190, 193, 196, 198, 201, 203, 206, 209, 211, 214, 217, 220, 223, 225, 228, 231, 234, 237, 240, 243, 246, 250, 253, 256, 260, 263, 267, 270, 274, 278, 282, 286, 290, 294, 299, 303, 308, 313, 318, 324, 329, 335, 342, 348, 355, 363, 371, 380, 389, 400, 412, 425, 440, 458, 479, 507, 753, 1609, 2871];
c = 0.9;
T = 0.95;
E0 = 0.04;
PDR_EWMA_IDX = 6;
PDR_SAMPLE_IDX = 7;
% dB
DELTA_I_UPPER_BOUND = 10;

diff = [];
% signed
x = t(:, 9);
ix = (x >= 2 ^ 15);
x(ix) = x(ix) - 2 ^ 16;
t(:, 9) = x;

for line = 1 : size(t, 1)
%     if t(line, 9) <= 2 ^ 15
%         is_neg = false;
%     else
%         is_neg = true;
%         continue;
%     end


pdr = t(line, PDR_EWMA_IDX) / 100;
pdr_sample = t(line, PDR_SAMPLE_IDX) / 100;

if abs(pdr - T) > E0
    slope = (pdr_inv_table(round(100 * pdr + 1)) - pdr_inv_table(round(100 * T + 1))) / (pdr - T) / 100;
else
    slope = pdr_slope_table(round(100 * pdr + 1)) / 10;
end
nominator = (c * pdr + (1 - c) * pdr_sample - T);
delta_i = nominator * slope / (1 - c);
%fprintf('slope: %f vs %f, nominator: %f vs %f, delta_i: %f vs %f\n', slope, t(line, 8), nominator, (t(line, 9)) / 128, delta_i, (t(line, 10)) / 128); % - 2 ^ 32
% clc;
% t(line, :)
% [slope, t(line, 7) / 2 ^ 10]
% % [nominator, (t(line, 8) - is_neg * 2 ^ 16) / 100, (t(line, 9) - is_neg * 2 ^ 16) / 128]
% online_delta_i = (t(line, 9) - is_neg * 2 ^ 16) / 128;
online_delta_i = t(line, 9);
diff = [diff; delta_i online_delta_i];
end
%
s = diff;
s = s(s(:, 2) ~= DELTA_I_UPPER_BOUND, :);
cdfplot((s(:, 2) - s(:, 1))); % ./ abs(s(:, 1)));

%% pdr display
% NUM_OF_PDRS = 3;
% pdrs = cell(NUM_OF_PDRS, 1);

pdrs{3} = pdr;
% 
data = pdrs;
% legend({'50', '70', '90'});



%% aggregate
% s = t;
% miss_cnt = 0;
% sync_cnt = 0;
% async_cnt = 0;
% 
% tx_win_ixs = find(s(:, 3) == 1);
% len = size(tx_win_ixs, 1);
% for i = 1 : len
%     idx = tx_win_ixs(i);
%     node = s(idx, 1);
%     nb = s(idx, 2);
%     slot = s(idx, 5);
%     
%     % find the receiver at the slot
%     IX = find(s(:, 1) == nb & s(:, 5) == slot, 1);
%     if isempty(IX)
%         miss_cnt = miss_cnt + 1;
%         continue;
%     end
%     if s(IX, 4) == 1 && s(IX, 2) == node
%         sync_cnt = sync_cnt + 1;
%     else
%         async_cnt = async_cnt + 1;
%     end
% end
% miss_cnt / len
% sync_cnt / len
% async_cnt / len
% len

%% %% channel distribution per node
s = t;
% assert
unique(s(:, 8))
unique(s(s(:, 8) == 2, 5))
unique(s(s(:, 8) ~= 2, 5))
% [ node total ratio0 ratio1 ratio2]
nodes = unique(s(:, 2));
node_channel = [];
for i = 1 : length(nodes)
    node = nodes(i);
    r = s(s(:, 2) == node, 8);
    total = length(r);
    node_channel = [node_channel; node total sum(r == 0) / total sum(r == 1) / total sum(r == 2) / total];
end
%save('node_channel.mat', 'node_channel');
%%
THRESHOLD = 0.3;
s = node_channel;
cdfplot(s(:, 5));
no_ctrl_nodes = s(s(:, 5) < THRESHOLD, 1);
save('node_channel.mat', 'node_channel', 'no_ctrl_nodes');
%% channel time series for a node
s = t;
s = s(s(:, 2) == 42, 5);
plot(s);

%% concurrency time series for a sender
s = t;
% tx
s = s(s(:, 8) == 0, :);
% tagged sender
r = s(s(:, 2) == link_pdrs(1, 1), :);
slots = r(:, 10);
concu = [];
for i = 1 : length(slots)
    concu = [concu; sum(s(:, 10) == slots(i))];
end
plot(concu);

%% ER not sync per receiver
s = t;
% assert
ver_diff = s(:, 3) - s(:, 10);
% 0 / 1
unique(ver_diff)

% [node total sync_ratio]
nodes = unique(s(:, 2));
node_sync_er = [];
for i = 1 : length(nodes)
    node = nodes(i);
    r = s(s(:, 2) == node, :);
    total = length(r);
    node_sync_er = [node_sync_er; node total sum(r(:, 3) == r(:, 10)) / total];
end

%% link-level ER version discrepancy ratio per receiver
s = t;
nodes = unique(link_pdrs(:, 2));
MATCH_IDX = 4;
MISMATCH_IDX = 10;
% [node total async_ratio]
node_sync_er = [];
for i = 1 : length(nodes)
    node = nodes(i);
    r = s(find(s(:, 2) == node, 1, 'last'), :);
    if isempty(r)
        continue;
    end
    total = r(MATCH_IDX) + r(MISMATCH_IDX);
    node_sync_er = [node_sync_er; node total r(:, MISMATCH_IDX) / total];
end
cdfplot(node_sync_er(:, end));

%% tx success ratio per node
s = t;
% assert
unique(s(:, 9))
% [ node total ratio]
nodes = unique(s(:, 2));
node_tx_succ = [];
for i = 1 : length(nodes)
    node = nodes(i);
    r = s(s(:, 2) == node, 9);
    total = length(r);
    node_tx_succ = [node_tx_succ; node total sum(r == 0) / total];
end

%% %% compute link pdrs and signal map
[link_pdrs signal_map] = linkPdr(txs, rxs, 127);
save('link_pdrs.mat', 'link_pdrs');
save('signal_map.mat', 'signal_map');
%% compute link pdrs
[link_pdrs] = linkPdr(txs, rxs, 127);
%%
t = rxs;
t = t(t(:, 4) == 1, :);
t = t(t(:, 2) == 105, :);
plot(t(:, 3) - 256)

%% compare pdr against groundtruth
% groundtruth = link_pdrs;
t = link_pdrs;
len = size(t, 1);
ground = zeros(len, 1);
for i = 1 : len
    ground(i) = groundtruth(t(i, 1), t(i, 2));
end
t = [t, ground];

%%
t = link;
for i = 1 : size(t, 1)
    if mod(t(i, 1) - 1, 15) >= 10 && mod(t(i, 2) - 1, 15) >= 10
        fprintf('{%d, %d}, ', t(i, 1), t(i, 2));
    end
end


%% dbm & mW
dbm1 = - 10496 / 128;
dbm2 = - 10556 / 128;
delta = 10 ^ (dbm1 / 10) - 10 ^ (dbm2 / 10);
delta_dbm = 10 * log10(delta)



%% integrity check
% (DBG_FLAG, DBG_ER_FLAG, __LINE__, from, lp->sender, lp->receiver, lp->rx_interference_threshold, lp->rx_er_version, seqno_)
TX_LINE = 212;
RX_LINE = 329;
SENDER_IDX = 5;
SEQ_IDX = 10;
load debugs;
t = debugs;
type = DBG_ER_FLAG;
t = t(t(:, 3) == type, :);
tx = t(t(:, 4) == TX_LINE, :);
rx = t(t(:, 4) == RX_LINE, :);
%rx = rx(1:100000, :);

miss = 0;
err = 0;
% each rx
for i = 1 : size(rx, 1)
    s = tx(tx(:, 2) == rx(i, SENDER_IDX) & tx(:, SEQ_IDX) == rx(i, SEQ_IDX), :);
    if isempty(s)
        miss = miss + 1;
        fprintf('miss %d\n', i);
        continue;
    end
    r = rx(i - 3 : i + 3, :);
    if ~all(rx(i, [6:9]) == s(1, [6:9]))
        err = err + 1;
        fprintf('err @ %d\n', i);
        rx(i, [6:10])
        s(1, [6:10])
    else
        fprintf('integral %d\n', i);
    end
end
fprintf('err %f, miss %f\n', err / size(rx, 1), miss / size(rx, 1));
disp('');


%% path loss per link
ALPHA = 0.1;
INVALID_GAIN = 255;
s = t;
% [ node total ratio]
nodes = unique(s(:, 2));
for i = 1 : length(nodes)
    node = nodes(i);
    r = s(s(:, 2) == node, :);
    
    senders = unique(r(:, 5));
    for j = 1 : length(senders)
        sender = senders(j);
        r_s = r(r(:, 5) == sender, :);
        figure;
        gain = r_s(:, [10]);
        gain = gain(gain ~= INVALID_GAIN);
        plot(gain);
        dev = norminv(1 - ALPHA / 2, 0, 1) * std(gain) / sqrt(size(gain, 1));
        [mean(gain) - dev, mean(gain), mean(gain) + dev]
%         legend({'Attenuation', 'Channel'}, 'Location', 'best');
        title([num2str(sender) ', ' num2str(node) ' channel ' num2str(r_s(1, 6))]);
    end
end

%%
t = link_pdrs(:, 1:2);
last_len = inf;
while size(t, 1) > 0 && size(t, 1) < last_len
    last_len = size(t, 1);
    tx = t(:, 1);
    rx = t(:, 2);
    ix = [];
    for i = 1 : last_len
        if sum(tx == rx(i)) == 0 || sum(rx == rx(i)) > 2
            ix = [ix; i];
        end
    end
    t(ix, :) = [];
end
%%
for i = 1 : size(t, 1)
%     if sum(t(:, 1) == t(i, 2)) == 0
%         fprintf('%d\n', sum(t(:, 1) == t(i, 2)));
%     end
    fprintf('{%d, %d}, ', t(i, 1), t(i, 2));
end


%%
t = tmp;
seqnos = unique(t(:, 2));
x = 280;
t = t(t(:, 2) == seqnos(x + sync_idx + MARGIN - 1), :);
t(:, 3) = t(:, 3) - min(t(:, 3));

%% 
SLOT_LEN = 128;
s = mod(t(:, 10), SLOT_LEN);
cdfplot(s);

%%
er_size = t(:, 8);
ix = (er_size >= 2 ^ 15);
er_size(ix) = er_size(ix) - 2 ^ 16;
er_size = er_size + 1;
hold on;
cdfplot(er_size);


%% e2e reliability
node = 22;
sum(rxs(:, 3) == node)
sum(txs(:, 2) == node)
sum(rxs(:, 3) == node) / sum(txs(:, 2) == node)

%%
t = link_pdrs;
parents = unique(t(:, 2))
leaves = setdiff(t(:, 1), parents)

%% link set statistics
t = unique(tx_successes(:, [2 3]), 'rows');
t = [t(:, 1); t(:, 2);];
[c e] = hist(t, unique(t));
fprintf('max degree %d, unique node number %d\n', max(c), length(e));

%% tx fail cause
node = 14;
t = tx_successes;
u1 = t(t(:, 2) == node, 4);
t = txs;
u2 = t(t(:, 2) == node, 4);
t = tx_done_fails;
u3 = t(t(:, 2) == node, 4);

%%
t = link_pdrs;
t = t(t(:, 4) > 0, :);
cdfplot(t(:,end));

%% aggregate pdr
load link_pdrs;
t = link_pdrs;
sum(t(:, 4)) / sum(t(:, 3))

%%
t = txs; %_successes;
% seq = 551;    % 17441 17553
% t = t(t(:, 2) == 1 & t(:, 4) == seq, 10)
%plot(t(2 : end) - t(1 : end - 1));
t = t(t(:, 2) == 42, 10);
plot(t);

%% SCREAM sanity check
% is_any_tx_fail should be TRUE if any tx fails, i.e., no ack
% data
% scream
INTERFERENCE_DIAMETER = 3;
miss_cnt = 0;
for i = 1 : size(data, 1)
    % decision (INTERFERENCE_DIAMETER + 1) slots later
    slot_seq = data(i, 10) + INTERFERENCE_DIAMETER + 1;
    if ~sum(scream(:, 10) == slot_seq)
        miss_cnt = miss_cnt + 1;
    end
end
fprintf('scream false nagative %f\n', miss_cnt / size(data, 1));

miss_cnt = 0;
for i = 1 : size(scream, 1)
    % decision (INTERFERENCE_DIAMETER + 1) slots later
    slot_seq = scream(i, 10) - INTERFERENCE_DIAMETER - 1;
    if ~sum(data(:, 10) == slot_seq)
        miss_cnt = miss_cnt + 1;
    end
end
fprintf('scream false positive %f\n', miss_cnt / size(scream, 1));

%% type 3
MIN_PDR = 0.05;
load link_pdrs;
t = link_pdrs;
% filter out weak link even w/o interference
t = t(t(:, 5) > MIN_PDR, :);
hold on;
cdfplot(t(:, 5));
fprintf('total pdr %f\n', sum(t(:, 4)) / sum(t(:, 3)));

%%
INITIAL_LE_PKT_CNT = 1500;
SRC_IDX = 3;
DST_IDX = 3;
SEQ_IDX = 4;
load txrxs;
txs = txs(txs(:, SEQ_IDX) >= INITIAL_LE_PKT_CNT, :);
rxs = rxs(rxs(:, SEQ_IDX) >= INITIAL_LE_PKT_CNT, :);

link_pdrs = [];
srcs = unique(txs(:, 2));

for i = 1 : size(srcs, 1)
   src = srcs(i);
   
   tx = txs(txs(:, 2) == src, :);
   dst = tx(1, DST_IDX);
   
   rx = rxs(rxs(:, 2) == dst & rxs(:, SRC_IDX) == src, :);
   
   link_pdrs = [link_pdrs; src dst size(tx, 1) size(rx, 1) size(rx, 1) / size(tx, 1)];
end
hold on;
cdfplot(link_pdrs(:, end));
%%
% legend({'20', '50', '100', '200', '500', '1000', '2000'})
legend({'70', '80', '90', '95'});

%%
% data = cell(2);
data{2} = t;
%%
group = zeros(0);
dataDisp = zeros(0);
for i = 1 : size(data, 1)
    group = [group ; ones(size(data{i})) + i - 1];
    dataDisp = [dataDisp ; data{i}];
end
figure;
boxplot(dataDisp, group, 'notch', 'on');


%%
t = tx_rx_snr_pdr;
scatter(t(:, 3), t(:, 4));
hold on;
x = -5 : 0.1 : 30;
plot(x, 90);

%%
str = cell(0);
for i = 1 : 8
    str{i} = num2str(i);
end
% str{10} = 'PRKS';
legend(str);
%% sanity check concurrency
len = length(concurrency);
hold on;
for i = 1 : 2 : 5
    start_idx = ceil(len * (1 - 1 / i)) + 1;
    fprintf('start at %d\n', start_idx);
    cdfplot(concurrency(start_idx : end));
end

%%
[n, xout] = hist(s, unique(s)); 
bar(xout, 100 * n / sum(n));

%%
% s = [12 29 13 14 14 15 26 28 28 13 30 15 42 11 43 44 57 42 58 11 60 45 71 72 72 71 73 44];
s = [links(:, 1); links(:, 2)]';
[n, x] = hist(s, unique(s));
xn = [x' n'];

%% set PRKS link pdr using other protocols', e.g., SCREAM
clc;
DEFAULT_PDR = 10;
MIN_PDR = 10;
load link_pdrs;
t = link_pdrs;
cdfplot(t(:, end));
% for i = 1 : size(t, 1)
%     pdr_req = round(t(i, end) * 100);
%     % lower bound 
%     if pdr_req < 10
%         pdr_req = 10;
%     end
% 	fprintf('{%d, %d}, ', t(i, 1), pdr_req);
% end
x = [];
MAX_NODE_ID = 130;
for i = 1 : MAX_NODE_ID
    pdr_req = t(t(:, 1) == i, end);
    pdr_req = round(pdr_req * 100);
    
    if isempty(pdr_req)
        pdr_req = DEFAULT_PDR;
    else
        if pdr_req < MIN_PDR
            pdr_req = MIN_PDR;
        end
    end
	fprintf('%d, ', pdr_req);
    x = [x; pdr_req];
end
fprintf('\n');
%%
y = [98, 97, 54, 26, 84, 56, 10, 98, 99, 77, 92, 10, 74, 99, 98, 94, 26, 78, 97, 94, 10, 98, 58, 10, 10, 54, 74, 24, 100, 99, 98, 97, 46, 98, 10, 10, 10, 72, 86, 78, 57, 86, 98, 92, 99, 92, 69, 70, 69, 98, 98, 92, 75, 99, 62, 10, 71, 89, 10, 10, 77, 16, 97, 10, 100, 10, 99, 81, 92, 10, 65, 97, 10, 10, 38, 91, 98, 29, 89, 98, 99, 10, 97, 10, 10, 10, 10, 10, 10, 10, 81, 82, 96, 92, 98, 74, 10, 10, 59, 94, 70, 10, 99, 65, 35, 10, 10, 80, 10, 87, 96, 10, 90, 95, 45, 79, 10, 100, 98, 10, 10, 85, 69, 33, 99, 98, 47, 10, 89, 10];
%%
pdr = [];
for snr = -10 : 10
    pdr = [pdr; snr2Ber(snr)];
end
plot(pdr);
% set(gca, 'yscale', 'log');

%%
legend({'Indriya', 'NetEye'});
%% 
jobs = [20878 20879];
for job_id = 1 : length(jobs)
    fprintf('processing job %d\n', jobs(job_id));
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    cd(job_dir);
end

%% 
t = data{1, 1};
t = t(t > 0);
t = t(t < 700);
[n, xout] = hist(t); 
bar(xout, 100 * n / sum(n));


%% # of retx
load link_seq_tx_cnt_latency;
t = link_seq_tx_cnt_latency;
t = t(:, end - 1);
% t = t(t < 250);
hold on;
cdfplot(t);
%%
legend({'PRKS70', 'PRKS80', 'PRKS90', 'PRKS95', 'SCREAM'});

%%
cd ~/Projects/tOR/RawData/21013;
load txrxs;
load debugs;
cdfplot(uart_relis(:, end));

%% 
load link_seq_tx_cnt_latency;
% [sender receiver seqno tx_attempt_cnt tx_timestamp rx_timestamp latency]
t = link_seq_tx_cnt_latency;
% after convergence
t = t(t(:, 5) >= BOOTSTRAP_TIME, :);

% each link, i.e., each sender
senders = unique(t(:, 1));
for k = 1 : length(senders)
    sender = senders(k);
    s = t(t(:, 1) == sender, :);
    
    max_tx_cnt = quantile(s(:, 4), pdr_reqs(i) / 100);
    s = s(s(:, 4) <= max_tx_cnt, end);
end

% max # of transmissions, including retx
max_tx_cnt = quantile(t(:, 4), pdr_reqs(i) / 100);
t = t(t(:, 4) <= max_tx_cnt, end);

%%
s = -25 - t(:, 9) + t(:, 10);
cdfplot(s);

%%
plot(snr, pdr);
grid on;
set(gca, 'xtick', -5 : 20);
%%
snr_pdr = [
    2 40;
    3 50;
    4 60;
    5 70;
    7 80;
    9 90;
    10 95;
    ];
hold on;
plot(snr_pdr(:, 1), snr_pdr(:, 2) / 100);

%%
% t = setting_time_pdr90_neteye;
% t = link_settling_time;
t = data;
for i = 1 : size(t, 1)
    cdfplot(t{i, 3});
    hold on;
end
%
legend({'PRKS 70', 'PRKS 80', 'PRKS 90', 'PRKS 95', 'PRKS-L 70', 'PRKS-L 80', 'PRKS-L 90', 'PRKS-L 95'});
%%
% links1 = links;
% load link_pdr_rssi;
t = intersect(links(:, 1:2), links1(:, 1:2), 'rows');
%%
load link_pdrs;
load ~/Dropbox/iMAC/Xiaohui/links_pdr99.mat;
[c ia] = setdiff(links(:, 1), link_pdrs(:, 1));
%%
% load link_pdr_rssi;
% t = link_pdrs;
% t = connectivity;
% sum(sum(t))
t = link_rssi;
s = t(~isnan(t));
sum(s < -10^9) / length(s)
s = s(s > -10^9);
cdfplot(s);
%%
load concurrency.mat;
fprintf('concurrency median %f, mean %f\n', median(concurrency), mean(concurrency));
% figure;
plot(concurrency);
%%
load txrxs.mat;
t = tx_fails;
t = t(t(:, 2) == 18, :);
% cdfplot(t(:, 7));
% length(unique(t(:, [3 4]), 'rows'))
% length(unique(t(:, [4 5]), 'rows'))

%%
clc;
x = 0.01 : 0.01 : 1;
y = 1280 * log10(x);
scatter(x, y);
for x = 0.01 : 0.01 : 1
    fprintf('%.0f, ', -1280 * log10(x));
end
fprintf('\n');

%% 20849 21103
% cd('/home/xiaohui/Projects/tOR/RawData/22313');
% load link_pdrs;
load txrxs;
s = unique(tx_successes(:, 2));
t = unique(txs(:, 2));
setdiff(s, t)
t = tx_done_fails;
% cdfplot(t(t(:, 2) == 82, 9));
sum(t(:, 2) == 82)
% link_pdrs(50, :) = [];
% t = link_pdrs;
% load schedule_unique_concurrent_set;
% load rx_concurrency.mat;
% t = ucs_freq;
% t = t(t < 50);
% cdfplot(t);
% cdfplot(t(:, end) - link_pdrs(:, end));
% fprintf('%f %f\n', mean(t), median(t));

%%
load e2e_link_pdrs;
load link_seq_tx_cnt_retx_latency;
load link_seq_latency;
% t = link_seq_latency;
t = link_seq_tx_cnt_retx_latency * 5 / 512 / 1000;
% t = e2e_link_pdrs;
t = t(:, end);
% hold on;
cdfplot(t);
%%
clc;
hold off;
M = 3;
N = 6;
x = rand(M, N);
lo = rand(M, N) * 0.5;
hi = rand(M, N) * 0.5;
bar(1:M, x);
% hold on;
% errorbar(repmat((1:M)', 1, N), x, lo, hi, 'rx');
% barerrorbar({1:M, x}, {repmat((1:M)', 1, N), x, lo, hi, 'rx'});

%%
% barerrorbar({1:6, barvalue'}, {1:6, barvalue', lo_err', hi_err', 'rx'});
% legend(bw_legend);
hold on;
errorbar(1:6, barvalue, lo_err, hi_err, 'rx');
%%
data = 1:10;
 eH = rand(10,1);
 eL = rand(10,1);

 figure;
 hold all;
 bar(1:10, data)
 errorbar(1:10, data, eL, eH, '.')

%%
load rx_concurrency;
t = rx_concurrency;
% t = t(end - 10000 : end);
plot(t);
mean(t)

%%
load schedule_unique_concurrent_set;
t = ucs(1:end, 1);
s = [];
for i = 1 : size(t, 1)
    s = [s; length(t{i})];
end
max(s)

%%
load link_pdrs;
% cdfplot(link_pdrs(:, end));
%
JOB_DURATION_MINS = 240;
BOOTSTRAP_SECONDS = 500;
SCALE = 1024 / 512;
thruput = sum(link_pdrs(:, 4)) / ((JOB_DURATION_MINS * 60 - BOOTSTRAP_SECONDS) * SCALE);
fprintf('throughput %f\n', thruput);

%% 
pdr_vs_snr = [snr' pdr'];
save('pdr_vs_snr_theory.mat', 'pdr_vs_snr');

%%
N = 100;
s = zeros(N, 1);
s(1) = 1;
s(2) = 2;
s(3) = 2;

for i = 4 : N
    s(i) = s(i - 2) + s(i - 3);
end
plot(s);
set(gca, 'yscale', 'log');



%%
load trimmed_link_pdrs;
t = trimmed_link_pdrs;
cdfplot(t(:, end) * 100);

