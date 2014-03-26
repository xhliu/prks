%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/15/2014
%   Function: compute latency
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
    cd(job_dir);
    if exist('link_seq_latency.mat', 'file')
        fprintf('skip processed %d-th job %d\n', job_id, jobs(job_id));
        continue;
    else
        fprintf('processing %d-th job job %d\n', job_id, jobs(job_id));
    end

%%
SRC_IDX = 3;
DST_IDX = 3;
SEQ_IDX = 4;
TIMESTAMP_IDX = 10;
load txrxs;

%% strech out wrapped around time
SLOT_WRAP_LEN = 2 ^ 32;
MIN_GAP = SLOT_WRAP_LEN / 2;

if is_sync_protocol
    t = rxs;
else
    t = txs;
end
nodes = unique(t(:, 2));
for j = 1 : length(nodes)
    node_ix = (t(:, 2) == nodes(j));
    s = t(node_ix, TIMESTAMP_IDX);
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
    t(node_ix, TIMESTAMP_IDX) = s;
end
if is_sync_protocol
    rxs = t;
else
    txs = t;
end

%
t = tx_successes;
nodes = unique(t(:, 2));
for j = 1 : length(nodes)
    node_ix = (t(:, 2) == nodes(j));
    s = t(node_ix, TIMESTAMP_IDX);
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
    t(node_ix, TIMESTAMP_IDX) = s;
end
tx_successes = t;

if is_sync_protocol
%% latency = rx_timestamp - tx_timestamp
% tx_timestamp from tx_successes
% rx_timestamp from TDMA-based:  rxs; async protocol: txs, i.e., sendDone

len = size(rxs, 1);
% [sender receiver seqno latency]
link_seq_latency = zeros(len, 4);
idx = 1;
miss_cnt = 0;
dup_cnt = 0;

senders = unique(tx_successes(:, 2));
% each link, i.e., sender
for i = 1 : size(senders, 1)
    sender = senders(i);
    tx = tx_successes(tx_successes(:, 2) == sender, :);
    receiver = tx(1, DST_IDX);
    rx = rxs(rxs(:, 2) == receiver & rxs(:, SRC_IDX) == sender, :);
    
    % each packet across that link
    for j = 1 : size(rx, 1)
        fprintf('processing packet %d for link %d\n', j, i);
        rx_timestamp = rx(j, TIMESTAMP_IDX);
        
        ix = tx(:, SEQ_IDX) == rx(j, SEQ_IDX);
        % sanity check
        match_cnt = sum(ix);
        if match_cnt == 0
            miss_cnt = miss_cnt + 1;
            continue;
        else
            if match_cnt > 1
                % duplicate
                dup_cnt = dup_cnt + 1;
                continue;
            end
        end

        tx_timestamp = tx(ix, TIMESTAMP_IDX);
        latency = rx_timestamp - tx_timestamp;
        link_seq_latency(idx, :) = [sender receiver rx(j, SEQ_IDX) latency];
        idx = idx + 1;
    end
end
link_seq_latency(idx : end, :) = [];
fprintf('miss ratio: %f, negative ratio: %f, duplicate ratio: %f\n', miss_cnt / len, sum(link_seq_latency(:, end) <= 0) / len, dup_cnt / len);

else

len = size(rxs, 1);
% [sender receiver seqno latency]
link_seq_latency = zeros(len, 4);
idx = 1;
miss_cnt = 0;
dup_cnt = 0;

senders = unique(tx_successes(:, 2));
% each link, i.e., sender
for i = 1 : size(senders, 1)
    sender = senders(i);
    tx = tx_successes(tx_successes(:, 2) == sender, :);
    tx_done = txs(txs(:, 2) == sender, :);
    receiver = tx(1, DST_IDX);
    rx = rxs(rxs(:, 2) == receiver & rxs(:, SRC_IDX) == sender, :);
    
    % each packet across that link
    for j = 1 : size(rx, 1)
        fprintf('processing packet %d for link %d\n', j, i);
        seqno = rx(j, SEQ_IDX);
        
        %rx_timestamp = rx(j, TIMESTAMP_IDX);
        ix = tx_done(:, SEQ_IDX) == seqno;
        match_cnt = sum(ix);
        if match_cnt == 0
            miss_cnt = miss_cnt + 1;
            continue;
        else
            if match_cnt > 1
                % duplicate
                dup_cnt = dup_cnt + 1;
                continue;
            end
        end
        rx_timestamp = tx_done(ix, TIMESTAMP_IDX);
        
        ix = tx(:, SEQ_IDX) == rx(j, SEQ_IDX);
        % sanity check
        match_cnt = sum(ix);
        if match_cnt == 0
            miss_cnt = miss_cnt + 1;
            continue;
        else
            if match_cnt > 1
                % duplicate
                dup_cnt = dup_cnt + 1;
                continue;
            end
        end
        tx_timestamp = tx(ix, TIMESTAMP_IDX);
        
        latency = rx_timestamp - tx_timestamp;
        link_seq_latency(idx, :) = [sender receiver seqno latency];
        idx = idx + 1;
    end
end
link_seq_latency(idx : end, :) = [];
fprintf('miss ratio: %f, negative ratio: %f, duplicate ratio: %f\n', miss_cnt / len, sum(link_seq_latency(:, end) <= 0) / len, dup_cnt / len);

end

%%
% MAX_LATENCY = inf; %10 ^ 4;
t = link_seq_latency;
t = t(t(:, end) > 0, :);
% t = t(t(:, end) < MAX_LATENCY, :);
cdfplot(t(:, end));
set(gca, 'xscale', 'log');
%%
save('link_seq_latency.mat', 'link_seq_latency');

end
%% time-consuming
% len = size(rxs, 1);
% % [sender receiver seqno latency]
% link_seq_latency = zeros(len, 4);
% idx = 1;
% miss_cnt = 0;
% dup_cnt = 0;
% for i = 1 : len
%     fprintf('processing packet %d\n', i);
%     rx_timestamp = rxs(i, TIMESTAMP_IDX);
%     
%     ix = txs(:, 2) == rxs(i, SRC_IDX) & txs(:, SEQ_IDX) == rxs(i, SEQ_IDX);
%     % sanity check
%     match_cnt = sum(ix);
%     if match_cnt == 0
%         miss_cnt = miss_cnt + 1;
%         continue;
%     else
%         if match_cnt > 1
%             % duplicate
%             dup_cnt = dup_cnt + 1;
%             continue;
%         else
%             if txs(ix, DST_IDX) ~= rxs(i, 2)
%                 fprintf('error\n');
%                 return;
%             end
%         end
%     end
%     
%     tx_timestamp = txs(ix, TIMESTAMP_IDX);
%     latency = rx_timestamp - tx_timestamp;
%     link_seq_latency(idx, :) = [txs(ix, 2) rxs(i, 2) rxs(i, SEQ_IDX) latency];
%     idx = idx + 1;
% end
% link_seq_latency(idx : end, :) = [];
% fprintf('miss ratio: %f, negative ratio: %f, duplicate ratio: %f\n', miss_cnt / len, sum(link_seq_latency(:, end) <= 0) / len, dup_cnt / len);
