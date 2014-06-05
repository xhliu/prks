%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/15/2014
%   Function: compute latency, no retx
%%
%% whether sync protocol or not
% CSMA, RTS-CTS, CMAC: async
% is_sync_protocol = false;
% jobs = async_jobs;
MAIN_DIR = '~/Projects/tOR/RawData/';
% PRKS, SCREAM, and RIDB (w/ or w/o OLAMA): sync
is_sync_protocol = false;
if is_sync_protocol
    jobs = sync_jobs;
else
    jobs = async_jobs;
end
% jobs = [22329];

for job_id = 1 : length(jobs)
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    cd(job_dir);
% fprintf('debugging negative delay\n');
    if exist('link_seq_latency.mat', 'file')
        fprintf('skip processed %d-th job %d\n', job_id, jobs(job_id));
        continue;
    else
        fprintf('processing %d-th job job %d\n', job_id, jobs(job_id));
    end

%%
% SRC_IDX = 3;
% DST_IDX = 3;
% SEQ_IDX = 4;
SRC_IDX = 5;
SEQ_IDX = 4;
% DST_IDX = 3;
ROOT_NODE_ID = 15;
TIMESTAMP_IDX = 10;
load txrxs;

%% strech out wrapped around time
SLOT_WRAP_LEN = 2 ^ 32;
rxs = unwrap(rxs, SLOT_WRAP_LEN);
txs = unwrap(txs, SLOT_WRAP_LEN);
tx_successes = unwrap(tx_successes, SLOT_WRAP_LEN);

t = tx_successes;
t = t(t(:, 2) == t(:, SRC_IDX), :);
% filter retx
[a ix] = unique(t(:, [SRC_IDX SEQ_IDX]), 'rows', 'first');
tx_successes = t(ix, :);

miss_cnt = 0;
dup_cnt = 0;

if is_sync_protocol
%% latency = rx_timestamp - tx_timestamp
% tx_timestamp from tx_successes
% rx_timestamp from TDMA-based:  rxs; async protocol: txs, i.e., sendDone
t = rxs;
t = t(t(:, 2) == ROOT_NODE_ID, :);
% last occurrence by default
[a ix] = unique(t(:, [SRC_IDX SEQ_IDX]), 'rows', 'first');
rxs = t(ix, :);


len = size(rxs, 1);
% [sender receiver seqno latency]
link_seq_latency = zeros(len, 4);
idx = 1;

senders = unique(tx_successes(:, 2));
% each link, i.e., sender
for i = 1 : size(senders, 1)
    sender = senders(i);
    tx = tx_successes(tx_successes(:, 2) == sender, :);
    %receiver = tx(1, DST_IDX);
    receiver = ROOT_NODE_ID;
    rx = rxs(rxs(:, 2) == receiver & rxs(:, SRC_IDX) == sender, :);
    
    % each packet across that link
    for j = 1 : size(rx, 1)
        fprintf('processing packet %d for src %d\n', j, i);
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

else

clc;
%% due to lack of global time, sum latency at each hop, which is the
%% interval from rx to ACKed txs (excepted at src, from tx_successes)
ACK_IDX = 7;
DST_IDX = 3;

% consider 1st ACKed tx as when a pkt leaves a node
t = txs;
t = t(t(:, ACK_IDX) ~= 0, :);
txs = t;

% t = rxs;
% % last occurrence by default
% [a ix] = unique(t(:, [SRC_IDX SEQ_IDX]), 'rows', 'first');
% rxs = t(ix, :);

% each received pkt at root
t = rxs;
t = t(t(:, 2) == ROOT_NODE_ID, :);
src_seqs = unique(t(:, [SRC_IDX SEQ_IDX]), 'rows');
len = size(src_seqs, 1);
% [sender receiver seqno latency]
link_seq_latency = zeros(len, 4);
idx = 1;

% each received pkt
for i = 1 : len
    src = src_seqs(i, 1);
    seq = src_seqs(i, 2);

    % start from src
    node = src;
    e2e_latency = 0;
    % each hop
    while node ~= ROOT_NODE_ID
        fprintf('<%d, %d> @ %d\n', src, seq, node);
        % arrival
        if node ~= src
            t = rxs;
        else
            % special case @ src
            t = tx_successes;
        end
        ix = t(:, 2) == node & t(:, SRC_IDX) == src & t(:, SEQ_IDX) == seq;
        % sanity check
        match_cnt = sum(ix);
        if match_cnt == 0
            miss_cnt = miss_cnt + 1;
            break;
%         else
%             % can be duplicate bcoz of failed ACK
%             if match_cnt > 1
%                 % duplicate
%                 dup_cnt = dup_cnt + 1;
%                 break;
%             end
        end
        arrival_time = t(ix, TIMESTAMP_IDX);
        arrival_time = arrival_time(1);
        
        % departure
        ix = txs(:, 2) == node & txs(:, SRC_IDX) == src & txs(:, SEQ_IDX) == seq;
        match_cnt = sum(ix);
        if match_cnt == 0
            miss_cnt = miss_cnt + 1;
            break;
        else
            % tx should be no duplicate since dequeue after ACKed
            if match_cnt > 1
                % duplicate
                dup_cnt = dup_cnt + 1;
                break;
            end
        end
        departure_time = txs(ix, TIMESTAMP_IDX);
        hop_delay = departure_time - arrival_time;
        e2e_latency = e2e_latency + hop_delay;
        
        % next hop
        node = txs(ix, DST_IDX);
    end
    
    % only for pkt reaching root
    if node == ROOT_NODE_ID
        fprintf('<%d, %d> @ %d\n', src, seq, node);
        link_seq_latency(idx, :) = [src ROOT_NODE_ID seq e2e_latency];
        idx = idx + 1;
    end
end

end

link_seq_latency(idx : end, :) = [];
fprintf('miss ratio: %f, negative ratio: %f, duplicate ratio: %f\n', miss_cnt / len, sum(link_seq_latency(:, end) <= 0) / len, dup_cnt / len);
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

%%
% len = size(rxs, 1);
% % [sender receiver seqno latency]
% link_seq_latency = zeros(len, 4);
% idx = 1;
% miss_cnt = 0;
% dup_cnt = 0;
% 
% senders = unique(tx_successes(:, 2));
% % each link, i.e., sender
% for i = 1 : size(senders, 1)
%     sender = senders(i);
%     tx = tx_successes(tx_successes(:, 2) == sender, :);
%     tx_done = txs(txs(:, 2) == sender, :);
%     %receiver = tx(1, DST_IDX);
%     receiver = ROOT_NODE_ID;
%     rx = rxs(rxs(:, 2) == receiver & rxs(:, SRC_IDX) == sender, :);
%     
%     % each packet across that link
%     for j = 1 : size(rx, 1)
%         fprintf('processing packet %d for src %d\n', j, i);
%         seqno = rx(j, SEQ_IDX);
%         
%         %rx_timestamp = rx(j, TIMESTAMP_IDX);
%         ix = tx_done(:, SEQ_IDX) == seqno;
%         match_cnt = sum(ix);
%         if match_cnt == 0
%             miss_cnt = miss_cnt + 1;
%             continue;
%         else
%             if match_cnt > 1
%                 % duplicate
%                 dup_cnt = dup_cnt + 1;
%                 continue;
%             end
%         end
%         rx_timestamp = tx_done(ix, TIMESTAMP_IDX);
%         
%         ix = tx(:, SEQ_IDX) == rx(j, SEQ_IDX);
%         % sanity check
%         match_cnt = sum(ix);
%         if match_cnt == 0
%             miss_cnt = miss_cnt + 1;
%             continue;
%         else
%             if match_cnt > 1
%                 % duplicate
%                 dup_cnt = dup_cnt + 1;
%                 continue;
%             end
%         end
%         tx_timestamp = tx(ix, TIMESTAMP_IDX);
%         
%         latency = rx_timestamp - tx_timestamp;
%         link_seq_latency(idx, :) = [sender receiver seqno latency];
%         idx = idx + 1;
%     end
% end
% link_seq_latency(idx : end, :) = [];