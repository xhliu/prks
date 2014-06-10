%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/15/2014
%   Function: compute latency, no retx
%%
clc;
MAIN_DIR = '~/Projects/tOR/RawData/';
fprintf('processing multihop\n');
%% whether sync protocol or not
% CSMA, RTS-CTS, CMAC: async
% PRKS, SCREAM, and RIDB (w/ or w/o OLAMA): sync
is_sync_protocol = false;
if is_sync_protocol
    jobs = sync_jobs;
else
    jobs = async_jobs;
end
% jobs = [22325];

for job_id = 1 : length(jobs)
    job_dir = [MAIN_DIR num2str(jobs(job_id))];
    cd(job_dir);
%     if exist('link_seq_tx_cnt_retx_latency.mat', 'file')
%         fprintf('skip processed %d-th job %d\n', job_id, jobs(job_id));
%         continue;
%     else
%         fprintf('processing %d-th job job %d\n', job_id, jobs(job_id));
%     end

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
% needed for non-TDMA protocol
load link_seq_latency;
load e2e_link_pdrs;

%% strech out wrapped around time
SLOT_WRAP_LEN = 2 ^ 32;
rxs = unwrap(rxs, SLOT_WRAP_LEN);
txs = unwrap(txs, SLOT_WRAP_LEN);
tx_successes = unwrap(tx_successes, SLOT_WRAP_LEN);

%% filter out redudant tx/rx
t = tx_successes;
t = t(t(:, 2) == t(:, SRC_IDX), :);
% filter retx
[a ix] = unique(t(:, [SRC_IDX SEQ_IDX]), 'rows', 'first');
tx_successes = t(ix, :);

t = rxs;
t = t(t(:, 2) == ROOT_NODE_ID, :);
% last occurrence by default
[a ix] = unique(t(:, [SRC_IDX SEQ_IDX]), 'rows', 'first');
rxs = t(ix, :);

miss_cnt = 0;
dup_cnt = 0;

%% latency = rx_timestamp - tx_timestamp
% tx_timestamp from tx_successes
% rx_timestamp from TDMA-based:  rxs; async protocol: txs, i.e., sendDone
%% approach 0
MAX_RETRIES = 8;
% delay for lost pkt after MAX_RETRIES e2e retries
LOST_PKT_LATENCY = inf;
len = size(rxs, 1);
% [sender receiver seqno latency]
% [sender receiver seqno tx_attempt_cnt tx_timestamp rx_timestamp latency]
link_seq_tx_cnt_retx_latency = zeros(len, 7);
idx = 1;

senders = unique(tx_successes(:, 2));
% each link, i.e., sender
for i = 1 : size(senders, 1)
    sender = senders(i);
    tx = tx_successes(tx_successes(:, 2) == sender, :);
    %receiver = tx(1, DST_IDX);
    receiver = ROOT_NODE_ID;
    rx = rxs(rxs(:, 2) == receiver & rxs(:, SRC_IDX) == sender, :);
    
    % [sender receiver seq latency]
    t = link_seq_latency;
    t = t(t(:, 1) == sender & t(:, 2) == receiver, :);
    
    % each tx, including retx
    j = 1;
    while j <= size(tx, 1)
        seq = tx(j, SEQ_IDX);
        fprintf('processing %d-th packet %d from src %d\n', j, seq, i);
        tx_timestamp = tx(j, TIMESTAMP_IDX);
        
        % search its rx
        rx_idx = find(rx(:, SEQ_IDX) >= seq & rx(:, SEQ_IDX) <= (seq + MAX_RETRIES) , 1);
        if isempty(rx_idx)            
            % lost pkt after MAX_RETRIES retries
            j = j + MAX_RETRIES + 1;
            link_seq_tx_cnt_retx_latency(idx, :) = [sender receiver seq (MAX_RETRIES + 1) tx_timestamp nan LOST_PKT_LATENCY];
            idx = idx + 1;
            continue;
        end
        
        % approach (2): 0 0 0 1, only 1 sample
        rx_seq = rx(rx_idx, SEQ_IDX);
        % next tx
        j = find(tx(:, SEQ_IDX) == rx_seq, 1);
        if isempty(j)
            miss_cnt = miss_cnt + 1;
            break;
        end
        
        if is_sync_protocol
            rx_timestamp = rx(rx_idx, TIMESTAMP_IDX);
            latency = rx_timestamp - tx_timestamp;
        else
            % latency = next_received_tx_delay + (next_received_tx_timestamp - my_tx_timestamp)
            latency = t(t(:, 3) == rx_seq, end);
            if isempty(latency)
                miss_cnt = miss_cnt + 1;
                j = j + 1;
                continue;
            else
                if size(latency, 1) > 1
                    dup_cnt = dup_cnt + 1;
                    continue;
                end
            end
            latency = latency(1) + tx(j, TIMESTAMP_IDX) - tx_timestamp;
        end
        
        tx_cnt = rx_seq - seq + 1;
        link_seq_tx_cnt_retx_latency(idx, :) = [sender receiver seq tx_cnt tx_timestamp rx_timestamp latency];
        idx = idx + 1;
        
        % next
        j = j + 1;
        % optimize
        %rx(1 : rx_idx, :) = [];
    end
end


%% approach 1: doomed bcoz all later pkts are lost, leaving no latency sample
% len = size(rxs, 1);
% [sender receiver seqno latency]
% [sender receiver seqno tx_attempt_cnt tx_timestamp rx_timestamp latency]
% link_seq_tx_cnt_retx_latency = zeros(len, 7);
% idx = 1;
% 
% senders = unique(tx_successes(:, 2));
% each link, i.e., sender
% for i = 1 : size(senders, 1)
%     sender = senders(i);
%     tx = tx_successes(tx_successes(:, 2) == sender, :);
%     receiver = tx(1, DST_IDX);
%     receiver = ROOT_NODE_ID;
%     rx = rxs(rxs(:, 2) == receiver & rxs(:, SRC_IDX) == sender, :);
%     
%     each packet across that link
%     for j = 1 : size(rx, 1)
%         seq = rx(j, SEQ_IDX);
%         fprintf('processing %d-th packet %d from src %d\n', j, seq, i);
%         rx_timestamp = rx(j, TIMESTAMP_IDX);
%         
%         search its tx
%         tx_idx = find(tx(:, SEQ_IDX) == seq, 1);
%         if isempty(tx_idx)
%             miss_cnt = miss_cnt + 1;
%             continue;
%         end
% 
%         approach (2): 0 0 0 1, only 1 sample
%         for k = 1 : tx_idx
%         k = 1;
%         tx_timestamp = tx(k, TIMESTAMP_IDX);
%         latency = rx_timestamp - tx_timestamp;
%         tx_cnt = (tx_idx - k + 1);
%         if tx_cnt > 5
%             disp('');
%         end
%         link_seq_tx_cnt_retx_latency(idx, :) = [sender receiver tx(k, SEQ_IDX) tx_cnt tx_timestamp rx_timestamp latency];
%         idx = idx + 1;
%         end
%          
%         optimize
%         tx(1 : tx_idx, :) = [];
%     end
% end


%%
link_seq_tx_cnt_retx_latency(idx : end, :) = [];
fprintf('miss ratio: %f, negative ratio: %f, duplicate ratio: %f\n', miss_cnt / len, sum(link_seq_tx_cnt_retx_latency(:, end) <= 0) / len, dup_cnt / len);
save('link_seq_tx_cnt_retx_latency.mat', 'link_seq_tx_cnt_retx_latency');
%%
% load link_seq_tx_cnt_retx_latency;
% MAX_LATENCY = inf; %10 ^ 4;
t = link_seq_tx_cnt_retx_latency;
t = t(t(:, end) > 0, :);
% t = t(t(:, end) < MAX_LATENCY, :);
cdfplot(t(:, end));
set(gca, 'xscale', 'log');
%%

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