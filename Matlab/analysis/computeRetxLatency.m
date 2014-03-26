%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/21/2014
%   Function: compute retx latency to reach a certain pdr
%% whether sync protocol or not
% is_sync_protocol = false;
function computeRetxLatency(job_dir, is_sync_protocol)

if ~exist(job_dir, 'dir')
    fprintf('directory %s nonexistent\n', job_dir);
    return;
end

cd(job_dir);
% fprintf('warning\n');
if exist('link_seq_tx_cnt_latency_2.mat', 'file')
    fprintf('job @ %s already computed\n', job_dir);
    return;
end

%%
SRC_IDX = 3;
DST_IDX = 3;
SEQ_IDX = 4;
TIMESTAMP_IDX = 10;
load txrxs;

%% strech out wrapped around time
SLOT_WRAP_LEN = 2 ^ 32;
tx_successes = unwrap(tx_successes, SLOT_WRAP_LEN);
rxs = unwrap(rxs, SLOT_WRAP_LEN);
if ~is_sync_protocol
    txs = unwrap(txs, SLOT_WRAP_LEN);
end


%% latency = rx_timestamp - tx_timestamp
% tx_timestamp from tx_successes
% rx_timestamp from TDMA-based:  rxs; async protocol: txs, i.e., sendDone

if is_sync_protocol

len = size(tx_successes, 1);
% tx_attempt_cnt: # of tx it takes to be delivered
% [sender receiver seqno tx_attempt_cnt tx_timestamp rx_timestamp latency]
% link_seq_tx_cnt_latency = zeros(len, 7);
link_seq_tx_cnt_latency_2 = zeros(len, 7);
idx = 1;
miss_cnt = 0;

senders = unique(tx_successes(:, 2));
% each link, i.e., sender
for i = 1 : size(senders, 1)
    sender = senders(i);
    tx = tx_successes(tx_successes(:, 2) == sender, :);
    receiver = tx(1, DST_IDX);
    rx = rxs(rxs(:, 2) == receiver & rxs(:, SRC_IDX) == sender, :);
    
    % each packet rx
    for j = 1 : size(rx, 1)
        fprintf('processing link %d, packet %d\n', i, j);
        seq = rx(j, SEQ_IDX);
        rx_timestamp = rx(j, TIMESTAMP_IDX);
        
        % search its tx
        tx_idx = find(tx(:, SEQ_IDX) == seq, 1);
        if isempty(tx_idx)
            miss_cnt = miss_cnt + 1;
            continue;
        end
        
        % approach (1): 0 0 0 1, 3 samples
%         % all transmissions up to tx_idx are "received" in this rx
%         for k = 1 : tx_idx
%             tx_timestamp = tx(k, TIMESTAMP_IDX);
%             latency = rx_timestamp - tx_timestamp;
% %             if latency < -10000
% %                 disp('');
% %             end
%             link_seq_tx_cnt_latency(idx, :) = [sender receiver tx(k, SEQ_IDX) (tx_idx - k + 1) tx_timestamp rx_timestamp latency];
%             idx = idx + 1;
%         end
        % approach (2): 0 0 0 1, only 1 sample
        %for k = 1 : tx_idx
        k = 1;
            tx_timestamp = tx(k, TIMESTAMP_IDX);
            latency = rx_timestamp - tx_timestamp;
            link_seq_tx_cnt_latency_2(idx, :) = [sender receiver tx(k, SEQ_IDX) (tx_idx - k + 1) tx_timestamp rx_timestamp latency];
            idx = idx + 1;
        %end
         
        % optimize
        tx(1 : tx_idx, :) = [];
    end
end
fprintf('approach changed\n');

%%
else
%% ~is_sync_protocol

len = size(tx_successes, 1);
% tx_attempt_cnt: # of tx it takes to be delivered
% [sender receiver seqno tx_attempt_cnt tx_timestamp rx_timestamp latency]
%link_seq_tx_cnt_latency = zeros(len, 7);
link_seq_tx_cnt_latency_2 = zeros(len, 7);
idx = 1;
miss_cnt = 0;

senders = unique(tx_successes(:, 2));
% each link, i.e., sender
for i = 1 : size(senders, 1)
    sender = senders(i);
    tx = tx_successes(tx_successes(:, 2) == sender, :);
    tx_done = txs(txs(:, 2) == sender, :);
    receiver = tx(1, DST_IDX);
    rx = rxs(rxs(:, 2) == receiver & rxs(:, SRC_IDX) == sender, :);
    
    % each packet rx
    for j = 1 : size(rx, 1)
        fprintf('processing link %d, packet %d\n', i, j);
        seq = rx(j, SEQ_IDX);
        
        % rx_timestamp = rx(j, TIMESTAMP_IDX);
%         ix = tx_done(:, SEQ_IDX) == seq;
%         match_cnt = sum(ix);
%         if match_cnt == 0
%             miss_cnt = miss_cnt + 1;
%             continue;
%         end
%         rx_timestamp = tx_done(ix, TIMESTAMP_IDX);
        
        % 1) search its tx_done
        tx_idx = find(tx_done(:, SEQ_IDX) == seq, 1);
        if isempty(tx_idx)
            miss_cnt = miss_cnt + 1;
            continue;
        end
        rx_timestamp = tx_done(tx_idx, TIMESTAMP_IDX);        
        % optimize
        tx_done(1 : tx_idx, :) = [];
        
        
        % 2) search its tx
        tx_idx = find(tx(:, SEQ_IDX) == seq, 1);
        if isempty(tx_idx)
            miss_cnt = miss_cnt + 1;
            continue;
        end
        
        % all transmissions up to tx_idx are "received" in this rx
        %for k = 1 : tx_idx
        k = 1;
            tx_timestamp = tx(k, TIMESTAMP_IDX);
            latency = rx_timestamp - tx_timestamp;
%             if latency < -10000
%                 disp('');
%             end
            link_seq_tx_cnt_latency_2(idx, :) = [sender receiver tx(k, SEQ_IDX) (tx_idx - k + 1) tx_timestamp rx_timestamp latency];
            idx = idx + 1;
        %end
        
        % optimize
        tx(1 : tx_idx, :) = [];
    end
end


end % is_sync_protocol

%% 
%link_seq_tx_cnt_latency(idx : end, :) = [];
link_seq_tx_cnt_latency_2(idx : end, :) = [];
% 
t = link_seq_tx_cnt_latency_2;
fprintf('ratios: negative %f, missing %f\n', sum(t(:, end) <= 0) / len, miss_cnt / len);
save('link_seq_tx_cnt_latency_2.mat', 'link_seq_tx_cnt_latency_2');

end
%% approach 1: kinda slow
% s = t(:, end) * (5 / 512 / 1000);
% cdfplot(s);
% mean(s)
% median(s)
% 
% 
% len = size(tx_successes, 1);
% % tx_attempt_cnt: # of tx it takes to be delivered
% % [sender receiver seqno tx_attempt_cnt latency]
% link_seq_tx_cnt_latency = zeros(len, 5);
% idx = 1;
% % miss_cnt = 0;
% % dup_cnt = 0;
% 
% senders = unique(tx_successes(:, 2));
% % each link, i.e., sender
% for i = 1 : size(senders, 1)
%     sender = senders(i);
%     tx = tx_successes(tx_successes(:, 2) == sender, :);
%     receiver = tx(1, DST_IDX);
%     rx = rxs(rxs(:, 2) == receiver & rxs(:, SRC_IDX) == sender, :);
%     if isempty(rx)
%         fprintf('link %d receives none\n', i);
%         continue;
%     end
%     min_rx_seq = min(rx(:, SEQ_IDX));
%     max_rx_seq = max(rx(:, SEQ_IDX));
%     
%     % each packet sent
%     for j = 1 : size(tx, 1)
%         seq = tx(j, SEQ_IDX);
%         % up to max_rx_seq, bcoz last few packets may never be received
%         if seq < min_rx_seq || seq > max_rx_seq
%             break;
%         end
%         
%         tx_timestamp = tx(j, TIMESTAMP_IDX);
%         tx_attempt_cnt = 0;
% %         last_found_idx = 1;
%         % search its rx, including retx
%         while true
%             rx_idx = find(rx(:, SEQ_IDX) == seq + tx_attempt_cnt, 1);
%             tx_attempt_cnt = tx_attempt_cnt + 1;
%             fprintf('processing link %d, packet %d, %d-th tx\n', i, j, tx_attempt_cnt);
%             if tx_attempt_cnt > 30
%                 disp('');
%             end
%             if isempty(rx_idx)
%                 continue;
%             end
%             
%             rx_timestamp = rx(rx_idx, TIMESTAMP_IDX);
%             latency = rx_timestamp - tx_timestamp;
%             link_seq_tx_cnt_latency(idx, :) = [sender receiver seq tx_attempt_cnt latency];
%             idx = idx + 1;
%             
%             % optimize using last found ix
%             if rx_idx > 1
%                 rx(1 : rx_idx - 1, :) = [];
%             end
% 
%             % found rx
%             break;
%         end
%     end
% end
% 
% link_seq_tx_cnt_latency(idx : end, :) = [];