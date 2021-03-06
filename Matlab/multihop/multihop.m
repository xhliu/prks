%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute convergence time, both for link and network
%%
clc;
ROOT_NODE_ID = 15;
fprintf('swap the 2 indices below\n')
SRC_IDX = 5;
SEQ_IDX = 4;
% DST_IDX = 3;
% time in us
SLOT_WRAP_LEN = 2 ^ 32;

if 0
%% e2e pdr
% fprintf('to change log idx here\n');
load txrxs;
% proprocessing: time wrap around
rxs = unwrap(rxs, SLOT_WRAP_LEN);
txs = unwrap(txs, SLOT_WRAP_LEN);

% (RX_FLAG, getHeader(msg)->origin, getHeader(msg)->originSeqNo, call
% SubAMPacket.source(msg), len, __LINE__, len > call SubSend.maxPayloadLength(), is_root_, getGlobalTime())
rxs = rxs(rxs(:, 2) == ROOT_NODE_ID, :);
% last occurrence by default
[a ix] = unique(rxs(:, [SRC_IDX SEQ_IDX]), 'rows', 'first');
rxs = rxs(ix, :);
% (TX_DONE_FLAG, call Util.getReceiver(), qe.originSeqNo, qe.origin, call
% SendQueue.size(), call Acks.wasAcked(msg), 0, 0, getGlobalTime())
txs = txs(txs(:, 2) == txs(:, SRC_IDX), :);
% filter retx
[a ix] = unique(txs(:, [SRC_IDX SEQ_IDX]), 'rows', 'first');
txs = txs(ix, :);
miss_cnt = size(txs, 1) - size(rxs, 1);
fprintf('%d out of %d are missing based on tx/rx: %f\n', miss_cnt, size(txs, 1), miss_cnt / size(txs, 1));

link_pdrs = [];
srcs = unique(txs(:, 2));

for i = 1 : size(srcs, 1)
   src = srcs(i);
   
   tx = txs(txs(:, 2) == src, :);
%    dst = tx(1, DST_IDX);
   dst = ROOT_NODE_ID;
   
   rx = rxs(rxs(:, 2) == dst & rxs(:, SRC_IDX) == src, :);
   
   link_pdrs = [link_pdrs; src dst size(tx, 1) size(rx, 1) size(rx, 1) / size(tx, 1)];
end
cdfplot(link_pdrs(:, end));
e2e_link_pdrs = link_pdrs;
%% loss causes: overflow or loss in air
load debugs;
t = debugs;
type = DBG_LOSS_FLAG;
% line = 186;
t = t(t(:, 3) == type, :);
% t = t(t(:, 4) == line, :);
% fprintf('%d are missing from log: some can still in queue\n', size(t, 1));
if ~isempty(t)
    % #line around 250  means queue overflow
    s = t;
    s = s(:, 4);
    cdfplot(s);
    % unique(s)
end
% size(unique(t(:, 9:10), 'rows'), 1)
miss_cnt = size(t, 1);
fprintf('%d out of %d are missing based on loss log: %f\n', miss_cnt, size(txs, 1), miss_cnt / size(txs, 1));
save('e2e_link_pdrs.mat', 'e2e_link_pdrs');

%% tx cost
job_dir = [MAIN_DIR num2str(22329)];
cd(job_dir);
load txrxs;
rxs = rxs(rxs(:, 2) == ROOT_NODE_ID, :);
unique_rx_cnt = size(unique(rxs(:, [SRC_IDX SEQ_IDX]), 'rows'), 1);
tx_cnt = size(txs, 1);
tx_cnt / unique_rx_cnt
end
% %% throughput
% % for TDMA only
% SLOT_LEN = 32;
% fprintf('slot length %d ms\n', SLOT_LEN);
% TIMESTAMP_IDX = 10;
% 
% s = floor(rxs(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));
% % 
% [c e] = hist(s, unique(s));
% plot(e);
% % add slots w/o any rx
% total = max(s) - min(s) + 1;
% rx_concurrency = [c'; zeros(total - length(c), 1)];
% save('rx_concurrency.mat', 'rx_concurrency');
% % cdfplot(rx_concurrency);
% fprintf('total %d, rx_concurrency median %f, mean %f\n', total, median(rx_concurrency), mean(rx_concurrency));
% 
% 
% %% delay
% 
% %% queue level
% load txrxs.mat;
% cdfplot(txs(:, 6));