%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute convergence time, both for link and network
%%
ROOT_NODE_ID = 15;
SRC_IDX = 4;
SEQ_IDX = 5;
% DST_IDX = 3;

%% e2e pdr
% fprintf('to change log idx here\n');
load txrxs;
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
fprintf('%d out of %d are missing: %f\n', miss_cnt, size(txs, 1), miss_cnt / size(txs, 1));

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

%% loss causes: overflow or loss in air
load debugs;
t = debugs;
type = DBG_LOSS_FLAG;
% line = 186;
t = t(t(:, 3) == type, :);
% t = t(t(:, 4) == line, :);
fprintf('%d are missing from log\n', size(t, 1));
if ~isempty(t)
    % #line around 250  means queue overflow
    s = t;
    s = s(:, 4);
    cdfplot(s);
    % unique(s)
end
size(unique(t(:, 9:10), 'rows'), 1)
%% queue level
load txrxs.mat;
cdfplot(txs(:, 6));