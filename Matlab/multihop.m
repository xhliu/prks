%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute convergence time, both for link and network
%%
ROOT_NODE_ID = 15;

%% e2e pdr
load txrxs;
% (RX_FLAG, getHeader(msg)->origin, getHeader(msg)->originSeqNo, call
% SubAMPacket.source(msg), 0, 0, 0, 0, getGlobalTime())
rxs = rxs(rxs(:, 2) == ROOT_NODE_ID, :);
[a ix] = unique(rxs(:, [3 4]), 'rows');
rxs = rxs(ix, :);
% (TX_DONE_FLAG, call Util.getReceiver(), qe.originSeqNo, qe.origin, call
% SendQueue.size(), call Acks.wasAcked(msg), 0, 0, getGlobalTime())
txs = txs(txs(:, 2) == txs(:, 5), :);
% filter retx
[a ix] = unique(txs(:, [5 4]), 'rows');
txs = txs(ix, :);
miss_cnt = size(txs, 1) - size(rxs, 1);
fprintf('%d out of %d are missing: %f\n', miss_cnt, size(txs, 1), miss_cnt / size(txs, 1));

SRC_IDX = 3;
% DST_IDX = 3;
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

%% queue level
load txrxs.mat;
cdfplot(txs(:, 6));