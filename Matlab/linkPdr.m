function [link_pdrs signal_map] = linkPdr(txs, rxs, MAX_NODE_ID)
%LINKPDR Summary of this function goes here
%   Detailed explanation goes here

SRC_IDX = 3;
RSSI_IDX = 10;

link_pdrs = NaN(MAX_NODE_ID);
signal_map = NaN(MAX_NODE_ID);

for i = 1 : MAX_NODE_ID
   src = i;
   
   tx_cnt = sum(txs(:, 2) == src);
   r = rxs(rxs(:, SRC_IDX) == src, :);
   for j = 1 : MAX_NODE_ID
       dst = j;
       
       if j == i
           continue;
       end

       rx_cnt = sum(r(:, 2) == dst);
       link_pdrs(i, j) = rx_cnt / tx_cnt;
       
       signal_map(i, j) = median(r(r(:, 2) == dst, RSSI_IDX) - 2 ^ 32 - 45);
   end
end

end
