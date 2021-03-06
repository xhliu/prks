%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   9/1/2011
%   Function: parse raw data from Indriya w/
%   $Dropbox/Programming/parserIndriya job# #.dat
%   Attention: MUST remove the table header at first line
%   [TX/RX; NodeID; SourceID; SeqNum; Last_Hop_Sender; Last_Hop_Ntw_Seq; Last_Hop_MAC_Seq; Local_Ntw_Seq;
%   Local_MAC_Seq; Timestamp, serial_seqno]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc

jobs = [43679 43681];

for job = 1 : length(jobs)
    job_id = jobs(job);
    
fprintf('processing job %d: ', job_id);

% srcDir = '~/Downloads/Indriya/';
srcDir = '~/Projects/tOR/RawData/Indriya/';
dest = [srcDir num2str(job_id)];
cd(dest);
output = fopen('output.txt', 'w');
load 7857.dat;
t = X7857;
% load 24915.dat;
% t = X24915;
%% parse
ENTRY_PER_ROW = 5;
ENTRY_LEN = 11;
%%
s = zeros(size(t, 1) * ENTRY_PER_ROW, ENTRY_LEN);

for i = 1 : size(t, 1)
    for j = 1 : ENTRY_PER_ROW
        s((i - 1) * ENTRY_PER_ROW + j, :) = t(i, (j - 1) * ENTRY_LEN + 1 : j * ENTRY_LEN);
    end
end
% s now stores everything

%% trim into corresponding matrices
SEND_FAIL_FLAG = 0;
SEND_FLAG = 1;
RCV_FLAG = 3;
INTERCEPT_FLAG = 2;

TX_FLAG = 2;
RX_FLAG = 3;


% packet loss
SW_FULL_FLAG = 5;
REJECT_FLAG = 6;
EXPIRE_FLAG = 11;

DEBUG_FLAG = 255;
DBG_LOSS_IN_AIR_FLAG = 5;
%% remove unused entries
tx_successes = s(s(:, 1) == SEND_FLAG, :);
srcFailPkts = s(s(:, 1) == SEND_FAIL_FLAG, :);
intercepts = s(s(:, 1) == INTERCEPT_FLAG, :);
destPkts = s(s(:, 1) == RCV_FLAG, :);

txs = s(s(:, 1) == TX_FLAG, :);
rxs = s(s(:, 1) == RX_FLAG, :);

rejs = s(s(:, 1) == REJECT_FLAG, :);
overflows = s(s(:, 1) == SW_FULL_FLAG, :);
expires = s(s(:, 1) == EXPIRE_FLAG, :);

debugs = s(s(:, 1) == DEBUG_FLAG, :);

%% uart reliability 
uart_relis = [];
nodes = unique(s(:, 2));
for i = 1 : length(nodes)
    node = nodes(i);
    node_uart = s(s(:, 2) == node, :);
    
    log_cnts = size(node_uart, 1);
    req_cnts = node_uart(end, end) + 1;
    reli = log_cnts / req_cnts;
    uart_relis = [uart_relis; node, log_cnts, req_cnts, reli];
end

save('debugs.mat', 'debugs');
save('txrxs.mat', 'txs', 'rxs', 'tx_successes', 'uart_relis');


%% compute link reliability
% SRC_IDX = 4;
% DST_IDX = 3;
% SEQ_IDX = 10;
SRC_IDX = 3;
DST_IDX = 3;
SEQ_IDX = 4;

link_pdrs = [];
srcs = unique(txs(:, 2));

for i = 1 : size(srcs, 1)
   src = srcs(i);
   
   tx = txs(txs(:, 2) == src, :);
   dst = tx(1, DST_IDX);
   
   rx = rxs(rxs(:, 2) == dst & rxs(:, SRC_IDX) == src, :);
   
   link_pdrs = [link_pdrs; src dst size(tx, 1) size(rx, 1) size(rx, 1) / size(tx, 1)];
end
save('link_pdrs.mat', 'link_pdrs');
fprintf('min uart reliability: %f\n', min(uart_relis(:, end)));

end
%%
if ~isempty(link_pdrs)
    cdfplot(link_pdrs(:, end));
    % hold on;
end
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