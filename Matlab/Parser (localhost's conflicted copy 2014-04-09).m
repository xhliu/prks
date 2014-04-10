clear
clc

jobs = [22094];
% initial pkts for link estimation; excluded from analysis
% applies to iMAC only
CONVERGE_STEP = 60;
CTRL_STEP_SIZE = 20;
INITIAL_LE_PKT_CNT = 0; %1200; %CONVERGE_STEP * CTRL_STEP_SIZE; % 0

TX_FAIL_FLAG = 0;
TX_SUCCESS_FLAG = 1;
TX_DONE_FLAG = 2;
RX_FLAG = 3;
TX_DONE_FAIL_FLAG = 4;
DEBUG_FLAG = 255;

for job = 1 : length(jobs)
    job_id = jobs(job);
    
    exceptions = [];
    if ~isempty(find(exceptions == job_id, 1))
        continue;
    end
fprintf('processing job %d: \n', job_id);
DirDelimiter='/';

% srcDir = '~/Projects/tOR/RawData';
srcDir = '~/Downloads/Jobs';
srcDir2 = num2str(job_id); % Defined by users
SRC_ID = 76;

MAX_FCS_SIZE = 0; %0, 5; 3;5
Colmn_Packet = 11; %11, 16; 14;
srcDir3 = '';
dest = [srcDir DirDelimiter srcDir2 srcDir3 DirDelimiter];
files = dir([dest '*.txt']);

cd(dest);

% preallocation
MAX_ENTRIES = 1000000;
% fprintf('entry %d, not 1000000\n', MAX_ENTRIES);

ROOT_ID = 15;

Control_Data = 8; %NetEye 8, Kansei 10: 2 Columns that are useless

IsOR = 1;

tx_fails = zeros(MAX_ENTRIES, Colmn_Packet);
tx_fail_cnts = 1;
tx_successes = zeros(MAX_ENTRIES, Colmn_Packet);
tx_success_cnts = 1;
txs = zeros(MAX_ENTRIES, Colmn_Packet);
tx_cnts = 1;
tx_done_fails = zeros(MAX_ENTRIES, Colmn_Packet);
tx_done_fail_cnts = 1;
rxs = zeros(MAX_ENTRIES, Colmn_Packet);
rx_cnts = 1;
debugs = zeros(MAX_ENTRIES, Colmn_Packet);
debugs_cnts = 1;
% UART reliability
uart_relis = [];
UART_SEQ_IDX = 11;


Num_Tx = 1;
Num_NodeID = 1;
Num_SourceID = 1;
Num_SeqNum = 2;
Num_FC_each = 1;

if IsOR == 1
%     MAX_FCS_SIZE = 3; %5;
%     Colmn_Packet = 14; %16
    EletPerLog = 5; 
else
%     MAX_FCS_SIZE = 1;
%     Colmn_Packet = 11;  %11
    EletPerLog = 6; %6
end

Num_LastSender = 1;
Num_LastSenderSeq = 2;
Num_LocalSeq = 2;

Num_LastNtwSeq = 2;
Num_LocalNtwSeq = 2;

Num_TimeStamp = 4;
Num_RetxTime = 4;

Num_PerElet = Num_Tx + Num_NodeID + Num_SourceID + Num_SeqNum + Num_FC_each * MAX_FCS_SIZE + Num_LastSender + ...
    Num_LastSenderSeq + Num_LocalSeq + Num_LastNtwSeq + Num_LocalNtwSeq + Num_TimeStamp + Num_RetxTime;

% Control_Data = 10; %8 -> Kansei Columns that are useless
NUM_COLUMNS = Control_Data + Num_PerElet * EletPerLog;  % Number of Columns in Log
SENDER_DATA_FORMAT = repmat('%x ', 1, NUM_COLUMNS); % '%x' means HEX

% all participating nodes, including those do not tx/forward data
nodes = [];

%%
for fileIndex = 1:length(files)
    indexedFile = files(fileIndex).name;
    fid = fopen([srcDir DirDelimiter srcDir2 DirDelimiter srcDir3 indexedFile]);
    Raw_Data = fscanf(fid, SENDER_DATA_FORMAT,[NUM_COLUMNS inf]);
    Raw_Data = Raw_Data';
    disp (['Loading file ' indexedFile]);
    
    Entry_Num = size(Raw_Data, 1) * EletPerLog;
    
    if ~isempty(Raw_Data)
        Packet_Log = zeros(Entry_Num, Colmn_Packet);
        CurrentIndex = 0;
        for temp_Entry = 1:size(Raw_Data, 1)
            Current_R_Index = 0;
            for temp_Ele = 1:EletPerLog
                PL_Index = 1;
                for i = 1:1:Num_Tx        %TX/RX
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_Tx - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_Tx;
                PL_Index = PL_Index + 1;
%                 for index = 2:9
%                     Packet_Log((CurrentIndex + 1), index) = Raw_Data(temp_Entry, (index - 1) * 2 + 8 + (temp_Ele - 1) * 17) * power(16, 2)...
%                                                         + Raw_Data(temp_Entry, (index - 1) * 2 + 9 + (temp_Ele - 1) * 17) * power(16, 0);
%                 end

                for i = 1:1:Num_NodeID    %NodeID
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_NodeID - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_NodeID;
                PL_Index = PL_Index + 1;
                
                for i = 1:1:Num_SourceID  %SourceID
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_SourceID - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_SourceID;
                PL_Index = PL_Index + 1;
                
                for i = 1:1:Num_SeqNum    %Sequence Number
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_SeqNum - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_SeqNum + Num_LastSender + Num_LastSenderSeq + Num_LocalSeq + Num_LastNtwSeq + Num_LocalNtwSeq;   %Jump New Columns
                PL_Index = PL_Index + 1;
                
                for j = 1:1:MAX_FCS_SIZE        %FC Set
                    for i = 1:1:Num_FC_each
                        Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                            Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_FC_each - i) * 2);
                    end
                    Current_R_Index = Current_R_Index + Num_FC_each;
                    PL_Index = PL_Index + 1;
                end
                
                %%Adding New Columns
                Current_R_Index = Current_R_Index - MAX_FCS_SIZE * Num_FC_each - (Num_LastSender + Num_LastSenderSeq + Num_LocalSeq + Num_LastNtwSeq + Num_LocalNtwSeq);
                for i = 1:1:Num_LastSender    %Last Hop Sender
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_LastSender - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_LastSender;
                PL_Index = PL_Index + 1;
                
                for i = 1:1:Num_LastNtwSeq    %Last Hop Network Sequence Number
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_LastNtwSeq - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_LastNtwSeq;
                PL_Index = PL_Index + 1;
                
                for i = 1:1:Num_LastSenderSeq    %Last Hop MAC Sequence Number
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_LastSenderSeq - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_LastSenderSeq;
                PL_Index = PL_Index + 1;
                
                for i = 1:1:Num_LocalNtwSeq    %Local Network Sequence Number
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_LocalNtwSeq - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_LocalNtwSeq;
                PL_Index = PL_Index + 1;
                
                for i = 1:1:Num_LocalSeq    %Local MAC Sequence Number
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_LocalSeq - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_LocalSeq;
                PL_Index = PL_Index + 1;
                
                Current_R_Index = Current_R_Index + MAX_FCS_SIZE * Num_FC_each;   %Add back to the latest index
                
                for i = 1:1:Num_TimeStamp    %Time Stamp
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_TimeStamp - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_TimeStamp;
                PL_Index = PL_Index + 1;                
                
                % time for next retx
                for i = 1:1:Num_RetxTime
                    Packet_Log((CurrentIndex + 1), PL_Index) = Packet_Log((CurrentIndex + 1), PL_Index) +...
                                                        Raw_Data(temp_Entry, (temp_Ele - 1) * Num_PerElet + Control_Data + Current_R_Index + i) * power(16, (Num_RetxTime - i) * 2);
                end
                Current_R_Index = Current_R_Index + Num_RetxTime;
                PL_Index = PL_Index + 1;
                
                CurrentIndex = CurrentIndex + 1;
                
                Current_R_Index = 0;
            end
        end
        
        nodeId = Packet_Log(1, 2);
%         fprintf('#warning at line 221\n');
%         if mod(nodeId, 4) ~= 0
%             continue;
%         end
        save([num2str(nodeId) '.mat'], 'Packet_Log');

       % UART reliability
        uart_seq_gap = Packet_Log(end, end) - Packet_Log(1, end) + 1;
        uart_relis = [uart_relis; nodeId size(Packet_Log, 1) uart_seq_gap size(Packet_Log, 1) / uart_seq_gap];
        
        len = length(find(Packet_Log(:, 1) == TX_FAIL_FLAG));
        tx_fails(tx_fail_cnts : tx_fail_cnts + len - 1, :) = Packet_Log(Packet_Log(:, 1) == TX_FAIL_FLAG, :);
        tx_fail_cnts = tx_fail_cnts + len;

        len = length(find(Packet_Log(:, 1) == TX_SUCCESS_FLAG));
        tx_successes(tx_success_cnts : tx_success_cnts + len - 1, :) = Packet_Log(Packet_Log(:, 1) == TX_SUCCESS_FLAG, :);
        tx_success_cnts = tx_success_cnts + len;

        len = length(find(Packet_Log(:, 1) == TX_DONE_FLAG));
        txs(tx_cnts : tx_cnts + len - 1, :) = Packet_Log(Packet_Log(:, 1) == TX_DONE_FLAG, :);
        tx_cnts = tx_cnts + len;        

        len = length(find(Packet_Log(:, 1) == TX_DONE_FAIL_FLAG));
        tx_done_fails(tx_done_fail_cnts : tx_done_fail_cnts + len - 1, :) = Packet_Log(Packet_Log(:, 1) == TX_DONE_FAIL_FLAG, :);
        tx_done_fail_cnts = tx_done_fail_cnts + len;
        
        len = length(find(Packet_Log(:, 1) == RX_FLAG));
        rxs(rx_cnts : rx_cnts + len - 1, :) = Packet_Log(Packet_Log(:, 1) == RX_FLAG, :);
        rx_cnts = rx_cnts + len;
        
%         fprintf('#warning at line 249\n');
        debug = Packet_Log(Packet_Log(:, 1) == DEBUG_FLAG, :);
        %debug(debug(:, 3) == 17 & debug(:, 4) == 339, :) = [];
        len = size(debug, 1);
        debugs(debugs_cnts : debugs_cnts + len - 1, :) = debug;
        debugs_cnts = debugs_cnts + len;
        
        %[pathstr, prename, ext, versn] = fileparts(indexedFile);
        [pathstr, prename, ext] = fileparts(indexedFile);
%         save([srcDir DirDelimiter srcDir2 DirDelimiter srcDir3 num2str(nodeId) '.mat'], 'Packet_Log'); %, 'Unique_Packet_Log');
%         disp (['Done with ' indexedFile ', go to next']);
%     else
%         disp (['File ' indexedFile ' is empty, go to next']);
    end
    fclose(fid);
end
% remove unused entries
tx_fails(tx_fail_cnts : end, :) = [];
tx_successes(tx_success_cnts : end, :) = [];
txs(tx_cnts : end, :) = [];
tx_done_fails(tx_done_fail_cnts : end, :) = [];
rxs(rx_cnts : end, :) = [];
debugs(debugs_cnts : end, :) = [];
save('debugs.mat', 'debugs');
% save('debugs.mat', '-v7.3', 'debugs');
save('txrxs.mat', 'txs', 'tx_fails', 'tx_done_fails', 'tx_successes', 'rxs', 'uart_relis');
%% compute link reliability
% fprintf('changed format\n');
% SRC_IDX = 4;
% DST_IDX = 3;
% SEQ_IDX = 10;
SRC_IDX = 3;
DST_IDX = 3;
SEQ_IDX = 4;

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
save('link_pdrs.mat', 'link_pdrs');
fprintf('min uart reliability: %f\n', min(uart_relis(:, end)));
%% debug large skew
% t = debugs;
% t = t(t(:, 3) == 15 & t(:, 4) == 0, :);
% root = mode(t(:, 5));
% s = t(t(:, 2) == root, :);
% fprintf('job %d: skew %d, root %d, root table size %d, root skew %f\n', job_id, mode(t(:, 10)), root, mode(s(:, 6)), mode(s(:, 10)));
end
%% 
if INITIAL_LE_PKT_CNT > 0
    fprintf('caution: has to be for PRKS!\n');
end
%%
clear;
load debugs;
load link_pdrs;
load txrxs;
%% reset check
load txrxs;
t = uart_relis;
t = t(t(:, end) > 1, :);
nodes = t(:, 1);
for i = 1 : size(nodes, 1)
%     clc;
    node = nodes(i);
    load([num2str(node) '.mat']);
    s = Packet_Log;
    plot(s(:, [9 11]));
    s = Packet_Log(:, end);
    title(num2str(node));
    diff = s(2:end) - s(1:end-1);
    idx = find(diff < -1);
    tmp = [Packet_Log(idx, :); Packet_Log(idx + 1, :)];
    fprintf('error: node %d reset!!\n', node);
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

%% %% assert
load debugs;
t = debugs;
t = t(t(:, 3) == DBG_ERR_FLAG, :);
% fprintf('assertion:\n');
s = unique(t(:, 4));
for i = 1 : length(s)
    fprintf('assertion %d: %d\n', s(i), sum(t(:, 4) == s(i)));
end
fprintf('\n');

%%
%hold on;
if ~isempty(link_pdrs)
    cdfplot(link_pdrs(:, end));
end