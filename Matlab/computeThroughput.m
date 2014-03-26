%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/12/2014
%   Function: compute throughput
%% SCREAM
JOB_MINS = 60;

ACTIVE_LINK_SIZE = 100; %59; %100;
INTERFERENCE_DIAMETER = 3; %8; % 3;

ROUND_LEN = 1 + INTERFERENCE_DIAMETER + 1;
FRAME_LEN = ROUND_LEN * ACTIVE_LINK_SIZE;
SLOT_LEN = 32;
BOOTSTRAP_SECONDS = (FRAME_LEN * 4 * 2 + FRAME_LEN * ACTIVE_LINK_SIZE) * SLOT_LEN / 1024;
fprintf('BOOTSTRAP_SECONDS %d\n', BOOTSTRAP_SECONDS);
%
SCALE = 128 / 100 * 32 / 5;
load link_pdrs;

thruput = sum(link_pdrs(:, 4)) / (JOB_MINS * 60 - BOOTSTRAP_SECONDS) * SCALE;
fprintf('throughput %f\n', thruput);

%% PRKS
BOOTSTRAP_SECONDS = 500;
SCALE = 512 / 5;
JOB_MINS = 240;

%% RIDB
% not 900 
BOOTSTRAP_SECONDS = 500;
SCALE =  1.28 * 32 / 5;

%% RIDB_OLAMA
BOOTSTRAP_SECONDS = 500;
SCALE = 512 / 5;

%% CMAC
BOOTSTRAP_SECONDS = 420;
SCALE = 1;
JOB_MINS = 180;

%%
load link_pdrs;

thruput = sum(link_pdrs(:, 4)) / (JOB_MINS * 60 - BOOTSTRAP_SECONDS) * SCALE;
fprintf('throughput %f\n', thruput);


%% %% PRKS after convergence
load txrxs;

TIMESTAMP_IDX = 10;
SLOT_WRAP_LEN = 2 ^ 32;
MIN_GAP = SLOT_WRAP_LEN / 2;

t = rxs;
nodes = unique(t(:, 2));
for j = 1 : length(nodes)
    node_ix = (t(:, 2) == nodes(j));
    s = t(node_ix, TIMESTAMP_IDX);
%     % sanity check if time is increasing
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

%%
BOOTSTRAP_SECONDS = 3600 * 3;
% us
BOOTSTRAP_TIME = BOOTSTRAP_SECONDS * 2 ^ 20;
JOB_MINS = 180;
SCALE = 512 / 5;

s = t(t(:, TIMESTAMP_IDX) > BOOTSTRAP_TIME, :);
thruput = size(s, 1) / (JOB_MINS * 60 - BOOTSTRAP_SECONDS) * SCALE;
fprintf('throughput %f\n', thruput);

%% 
load link_pdrs;
load txrxs;
% sanity
fprintf('# of rxs %d, %d, %f\n', sum(link_pdrs(:, 4)), size(rxs, 1), sum(link_pdrs(:, 4)) / size(rxs, 1));
%%
min(uart_relis(:, end))
length(unique(link_pdrs(:, 2)))
length(unique(rxs(:, 2)))

%% correlation btw. # of tx and pdr of a link
% to see if reliable links tx more
cd ~/Projects/tOR/RawData/20793;
load link_pdrs;
corrcoef(link_pdrs(:, 3), link_pdrs(:, 5))
%%
data = cell(3, 1);
data{1} = [0.48
0.51

0.44
0.45

0.41
0.39

0.39
0.4];

data{2} = [
   0.45
0.46
0.42
0.42
0.47
0.35];

data{3} = [
0.02

0.1
0.13

0.28

0.26];