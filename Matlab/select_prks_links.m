%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   5/23/2013
%   Function: select links meeting pdr req. in the absence of interference
%   Updated: 6/25/13 consider link snr, i.e., rssi, as well
%   Updated: 7/23/13 select reliable link based on link SNR and PDR w/o interference
%                and then construct min hop tree after pruning unreliable links
%% each iteration
%jobs = [19235 19236 19237 19238 19239 19248 19251 19258 19260];
%jobs = [21890:2:21898 21900 21903 21904 21907:21910 21915 21916 21918 21919 21921 21926 21928 21937];
jobs = [22537 : 2 : 22543 22546 22559];
MAIN_DIR = '~/Downloads/Jobs/';
% MAIN_DIR = '~/Projects/tOR/RawData/';
for job_id = 1 : length(jobs)
    job = jobs(job_id);
    fprintf('processing job %d\n', job);
    dest = [MAIN_DIR num2str(job)];
    cd(dest);
%% groundtruth pdr & snr
load txrxs;
SRC_IDX = 3;
RSSI_IDX = 10;
RSSI_OFFSET = 45;

nodes = unique(txs(:, 2));
len = size(nodes, 1);
% entry (i, j) stores link <i, j> pdr
link_pdrs = repmat(nan, len, len);
% receiver rssi
link_rssi = repmat(nan, len, len);

for i = 1 : len
   src = nodes(i);  
   tx_cnt = sum(txs(:, 2) == src);
   rx = rxs(rxs(:, SRC_IDX) == src, :);
   
   for j = 1 : len
       if j == i
           continue;
       end
       dst = nodes(j);

       rx_cnt = sum(rx(:, 2) == dst);

       link_pdrs(i, j) = rx_cnt / tx_cnt;

       if i == 1 && j == 1
           fprintf('change according to raw RSSI log\n');
       end
       % indriya
       rssi = rx(rx(:, 2) == dst, RSSI_IDX) - 2 ^ 32 - RSSI_OFFSET;
       % neteye
       %rssi = rx(rx(:, 2) == dst, RSSI_IDX) - 256;
       if isempty(rssi)
           continue;
       end
%        plot(rssi);
%        title(num2str(median(rssi)));
       link_rssi(i, j) = median(rssi);
   end
end

%
save('link_pdr_rssi.mat', 'link_pdrs', 'link_rssi', 'nodes');
%%
end

%% %% link selection 
%% 1) 
load link_pdr_rssi;
% constraints: implicit, one node at most one outgoing link
MAX_ACTIVE_LINK_SIZE = inf; %100;
MAX_INCIDENT_LINK_SIZE = 2;
% job 19976 generates exactly 100 links
MIN_PDR = 0.99;

links = [];

for i = 1 : len
   for j = 1 : len
       if j == i
           continue;
       end
       if link_pdrs(i, j) >= MIN_PDR
           % node i does not have to be i-th node
           links = [links; nodes(i) nodes(j) link_pdrs(i, j)];
           s = [links(:, 1); links(:, 2)];
           [cnt element] = hist(s, unique(s));
           %
           if max(cnt) <= MAX_INCIDENT_LINK_SIZE
               break;
           else
               links(end, :) = [];
           end
       end
   end
   if size(links, 1) >= MAX_ACTIVE_LINK_SIZE
       break;
   end
end


%% 2) factor in RSSI
% choose link w/ max RSSI to break ties
MAX_ACTIVE_LINK_SIZE = inf; %100;
MAX_INCIDENT_LINK_SIZE = 2; %4;
MIN_PDR = 0.95;

links = [];

for i = 1 : len
    % pdr
    IX = find(link_pdrs(i, :) >= MIN_PDR);
    if isempty(IX)
        continue;
    end
    
    % choose the receiver w/ max RSSI
    max_rssi = max(link_rssi(i, IX));
    j = find(link_pdrs(i, :) >= MIN_PDR & link_rssi(i, :) == max_rssi, 1);
    
       % node i does not have to be i-th node
       links = [links; nodes(i) nodes(j) link_pdrs(i, j) link_rssi(i, j)];
       s = [links(:, 1); links(:, 2)];
       [cnt element] = hist(s, unique(s));
       %
       if max(cnt) > MAX_INCIDENT_LINK_SIZE
           links(end, :) = [];
       end


   if size(links, 1) >= MAX_ACTIVE_LINK_SIZE
       break;
   end
end

%%
save('links_indriya_iorder.mat', 'links');
%% format
clc;
% links = node_parent;
% to select a smaller topology, for debug purpose
MAX_NODE_ID = 130;  % for control signalling TDMA to work
for i = 1 : size(links, 1)
    if links(i, 1) < MAX_NODE_ID && links(i, 2) < MAX_NODE_ID
        fprintf('{%d, %d}, ', links(i, 1), links(i, 2));
    end
end
% disp('\n');
% 5 * 5
% for i = 1 : size(links, 1)
%     if mod(links(i, 1), 15) > 10 && links(i, 2) > 10
%         fprintf('{%d, %d}, ', links(i, 1), links(i, 2));
%     end
% end

%% %% construct collection tree based on SNR threshold w/o interference
%% node noise
RSSI_OFFSET = 127 + 45;
load debugs;
t = debugs;

nodes = unique(t(:, 2));
% [node, noise]
node_noise = zeros(length(nodes), 2);
% each sender
for i = 1 : length(nodes)
    node = nodes(i);
    s = t(t(:, 2) == node, 10) - RSSI_OFFSET;
    %hist(s);
    node_noise(i, :) = [node median(s)];
end
save('node_noise.mat', 'node_noise');


%% link SNR
% neteye: in job 18654/19012/19936
% indriya: 46644
load node_noise.mat;
%% neteye: in job 18651/19011/19935 of tx power 0 dBm
% indriya: 46654
load link_pdr_rssi.mat;
TX_POWER_OFFSET = 0; % - (-25);
% power level 31
TX_POWER_DBM = 0;

% signal and SNR when tx power is -25 dBm
link_signal = nan(size(link_pdrs));
signal_map = nan(size(link_pdrs));
link_snr = nan(size(link_pdrs));
for i = 1 : length(nodes)
    % i-th column, i.e., receiver
    node = nodes(i);
    
    % look up noise
    noise = node_noise(node_noise(:, 1) == node, 2);
    if isempty(noise)
        fprintf('node %d noise not found\n', node);
        continue;
    end
    
    % signal = rssi - noise
    % convert from dBm to mW first
    link_signal_mW = 10 .^ (link_rssi(:, i) / 10) - 10 .^ (noise / 10);
    link_signal_dBm = 10 * log10(link_signal_mW);
    link_signal(:, i) = link_signal_dBm - TX_POWER_OFFSET;
    
    % link attenuation
    signal_map(:, i) = TX_POWER_DBM - link_signal(:, i);
    
    % SNR
    link_snr(:, i) = link_signal(:, i) - noise;
end
fprintf('unknown link path loss ratio %f\n', (sum(sum(isinf(signal_map) | isnan(signal_map))) - length(nodes)) / length(nodes) ^ 2);
save('link_snr_gain.mat', 'nodes', 'link_snr', 'signal_map');

%% select reliable links, in terms of both SNR and PDR
PDR_THRESHOLD = 0.9;
% SNR corresponding to PDR_THRESHOLD
GAMMA_T = 4;
GAMMA_B = 1;
SNR_THRESHOLD = GAMMA_T + GAMMA_B;
connectivity = (link_snr > SNR_THRESHOLD);

% additional PDR filter bcoz of pdr_vs_sinr error
% tx power 3 job 18646
load link_pdr_rssi.mat;
fprintf('assuming nodes involved are identical, including order\n');
reliable_connectivity = (link_pdrs > PDR_THRESHOLD);

connectivity = connectivity & reliable_connectivity;
bar(sum(connectivity));
% connectivity matrix
save('connectivity.mat', 'nodes', 'connectivity');

%% prune the connectivity graph to reduce degree
MAX_DEGREE = 4;
MIN_LINK_SIZE = 40;
MAX_LINK_SIZE = inf;
for k = 1 : 10000
    fprintf('round %d\n', k);
    load long_term_connectivity;
    % remove prob.
    p = 0.5;
    for i = 1 : size(connectivity, 1)
        if rand() < p
            % remove
            connectivity(i, :) = 0;
            connectivity(:, i) = 0;
        end
    end

    % min hop tree
    % tmp = [];
    %for j = 1 : length(nodes)
    BASESTATION = 15; % nodes(j);
    % convert connectivity matric into link cost matrix
    link_cost = ones(size(connectivity));
    % no existent link
    link_cost(~connectivity) = inf;
    % parent is index, not necessarily equal to node id
    [hopcount, parent] = dijkstra(link_cost, BASESTATION, size(link_cost, 1));

    % final tree
    node_parent = [];
    for i = 1 : length(nodes)
        node = nodes(i);

        if node == BASESTATION
            ;%node_parent = [node_parent; node node];
        else
            if ~isnan(parent(i))
                node_parent = [node_parent; node nodes(parent(i))];
            end
        end
    end
    save('node_parent.mat', 'node_parent');

    if isempty(node_parent)
        continue;
    end
    % node degree
    links = node_parent;
    if size(links, 1) < MIN_LINK_SIZE || size(links, 1) > MAX_LINK_SIZE
        continue;
    end
    node = [links(:, 1); links(:, 2)];
    [cnt x] = hist(node, unique(node));
    xcnt = [x cnt'];
    fprintf('max degree: %d\n', max(cnt));
    if max(cnt) <= MAX_DEGREE
        save('long_term_tree.mat', 'node_parent');
        fprintf('tree found\n');
        return;
    end
    % hist(cnt);
    % tmp = [tmp; nodes(j) max(cnt)];
end
fprintf('tree not found\n');


%% %% select node tx power to ensure SNR_THRESHOLD
% any valid pkrs job, e.g., 19007
% lama
% load ~/Dropbox/iMAC/Xiaohui/links100.mat;
% olama
load ~/Dropbox/iMAC/Xiaohui/links_olama.mat;
% job 19021/20768, power level 31 
load link_snr_gain;
% ~98%
SNR_THRESHOLD = 10;
DEFAULT_TX_POWER = -25;
MIN_POWER_LEVEL = 3;
% based on link_snr for a given tx power, i.e., 0 dBm
LINK_SNR_TX_POWER = 0;
% from level 0 to 31
level_power_table = [-38, -33, -29, -25, -22, -19, -17, -15, -13, -12, -11, -10, -9, -8, -8, -7, -6, -6, -5, -5, -5, -4, -4, -3, -2, -2, -1, -1, -1, 0, 0, 0];
link_tx_power_level = zeros(size(links, 1), 4);
for i = 1 : size(links, 1)
    sender = links(i, 1);
    receiver = links(i, 2);
    fprintf('%d-th link: %d -> %d\n', i, sender, receiver);
    tx_idx = find(nodes == sender);
    rx_idx = find(nodes == receiver);
    
    %  tx_0 - snr_0 = tx_1 - snr_1 = attenuation + noise
    % tx_1 = tx_0 - snr_0 + snr_1
    if isempty(tx_idx) || isempty(rx_idx)
        tx_power = DEFAULT_TX_POWER;
    else
        tx_power = LINK_SNR_TX_POWER - link_snr(tx_idx, rx_idx) + SNR_THRESHOLD;
    end
    
    len = length(level_power_table);
    tx_power_level = len - 1;
    for j = 1 : len
        if tx_power <= level_power_table(j)
            tx_power_level = j - 1;
            % lower than 3 can cause disconnectivity; this occurs bcoz of
            % modelling error
            if tx_power_level < MIN_POWER_LEVEL
                tx_power_level = MIN_POWER_LEVEL;
            end
            break;
        end
    end
    link_tx_power_level(i, :) = [sender receiver tx_power tx_power_level];
end
save('link_tx_power_level.mat', 'link_tx_power_level');
%% format
clc;
t = link_tx_power_level;
for i = 1 : size(t, 1)
    % sender receiver tx_power_level
    fprintf('{%d, %d, %d}, ', t(i, 1), t(i, 2), t(i, 4));
end


%% %% select long-term reliable links
% this works even nodes change over jobs
PDR_THRESHOLD = 0.97;
NODE_NUM = 130;
connectivity = ones(NODE_NUM, NODE_NUM);

% jobs = [19235 19236 19237 19238 19239 19248 19251 19258 19260];
% jobs = [21811 21814 21832 21833 21836 21838];
% jobs = [21890:2:21898 21900 21903 21904 21907:21910 21915 21916 21918 21919 21921 21926 21928 21937];
% compare PRKS against SCREAM
jobs = [22537 : 2 : 22543 22546 22559];

for k = 1 : length(jobs)
    job = jobs(k);
    fprintf('processing job %d\n', job);
    dest = [MAIN_DIR num2str(job)];
    cd(dest);

%% check each job
load link_pdr_rssi;
for i = 1 : NODE_NUM
    tx_idx = find(nodes == i);
    if isempty(tx_idx)
        % conservative
        connectivity(i, :) = 0;
    end

    for j = 1 : NODE_NUM
        fprintf('%d -> %d\n', i, j);
        if 0 == connectivity(i, j)
            continue;
        end
        
        rx_idx = find(nodes == j);
        
        if ~isempty(rx_idx)
            % only connected if all jobs say so
            if link_pdrs(tx_idx, rx_idx) < PDR_THRESHOLD
                connectivity(i, j) = 0;
            end
        else
            % conservative
            connectivity(:, j) = 0;            
        end
    end
end

%%
end
%%
%%
nodes = 1 : NODE_NUM;
save('long_term_connectivity.mat', 'connectivity', 'nodes');
%% format
links = [];
t = connectivity;
len = size(t, 1);
for i = 1 : len
   fprintf('processing node %d \n', i);
    for j = 1 : len
       if j == i
           continue;
       end
       if t(i, j)
           links = [links; i j];
           % at most one outgoing link
           break;
       end
   end
end

%%
% constraints: implicit, one node at most one outgoing link
MAX_ACTIVE_LINK_SIZE = 100;
MAX_INCIDENT_LINK_SIZE = 2;
% job 19976 generates exactly 100 links
MIN_PDR = 0.99;

t = connectivity;
len = size(t, 1);
links = [];

for i = 1 : len
   for j = 1 : len
       if j == i
           continue;
       end
       if t(i, j)
           % node i does not have to be i-th node
           links = [links; i j];
           s = [links(:, 1); links(:, 2)];
           [cnt element] = hist(s, unique(s));
           %
           if max(cnt) <= MAX_INCIDENT_LINK_SIZE
               break;
           else
               links(end, :) = [];
           end
       end
   end
   if size(links, 1) >= MAX_ACTIVE_LINK_SIZE
       break;
   end
end
