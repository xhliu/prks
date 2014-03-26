%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute throughput in each slot; mainly to compare PRKS w/
%   SCREAM pdr req w/ SCREAM
%% to cope w/ slot/time wrap around
SLOT_LEN = 32; %512; %512;
fprintf('slot length %d\n', SLOT_LEN);

%% 
load txrxs;
% time in us
SLOT_WRAP_LEN = 2 ^ 32;
TIMESTAMP_IDX = 10;

%% time wrap around
t = unwrap(rxs, SLOT_WRAP_LEN);
t(:, TIMESTAMP_IDX) = floor(t(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));


%%
SRC_IDX = 3;
% DST_IDX = 2;
MIN_SAMPLE_CNT = 1000;

% random sampling each link
% unique sender
senders = unique(t(:, SRC_IDX));
% [sender slot]
sampled_t = zeros(size(t, 1), 2);
idx = 1;
for i = 1 : length(senders)
    sender = senders(i);    
    
    sender_slots = t(t(:, SRC_IDX) == sender, TIMESTAMP_IDX);
    len = length(sender_slots);
    %rx_concurrency = zeros(len, 1);
    if len < MIN_SAMPLE_CNT
        continue;
    end
    
    tmp = zeros(MIN_SAMPLE_CNT, 2);
    for j = 1 : MIN_SAMPLE_CNT
        len = length(sender_slots);
        % random sampling
        ix = ceil(len * rand);
        tmp(j, :) = [sender sender_slots(ix)];
        sender_slots(ix) = [];
    end
    
    sampled_t(idx : idx + MIN_SAMPLE_CNT - 1, :) = tmp;
    idx = idx + MIN_SAMPLE_CNT;
end
sampled_t(idx : end, :) = [];

s = sampled_t(:, 2);
% save('rx_concurrencys.mat', 'rx_concurrencys');

% t = sampled_t;
% % unique sender
% senders = unique(t(:, 1));
% rx_concurrencys = zeros(length(senders), 3);
% for i = 1 : length(senders)
%     sender = senders(i);    
%     sender_slots = t(t(:, SRC_IDX) == sender, 2);
%     len = length(sender_slots);
%     rx_concurrency = zeros(len, 1);
%     
%     % each slots
%     for j = 1 : len
%         fprintf('link %d slot %d\n', i, j);
%         slot = sender_slots(j);
%         
%         rx_concurrency(j) = sum(t(:, 2) == slot);
%     end
% %     cdfplot(rx_concurrency);
%     rx_concurrencys(i, :) = [sender len mean(rx_concurrency)];
% end

% %%
% SRC_IDX = 3;
% DST_IDX = 2;
% 
% % unique sender
% senders = unique(t(:, SRC_IDX));
% rx_concurrencys = zeros(length(senders), 3);
% for i = 1 : length(senders)
%     sender = senders(i);    
%     sender_slots = t(t(:, SRC_IDX) == sender, TIMESTAMP_IDX);
%     len = length(sender_slots);
%     rx_concurrency = zeros(len, 1);
%     
% %     % each slots
% %     for j = 1 : len
% %         fprintf('link %d slot %d\n', i, j);
% %         slot = sender_slots(j);
% %         
% %         rx_concurrency(j) = sum(t(:, TIMESTAMP_IDX) == slot);
% %     end
% %     cdfplot(rx_concurrency);
%     rx_concurrencys(i, :) = [sender len mean(rx_concurrency)];
% end
% % save('rx_concurrencys.mat', 'rx_concurrencys');
% 
% %%
% s = rx_concurrencys;
% s = s(:, 2);
% cdfplot(s);
% mean(s)

%% 
[c e] = hist(s, unique(s));
ce = [e c'];

%% figure;
rx_concurrency = ce(:, end);
% sanity check rx concurrency
load link_pdrs;
fprintf('total concurrency %d, total packets received %d, ratio: %f\n', sum(rx_concurrency), sum(link_pdrs(:, 4)), sum(rx_concurrency) / sum(link_pdrs(:, 4)));
save('rx_concurrency_sample.mat', 'rx_concurrency');
cdfplot(rx_concurrency);
fprintf('rx_concurrency median %f, mean %f\n', median(rx_concurrency), mean(rx_concurrency));


%%
% load rx_concurrencys;
% s = rx_concurrencys;
% % s = s(end - 30000 : end);
% plot(s);
% mean(s)
% median(s)
% mean(rx_concurrency) * 1024 / 5

