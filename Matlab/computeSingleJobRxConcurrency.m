%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute throughput in each slot

%% to cope w/ slot/time wrap around
SLOT_LEN = 32;
fprintf('slot length %d\n', SLOT_LEN);

%% 
load txrxs;
% time in us
SLOT_WRAP_LEN = 2 ^ 32;
TIMESTAMP_IDX = 10;

%% time wrap around
t = unwrap(rxs, SLOT_WRAP_LEN);
s = floor(t(:, TIMESTAMP_IDX) / (SLOT_LEN * 1024));
%% 
[c e] = hist(s, unique(s));
ce = [e c'];

%% figure;
rx_concurrency = ce(:, end);
% sanity check rx concurrency
load link_pdrs;
fprintf('total concurrency %d, total packets received %d, ratio: %f\n', sum(rx_concurrency), sum(link_pdrs(:, 4)), sum(rx_concurrency) / sum(link_pdrs(:, 4)));
% save('rx_concurrency.mat', 'rx_concurrency');
cdfplot(rx_concurrency);
fprintf('rx_concurrency median %f, mean %f\n', median(rx_concurrency), mean(rx_concurrency));


%%
% load rx_concurrency;
% s = rx_concurrency;
% % s = s(end - 30000 : end);
% plot(s);
% mean(s)
% median(s)
% mean(rx_concurrency) * 1024 / 5