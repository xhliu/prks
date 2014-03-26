%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   11/6/2013
%   Function: schedule using iOrder
%% 
snr_threshold_concurrency_median_mean = [];
% snr_thresholds = [5 7 9 10];
snr_thresholds = 1 : 10;
% for snr_threshold = 1 : 10 %
len = length(snr_thresholds);
iorder = cell(len, 1);
for j = 1 : len
%% 
% measurement: 2.8 3.2 4 4.5
% simulation: 6.6 7.6 9 10.3
% fit for neteye: 5 7 9 10
SNR_THRESHOLD = snr_thresholds(j);
% for evacuation
% QUEUE_LEN = 200;
%% 

% link_queue_len = ones(size(link_set, 1), 1) * QUEUE_LEN;
%% compute schedule
% schedule = iOrder(link_set, link_queue_len, signal_map, noises, SNR_THRESHOLD, TX_POWER);
schedule = iOrderOneSlot(link_set, signal_map, noises, SNR_THRESHOLD, TX_POWER);
%% statistics
concurrency = size(length(schedule), 1);
scheduled_link = zeros(1000000, 1);
idx = 1;
for i = 1 : length(schedule)
    len = length(schedule{i});
    concurrency(i) = len;
    scheduled_link(idx : idx + len - 1) = schedule{i};
    idx = idx + len;
end
% sanity check: ahould all be equal to QUEUE_LEN for iOrder()
scheduled_link(idx : end) = [];
% hist(scheduled_link, size(link_set, 1));
% figure;
% hold on;
% cdfplot(concurrency);
% empty concurrent set since the link of interest is unknown
concurrency(concurrency == 0) = [];
iorder{j} = concurrency;
fprintf('concurrency %f, %f\n', median(concurrency), mean(concurrency));
% snr_threshold_concurrency_median_mean = [snr_threshold_concurrency_median_mean; snr_threshold median(concurrency), mean(concurrency)];
end
%%
% iorder_concurrency = concurrency;
% save('iorder_concurrency.mat', 'iorder_concurrency');
% save('one_slot_schedule.mat', 'schedule');
% legend({'70', '80', '90', '95'});
% legend({'70 iOrder', '80 iOrder', '90 iOrder', '95 iOrder', '70 ns3', '80 ns3', '90 ns3', '95 ns3'});

%% simulation
% concurrency = concurrency95;
% cdfplot(concurrency);
% fprintf('concurrency %f, %f\n', median(concurrency), mean(concurrency));
