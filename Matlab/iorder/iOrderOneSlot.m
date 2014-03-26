%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   11/6/2013
%   Function: iOrder scheduling w/ one slot, i.e., find max concurrent set
%   for each link
%%
% @param signal_map & snr_threshold: in dB
% @param node_noise & tx_power: in dBm
% all sets are represented w/ indices relative in 'link'
function schedule = iOrderOneSlot(link, signal_map, node_noise, snr_threshold, tx_power)
    % preprocessing: convert dB and dBm
    signal_map = 10 .^ (signal_map / 10);
    node_noise = 10 .^ (node_noise / 10);
    snr_threshold = 10 ^ (snr_threshold / 10);
    tx_power = 10 ^ (tx_power / 10);
    
    len = size(link, 1);
    schedule = cell(len, 1);
    all_link = (1 : len)';
    
    for i = 1 : len
        % remove the start link
        residual_link = all_link;
        residual_link(residual_link == i) = [];
        slot_schedule = maxConcurrentSet(i, residual_link, link, signal_map, node_noise, snr_threshold, tx_power);
        
        % append
%         fprintf('processing link %d\n', i);
        schedule{i} = slot_schedule;
    end
end