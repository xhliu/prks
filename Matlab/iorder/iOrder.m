%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   11/6/2013
%   Function: iOrder scheduling: does all links' SNR w/o interference have to be above
%   threshold to work?
%%
% @param signal_map & snr_threshold: in dB
% @param node_noise & tx_power: in dBm
% all sets are represented w/ indices relative in 'link'
function schedule = iOrder(link, link_queue_len, signal_map, node_noise, snr_threshold, tx_power)
    % preprocessing: convert dB and dBm
    signal_map = 10 .^ (signal_map / 10);
    node_noise = 10 .^ (node_noise / 10);
    snr_threshold = 10 ^ (snr_threshold / 10);
    tx_power = 10 ^ (tx_power / 10);
    
    MAX_SCHEDULE_LEN = 1000000;
    schedule = cell(MAX_SCHEDULE_LEN, 1);
    schedule_len = 1;
    residue_link = (1 : size(link, 1))';
    
    while ~isempty(residue_link)
        [longest_queue longest_queue_idx] = max(link_queue_len);
        % remove the start link
        tmp = residue_link;
        tmp(tmp == longest_queue_idx) = [];
        slot_schedule = maxConcurrentSet(longest_queue_idx, tmp, link, signal_map, node_noise, snr_threshold, tx_power);
        
        % append
        fprintf('processing slot %d\n', schedule_len);
        schedule{schedule_len} = slot_schedule;
        schedule_len = schedule_len + 1;
        
        % 1 packet scheduled
        for i = 1 : length(slot_schedule)
            link_idx = slot_schedule(i);
            link_queue_len(link_idx) = link_queue_len(link_idx) - 1;
            % all packets scheduled
            if link_queue_len(link_idx) == 0
                residue_link(residue_link == link_idx) = [];
            end
        end
    end
    
    schedule(schedule_len : end) = [];    
end