%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/25/2014
%   Function: compute link snr of prks based on concurrent nodes, signal
%   map, and noise; to determine SNR threshold for iOrder comparison
%%
% @param: link_active_slot [link slot_this_link_is_active], link is just
% sender for sender has just 1 receiver
% obtained from RRKS measurement
% @param: signal_map: in dB
% @param: node_noise & tx_power: in dBm
% @return: link's snr time series for each link activation
% all sets are represented w/ indices relative in 'link'
function link_snrs = calcPrksSnr(link, link_active_slot, signal_map, node_noise, tx_power)
    len = size(link, 1);
    link_snrs = cell(len, 1);
    t = link_active_slot;
    
    % each link
    for i = 1 : len
        sender = link(i, 1);
        receiver = link(i, 2);
        signal = tx_power - signal_map(sender, receiver);
        % this link is active in these slots
        slots = t(t(:, 1) == sender, end);
        
        % each tx along this link
        % [slot, # of concurrent links, signal, noise+interference, snr]
        slot_snrs = zeros(size(slots, 1), 5);
        for j = 1 : size(slots, 1)
            slot = slots(j);
            fprintf('link %d, slot %d\n', i, slot);
            % exclude sender itself
            concurrent_nodes = t(t(:, end) == slot & t(:, 1) ~= sender, 1);
            
            interference_sum = -inf;
            for k = 1 : size(concurrent_nodes, 1)
                interference = tx_power - signal_map(concurrent_nodes(k), receiver);
                interference_sum = dBmSum(interference_sum, interference);
            end
            ni = dBmSum(interference_sum, node_noise(receiver));
            snr = signal - ni;
            %slot_snrs(j, :) =  [slot, size(concurrent_nodes, 1) - 1, signal, ni, snr];
            slot_snrs(j, :) =  [slot, size(concurrent_nodes, 1), signal, ni, snr];
        end
        link_snrs{i} = slot_snrs;
    end
    
    % sum of dBm
    function z = dBmSum(x, y)
        z = 10 ^ (x / 10) + 10 ^ (y / 10);
        z = 10 * log10(z);
    end
end