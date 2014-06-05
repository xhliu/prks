%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   11/6/2013
%   Function: iOrder scheduling w/ one slot, i.e., find max concurrent set
%   for each link
%%
% @param signal_map & snr_threshold: in dB
% @param node_noise & tx_power: in dBm
% all sets are represented w/ indices relative in 'link'
function [schedule thruput] = iOrderOneSlot(link, signal_map, node_noise, snr_threshold, tx_power, packet_len)
    % preprocessing: convert dB and dBm
    signal_map = 10 .^ (signal_map / 10);
    node_noise = 10 .^ (node_noise / 10);
    snr_threshold = 10 ^ (snr_threshold / 10);
    tx_power = 10 ^ (tx_power / 10);
    
    len = size(link, 1);
    schedule = cell(len, 1);
    thruput = zeros(len, 1);
    all_link = (1 : len)';
    
    for i = 1 : len
        % remove the start link
        residual_link = all_link;
        residual_link(residual_link == i) = [];
        slot_schedule = maxConcurrentSet(i, residual_link, link, signal_map, node_noise, snr_threshold, tx_power);
        
        % append
%         fprintf('processing link %d\n', i);
        schedule{i} = slot_schedule;
        
        [is_concurrent slot_thruput] = passConcurrencyTest(slot_schedule);
        thruput(i) = slot_thruput;
    end
    
    % copy from scream.m
    % @is_passed: can the set be concurrent?
    % @cs_thruput: concurrent set's thruput in a slot = \sum {link pdr};
    % only valid when is_passed is true
    function [is_passed, cs_thruput] = passConcurrencyTest(concurrent_set)
        cs_thruput = 0;
        % each link
        for m = 1 : length(concurrent_set)
            sender = link(concurrent_set(m), 1);
            receiver = link(concurrent_set(m), 2);
            path_loss = signal_map(sender, receiver);
            if isnan(path_loss) || isinf(path_loss)
                %fprintf('link attenuation unknown, skip\n');
                continue;
            end
            signal = tx_power / path_loss;
            noise = node_noise(receiver);

            % each interferer
            interference_sum = 0;
            for n = 1 : length(concurrent_set)
                if n == m
                    continue;
                end
                % from interferer
                interferer = link(concurrent_set(n), 1);
                interferer_receiver = link(concurrent_set(n), 2);
                % special case: primary interference
                if sender == interferer || sender == interferer_receiver ...
                       || receiver == interferer || receiver == interferer_receiver
                    is_passed = false;
                    return;
                end
                
                path_loss = signal_map(interferer, receiver);
                % unknown as too far away 
                if isnan(path_loss) || isinf(path_loss)
                    interference = 0;
                else
                    interference = tx_power / path_loss;
                end
                interference_sum = interference_sum + interference;
            end
            
            snr = signal / (interference_sum + noise);
            snr_db = 10 * log10(snr);
            pdr = snr2Pdr(snr_db, packet_len);
            cs_thruput = cs_thruput + pdr;    
            
%             % treat concurrent set of size 1 as special case
%             if 1 == length(concurrent_set)
%                 is_passed = true;
%                 return;
%             end
%             
%             if is_centralized
%                 if snr_db < snr_threshold
%                     is_passed = false;
%                     return;
%                 end
%             else
%                 if rand > pdr
%                     is_passed = false;
%                     return;
%                 end
%             end
        end
        % all links' tx succeed
        is_passed = true;
    end
end