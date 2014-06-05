%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   11/6/2013
%   Function: iOrder scheduling: does all links' SNR w/o interference have to be above
%   threshold to work?
%   Updated: 3/23/14, treat unknown path loss as inf path loss as is the
%   case for Indriya
%%
% find from the candidate set links that can be concurrent w/ start_link
% @param: both are indices
function slot_concurrent_set = maxConcurrentSet(start_link, candidate_set, link, signal_map, node_noise, snr_threshold, tx_power)
    scheduled_link = zeros(length(candidate_set) + 1, 1);
    
    % start link attenuation unknown; has to meet SNR threshold as well in
    % the absence of interference
    sender = link(start_link, 1);
    receiver = link(start_link, 2);
    path_loss = signal_map(sender, receiver);
    if isnan(path_loss) || isinf(path_loss)
        %fprintf('link attenuation unknown, skip\n');
        slot_concurrent_set = [];
        return;
    end
    
    scheduled_link(1) = start_link;
    idx = 2;

    remaining_link = candidate_set;

%     % sanity check
%     % computed concurrent set for link 1
%     max_concurrent_set = [27;52;23;79;85;90;4;17;36;81;39;82];
%     ib = concurrentSetIB(max_concurrent_set);
%     fprintf('ib %e\n', ib, ib);
%     s = (1:91)';
%     rest = setdiff(s, max_concurrent_set);
%     for i = 1 : length(rest)
%         ib = concurrentSetIB([max_concurrent_set; rest(i)]);
%         if ib >= 0
%             fprintf('adding node %d ib %e\n', rest(i), ib);
%         end
%     end
    
    while ~isempty(remaining_link)
        % find the link that maximizes IB
        max_ib = -inf;
        max_ib_idx = inf;           
        for k = 1 : length(remaining_link)
            ib = concurrentSetIB([scheduled_link(1 : idx - 1); remaining_link(k)]);

%                 if isinf(max_ib)
%                     break;
%                 end
            if max_ib < ib
                max_ib = ib;
                max_ib_idx = k;
            end
        end

%             % warning
%             if isinf(max_ib)
%                 break;
%             end
        % no valid schedule found, abort
        if max_ib < 0
            break;
        end

        scheduled_link(idx) = remaining_link(max_ib_idx);
        idx = idx + 1;
        remaining_link(max_ib_idx) = [];
    end

    slot_concurrent_set = scheduled_link(1 : idx - 1);

    % @param concurrent_set: consists of link id's
    % @return interference budget of the concurrent set
    function ib = concurrentSetIB(concurrent_set)
        ib = inf;

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
                    ib = -inf;
                    return;
                end

                path_loss = signal_map(interferer, receiver);
                % unknown (nan, no sample received) or invalid (-inf, rssi < noise)
% v1
%                 if isnan(path_loss) || isinf(path_loss)
%                     ib = -inf;
%                     return;
%                 end
%                 interference = tx_power / path_loss;
% v2
                % unknown as too far away 
                if isnan(path_loss) || isinf(path_loss)
                    interference = 0;
                else
                    interference = tx_power / path_loss;
                end
                interference_sum = interference_sum + interference;
            end

            link_ib = signal / snr_threshold - noise - interference_sum;

            % min of all link IB
            if ib > link_ib
                ib = link_ib;
            end
        end
    end % concurrentSetIB()

end % maxConcurrentSet()