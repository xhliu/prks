%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/10/2014
%   Function: generate pdr vs snr theoretical curve for CC2420 radio; 
%   w/ fading
%% all in dB
function mean_pdr = generatePdrFromSnr(mean_snr, sigma_s, sigma_n, packet_len_byte, packet_cnt)
    sigma_snr = sqrt(sigma_s ^ 2 + sigma_n ^ 2);
    fprintf('reduce processing by assume bit in the same byte enjoys identical snr\n');
    %len_in_bit = packet_len_byte * 8;
    
    success_cnt = 0;    
    % each packet
    for i = 1 : packet_cnt
        is_packet_received = true;
        
        % each bit
        %for j = 1 : len_in_bit
        for j = 1 : packet_len_byte
            snr = mean_snr + randn * sigma_snr;
            ber = snr2Ber(snr);
            % any bit corrupted
            if rand < ber
                is_packet_received = false;
                break;
            end
        end
        
        if is_packet_received
            success_cnt = success_cnt + 1;
        end
    end
    
    mean_pdr = success_cnt / packet_cnt;
end