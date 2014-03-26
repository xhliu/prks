%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/10/2014
%   Function: CC2420 radio model based on 802.14.5 model
function ber = snr2Ber(snr_db)
    % convert
    snr = 10 ^ (snr_db / 10);
    
    sum = 0;
    for k = 2 : 16
        sum = sum + (-1) ^ k * nchoosek(16, k) * exp(20 * snr * (1 / k - 1));
    end
    ber = 8 / 15 * 1 / 16 * sum;
end