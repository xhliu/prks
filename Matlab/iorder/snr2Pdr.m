%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   6/4/2014
%   Function: generate pdr vs snr theoretical curve for CC2420 radio; 
%   w/ fading
%%
function pdr = snr2Pdr(snr_db, packet_cnt)
    % [snr pdr]
    load ~/Dropbox/Projects/PRK/Matlab/matdata/pdr_vs_snr; % pdr_vs_snr_theory
    t = pdr_vs_snr;
    
    for i = 1 : size(t, 1)
        % round downward
        if snr_db < t(i, 1)
            if i > 1
                pdr = t(i - 1, 2);
            else
                pdr = t(i, 2);
            end
            return;
        end
    end
    pdr = t(end, 2);
end