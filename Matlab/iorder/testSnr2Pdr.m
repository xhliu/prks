%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/10/2014
%   Function: generate pdr vs snr theoretical curve for CC2420 radio
%%
SIGMA_S = 3.2;
SIGMA_N = 4;
PACKET_LEN = 128;
PACKET_CNT = 1000;

snr = -5 : 1 : 20;
snr = snr';
len = length(snr);
pdr = nan(len, 1);
for i = 1 : len
    pdr(i) = snr2Pdr(snr(i), SIGMA_S, SIGMA_N, PACKET_LEN, PACKET_CNT); 
end

plot(snr, pdr);
%%
save('pdr_vs_snr_theory.mat', 'snr', 'pdr');