%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/10/2014
%   Function: generate pdr vs snr theoretical curve for CC2420 radio
%%
SIGMA_S = 3.2;
SIGMA_N = 4;
PACKET_LEN = 128;
PACKET_CNT = 1000;

snr = -10 : 0.5 : 100;
% snr = snr';
len = length(snr);
pdr = nan(len, 1);
% for i = 1 : len
%     pdr(i) = generatePdrFromSnr(snr(i), SIGMA_S, SIGMA_N, PACKET_LEN, PACKET_CNT); 
% end

% no fading channel
for i = 1 : len
%     pdr(i) = snr2PdrNoFading(snr(i), PACKET_LEN); 
    pdr(i) = snr2Pdr(snr(i), PACKET_LEN); 
end

plot(snr, pdr);
%%
%save('pdr_vs_snr_theory.mat', 'snr', 'pdr');

%%
pdr = get(get(gca,'Children'),'YData');
snr = get(get(gca,'Children'),'XData');
pdr_vs_snr = [snr' pdr'];
save('pdr_vs_snr.mat', 'pdr_vs_snr');
t = pdr_vs_snr;
figure;
plot(t(:, 1), t(:, 2));