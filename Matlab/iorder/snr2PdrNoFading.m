%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   6/1/2014
%   Function: no fading, constant signal & noise; also assumes bit indepedence
%% all in dB
function pdr = snr2PdrNoFading(snr_db, packet_len_byte)
    len = packet_len_byte * 8;
    pdr = (1 - snr2Ber(snr_db)) ^ len;
end