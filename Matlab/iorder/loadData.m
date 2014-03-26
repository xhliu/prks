%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/25/2014
%   Function: load jobs and do some pre-processing
%% link set
load ~/Dropbox/iMAC/Xiaohui/links_olama; % link100.mat;
% indriya
% load ~/Dropbox/iMAC/Xiaohui/links_olama_indriya;
link = links;
%% signal map: neteye   job 19935/20768;    indriya     job 46280/46281
load ~/Projects/tOR/RawData/20768/link_snr_gain;
% load ~/Projects/tOR/RawData/Indriya/46281/link_snr_gain;
t = signal_map;
cnt = sum(sum(isnan(t))) - size(t, 1);
fprintf('nan ratio %f\n', cnt / (size(t, 1) * size(t, 2)));
%% noise floor: neteye job 18654/19012/19936/20769;     indriya     job
%% 46279/46282
% load ~/Projects/tOR/RawData/20769/node_noise;
load ~/Projects/tOR/RawData/Indriya/46282/node_noise;
%% power level 3
TX_POWER = -25;

%%
len = size(link, 1);
link_set = zeros(len, 2);
idx = 1;
% convert node's id to be based on 'nodes'
for i = 1 : len
    sender_idx = find(nodes == link(i, 1));
    receiver_idx = find(nodes == link(i, 2));
    if isempty(sender_idx) || isempty(receiver_idx)
        continue;
    end
    link_set(idx, 1) = sender_idx;
    link_set(idx, 2) = receiver_idx;
    idx = idx + 1;
end
link_set(idx : end, :) = [];

%%
noises = [];
t = node_noise;
for i = 1 : size(nodes, 1)
    noises(i) = t(t(:, 1) == nodes(i), 2);
end
