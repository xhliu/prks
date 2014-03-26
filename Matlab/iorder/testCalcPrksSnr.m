%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/25/2014
%   Function: to determine SNR threshold for iOrder comparison; run
%   loadData.m first
%%
fprintf('run loadData.m first\n');
%% %% compute link_active_slot
load debugs;
t = debugs;
type = DBG_TDMA_FLAG;
line = 534;
t = t(t(:, 3) == type, :);
t = t(t(:, 4) == line, :);

%% line # if changed
% s = t;
% s = s(:, 4);
% cdfplot(s);
% unique(s)

%%
DATA_PENDING_IDX = 5;
SLOT_IDX = 10;
SLOT_LEN = 512 * 2 ^ 10;
SLOT_WRAP_LEN = 2 ^ 32 / SLOT_LEN;

% filter slots w/o data
t = t(t(:, DATA_PENDING_IDX) == 1, :);
% unwrap slots
t = unwrap(t, SLOT_WRAP_LEN);

%% convert node's id to be based on 'nodes'
len = size(t, 1);
link_active_slot = zeros(len, 2);
idx = 1;
for i = 1 : len
    sender_idx = find(nodes == t(i, 2));
    if isempty(sender_idx)
        continue;
    end
    link_active_slot(idx, :) = [sender_idx t(i, SLOT_IDX)];
    idx = idx + 1;
end
link_active_slot(idx : end, :) = [];

%% [slot, # of concurrent links, snr]
link_snrs = calcPrksSnr(link_set, link_active_slot, signal_map, noises, TX_POWER);
%%
save('link_snrs.mat', 'link_snrs');
%% 
% s = link_snrs{91};
% plot(s(:, end));