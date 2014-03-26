%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   2/16/2012
%   Function: analyze the accuracy of link attenuation estimation (i.e.,
%   signal map)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
TX_FLAG = 0;
RX_FLAG = 1;
DBG_FLAG = 2;

SCALAR = 128;

INBOUND_IDX = 9;
OUTBOUND_IDX = 10;

%%
if 0
    
node = 76; %[11:15 27:30 41:45 56:60 71:75];

load([num2str(node) '.mat']);
TYPE = 2;
NB_IDX = 3;
t = Packet_Log;
t = t(t(:, 1) == TYPE, :);

% filter packets w/ invalid noise RSSI
INVALID_RSSI = 0;
t = t(t(:, 6) ~= INVALID_RSSI, :);

neighbors = unique(t(:, NB_IDX));
for i = 1 : length(neighbors)
    neighbor = 111; %neighbors(i);
    s = t(t(:, NB_IDX) == neighbor, :);
% pkt: -71  noise: -92 (minus 127)
% t = t(t(:, 1) == 2 & t(:, 4) == 15, :);
% openvar('s');
%     s = s(s(:, 6) > 65000, :);
    total = s(:, 5) - 256;
if length(total) < 100
    continue;
end
    figure;
    plot(total);
    noise = s(:, 6) - 172;
    plot(noise);
    rss = exp(log(10) * total / 10) - exp(log(10) * noise / 10);
    rss = rss(rss > 0);
    rss_dbm = 10 * log10(rss);
    plot(rss_dbm);
    title(['link ' num2str(node) ' to ' num2str(neighbor)]);
end
% 15 -> 30: 59 dB
% 30 -> 15: 61 dB
%%
% 10 * log10(10 ^ (-71 / 10) - 10 ^ (-72 / 10))
%% read rssi immediately success ratio
nodes = [11:15 27:30 41:45 56:60 71:75];
ratios = [];
for i = 1 : length(nodes)
load([num2str(nodes(i)) '.mat']);
TYPE = 2;
NB_IDX = 3;
t = Packet_Log;
t = t(t(:, 1) == TYPE, :);
% s = t(t(:, 10) < 100000, 10);
s = t(:, 10) - 172;
% figure;
% cdfplot(s);
suc = t(end, 4);
total = t(end, 10);
fprintf('%d out of %d: %f\n', suc, total, suc / total);
ratios = [ratios; suc / total];
end
[n xout] = hist(ratios);
bar(xout, n / sum(n) * 100);

%% abnormal convergence
load debugs;
INVALID_VAL = 511;
ranges = [];
nodes = unique(debugs(:, 2));
for j = 1 : length(nodes)
    node = nodes(j);
    load([num2str(node) '.mat']);
    TYPE = 1;
    NB_IDX = 8;
    t = Packet_Log;
    t = t(t(:, 1) == TYPE, :);
    neighbors = unique(t(:, NB_IDX));
    for i = 1 : length(neighbors)
        neighbor = neighbors(i);
        s = t(t(:, NB_IDX) == neighbor, OUTBOUND_IDX);
        s = floor(s / 128);
    %     s = s(200:end);
    %     s = s(s ~= 511);
        if size(s, 1) < 20
            continue;
        end
        ranges = [ranges; max(s) - min(s)];
        plot(s);
        title(['link ' num2str(neighbor) ' to ' num2str(node)]);
        idx = find(s ~= INVALID_VAL, 1);
        if sum(s(idx + 1 : end) == INVALID_VAL) > 0
    %     if sum(s == 511) > 1 && sum(s ~= 511) > 10
            plot(s);
            title(['link ' num2str(node) ' to ' num2str(neighbor)]);
        end
    end
end

%% sanity check: inbound at receiver v.s. outbound at sender
errs = [];
load debugs;
INVALID_VAL = 511;  % 65535 / 128
TYPE = 1;
nodes = unique(debugs(:, 2));
for i = 1 : length(nodes)
    tx = nodes(i);
    for j = 1 : length(nodes)
        rx = nodes(j);
        if j == i
            continue;
        end
        
        % inbound
        load([num2str(rx) '.mat']);
        t = Packet_Log;
        t = t(t(:, 1) == TYPE, :);
        s = floor(t(t(:, 8) == tx, INBOUND_IDX) / 128);
        s = s(s ~= INVALID_VAL, :);
        if isempty(s)
            continue;
        end
        r = s;
%         figure;
%         plot(s);
%         title(['inbound link ' num2str(tx) ' to ' num2str(rx)]);

        % outbound
        load([num2str(tx) '.mat']);
        t = Packet_Log;
        t = t(t(:, 1) == TYPE, :);
        s = floor(t(t(:, 8) == rx, OUTBOUND_IDX) / 128);
        s = s(s ~= INVALID_VAL, :);
        if isempty(s)
            continue;
        end
        diff = setdiff(s, r);
        if ~isempty(diff)
            fprintf('link (%d, %d) error: %d, %d\n', tx, rx, diff(1), length(diff));
            errs = [errs; length(diff)];
        end
%         figure;
%         plot(s);
%         title(['outbound link ' num2str(tx) ' to ' num2str(rx)]);
    end
end

%%
load debugs;
tx = 22;
rx = 1; % 3 18, 31
% hold all;
% link 3 -> 4
% receive
t = debugs;
t = t(t(:, 6) ~= 0, :);
t = t(t(:, 2) == rx & t(:, 3) == tx, :);
% figure;
% s = floor(t(:, 10) / 128);
total = t(:, 5) - 256;
noise = t(:, 4) - 65536;
rss = exp(log(10) * total / 10) - exp(log(10) * noise / 10);
rss = rss(rss > 0);
s = 10 * log10(rss);
attnu = -25 - s;
plot(attnu);
% attnu = attnu(attnu > 50 & attnu < 70);
% figure;
% plot(attnu);
title(['link ' num2str(tx) ' to ' num2str(rx) ' mean: ' num2str(mean(attnu))]);

%%
% a = attnu(1:200);
b = attnu(1:200);
%%
r_ = r(:, [8 10]);
t_ = t(:, [8 10]);
[c ia ib] = intersect(r_(:, 1), t_(:, 1));
x = [r_(ia, 1) r_(ia, 2) - t_(ib, 2) r_(ia, 2) t_(ib, 2)];
fprintf('rx <%d, %f> not sent, err <%d, %f> \n', size(r_, 1) - size(ia, 1), 1 - size(ia, 1) / size(r_, 1), ...
    sum(x(:, 2) ~= 0), sum(x(:, 2) ~= 0) / size(x, 1));


%%
[n xout] = hist(ranges);
bar(xout, n / sum(n) * 100);

end
%% accuracy of link gain estimation
% 14->15:47      15->14:51
% close all;
clc;
tx_rx_gains = [];
NB_IDX = 3;
IDX = 10;
NODE_CNT = 130;
MIN_SAMPLE_SIZE = 100;

load debugs;
r = debugs;
% filter packets w/ invalid noise RSSI
INVALID_RSSI = 0;
r = r(r(:, 6) ~= INVALID_RSSI, :);
% r = r(r(:, 10) == 0, :);
nodes = unique(r(:, 2));

% -25 in NetEye; -10 in Indriya
tx_power = -25;
for i = 1 : length(nodes)
    rx = nodes(i);    

    fprintf('rx %d\n', rx);
    t = r(r(:, 2) == rx, :);
    
    txs = unique(t(:, NB_IDX));
    
    for j = 1 : length(txs)
        tx = txs(j);
        s = t(t(:, NB_IDX) == tx, :);
        % enough samples to let mean make sense
        if size(s, 1) < MIN_SAMPLE_SIZE
            continue;
        end
        total = s(:, 5) - 256;
        noise = s(:, 4) - 65536;
        rss = exp(log(10) * total / 10) - exp(log(10) * noise / 10);
        ratio = sum(rss <= 0) / length(rss);
%         plot(rss);
%         close all;
        % filter packets w/ invalid received signal strength
        rss = rss(rss > 0);
        rss_dbm = 10 * log10(rss);
        attnu = tx_power - rss_dbm;
        plot(attnu);
%         figure;
%         cdfplot(rss_dbm);
%         fprintf('%f %f %f\n', mean(rss_dbm), median(rss_dbm), mode(rss_dbm));
        % discrete
%         rss_dbm = round(rss_dbm);
%         if (max(rss_dbm) - min(rss_dbm)) < 20
%             continue;
%         end

%         figure;
%         plot([total noise rss_dbm]);
%         legend({'total', 'noise', 'rss_dbm'});
%         title(['Received signal strength of link ' num2str(tx) ' to ' num2str(rx)]);
        fprintf('link <%d, %d> gain: %f\n', tx, rx, mode(attnu));
        tx_rx_gains = [tx_rx_gains; tx, rx, mean(attnu), median(attnu), mode(attnu), ratio];
    end
end
save('tx_rx_gains.mat', 'tx_rx_gains');

%%
% NetEye
% 8475: benchmark 8480: 10s  8478: 2500ms   8477: 100ms
%cd('/home/xiaohui/Projects/tOR/RawData/8477');

% Indriya
% 19292: benchmark  19287: 10 s     19286: 2500 ms      19284: 100 ms
cd('/home/xiaohui/Projects/tOR/RawData/Indriya/19284');

load tx_rx_gains.mat;
%
% light_tx_rx_gains = tx_rx_gains;
% figure; hold on;
% diff
IDX = 3;
t = light_tx_rx_gains;  % benchmark
s = tx_rx_gains;
[x, it, is] = intersect(t(:, 1:2), s(:, 1:2), 'rows');
r = [t(it, 1:2), t(it, IDX), s(is, IDX)];
% r0 = exp(log(10) * (-r(:, 3)) / 10);
% r1 = exp(log(10) * (-r(:, 4)) / 10);
% result = (r1 - r0) ./ r0;
% absolute error
result = (r(:, 4) - r(:, 3));
cdfplot(result);
% relative error
% result = (r(:, 4) - r(:, 3)) ./ r(:, 3);
% cdfplot(result * 100);
%% figure;
% boxplot(result, 'notch', 'on');
% title('mean');
% title('median');
% title('mode');
% c = result;
% 1-sided lower
% ALPHA = 0.1;
% t = abs(result);
% cdfplot(t);
% CDF
% hold on;
%% sparser cdf
t = result;
DOWN_SCALE = 50;
[f, x] = ecdf(t);
lo = 1;
hi = length(x);
% 5923 6124
% IDX = [1:DOWN_SCALE:lo (lo + 1):hi (hi + 1):length(x)];
IDX = [1:DOWN_SCALE:length(x)];
plot(x(IDX), f(IDX), '*');

%%
% legend('Light', 'Medium', 'Heavy');
% lengthen legend line by 4 times
fprintf('legendflex seems incompatible w/ export_fig, save manually\n');
legendflex({'Light', 'Medium', 'Heavy'}, 'xscale', 4);
% mu = 1;
% z = (mean(t) - mu) / (std(t) / sqrt(size(t, 1)))
% normcdf(z, 0, 1)
% dev =  norminv(1 - ALPHA, 0, 1) * std(t) / sqrt(size(t, 1));
% fprintf('[%.4f, %.4f]\n', mean(t) - dev, mean(t));
% 
% r = sort(t);
% % err bound
% lo_idx = ceil(0.5 * size(r, 1) - 0.5 * norminv(1 - ALPHA / 2, 0, 1) * sqrt(size(r, 1)));
% lo = r(lo_idx);
% hi_idx = ceil(0.5 * size(r, 1) + 0.5 * norminv(1 - ALPHA / 2, 0, 1) * sqrt(size(r, 1)));
% hi = r(hi_idx);
% lo
% hi
% median(t)

%% boxplot
data = cell(3, 1);
data{1} = a_;
data{2} = b_;
data{3} = c_;
group = zeros(0);
dataDisp = zeros(0);
for i = 1 : size(data, 1)
    group = [group ; ones(size(data{i})) + i - 1];
    dataDisp = [dataDisp ; data{i}];
end
figure;
set(gca, 'FontSize', 40);
title('');
boxplot(dataDisp, group, 'notch', 'on');
%%
set(gca, 'ygrid', 'on', 'xticklabel', {'Light', 'Medium', 'Heavy'});
ylabel('Estimation error (dB)');

%% %% RSSI read latency distribution
load debugs.mat;
t = debugs(:, 10);
t = t(t < 100000);
%% async
a = t;
%% sync
s = t;
%% CDF
% hold on;
t = result;
DOWN_SCALE = 20; %1000;  % 2
[f, x] = ecdf(t);
lo = 1;
hi = length(x);
% 5923 6124
% IDX = [1:DOWN_SCALE:lo (lo + 1):hi (hi + 1):length(x)];
IDX = [1:DOWN_SCALE:length(x)];
plot(x(IDX), f(IDX), '*');

%%
legend({'Sync read', 'Async read'});
%%
set(gca, 'xscale', 'log');
set(gca, 'xtick', [10 100 1000 10000 100000]);
xlim([100 100000]);
grid on;

%%
set(gca, 'yscale', 'log', 'ygrid', 'on', 'xticklabel', {'Synchronous', 'Asynchronous'});
set(gca, 'ytick', [100 1000 10000 100000]);
ylabel('RSSI register reading latency (\mus)');
ylim([10 ^ 1.9 10 ^ 5]);
