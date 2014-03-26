%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   10/30/2013
%   Function: compute concurrency in pkts/s for a single job

%% to cope w/ slot/time wrap around
SLOT_LEN = 32;
fprintf('slot length %d\n', SLOT_LEN);

%% 
load debugs;
t = debugs;
type = DBG_TDMA_FLAG;
line = 543; %539; %543;
t = t(t(:, 3) == type, :);
t = t(t(:, 4) == line, :);

%% line # if changed
% s = t;
% s = s(:, 4);
% cdfplot(s);
% unique(s)

%% concurrency
% filter slots w/o data by default
% fprintf('filter slots w/o data\n');
% t = t(t(:, 5) == 1, :);

load link_pdrs;
SLOT_WRAP_LEN = 2 ^ 32 / (SLOT_LEN * 2 ^ 10);

%%
t = unwrap(t, SLOT_WRAP_LEN);
s = t(:, 10);
%% 
[c e] = hist(s, unique(s));
ce = [e c'];

%% figure;
concurrency = ce(:, end);
% sanity check concurrency
fprintf('total concurrency %d, total packets sent %d, ratio: %f\n', sum(concurrency), sum(link_pdrs(:, 3)), sum(concurrency) / sum(link_pdrs(:, 3)));
save('concurrency.mat', 'concurrency');
cdfplot(concurrency);
fprintf('concurrency median %f, mean %f\n', median(concurrency), mean(concurrency));


%%
% MIN_GAP = SLOT_WRAP_LEN / 2;
% nodes = unique(t(:, 2));
% tx_slots = [];
% for j = 1 : length(nodes)
%     s = t(t(:, 2) == nodes(j), 10);
%     % sanity check if time is increasing
% %     plot(s);
%     
%     % wrap around points; add MIN_GAP bcoz sometimes a entry is larger than
%     % its next entry somehow
%     %IX = find(s(1:end-1) > s(2:end));
%     IX = find(s(1:end-1) > (s(2:end) + MIN_GAP));
%     len = length(IX);
%     for i = 1 : len
%         if i < len
%             s(IX(i) + 1 : IX(i + 1)) = s(IX(i) + 1 : IX(i + 1)) + i * SLOT_WRAP_LEN;
%         else
%             s(IX(i) + 1 : end) = s(IX(i) + 1 : end) + i * SLOT_WRAP_LEN;
%         end
%     end
% %     plot(s);
%     tx_slots = [tx_slots; s];
% end
% 
% s = tx_slots;

%% present
%{
job{1} = [19479 19475 19471 19518 19517 19881 19895 19898];
job{2} = [19802 19815 19834 19858 19896 19904];
job{3} = [19891 19897 19903 19905];
main_dir = '~/Projects/tOR/RawData/';

len = length(job);
data = cell(len, 1);
% each pdr req
for i = 1 : len
    pdr_job = job{i};
    % each job
    for j = 1 : length(pdr_job)
        fprintf('processing job %d\n', pdr_job(j));
        job_dir = [main_dir num2str(pdr_job(j))];
        cd(job_dir);
        load concurrency;
        data{i} = [data{i}; concurrency];
    end
end
%% boxplot
group = zeros(0);
dataDisp = zeros(0);
for i = 1 : size(data, 1)
    group = [group ; ones(size(data{i})) + i - 1];
    dataDisp = [dataDisp ; data{i}];
end
% figure;
boxplot(dataDisp, group, 'notch', 'on');
set(gca, 'xticklabel', {'PRKS', 'PRKS-L', 'PRKS-R'});

%% cdf
figure;
hold on;
for i = 1 : size(data, 1)
    cdfplot(data{i});
end
legend({'PRKS', 'PRKS-L', 'PRKS-R'});

%}