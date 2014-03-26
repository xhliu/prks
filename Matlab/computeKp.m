%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   4/14/12
%   Function: compute Kp in the controller according to PDR vs SINR curve
%   updated: 1/14/13 to compute inverse function inv_pdrs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
t = snr_pdr;
for i = 1 : size(t, 1) - 1
    if t(i) > t(i + 1)
        break;
    end
end

s = t(1:761, :);
r = t(762:end, :);

y = 0.976;
x1 = find(s(:, 2) > y, 1);
x2 = find(r(:, 2) > y, 1);

t = [s(1:x1 - 1, :); r(x2 : end, :)];
figure;
plot(t(:, 1), t(:, 2));

%%
snr = t(:, 1);
pdr = t(:, 2);
s = (snr(2 : end) - snr(1 : end - 1)) ./ (pdr(2 : end) - pdr(1 : end - 1));
IX = find(s < 0);
% out of order
IX = [1:5 690:691];
snr(IX, :) = [];
pdr(IX, :) = [];
s = (snr(2 : end) - snr(1 : end - 1)) ./ (pdr(2 : end) - pdr(1 : end - 1));
IX = find(s < 0);

%%
clc;
Kp = zeros(101, 1);
inv_pdrs = zeros(101, 1);
for j = 1 : 101
    if j == 1
        % Kp(0) is astronomical
        i = 0.0001;
    else
        i = (j - 1) * 0.01;
    end
    idx = j;
    
    ix = find(pdr == i, 1);
    if ~isempty(ix)
        Kp(idx) = s(ix);
    else
        % find right
        r = find(pdr > i, 1);
        if ~isempty(r)
            if r > 1
                % choose closer
                l_diff = i - pdr(r - 1);
                r_diff = pdr(r) - i;
                if l_diff < r_diff
                    Kp(idx) = s(r - 1);
                    ix = r - 1;
                else
                    Kp(idx) = s(r);
                    ix = r;
                end
            else
                Kp(idx) = s(r);
                ix = r;
            end
        else
            Kp(idx) = s(end);
            ix = length(s);
        end
    end
%     fprintf('%d: %f; %d, %f, %f\n', idx, Kp(idx), ix, pdr(ix), s(ix));
%     fprintf('%d, ', round(snr(ix) * 100));
    inv_pdrs(idx) = snr(ix);
end
% absolute value
save('Kp.mat', 'Kp'); 
save('inv_pdrs.mat', 'inv_pdrs');

%%
for i = 1 : 101
    fprintf('%d, ', round(Kp(i) * 10));
end
%% max slope
REFERENCE_PDR = 0.9;
REFERENCE_IDX = round(REFERENCE_PDR * 100 + 1)
slopes = [];
for i = 1 : 101
    pdr = (i - 1) * 0.01;
    slopes = [slopes; (inv_pdrs(i) - inv_pdrs(REFERENCE_IDX)) / (pdr - REFERENCE_PDR)];
end
max(slopes)
max(Kp)