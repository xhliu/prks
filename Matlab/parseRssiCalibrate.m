%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% author: Xiaohui Liu (whulxh@gmail.com)
% date: 3/24/13
% description: parse CC2420 reference curve
% http://hinrg.cs.jhu.edu/joomla/software/166-calibrating-rssi-values-reported-by-cc2420-radios.html
% input: name of the file
% output: reference curve
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
filename = 'cc2420-rssi-calibration-table.txt';
type_value = [];

% read all in
str = textread(filename, '%s', 'delimiter', '\n');
len = size(str, 1);
calib_raw_rssi = [];
% each line added
% skip 1st line
for i = 2 : len
    remain = str{i};
    entry = [];
    for j = 1 : 2
        [token, remain] = strtok(remain, ' ');
        entry = [entry str2num(token)];
    end
    calib_raw_rssi = [calib_raw_rssi; entry];
end
save('calib_raw_rssi.mat', 'calib_raw_rssi');
plot(calib_raw_rssi(:, 1), calib_raw_rssi(:, 2));
grid on;