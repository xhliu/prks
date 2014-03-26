%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   8/28/2013
%   Function: calc elapsed time by removing control slots in between
%   @param start_time_us, end_time_us: start time and end time in us
%   @return: time interval from start to end in ms

%   updated: 9/10/2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function time_diff_ms = globalTimeDiff(start_time_us, end_time_us)

% in us
SLOT_LEN = 32 * 1024;

% # of links scheduled
ACTIVE_LINK_SIZE = 99;
% least SLOT_MASK = (2^n - 1), such that 2^n >= active_link_size
SLOT_MASK = 127; %0x7F;
% max # of slot search forward; to prevent inf loop
MAX_SLOT_FORWARD = SLOT_MASK + 1;

start_slot = floor(start_time_us / SLOT_LEN);
end_slot = floor(end_time_us / SLOT_LEN);

skip_slot_cnt = 0;
%for slot = start_slot : end_slot
for slot = (start_slot + 1) : end_slot
    if mod(slot, MAX_SLOT_FORWARD) < (MAX_SLOT_FORWARD - ACTIVE_LINK_SIZE)
        fprintf('%d, %d\n', slot, mod(slot, MAX_SLOT_FORWARD));
        skip_slot_cnt = skip_slot_cnt + 1;
    end
end

time_diff_ms = ((end_time_us - start_time_us) - skip_slot_cnt * SLOT_LEN) / 1024;

end
