%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/15/14
%   Function: record all job parameters; run this before any other .m files
%   under this directory
%% format
% [job_id job_duration_in_mins]
MAIN_DIR = '~/Projects/tOR/RawData/';
% MAIN_DIR = '~/Downloads/Indriya/';
PROTOCOL_CNT = 2;
PDR_REQ_CNT = 4;

% display shared variable
groupnames = {'70', '80', '90', '95'};
bw_xlabel = 'PDR requirement (%)';
%bw_legend = {'PRKS', 'CSMA', 'RTS-CTS', 'RIDB', 'CMAC', 'SCREAM'};
bw_legend = {'PRKS', 'SCREAM'};

%% PRKS
PRKS_SLOT_LEN = 512;
job = cell(0);
% pdr req 70
job{1} = [20858 240;
          20867 240;
          ];
% 80
job{2} = [20859 240;
          20866 240;
          ];
% 90
job{3} = [20857 240;
          20865 240;
          ];
% 95
job{4} = [20856 240;
          20860 360;
          ];
prks_job = job;

%% SCREAM
SCREAM_SLOT_LEN = 32;
scream_job = [(20849 : 20853)' repmat(90, 5, 1)];