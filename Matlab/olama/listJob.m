%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Author: Xiaohui Liu (whulxh@gmail.com)
%   Date:   1/15/14
%   Function: record all job parameters; run this before any other .m files
%   under this directory
%% 

% select testbed: NetEye or Indriya
is_neteye = false;
% is_neteye = true;

% directory of raw job data
if is_neteye
    MAIN_DIR = '~/Projects/tOR/RawData/';
else
    MAIN_DIR = '~/Projects/tOR/RawData/Indriya/';
end

% directory to store figures
FIGURE_DIR = '~/Dropbox/Projects/onama/figure/';
fprintf('Warning: please change the directory to prevent overwriting\n');

% # of protocols to compare
PROTOCOL_CNT = 2;
% # of pdr requirements
pdr_reqs = [70 80 90 95];
PDR_REQ_CNT = length(pdr_reqs);

% display shared variable
groupnames = {'70', '80', '90', '95'};
bw_xlabel = 'PDR requirement (%)';
fprintf('Warning: protocol orders in compareProtocol*.m must match bw_legend here\n');
%bw_legend = {'PRKS', 'CSMA', 'RTS-CTS', 'RIDB', 'RIDB-OLAMA', 'CMAC', 'SCREAM'};
bw_legend = {'PRKS-NAMA', 'PRKS-ONAMA'};

%% format: [job_id job_duration_in_mins]

%%
%bootstrap_in_seconds = [500 30 30 900]

% jobs = [20878 20879];
% for job_id = 1 : length(jobs)
%     fprintf('processing job %d\n', jobs(job_id));
%     job_dir = [MAIN_DIR num2str(jobs(job_id))];
%     cd(job_dir);
% end

PRKS_ONAMA_SLOT_LEN = 512;
PRKS_NAMA_SLOT_LEN = 32;

if is_neteye
%% PRKS NAMA
job = cell(0);

% pdr req 70
job{1} = [21514 90;
          21516 90;
          21520 90;
          ];
% 80
job{2} = [21515 90;
          21517 90;
          21521 90;
          ];
% 90
job{3} = [21512 90;
          21518 90;
          21522 90;
          ];
% 95
job{4} = [21513 90;
          21519 90;
          21523 90;
          ];
prks_nama_job = job;

%% PRKS ONAMA
job = cell(0);

% pdr req 70
job{1} = [20858 240;
          20867 240;
          20894 240;
          20891 240;
          ];
% 80
job{2} = [20859 240;
          20866 240;
          20914 240;
          20892 240;
          ];
% 90
job{3} = [20857 240;
          20865 240;
          20926 240;
          20913 240;
          20890 240;
          ];
% 95
job{4} = [20856 240;
          20860 360;
          20925 240;
          20912 240;
          20893 240;
          ];
prks_onama_job = job;


else

%% PRKS NAMA
job = cell(0);

% pdr req 70
job{1} = [45329 60;
          45376 60;
          45395 60;
          ];
% 80
job{2} = [45328 60;
          45377 60;
          45396 60;
          ];
% 90
job{3} = [45312 60;
          45381 60;
          45424 60;
          ];
% 95
job{4} = [45313 60;
          45382 60;
          45425 60;
          ];
prks_nama_job = job;

%% PRKS ONAMA
job = cell(0);

% mobihoc
% % pdr req 70
% job{1} = [43320 120;
%           43302 60;
%           ];
% % 80
% job{2} = [43330 120;
%           43311 90;
%           ];
% % 90
% job{3} = [43324 120;
%           43301 60;
%           ];
% % 95
% job{4} = [43329 120;
%           43300 120;
%           43295 60;
%           ];

% globecom
% % pdr req 70
% job{1} = [43320 120;
%           43302 60;
%           43635 90;
%           ];
% % 80
% job{2} = [43330 120;
%           43311 90;
%           43678 60;
%           43705 90;
%           ];
% % 90
% job{3} = [43324 120;
% %           43301 60;
%           43677 60;
%           43704 90;
%           ];
% % 95
% job{4} = [43329 120;
% %           43300 120;
%           43295 60;
%           43634 90;
%           ];

% specifically to compare concurrency w/ iOrder
% pdr req 70
job{1} = [46790 55;
          46949 55;
          ];
% 80
job{2} = [46792 55;
          46950 55;
          ];
% 90
job{3} = [46791 55;
          46941 55;
          ];
% 95
job{4} = [46825 60;
          46930 55;
          ];
prks_onama_job = job;


end

%% sync protocols
jobs = [];

for i = 1 : length(prks_nama_job)
    jobs = [jobs; prks_nama_job{i}(:, 1)];
end

for i = 1 : length(prks_onama_job)
    jobs = [jobs; prks_onama_job{i}(:, 1)];
end
%{
for i = 1 : length(ridbolama_job)
    jobs = [jobs; ridbolama_job{i}(:, 1)];
end
%}
sync_jobs = jobs;

%% DBG constants
DBG_LOSS_FLAG = 0;
DBG_TX_FLAG = DBG_LOSS_FLAG + 1;
DBG_RX_FLAG = DBG_TX_FLAG + 1;
DBG_BACKOFF_FLAG = DBG_RX_FLAG + 1;
DBG_ER_FLAG = DBG_BACKOFF_FLAG + 1;
% 5
DBG_SM_FLAG = DBG_ER_FLAG + 1;
DBG_TX_FAIL_FLAG = DBG_SM_FLAG + 1;
DBG_TIMEOUT_FLAG = DBG_TX_FAIL_FLAG + 1;
DBG_BI_ER_FLAG = DBG_TIMEOUT_FLAG + 1;
DBG_EXEC_TIME_FLAG = DBG_BI_ER_FLAG + 1;
% 10
DBG_DELAY_FLAG = DBG_EXEC_TIME_FLAG + 1;
DBG_CANCEL_FLAG = DBG_DELAY_FLAG + 1;
DBG_CONTROLLER_FLAG = DBG_CANCEL_FLAG + 1;
DBG_COUNTER_NAV_FLAG = DBG_CONTROLLER_FLAG + 1;
DBG_CALC_FLAG = DBG_COUNTER_NAV_FLAG + 1;
% 15
DBG_HEARTBEAT_FLAG = DBG_CALC_FLAG + 1;
DBG_FTSP_FLAG = DBG_HEARTBEAT_FLAG + 1;
DBG_TDMA_FLAG = DBG_FTSP_FLAG + 1;
DBG_SPI_FLAG = DBG_TDMA_FLAG + 1;
DBG_DRIVER_FLAG = DBG_SPI_FLAG + 1;
% 20
DBG_ERR_FLAG = DBG_DRIVER_FLAG + 1;
DBG_OVERFLOW_FLAG = DBG_ERR_FLAG + 1;
