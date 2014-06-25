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
% if is_neteye
%     MAIN_DIR = '~/Projects/tOR/RawData/';
% else
%     MAIN_DIR = '~/Projects/tOR/RawData/Indriya/';
% end
MAIN_DIR = '~/Downloads/Indriya/';
% MAIN_DIR = '~/Downloads/Jobs/';

% directory to store figures
FIGURE_DIR = '~/Dropbox/iMAC/Xiaohui/figure/';
% FIGURE_DIR = '~/Dropbox/iMAC/Yu/Figure/Indriya/';
% FIGURE_DIR = '~/Dropbox/iMAC/Yu/Figure/MobiHoc/';
fprintf('Warning: please change the directory to prevent overwriting\n');

% # of protocols to compare
PROTOCOL_CNT = 2;
% # of pdr requirements
pdr_reqs = [70];
PDR_REQ_CNT = length(pdr_reqs);

% display shared variable
groupnames = cell(1);
% bw_xlabel = 'PDR requirement (%)';
bw_xlabel = '';
fprintf('Warning: protocol orders in compareProtocol*.m must match bw_legend here\n');
%bw_legend = {'PRKS', 'CSMA', 'RTS-CTS', 'RIDB', 'RIDB-OLAMA', 'CMAC', 'SCREAM'};
bw_legend = {'PRKS', 'RIDB'};
%bw_legend = {'PRKS', 'SCREAM'};

%% format: [job_id job_duration_in_mins]

%%
%bootstrap_in_seconds = [500 30 30 900]

% jobs = [20878 20879];
% for job_id = 1 : length(jobs)
%     fprintf('processing job %d\n', jobs(job_id));
%     job_dir = [MAIN_DIR num2str(jobs(job_id))];
%     cd(job_dir);
% end

if is_neteye
%% PRKS
PRKS_SLOT_LEN = 512;
job = cell(0);
% 
% fprintf('Jobs for PRKS K parameters\n');
% job{1} = [21016 180];
% job{2} = [21015 180];
% job{3} = [21037 180];
% job{4} = [21001 240];
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
prks_job = job;

%% CSMA
CSMA_SLOT_LEN = 5;
csma_job = [20288 20;
            20313 20;
            20325 20;
            20310 20;];

%% RTSCTS
RTSCTS_SLOT_LEN = 5;
rtscts_job = [20287 30;
              20324 30;
              20311 30;];


%% RIDB
RIDB_SLOT_LEN = 32;
job = cell(0);
% pdr req 70
job{1} = [20278 30;
          20327 30;
          ];
% 80
job{2} = [20276 30;
          20383 60;
          20319 30;
          ];
% 90
job{3} = [20275 30;
          20382 60;
          20317 30;
          ];
% 95
job{4} = [20277 30;
          20318 30;
          ];
ridb_job = job;



%% RIDB_OLAMA
% RIDBOLAMA_SLOT_LEN = 512;
% job = cell(0);
% % pdr req 70
% job{1} = [20793 60;
%           ];
% % 80
% job{2} = [20794 60;
%           ];
% % 90
% job{3} = [20778 60;
%           20776 60;
%           ];
% % 95
% job{4} = [20792 60;
%           ];
% ridbolama_job = job;

%% CMAC
CMAC_SLOT_LEN = 5;
job = cell(0);
% pdr req 70
job{1} = [20291 30;
          20326 30;
          ];
% 80
job{2} = [20290 30;
          20323 30;
          ];
% 90
job{3} = [20286 30;
          20320 30;
          20292 30;
          ];
% 95
job{4} = [20289 30;
          20322 30;
          20321 30;
          ];
cmac_job = job;


%% SCREAM
SCREAM_SLOT_LEN = 32;
% scream_job = [20849 90;
%               20850 90;
%               20851 90;
%               20852 90;
%               20853 90;
%               20881 90];
job = cell(0);
% pdr req 70
job{1} = [23054 180;
          23057 240;
          23061 240;
          23068 240;
          ];
% 80
job{2} = [23055 240;
          23060 240;
          23062 240;
          23069 240;
          ];
% 90
job{3} = [23053 240;
          23058 240;
          23063 240;
          23070 240;
          ];
% 95
job{4} = [23056 240;
          23059 240;
          23064 240;
          23071 240;
];
scream_job = job;

else
%% PRKS
PRKS_SLOT_LEN = 512;
job = cell(0);
% 
% fprintf('Jobs for PRKS K parameters\n');
% job{1} = [43635 90];
% job{2} = [43678 60];
% job{3} = [43677 60];
% job{4} = [43634 90];

% pdr req 70
job{1} = [49391 60;
          ];
prks_job = job;




%% RIDB
RIDB_SLOT_LEN = 32;
job = cell(0);
% pdr req 70
job{1} = [49299 60;
          ];

ridb_job = job;

end
          
% %% sync protocols
% jobs = [];
% % jobs = [jobs; scream_job(:, 1)];
% 
% for i = 1 : length(prks_job)
%     jobs = [jobs; prks_job{i}(:, 1)];
% end
% 
% for i = 1 : length(ridb_job)
%     jobs = [jobs; ridb_job{i}(:, 1)];
% end
% 
% for i = 1 : length(scream_job)
%     jobs = [jobs; scream_job{i}(:, 1)];
% end
% %{
% for i = 1 : length(ridbolama_job)
%     jobs = [jobs; ridbolama_job{i}(:, 1)];
% end
% %}
% sync_jobs = jobs;
% 
% 
% %% async protocols
% jobs = [];
% jobs = [jobs; csma_job(:, 1); rtscts_job(:, 1);];
% for i = 1 : length(cmac_job)
%     jobs = [jobs; cmac_job{i}(:, 1)];
% end
% async_jobs = jobs;

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
