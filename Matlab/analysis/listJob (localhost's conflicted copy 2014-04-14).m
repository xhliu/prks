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
% MAIN_DIR = '~/Projects/tOR/RawData/';
% MAIN_DIR = '~/Projects/tOR/RawData/Indriya/';
MAIN_DIR = '~/Downloads/Indriya/';
%% MAIN_DIR = '~/Downloads/';

% directory to store figures
FIGURE_DIR = '~/Dropbox/iMAC/Xiaohui/figure/';
% FIGURE_DIR = '~/Dropbox/iMAC/Yu/Figure/Indriya/';
% FIGURE_DIR = '~/Dropbox/iMAC/Yu/Figure/MobiHoc/';
fprintf('Warning: please change the directory to prevent overwriting\n');

% # of protocols to compare
PROTOCOL_CNT = 6;
% # of pdr requirements
pdr_reqs = [70 80 90 95];
PDR_REQ_CNT = length(pdr_reqs);

% display shared variable
groupnames = {'70', '80', '90', '95'};
bw_xlabel = 'PDR requirement (%)';
fprintf('Warning: protocol orders in compareProtocol*.m must match bw_legend here\n');
%bw_legend = {'PRKS', 'CSMA', 'RTS-CTS', 'RIDB', 'RIDB-OLAMA', 'CMAC', 'SCREAM'};
bw_legend = {'PRKS', 'CSMA', 'RTS-CTS', 'RIDB', 'CMAC', 'SCREAM'};
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
%{
RIDBOLAMA_SLOT_LEN = 512;
job = cell(0);
% pdr req 70
job{1} = [20793 60;
          ];
% 80
job{2} = [20794 60;
          ];
% 90
job{3} = [20778 60;
          20776 60;
          ];
% 95
job{4} = [20792 60;
          ];
ridbolama_job = job;
%}
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
scream_job = [20849 90;
              20850 90;
              20851 90;
              20852 90;
              20853 90;
              20881 90];

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
job{1} = [43320 120;
          43302 60;
          ];
% 80
job{2} = [43330 120;
          43311 90;
          ];
% 90
job{3} = [43324 120;
          43301 60;
          ];
% 95
job{4} = [43329 120;
          43300 120;
          43295 60;
          ];
prks_job = job;

%% CSMA
CSMA_SLOT_LEN = 5;
csma_job = [43351 30;
            ];

%% RTSCTS
RTSCTS_SLOT_LEN = 5;
rtscts_job = [43352 30;
              43353 30;
              ];


%% RIDB
RIDB_SLOT_LEN = 32;
job = cell(0);
% pdr req 70
job{1} = [43514 30;
          ];
% 80
job{2} = [43513 30;
          ];
% 90
job{3} = [43353 30;
          43390 60;
          43456 60;
          43512 30;
          ];
% 95
job{4} = [43511 30;
          ];
ridb_job = job;



%% RIDB_OLAMA
%{
RIDBOLAMA_SLOT_LEN = 512;
job = cell(0);
% pdr req 70
job{1} = [43486 60;
          ];
% 80
job{2} = [43485 60;
          43588 60;
          ];
% 90
job{3} = [43484 60;
          43587 60;
          ];
% 95
job{4} = [43483 60;
          43523 60;
          43586 60;
          ];
ridbolama_job = job;
%}

%% CMAC
CMAC_SLOT_LEN = 5;
job = cell(0);
% pdr req 70
job{1} = [43422 30;
          43556 30;
          ];
% 80
job{2} = [43423 30;
          ];
% 90
job{3} = [43354 30;
          43389 60;
          43431 30;
          ];
% 95
job{4} = [43430 30;
          ];
cmac_job = job;


%% SCREAM
SCREAM_SLOT_LEN = 32;
scream_job = [43336 30;
              43337 60;
              43347 60;
              ];
end
          
%% sync protocols
jobs = [];
jobs = [jobs; scream_job(:, 1)];

for i = 1 : length(prks_job)
    jobs = [jobs; prks_job{i}(:, 1)];
end

for i = 1 : length(ridb_job)
    jobs = [jobs; ridb_job{i}(:, 1)];
end
%{
for i = 1 : length(ridbolama_job)
    jobs = [jobs; ridbolama_job{i}(:, 1)];
end
%}
sync_jobs = jobs;


%% async protocols
jobs = [];
jobs = [jobs; csma_job(:, 1); rtscts_job(:, 1);];
for i = 1 : length(cmac_job)
    jobs = [jobs; cmac_job{i}(:, 1)];
end
async_jobs = jobs;
