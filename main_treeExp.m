%% MAIN: COST-AWARE PATH PLANNING WITH SC-LTL CONSTRAINTS (CAPP-LTL)
%   Run tree-expansion scenario.
%
%   - Reference:
%       Kyunghoon Cho, Junghun Suh, Clarie J. Tomlin, and Songhwai Oh,
%       "Cost-Aware Path Planning under Co-Safe Temporal Logic
%       Specifications," IEEE Robotics and Automation Letters, 
%       vol. 2, no. 4, pp. 2308-2315, Oct. 2017.


clc;
clear all; %#ok<CLALL>
close all;

global path_to_ltl2ba path_to_triangulate path_to_triangle use_mex

% location of extenral executable file
path_to_ltl2ba         = './external/ltl2ba/ltl2ba';
path_to_triangulate    = './external/siedel/triangulate';
path_to_triangle       = './external/triangle/triangle';

% whether to use mex-file (MATLAB version must be higher than 2016a)
use_mex = true;

% add path
addpath('./src');
addpath('./src/mex_part');
addpath('./src_utils');
if use_mex
    addpath('./ce_mex');
else
    addpath('./ce');
end

if ~exist('./result', 'dir')
   mkdir('./result')
end

% scenario specified by ltl
LTLSPEC_ARRAY       = {'<>(a&&<>(b))','(<>a) && (<>b)'};
AP_LTLSPEC_ARRAY    = {{'a' 'b'},{'a' 'b'}};

RND_SEED_ARRAY      = 1;    % random seeds
SCENARIO            = 1;    % selected scenario

initial_pos         = [1 1 pi/4]';  % initial pose (x, y, theta)

%% SET PARAMETERS
do_plot = 1;    % whether to plot
load_DP = 1;    % whether to load DiscretePlanner

triangle_type = 4;  % choose triangular decomposition method
cost_type     = 'costmap'; % chosse cost_type: 'distance', 'costmap'

% whether to allow long-extension
allow_longExtension = 1;

% dynamic constraints 'pointmass' or 'unicycle_part'
dynamic_type = 'unicycle_part';

iterMax = 1000;  % iteration number
texplore = 1;   % tree-extention number per iteration

% simulation-environment file
workspacefile  = './map/workspace_tree_exp.mat';
costmapfile    = './map/costmap_tree_exp.mat';

%% RUN
% set initial pos
x_init          = initial_pos;

% set LTL formula
LTLSPEC         = LTLSPEC_ARRAY{1,SCENARIO};
AP_LTLSPEC      = AP_LTLSPEC_ARRAY{1,SCENARIO};

% wheter to compute coverage
do_compute_coverage = 0;

% figure black
fig_black = false;

% mode for saving picture
save_pic_mode = 0;  % 0: low-resolution, 1: high-resolution

for nidx_rnd = 1:1:size(RND_SEED_ARRAY,2)
    % set seed number
    rnd_seed = RND_SEED_ARRAY(1,nidx_rnd);
    
    RUN_CAPP_LTL(rnd_seed,SCENARIO,iterMax,texplore,allow_longExtension,dynamic_type,...
            triangle_type,cost_type,LTLSPEC,AP_LTLSPEC,x_init,...
            workspacefile,costmapfile,do_plot,load_DP,do_compute_coverage,fig_black,save_pic_mode);        
    load_DP = 1;
end
