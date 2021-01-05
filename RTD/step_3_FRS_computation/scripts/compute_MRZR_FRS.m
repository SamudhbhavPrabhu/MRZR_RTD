%% description
% This script computes a Forward-Reachable Set (FRS) for the sgway. The
% user specifies the range of initial speeds; all other info is loaded from
% the relevant .mat files.
%
% Note, you need MOSEK and spotless on your MATLAB path to run this script!
%
% Author: Shreyas Kousik
% Created: 10 Mar 2020
% Updated: 14 Mar 2020

clear ; clc ; close all ;

%% user parameters
% degree of SOS polynomial solution
degree = 6 ; % this should be 6 or less unless you have like 100+ GB of RAM

% whether or not to include tracking error
include_tracking_error = true ;

% speed range (uncomment one of the following)
% v_0_range = [0.0, 0.5] ;
% v_0_range = [0.5, 1.0] ;
% v_0_range = [1.0, 1.5] ;
% v_0_range = [1.5, 2.0] ;
% v_0_range = [2.0, 2.5] ;
% v_0_range = [2.5, 3.0] ;
% v_0_range = [3.0, 3.5] ;
% v_0_range = [3.5, 4.0] ;
% v_0_range = [4.0, 4.5] ;
% v_0_range = [4.5, 5.0] ;

% whether or not to save output
save_data_flag = true ;

% whether or not to run the solver
run_solver_flag = true ; 

%% automated from here
% load timing
load('MRZR_timing.mat')

% load the error functions and distance scales
switch v_0_range(1)
    case 0.0
        load('MRZR_error_functions_v_0_0.0_to_0.5.mat')
        load('MRZR_FRS_scaling_v_0_0.0_to_0.5.mat')
    case 0.5
        load('MRZR_error_functions_v_0_0.5_to_1.0.mat')
        load('MRZR_FRS_scaling_v_0_0.5_to_1.0.mat')
    case 1.0
        load('MRZR_error_functions_v_0_1.0_to_1.5.mat')
        load('MRZR_FRS_scaling_v_0_1.0_to_1.5.mat')
    case 1.5
        load('MRZR_error_functions_v_0_1.5_to_2.0.mat')
        load('MRZR_FRS_scaling_v_0_1.5_to_2.0.mat')
    case 2.0
        load('MRZR_error_functions_v_0_2.0_to_2.5.mat')
        load('MRZR_FRS_scaling_v_0_2.0_to_2.5.mat')
    case 2.5
        load('MRZR_error_functions_v_0_2.5_to_3.0.mat')
        load('MRZR_FRS_scaling_v_0_2.5_to_3.0.mat')
    case 3.0
        load('MRZR_error_functions_v_0_3.0_to_3.5.mat')
        load('MRZR_FRS_scaling_v_0_3.0_to_3.5.mat')
    case 3.5
        load('MRZR_error_functions_v_0_3.5_to_4.0.mat')
        load('MRZR_FRS_scaling_v_0_3.5_to_4.0.mat')
    case 4.0
        load('MRZR_error_functions_v_0_4.0_to_4.5.mat')
        load('MRZR_FRS_scaling_v_0_4.0_to_4.5.mat')
    case 4.5
        load('MRZR_error_functions_v_0_4.5_to_5.0.mat')
        load('MRZR_FRS_scaling_v_0_4.5_to_5.0.mat')
    otherwise
        error('Please pick a valid speed range for the MRZR FRS!')
end


% create agent to use for footprint
A = MRZR() ;
footprint = A.footprint ;

%% set up the FRS computation variables and dynamics
% set up the indeterminates
t = msspoly('t', 1) ; % time t \in T
z = msspoly('z', 2) ; % state z = (x,y) \in Z
k = msspoly('k', 2) ; % parameters k \in K

x = z(1) ; y = z(2) ;

% create polynomials that are positive on Z, and K, thereby
% defining them as semi-algebraic sets; h_T is automatically generated
Z_range = [-1, 1 ; -1, 1] ; % z \in [-1,1]^2

%Z0_radius = footprint/distance_scale ; % z(0) \in Z_0

K_range = [-1, 1 ; -1, 1] ; % k \in [-1,1]^2

h_Z = (z - Z_range(:,1)).*(Z_range(:,2) - z) ;

L = [min(A.footprint_vertices(1,:)), max(A.footprint_vertices(1,:))];
W = [min(A.footprint_vertices(2,:)), max(A.footprint_vertices(2,:))];

h_Z0 = [(x-L(1))*(L(2)-x);(y-W(1))*(W(2)-y)];
% h_Z0 = 1 - ((x - initial_x)/(footprint/distance_scale)).^2 + ...
%          - ((y - initial_y)/(footprint/distance_scale)).^2 ;

h_K = (k - K_range(:,1)).*(K_range(:,2) - k) ;

%% specify dynamics and error function
% set up w_des in terms of k_1
w_des = w_max*k(1) ;

% set up v_des in terms of k_2
v_range = [v_0_min - delta_v, v_0_max + delta_v] ;
v_range = bound_values(v_range,[0, max_speed]) ;
v_des = (diff(v_range)/2)*k(2) + mean(v_range) ;

% create dynamics
scale = (time_scale/distance_scale) ;
f = scale*[v_des - distance_scale*w_des*(y - initial_y) ;
                 + distance_scale*w_des*(x - initial_x)] ;

% create tracking error dynamics; first, make the monomials of time in
% decreasing power order
g_x_t_vec = t.^(length(g_x_coeffs)-1:-1:0) ;
g_y_t_vec = t.^(length(g_y_coeffs)-1:-1:0) ;
g = scale*[g_x_t_vec*g_x_coeffs', 0 ;
           0, g_y_t_vec*g_y_coeffs'] ;

%% create cost function
% this time around, we care about the indicator function being on Z x K
int_ZK = boxMoments([z;k], [Z_range(:,1);K_range(:,1)], [Z_range(:,2);K_range(:,2)]);

%% setup the problem structure
solver_input_problem.t = t ;
solver_input_problem.z = z ;
solver_input_problem.k = k ;
solver_input_problem.f = f ;
solver_input_problem.hZ = h_Z ;
solver_input_problem.hZ0 = h_Z0 ;
solver_input_problem.hK = h_K ;
solver_input_problem.cost = int_ZK ;
solver_input_problem.degree = degree ;

if include_tracking_error
    solver_input_problem.g = g ;
end

%% create filename for saving
filename = ['MRZR_FRS_deg_',num2str(degree),'_v_0_',...
    num2str(v_0_min,'%0.1f'),'_to_',...
    num2str(v_0_max,'%0.1f'),'.mat'] ;

%% compute FRS without tracking error
if run_solver_flag
    disp('Running solver to compute FRS!')
    solve_time = tic ;
    solver_output = compute_FRS(solver_input_problem) ;
    solve_time = toc(solve_time) ;
    
    % extract FRS polynomial result
    FRS_polynomial = solver_output.indicator_function ;
    FRS_lyapunov_function = solver_output.lyapunov_function ;
else
    disp('Not running solver! Loading existing FRS instead.')
    load(filename)
end

%% save result
if save_data_flag
    disp(['Saving FRS output to file: ',filename])
    
    % save output
    save(filename,'FRS_polynomial','FRS_lyapunov_function','t','z','k',...
        'time_scale','distance_scale',...
        'v_0_min','v_0_max','v_des','w_des',...
        'max_speed','footprint','f','g','initial_x','initial_y',...
        't_plan','v_range','delta_v','degree','h_Z','h_Z0','h_K',...
        'w_max','w_min','delta_w')
end