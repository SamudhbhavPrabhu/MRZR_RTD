%% description
% This script runs a simulation with the segway in the simulator
% framework, using RTD to plan online.
%
% Author: Shreyas Kousik
% Created: 13 Mar 2020
% Updated: 27 Mar 2020
%

rosshutdown;
ipAddr = '127.0.0.1';
rosinit(ipAddr)

[vel_Pub,vel_pubMsg] = rospublisher('/vel_topic','std_msgs/Float32MultiArray');
[steer_Pub,steer_pubMsg] = rospublisher('/steer_topic','std_msgs/Float32MultiArray');

%[vel_Pub,vel_pubMsg] = rospublisher('/velocity','std_msgs/Float32');
%[yaw_Pub,yaw_pubMsg] = rospublisher('/yaw','std_msgs/Float32');

ObsLoc_Sub = rossubscriber('/ObsLoc','std_msgs/String');
States_Sub = rossubscriber('/current_states','std_msgs/String');

%% user parameters
% world
%obstacle_size_bounds = [4, 4] ; % side length [min, max]
%N_obstacles = 8 ;
bounds = [-460,-330,-660,-530] ;  % [-460,-330,-660,-530];%[480,585,-145,-100]; %%% %bounds =  [-108, -49.7, 96, 148]; 
goal_radius = 3 ;

% planner
buffer = 0.001 ; % m
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;
FRS_degree = 6 ;
plot_FRS_flag = true ;
plot_HLP_flag = true ;
plot_waypoints_flag = true ;

% simulation
verbose_level = 10 ;
max_sim_time = 1000 ;
max_sim_iterations = 1000 ;

% plotting
plot_while_simulating_flag = true ;
animate_after_simulating_flag = false ;

%% automated from here
A = MRZR() ;
P = MRZR_RTD_planner('verbose',verbose_level,'buffer',buffer,...
    't_plan',t_plan,'t_move',t_move,...
    'FRS_degree',FRS_degree,...
    'plot_FRS_flag',plot_FRS_flag,...
    'plot_HLP_flag',plot_HLP_flag,...
    'plot_waypoints_flag',plot_waypoints_flag) ;

%agent_info = get_agent_info(A);
l = 4; % length
r = 0; % rotation
% obstacle rotation
R = [cos(r) sin(r) ; -sin(r) cos(r)] ;
world_info.goal = [-348.2; -545.4];%[574.2; -139]; % %% % [-60;136];
world_info.dimension = 2;
world_info.bounds = bounds;
agent_info.time = [];
agent_info.state = [];
agent_info.state1 = [];
agent_info.footprint = [3.6 1.6];

bbar = sqrt(agent_info.footprint(1)^2 + agent_info.footprint(2)^2);
P.point_spacing = P.compute_point_spacings(bbar,0.001) ;
P.agent_footprint = agent_info.footprint ;
%P.agent_max_accel = 3.75 ;
%P.agent_max_yaw_rate = 1 ;
P.bounds = world_info.bounds + [P.buffer -P.buffer P.buffer -P.buffer] ;
            
% create world bounds as an obstacle; note this passes the
% bounds in as a clockwise polyline, so everything outside of
% the world bounds counts as inside the polyline if using
% functions like inpolygon
xlo = P.bounds(1) ; xhi = P.bounds(2) ;
ylo = P.bounds(3) ; yhi = P.bounds(4) ;
            
B = [xlo, xhi, xhi, xlo, xlo ;
    ylo, ylo, yhi, yhi, ylo] ;
B = [B, nan(2,1), 1.01.*B(:,end:-1:1)] ;
            
P.bounds_as_obstacle = B ;

time_t = [];           
% 4. process the FRS polynomial
P.vdisp('Processing FRS polynomial',4)
            
P.FRS_polynomial_structure = cell(1,10) ;
            
for idx = 1:10
    I = P.FRS{idx}.FRS_polynomial - 1 ;
    z = P.FRS{idx}.z ;
    k = P.FRS{idx}.k ;
    P.FRS_polynomial_structure{idx} = P.get_FRS_polynomial_structure(I,z,k) ;
end

% 5. initialize the current plan as empty
P.vdisp('Initializing current plan',4)

P.current_plan.T = [] ;
P.current_plan.U = [] ;
P.current_plan.Z = [] ;

% 6. set up info structure to save replan dat
P.info = struct('agent_time',[],'agent_state',[],...
    'k_opt_found',[],...
    'FRS_index',[],...
    'waypoint',[],...
    'waypoints',[],...
    'obstacles',[],...
    'obstacles_in_world_frame',[],...
    'obstacles_in_FRS_frame',[],...
    'traj_opt_exitflag',[],...
    'T',[],'U',[],'Z',[]) ;

myRate = robotics.Rate(2); 
reset(myRate);

Z = zeros(5,50);
agent_state = [-441.8; -632.2; 0;0;0];
agent_estim_state = agent_state;
j = 0;

while(myRate.TotalElapsedTime<50)
    
    vel_pubMsg.Data = Z(4,:);
    steer_pubMsg.Data = Z(5,:);
    
    send(vel_Pub,vel_pubMsg);
    send(steer_Pub,steer_pubMsg);
        
    start_time1 = tic;
    States_Msg = receive(States_Sub,0.2);
    States_Msg = str2num(States_Msg.Data);
    agent_info.time = [agent_info.time, 0.5*j];
    agent_state = [States_Msg(1); States_Msg(2); States_Msg(3)*pi/180; States_Msg(4); States_Msg(5)];
    agent_info.state = [agent_info.state, agent_estim_state];
    
    ObsLoc_Msg = receive(ObsLoc_Sub,0.2);
    ObsLoc_Msg = str2num(ObsLoc_Msg.Data);
    N_obs = size(ObsLoc_Msg,2)/2;
    
    ObsX_Msg = ObsLoc_Msg(1:N_obs);
    ObsY_Msg = ObsLoc_Msg(N_obs+1:2*N_obs);
    
    if N_obs == 0
        O = [];
    else
        O = nan(2,6*N_obs);
        for idx = 1:6:(6*N_obs-1)
            % obstacle base
            o= [-l/2  l/2 l/2 -l/2 -l/2 ;
                -l/2 -l/2 l/2  l/2 -l/2 ] ;
            c = [ObsX_Msg(floor(idx/6)+1);
                 ObsY_Msg(floor(idx/6)+1)];
            O(:,idx:idx+4)= R*o + repmat(c,1,5);
        end
       
        if isnan(O(1,end))
            O = O(:,1:end-1) ;
        end
    end
    world_info.obstacles = O;


    P.vdisp('Setting up high-level planner',4)
    P.HLP.setup(agent_info,world_info) ;
    %P.HLP.default_lookahead_distance = P.lookahead_distance ;
    
    [T,U,Z] = P.replan(agent_info,world_info);
    Z(5,:) = bound_values(Z(5,:),A.max_wheelangle) ;
            
    [tout,zout] = A.integrator(@(t,z) A.dynamics(t,z,T,U,Z), [0 t_move], agent_state) ;
    agent_estim_state = zout(:,end);
    
%     z = agent_state(1:2,1) ;
%     dz = z - repmat(world_info.goal,1,size(z,2)) ;
%     out = min(vecnorm(dz,2)) <= goal_radius ;
%     
%     if out
%     Z(4,:) = 0;
%     Z(5,:) = 0;
%     vel_pubMsg.Data = Z(4,:);
%     steer_pubMsg.Data = Z(5,:);
%     
%     send(vel_Pub,vel_pubMsg);
%     send(steer_Pub,steer_pubMsg);
%     break
%     end
    
    time1 = toc(start_time1);
    j = j+1;
    pause(0.5 - time1);
    

    %%  Block execution based on the rate defined.
% End while-loop
end