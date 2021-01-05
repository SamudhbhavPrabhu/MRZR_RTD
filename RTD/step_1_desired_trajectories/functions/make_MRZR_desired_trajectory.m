function [T,U,Z] = make_MRZR_desired_trajectory(t_f,w_des,v_des)
% [T,U,Z] = make_segway_desired_trajectory(t_f,w_des,v_des)
%
% Create a Dubins path as a full-state trajectory for the TurtleBot.
%
% The inputs are:
%   t_f      planning time horizon
%   w_des    desired yaw rate
%   v_des    desired speed
%
% The outputs are:
%   T        timing for desired trajectory as a 1-by-N array
%   U        desired input (yaw rate and acceleration) as 2-by-N array
%   Z        desired trajectory (x,y,h,v) as a 4-by-N array
%
% Note, this is identical to the turtlebot desired trajectories in the RTD
% tutorial.
%
% Author: Shreyas Kousik
% Created: 9 Mar 2020
% Updated: -

    % set up timing
    t_sample = 0.01 ;
    T = unique([0:t_sample:t_f,t_f],'stable');
    N_t = length(T) ;
    l =  3;
    
    % get inputs for desired trajectories
    w_traj = w_des*ones(1,N_t) ;
    v_traj = v_des*ones(1,N_t) ;
    U_in = [w_traj ; v_traj] ;
    
    if v_des~=0
        wheelangle_traj = atan2(l*w_traj,v_traj);
    else
        wheelangle_traj = zeros(1,N_t);
    end

    % compute desired trajectory
    z0 = zeros(3,1) ;
    [~,Z] = ode45(@(t,z) MRZR_trajectory_producing_model(t,z,T,U_in),T,z0) ;

    % append velocity to (x,y,h) trajectory to make it a full-state
    % trajectory for the turtlebot
    %Z = [Z' ; w_traj ; v_traj] ;
    Z = [Z' ; v_traj ; wheelangle_traj] ;
    
    % compute inputs for robot
    %a_traj = [diff(v_traj)./t_sample, 0] ;
    %U = [w_traj ; a_traj] ;
    U = [v_traj;wheelangle_traj] ;
end