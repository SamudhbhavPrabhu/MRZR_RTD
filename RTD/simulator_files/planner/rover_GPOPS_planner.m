classdef rover_GPOPS_planner< planner
    properties
    % implementation-specific properties
        gpops_problem
        C % parallel cluster so we can run jobs with a timeout

        braking_flag
        stop_flag
        T_old
        U_old
        Z_old % plan from previous iteration
        agent_state % current state
        
        use_coarse_initial_guess = true
        T_min=0.5 % lower bound on timing, can be used to encourage safety
        T_max=1.5% upper bound on timing, encourages robot to go faster
        
        upper_boundary
        lower_boundary
        
        bounds_combined  
        
        max_heading = 0.6;
        
        lookahead_distance = 4;
        desired_speed = 2;
        max_speed = 2;
        min_speed = 0;
        max_wheelangle = 0.5;
        speed_weight =1;
        longitudinal_weight = 1;
        steering_weight = 1;
        lateral_weight = 1;
        
        obstacle_buffer = [0,0;0,0];
   
    end
    
    methods
    %% constructor
    function P = rover_GPOPS_planner(varargin)
        % set default parameters
        t_plan = 0.5 ;
        timeout = 0.5;
        t_move = 0.5 ;
        buffer = 0;
        
        % parse arguments
        P@planner('t_plan',t_plan,'timeout',timeout,'t_move',t_move,'buffer',buffer,varargin{:})
       
    end
    
    %% setup
    function setup(P,agent_info,world_info)
        
        
        
        % get world bounds
        P.upper_boundary = world_info.road_upper_boundary;
        P.lower_boundary = world_info.road_lower_boundary;
        
                            
        % make GPOPS problem object
        P.agent_state = agent_info.state(:,end);
        P.gpops_problem = P.make_GPOPS_problem_object(agent_info,world_info) ;
        
        % initialize old state
        P.T_old = 0 ;
        P.U_old = [0 ; 0] ;
        P.Z_old = [world_info.start ; 0 ; 0] ;

        % call IPOPT to make sure GPOPS doesn't crash
        try
            ipopt
        catch
        end
        
        % set up structure for info
        I = struct('N_collocation_points',[],...
                   'N_constraints',[]) ;
        P.info = I ;
        
        % set up plotting info
        
        P.braking_flag=false;
        P.stop_flag=false;
        
        %get obstacle buffer
        rotated_vertices = [cos(P.max_heading) -sin(P.max_heading);sin(P.max_heading) cos(P.max_heading)]*agent_info.footprint_vertices;

        L = [min(rotated_vertices(1,:)),max(rotated_vertices(1,:))];
        W = [min(rotated_vertices(2,:)),max(rotated_vertices(2,:))];

        rotated_vertices = [cos(P.max_heading) -sin(P.max_heading);sin(P.max_heading) cos(P.max_heading)]'*agent_info.footprint_vertices;

        L = [min([L(1),rotated_vertices(1,:)]),max([L(2),rotated_vertices(1,:)])];
        W = [min([W(1),rotated_vertices(2,:)]),max([W(2),rotated_vertices(2,:)])];
        
        P.obstacle_buffer = P.buffer*[-1,1;-1 1]+[L;W];
        
    end

    
    %% replan
    function [Tout,Uout,Zout] = replan(P,agent_info,world_info)
        P.braking_flag=false;

        O = world_info.obstacles;
        % put obstacles into problem (obstacles must be in ccw order
        % separated by NaNs)
        if ~isempty(O)
            NObs = size(O,2)/6;
            
            buffer_stencil = [P.obstacle_buffer(1,[1 2 2 1 1]);...
                P.obstacle_buffer(2,[1 1 2 2 1])];
            buffer_stencil = repmat([buffer_stencil,NaN(2,1)],[1 NObs]);
            
            P.gpops_problem.auxdata.NObs = NObs ;
            
            P.current_obstacles = O+buffer_stencil;
            
            obs_nan_idxs=[0,find(isnan(O(1,:)))];
        else
            

            P.gpops_problem.auxdata.NObs=0;
            NObs=0;
            P.current_obstacles = [] ;
        end
        
        
        %generate constraint matrices for obstacles
        
        
        % get state information from agent
        P.agent_state = agent_info.state(:,end) ;
   
        xy = P.agent_state(agent_info.position_indices) ;
        h = P.agent_state(agent_info.heading_index);
        % get waypoints
        
        wp = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
       
 
       P.gpops_problem.auxdata.goal = wp;
        

   %we will just use limits on the y state for road boundaries
%           Npred=length(move_idxs)-1;   
%          Aineq_road = zeros( 4 * Npred, 2 );
%          bineq_road = zeros( 4 * Npred, 1 );
%          for i = 1:Npred
%           [ Aineq_road( ( 1:4 ) + ( i - 1 ) * 4, : ), bineq_road( ( 1:4 ) + ( i - 1 ) * 4 ), ~, ~ ]  = ...
%               vert2lcon( [ P.outer_bounds( 1:2, move_idxs(i) )'; P.outer_bounds(1:2, move_idxs(i+1) )';  P.inner_bounds(1:2, move_idxs(i) )';  P.inner_bounds(1:2, move_idxs(i+1) )' ] );
%          end% 

        
% % %        
% %     
 
        %generate_obstacle_constraint for obstacles that are within
        %waypoint distance and in front of the vehicle
        Aineq = cell(NObs,1);
        bineq = cell(NObs,1);
        k=1;
        for j=1:NObs
            Ocur=P.current_obstacles(:,obs_nan_idxs(j)+1:obs_nan_idxs(j+1)-1);
            Olocal= rotation_matrix_2D( h )'*(Ocur-xy);
            if min(dist_point_to_points(xy,Ocur))<P.lookahead_distance && any(Olocal(1,:)>-sqrt(sum(agent_info.footprint.^2)))
            [Aineq_temp,bineq_temp,~,~]=vert2lcon(Ocur');
            Aineq{k+1}=Aineq_temp;
            bineq{k+1}=bineq_temp;
            P.current_obstacles=[P.current_obstacles,[Ocur,NaN(2,1)]];
            
            
            k=k+1;
             else
                P.gpops_problem.auxdata.NObs = P.gpops_problem.auxdata.NObs-1;
             end
            
        end
        
        Lempty =  cellfun(@isempty,Aineq);
        
        Aineq = Aineq(~Lempty);
        bineq = bineq(~Lempty);
        
        %**road constraint should be negative, obstacle constraints should be
        %positive**
         P.gpops_problem.bounds.phase.path.lower = zeros(1,P.gpops_problem.auxdata.NObs);
         P.gpops_problem.bounds.phase.path.upper = Inf(1,P.gpops_problem.auxdata.NObs);
             
       
        P.gpops_problem.auxdata.Aineq=Aineq;
        P.gpops_problem.auxdata.bineq=bineq;

%        % set up planner problem
       P.gpops_problem.bounds.phase.initialstate.lower = P.agent_state' ;
       P.gpops_problem.bounds.phase.initialstate.upper = P.agent_state' ;
        
       
        
        % set up initial guess
        if P.use_coarse_initial_guess || isempty(P.Z_old) || length(P.T_old)<2
            P.vdisp('Using coarse initial guess!',3) ;
            x_guess = [xy(1); xy(1) + P.agent_state(agent_info.velocity_index)] ;
            y_guess = [xy(2); xy(2)] ;
            h_guess = [h; h] ;
            vx_guess = [P.agent_state(agent_info.velocity_index); P.agent_state(agent_info.velocity_index)] ;

            
            T = [0,1] ;
            U = zeros(2,2) ;
            Z = [x_guess, y_guess, h_guess, vx_guess,[0;0]]' ;
            P.use_coarse_initial_guess = false ;
        else
            P.vdisp('Using old trajectory as initial guess!',3) ;
            
            % find closest point in Zold to current state
            T = P.T_old ;
            U = zeros(2,length(T)) ; % note that we discard the braking row
            Z = [P.Z_old(1:4,:) ;P.U_old(2,:)];
            xy_old = Z(1:2,:) ;
            [~,idx] = min(dist_point_to_points(xy,xy_old)) ;
            
            % get the xy difference and shift Z by that much
            dxy = xy - xy_old(:,1) ;
            Z(1:2,:) = Z(1:2,:) + repmat(dxy,1,size(Z,2)) ;
            
            T = T(idx:end) - T(idx) ;
            U = U(:,idx:end) ;
            Z = Z(:,idx:end) ;
        end
        
        P.gpops_problem.guess.phase.time = T(:) ;
        P.gpops_problem.guess.phase.control = U' ;
        P.gpops_problem.guess.phase.state = Z' ;

        % run planner
        gp = P.gpops_problem ;

        % update planner info
        I = P.info ;
        % reset timer
       P.gpops_problem.auxdata.timer_start = tic ;
       
         try
            P.vdisp('Running GPOPS.',5)
            output = gpops2(gp) ;
         catch
              P.vdisp('GPOPS errored!',5)
              output = [] ;
          end

        if ~isempty(output)
            P.vdisp('GPOPS converged successfully!',3)
            solution = output.result.solution ;

            I.N_collocation_points = [I.N_collocation_points, length(solution.phase.time)] ;
            if ~isempty(O)
                I.N_constraints = [I.N_constraints, (sum(~isnan(O(1,:))) - sum(isnan(O(1,:))))] ;
            else
                I.N_constrains = [I.N_constraints,0] ;
            end

            Tout = solution.phase.time' ;
           
            Uout = solution.phase.control' ;
            Zout = solution.phase.state' ;
        
      
        else
            P.vdisp('GPOPS failed to solve in time! Slamming on the brakes!',4)

            I.N_collocation_points = [I.N_collocation_points, nan] ;
            I.N_constraints = [I.N_constraints, nan] ;
            P.braking_flag=true;
            P.stop_flag=true;
            Tout = [0,5] ;          
            Uout = [0,0;0,0] ;
            Zout = [P.agent_state,P.agent_state] ;
        end
        
        P.info = I ;
        
        P.T_old = Tout ;
        P.U_old = Uout ; % added brake row
        P.Z_old = Zout ;

        if ~isempty(Zout)
            P.current_plan = Zout(agent_info.position_indices,:) ;
        end
    end
    
    %% make GPOPS problem object
    function out = make_GPOPS_problem_object(P,~,world_info)
        bounds.phase.initialtime.lower = 0 ;
        bounds.phase.initialtime.upper = 0 ;
        bounds.phase.finaltime.lower   = P.T_min ;
        bounds.phase.finaltime.upper   = P.T_max ;

        % state bounds, states z = [x,y,h,vx,delta]

        xmin = -Inf;
        xmax = Inf ;
        ymin = min(world_info.road_lower_boundary(2,:));
        ymax = max(world_info.road_upper_boundary(2,:)) ;
        hmin = -P.max_heading ;
        hmax = P.max_heading ;
        vxmin = P.min_speed ;
        vxmax = P.max_speed ;
        deltamax = P.max_wheelangle;


        bounds.phase.state.lower        = [xmin, ymin, hmin, vxmin,-deltamax] ;
        bounds.phase.state.upper        = [xmax, ymax, hmax, vxmax,deltamax] ;
        
        bounds.phase.finalstate.lower   = [-Inf * ones(1, 4),-deltamax];
        bounds.phase.finalstate.upper   = [ Inf * ones(1, 4),deltamax];
        
        bounds.phase.path.lower = -Inf ;
        bounds.phase.path.upper = 0 ;
        
    
        bounds.phase.control.lower = [vxmin, -deltamax];
        bounds.phase.control.upper = [vxmax, deltamax];     
        bounds.phase.integral.lower=0;
        bounds.phase.integral.upper=1000000;
        
        % default initial guess
        t_guess = [0; P.T_max];
        spd_guess = [P.max_speed ; P.max_speed ];
        wa_guess = [0 ; 0 ];

        guess.phase.time = t_guess ;
        guess.phase.control = [spd_guess, wa_guess] ;
        guess.phase.integral=0;
        
        % mesh settings
        mesh.method = 'hp-LiuRao-Legendre';
        mesh.tolerance = 1e-3; 
        mesh.maxiterations = 4;
        mesh.colpointsmin = 2 ;
        mesh.colpointsmax = 10 ;
        mesh.phase.colpoints = 4*ones(1,10);
        mesh.phase.fraction = 0.1*ones(1,10);

        % obstacle gpops_problem
        auxdata.timer_start = tic ;
        auxdata.timeout = P.timeout ;
        auxdata.desired_speed = P.desired_speed;
        auxdata.speed_weight = P.speed_weight;
        auxdata.longitudinal_weight = P.longitudinal_weight;
        auxdata.lateral_weight = P.lateral_weight;
        auxdata.steering_weight = P.steering_weight;

        % finalize setting up GPOPS problem object
        out.bounds = bounds ;
        out.guess = guess ;
        out.mesh = mesh ;
        out.name = 'rover_simulation' ;
        out.functions.continuous = @dynamics_function ;
        out.functions.endpoint = @endpoint_function ;
        out.auxdata = auxdata ;
        out.nlp.solver = 'ipopt' ;
        out.displaylevel = 2 ;
        out.derivatives.derivativelevel = 'second' ;
        out.derivatives.supplier = 'sparseCD' ;
    end
    
   
    end
end

function out = dynamics_function(input)
% dynamics
l = 0.3265;
lr = 0.0765;

    % states z = [x,y,h,vx,vy,yr]
%     x = input.phase.state(:,1);
   y = input.phase.state(:,2);
    h = input.phase.state(:,3);
    vx = input.phase.state(:,4);
    delta = input.phase.state(:,5);


    delta_des = input.phase.control(:,2);
    v_des = input.phase.control(:,1);

    yawrate = tan(delta).*vx./(l+4.4e-7*vx.^2);
  
    vy = yawrate.*(lr-0.0140*vx.^2);
    
    cr = -0.0811;
    dvxdt = cr-1.4736*(vx-v_des)+0.1257*(vx-v_des).^2;
    
    deltad = -5*(delta-delta_des);
    
    dzdt = [vx.*cos(h)-vy.*sin(h), vx.*sin(h)+vy.*cos(h), yawrate, dvxdt,deltad];

    out.dynamics  = dzdt ;
    
    %add input cost
    gy = input.auxdata.goal(2) ;
    speed_weight = input.auxdata.speed_weight;
    steering_weight = input.auxdata.steering_weight;
    lateral_weight = input.auxdata.lateral_weight;
    
    out.integrand= speed_weight * (input.phase.control(:,1)-input.auxdata.desired_speed).^2+steering_weight*input.phase.control(:,2).^2 +lateral_weight*(y-gy).^2;
    

        
% obstacle check
    Npoints = size( input.phase.state( :, 1 ), 1 );
    
    
 %do not need these (use state constraint to check if we are on road)
% testpoints = input.auxdata.Aineq{1} * [ input.phase.state( :, 1 ) input.phase.state( :, 2 ) ]' - input.auxdata.bineq{1};
% road_check = min( reshape( max( reshape( testpoints, 4, Npoints * input.auxdata.Npred ) ), input.auxdata.Npred, Npoints ) )';

if input.auxdata.NObs >=1
obs_check=NaN(Npoints,input.auxdata.NObs);
for i=1:input.auxdata.NObs
    obs_testpoints=input.auxdata.Aineq{i}*[ input.phase.state( :, 1 ) input.phase.state( :, 2 ) ]'-input.auxdata.bineq{i};
    obs_check(:,i)= max(obs_testpoints)';
end
%     out.path=[road_check,obs_check];
 out.path= obs_check;
else
%     out.path=road_check;

end

end

function out = endpoint_function(input)
        x = input.phase.finalstate(1);


        gx = input.auxdata.goal(1) ;
    
        longitudinal_weight = input.auxdata.longitudinal_weight;
        

        out.objective = longitudinal_weight*(x-gx).^2 +input.phase.integral;

        if timeout_check(input)
             error('out of time')
        end
end

function out = timeout_check(input)
    out = toc(input.auxdata.timer_start) >= input.auxdata.timeout;
end

