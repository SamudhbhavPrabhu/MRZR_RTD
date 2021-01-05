classdef MRZR < RTD_agent_2D
    properties
        max_speed = 5;
        min_speed = 0; %3
        max_wheelangle = 1;
        speed_index = 4;
        wheelbase = 3;
        rear_axel_to_center_of_mass = 0.7;
        
    end
        
     methods
        %% constructor
        function A = MRZR(varargin)
            % set up default superclass values
            name = 'MRZR' ;
            default_footprint = [3.6 1.6] ;
            n_states = 5 ;
            n_inputs = 2 ; % two reference actually
            stopping_time = 3; % conservative estimate
            sensor_radius = 30;
            %LLC = rover_PD_LLC; % doesnt need a controller

            % create agent
            A@RTD_agent_2D('name',name,...
                'footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,'sensor_radius',sensor_radius,varargin{:}) ; %'LLC',LLC,varargin{:}
        end
        
        %% functions to convert yawrate and velocty to wheel angle and vice-versa
        
        function yawrate = wheelangle_to_yawrate(A,v,wheelangle)
            l = A.wheelbase;
            yawrate = tan(wheelangle).*v./l;
        end
        
        function wheelangle = yawrate_to_wheelangle(A,v,yawrate)
            l = A.wheelbase;
            wheelangle = atan(yawrate.*l./v);
            
            wheelangle(v==0) = sign(yawrate)*A.max_wheelangle;  
        end
        
        function vy = wheelangle_to_lateral_veloctity(A,v,wheelangle)
              w = A.wheelangle_to_yawrate(v,wheelangle);
              vy = w.*(A.rear_axel_to_center_of_mass);
        end
         %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            % handle no desired trajectory input
            if nargin < 6
                Z = [] ;
            end
    
            
            % extract the states
            h = z(A.heading_index) ;
            v = z(A.speed_index) ;
            wheelangle = z(5);
            
            % get nominal control inputs
            %u = A.LLC.get_control_inputs(A,t,z,T,U,Z) ;
            %u(isnan(u)) = 0 ; % safety check
            if isempty(Z)
                % if no desired trajectory is passed in, then we assume
                % that we are emergency braking
                u_des = match_trajectories(t,T,U,'previous') ;
                % create output
                if numel(T_des) == 1
                    U  = u_des;
                else
                    U  = match_trajectories(t,T,U,'previous');
                end
                
            else
                % otherwise, we are doing feedback about a desired
                % trajectory
                [u_des,z_des] = match_trajectories(t,T,U,T,Z,'previous') ;
            
                v_des = z_des(4);
                delta_des = z_des(5);
            end
            
            % saturate the inputs
            v_des = bound_values(v_des,A.min_speed,A.max_speed) ;
            delta_des = bound_values(delta_des,A.max_wheelangle) ;
                   
            % calculate the derivatives
            w = A.wheelangle_to_yawrate(v,wheelangle);
            vy = A.wheelangle_to_lateral_veloctity(v,wheelangle);
            
            
            xd = v*cos(h)-vy*sin(h);
            yd = v*sin(h)+vy*cos(h);
            
            hd = w ;
            if v_des >= 0 && v_des <= 1
              vd = -1.2*(v-v_des);            
            elseif v_des > 1 && v_des <= 2
              vd = -0.3-5*(v-v_des)-2.16*(v-v_des)^2;
            elseif v_des > 2 && v_des <= 3
              vd = -0.5-5*(v-v_des)-1.6*(v-v_des)^2;
            elseif v_des > 3 && v_des <= 4
              vd = -0.5-4.7*(v-v_des)-1.3*(v-v_des)^2; 
            else
              vd = -0.5-4.3*(v-v_des)-0.82*(v-v_des)^2;  
            end
            deltad = -25*(wheelangle-delta_des);
            % return state derivative
            zd = [xd ; yd ; hd ; vd;deltad] ;
            
        end
        
                %% get agent into
        function agent_info = get_agent_info(A)
            % call superclass method
            agent_info = get_agent_info@agent(A) ;
            
            % additional fields
            agent_info.input = A.input;
            agent_info.input_time = A.input_time;
            agent_info.heading_index = A.heading_index ;
            agent_info.velocity_index = 4;
            agent_info.yaw_rate = A.wheelangle_to_yawrate(A.state(4,:),A.state(5,:));
            agent_info.desired_time = A.desired_time ;
            agent_info.desired_input = A.desired_input ;
            agent_info.desired_trajectory = A.desired_trajectory ;
            agent_info.heading_index = A.heading_index ;
            agent_info.footprint = A.footprint ;
            agent_info.footprint_vertices = A.footprint_vertices ;
            agent_info.wheelbase = A.wheelbase;
            agent_info.rear_axel_to_center_of_mass = A.rear_axel_to_center_of_mass;
        end
     end
    
end