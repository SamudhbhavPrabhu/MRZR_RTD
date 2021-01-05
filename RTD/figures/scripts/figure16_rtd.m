%% description
% This script generates Fig. 16 in the paper, for the RTD planner 
%
% Author: Sean Vaskov
% Created: 10 Apr 2020

%% user parameters
save_pdf_flag = true ;
trial = 187; %% 22 for planners stop (b), 187 for trial where RRT crashes (a)
xplotlimits = [3 23]; %[-1 15] for trial 22, [3 23] for trial 187

%%

load('rover_simulation_worlds.mat')
load(['experiment_2/data_for_figures/rover_experiment_2_summary_world_',num2str(trial,'%04d'),'_with_info_for_RTD.mat'])

close all
fh = figure;

W = W_all{trial};

frs_color = [0,0.75,0.25];
ftprint_color = [0.8 0.8 1];
yplotlimits = [-1 1];
obs_size = 1;

W.bounds_as_polyline = NaN(2,5);
W.plot;

A = RoverAWD();


FRS_directory = '~/MATLAB/IJRR_bridging_the_gap/step_3_FRS_computation/data/rover_reconstructed';
buffer = 0.01;

   P = rover_RTD_planner('FRS_directory',FRS_directory,'buffer',buffer,'filtering_poly',reachable_set_poly);
   P.setup(A.get_agent_info,W.get_world_info(A.get_agent_info,P))
   
   O = buffer_box_obstacles(W.obstacles,P.FRS_buffer,'a',P.arc_point_spacing) ;
   O = interpolate_polyline_with_spacing(O,P.point_spacing) ;
   
plot(summary.trajectory(1,:),summary.trajectory(2,:),'b','LineWidth',1.0)
planning_indices = 1:3:length(summary.planner_info.FRS_index_used_for_plan);
plot(O(1,:),O(2,:),'r.','MarkerSize',obs_size)

for i = planning_indices
    
    z_act = summary.planner_info.agent_state_used_for_plan(:,i);
    P.agent_state = z_act;
    A.state = z_act;
    P.process_world_info(W.get_world_info(A,P))
    P.current_FRS_index = summary.planner_info.current_FRS_index(i);
    
    %plot footprint
     V_fp = z_act(1:2)+rotation_matrix_2D(z_act(3))*A.footprint_vertices;
            V_arrow = z_act(1:2)+rotation_matrix_2D(z_act(3))*A.arrow_vertices;
            
      
            hold on
            fill(V_fp(1,:),V_fp(2,:),ftprint_color,...
                'EdgeColor',A.plot_footprint_edge_color,...
                'FaceAlpha',1,...
                'EdgeAlpha',A.plot_footprint_edge_opacity) ;
            
            % plot arrow on footprint
            fill(V_arrow(1,:),V_arrow(2,:),A.plot_arrow_color,...
                'EdgeColor',A.plot_arrow_color,...
                'FaceAlpha',1,...
                'EdgeAlpha',A.plot_arrow_opacity) ;
     
            if ~any(isnan(summary.planner_info.k_opt_found(:,i)))
                FRS = P.FRS{P.current_FRS_index};
                
                z = FRS.z;
                zscale = FRS.zscale;
                zoffset = FRS.zoffset;
                k = FRS.k;
                psiend_k2 = (-z_act(3)-FRS.psi_end_min)*2/(FRS.psi_end_max-FRS.psi_end_min)-1;
                psiend_k2 = bound_values(psiend_k2,1);

               wx = subs(P.FRS{P.current_FRS_index}.w,k,[summary.planner_info.k_opt_found(1,i);psiend_k2;summary.planner_info.k_opt_found(2,i)]);
               
               wk = sub_z_into_w(P.w_polynomial_info,P.current_obstacles_in_FRS_coords) ;
                              
               wx_pts = eval_w(summary.planner_info.k_opt_found(:,i),wk.wkcoef,wk.wkpows);
               
               L = wx_pts <= 1;
               outside_pts = rotation_matrix_2D(z_act(3))* (repmat(zscale(1:2),[1 sum(L)]).*P.current_obstacles_in_FRS_coords(:,L)-zoffset(1:2))+z_act(1:2);
               
                
               h = get_2D_contour_points(wx,z(1:2),1,'Scale',zscale(1:2),'Offset',[zoffset(1);zoffset(2)],'Pose',z_act(1:3),'N',100);
                plot(h(1,:),h(2,:),'Color',frs_color,'LineWidth',1.0)
        
               
               
            end
            
           
      
end
textpos = [xplotlimits(2)-0.2,1];
text(textpos(1),textpos(2),'RTD', 'EdgeColor','none','Margin',1,...
  'HorizontalAlignment', 'right','VerticalAlignment','top','FontSize',14,'BackgroundColor','w')

axis equal
  set(gca,'Layer','Top',...
      'Box',    'on',...
      'TickDir', 'in',...
      'Xminortick', 'off',...
      'Yminortick', 'off',...
      'YGrid',  'off',...
      'XColor', [0 0 0],...
      'Ycolor', [0 0 0],...
      'Xtick',  linspace(xplotlimits(1),xplotlimits(2),5),...
       'Ytick',  linspace(yplotlimits(1),yplotlimits(2),3),...
      'Linewidth', 1.0 );
  set(gca,'Fontsize',15);
  set(gca,'fontname','Times New Roman')
  xlabel('x [m]')
 xlim(xplotlimits)
 ylim(yplotlimits)
  ylabel('y [m]')
  
  set_plot_linewidths(1.25)
   
  ax = gca;
  ax.YAxis.TickLabelFormat = '%.0f';
  ax.XAxis.TickLabelFormat = '%.0f';
   

 
if save_pdf_flag
    save_figure_to_pdf(fh,['RTD_rover_trial_',num2str(trial),'.pdf'])
end

