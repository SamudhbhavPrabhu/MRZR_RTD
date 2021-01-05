function [P,patch_data,N_vertices] = get_2D_contour_points(p,x,l,varargin)
% P = get_2D_contour_points(p,x,l,'keyword1',value1,'keyword2',value2,...)
% [P,patch_data] = get_2D_contour_points(...)
%
% Given an msspoly p in the 2D variable x, return a 2-by-N polyline of the
% set where p(x) = l, i.e. the l-level set of p.
%
% In addition to p, x, and l, additional inputs can be passed in as keyword
% arguments. The keywords allowed are:
%   Offset          a 2D position offset of the origin in the frame used
%                   for computing p (e.g., if p represents a semi-algebraic
%                   set)
%
%   Pose            a 3-by-1 in SE(2) where the contour should be centered
%                   at and rotated by
%
%   Scale           a 2-by-1 scalar representing how much 2D space is scaled in
%                   the x and y dimensions for computing p
%
%   Bounds          [xlo,xhi,ylo,yhi] lower and upper bounds of the space
%                   containing the contour (for now, these bounds are
%                   applied to both x and y axes) in the frame that p is
%                   computed in
%
%   GridDensity     the number of points in x and y to use for computing
%                   the contour (default is 100)
%
%   Scale, Offset, and Pose satisfy the following equality
%    x_global = rotation_matrix_2D(Pose(3))* [Scale.*x-Offset] + Pose(1:2)
%
% Authors: Shreyas Kousik and Sean Vaskov
% Created: 29 May 2019
% Updated: 20 July  2020
%
    %% parse input arguments
    if nargin < 3
        l = 0 ;
    end

    % create default inputs
    Offset = [0;0] ;
    Pose = [0;0;0] ;
    Scale = [1 ; 1] ;
    Bounds = [-1, 1, -1, 1] ;
    GridDensity = 100 ;

    for idx = 1:2:length(varargin)
        switch varargin{idx}
            case 'Offset'
                Offset = varargin{idx+1} ;
            case 'Pose'
                Pose = varargin{idx+1} ;
            case 'Scale'
                Scale = varargin{idx+1} ;
            case 'Bounds'
                Bounds = varargin{idx+1} ;
                if length(Bounds) == 1
                    Bounds = Bounds.*[-1 1 -1 1] ;
                end
            case 'GridDensity'
                GridDensity = varargin{idx+1} ;
        end
    end
    if numel(Scale) == 1
        Scale = [Scale;Scale];
    end
    %% create contour points
    % make 2D grid for plotting
    x_vec = linspace(Bounds(1),Bounds(2),GridDensity) ;
    y_vec = linspace(Bounds(3),Bounds(4),GridDensity) ;
    [X1,X2] = meshgrid(x_vec, y_vec) ;
    X = [X1(:) X2(:)]' ;
    F = reshape(full(msubs(p,x(1:2), X)),GridDensity,GridDensity) ;

    % create contour matrix
    P_raw = contourc(x_vec,y_vec,F,[l l]) ;
    
    if ~isempty(P_raw)

        % find the columns in the contour matrix that separate the individual
        % contours - the first entry in these columns will be the input l, and
        % the second entry will be an integer
        idxs = find((P_raw(1,:) == l) & (mod(P_raw(2,:),1) == 0)) ;
        N_vertices = P_raw(2,idxs) ;
        
        % set up for patch data output
        if nargout > 1
            N = length(idxs) ;
            patch_data = struct() ;
            patch_data(N).Faces = [] ;
            patch_data(N).Vertices = [] ;
        end
        
        % for each contour, extract the vertices
        P = [] ;
        patch_idx = 1 ;
        for idx = idxs
            N_idx = P_raw(2,idx) ;
            V_idx = P_raw(:, (idx+1):(idx + N_idx)) ;
            P = [P, nan(2,1), V_idx] ;
            if nargout > 1
                patch_data(patch_idx).Faces = [1:N_idx,1] ;
                patch_data(patch_idx).Vertices = V_idx' ;
            end
            patch_idx = patch_idx + 1 ;
        end
        
        % remove column of nans at end of P
        if isnan(P(1,end))
            P = P(:,1:end-1) ;
        end
        
        % scale, shift, and rotate the contour points
        Position = Pose(1:2) ;
        Rotation = Pose(3) ;
        
        Px = P(1,:) ;
        Py = P(2,:) ;
        
        x0 = Offset(1) ;
        y0 = Offset(2) ;
        
        % check the scale for if it's different in x and y
        if length(Scale) > 1
            Scale_x = Scale(1) ;
            Scale_y = Scale(2) ;
        else
            Scale_x = Scale ;
            Scale_y = Scale ;
        end
        
        % shift and scale the points appropriately
        x_shift = (Scale_x*Px - x0)*cos(Rotation) - sin(Rotation)*(Scale_x*Py - y0) ;
        y_shift = (Scale_y*Py - y0)*cos(Rotation) + sin(Rotation)*(Scale_y*Px - x0) ;
        
        % create the final output
        P = Position + [x_shift ; y_shift];
    else
        P = [] ;
    end
end