
function p = fit_bounding_polynomial_from_samples(p_data,x_data,x,degree,bound)
% Output: a polynomial function p(x) of degree degree that is the solution
% of the following optimization program (for bound = 'upper')

% p = argmin integral_hX p(x) dx
%        p
%          s.t. p(x_data)-x_data >= 0 for all x_data
%               p(x) > min(x_data) on hX
%        
%      where Hx = {x | (x-min(x_data)).*(max(x_data)-x)>=0}

% entering bound = 'lower', makes p a lower bound instead
% 
% note that the functions produced by this script are only garaunteed to
% bound the provided data. In practice if degree is low, this works for
% points in hX, but this is not garaunteed.
% 
% Author: Sean Vaskov
% Created: 8 Mar 2020
% Updated: -



if nargin < 5
    bound = 'upper';
end

N = size(x_data,2);

%scale data to [-1,1]
max_x = max(x_data,[],2);
min_x  = min(x_data,[],2);

max_p = max(p_data);
min_p = min(p_data);

p_data_scaled = 2*(p_data-repmat(min_p,[1 N]))./(repmat(max_p,[1 N])-repmat(min_p,[1 N]))-1;

x_data_scaled = 2*(x_data-repmat(min_x,[1 N]))./(repmat(max_x,[1 N])-repmat(min_x,[1 N]))-1;

hX = (x+1).*(1-x);

%set up program
prog = spotsosprog();

prog = prog.withIndeterminate(x);

pterms = monomials(x,0:degree);

[prog,p,coeff] = prog.newFreePoly(pterms); 

%set up objective
int_X = boxMoments(x,-ones(length(x),1),ones(length(x),1));

if strcmpi(bound,'lower')
    %upper bouund constraint on p
    prog = sosOnK(prog,1-p,x,hX,degree);
    
    %maximize integral
    obj = -int_X(pterms)'*coeff;
    
    %enforce that polynomial is less than data
    err = p_data_scaled-msubs(p,x,x_data_scaled);
else
    %lower bound on p
    prog = sosOnK(prog,p+1,x,hX,degree);
    
    %minimize integral
    obj = int_X(pterms)'*coeff;
    
    %enforce that polynomial is greater than data
    err = msubs(p,x,x_data_scaled)-p_data_scaled;
end

prog=prog.withPos(err);

options = spot_sdp_default_options();
options.verbose = 1;

sol = prog.minimize(obj, @spot_mosek, options);    

p_scaled = sol.eval(p);

p = (max_p-min_p)/2*(subs(p_scaled,x,2./(max_x-min_x).*(x-min_x)-1)+1)+min_p;

end
