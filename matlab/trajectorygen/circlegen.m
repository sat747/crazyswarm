%Circle trajectory generation
clear all
clc
%number of points
npts = 9;
npieces = npts - 1;

%polynomial degree
degree = 7;  
%continuity constraint (all derivatives through cont are continuous)
continuity = 4;  
%number of dimensions
dim = 4;

% mission with only position constraints
x = nan(3, npts, 4);

% start on ground with 0 vel, 0 acc at theta=0
x(:,1,:) = 0;         % initial zeros in everything up to acceleration
x(:,npts,:) = 0;
% 
% x(:,2,:) = 0;         % take off, come to rest again
% x(1,2,:) = [1 0 1 0];
% 
% x(:,3,:) = x(:,2,:);         % stabilization period. don't move

for i=2:npts
    theta = (pi/4) * (i-2);
    x(1,i,:) = [(0.5*cos(theta)) (0.5*sin(theta)) 0 0]  % position
end

t_takeoff = 2;
%t_settle = 2;
%t_circle = 1;
intervals = [t_takeoff 1 * ones(1,npieces-1)];
knot = [0 cumsum(intervals)]; %times of waypoints

[soln, free] = pp_waypoints(knot, degree, continuity, x, 'monomial')
fprintf('%d free dimensions\n', size(free, 2));

pp = polyvec2pp(knot, dim, soln)
pp2csv(pp, 'circtest.csv');
