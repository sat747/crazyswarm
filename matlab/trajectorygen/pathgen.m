function [duration, fx, fy, fz, fyaw] = pathgen(npts, px, py, pz)
    %general path trajectory generation
        
    if iscell(px) && iscell(py) && iscell(pz)
        X = cellfun(@double, px);
        Y = cellfun(@double, py);
        Z = cellfun(@double, pz);
    else
        X = px;
        Y = py;
        Z = pz;
    end
    
    %number of points
    npcs = npts - 1;

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
        x(1,i,:) = [X(i-1) Y(i-1) Z(i-1) 0];  % position [x y z yaw]
    end

    %t_takeoff = 2;
    t_settle = 1;
    %t_circle = 1;
    intervals = [t_settle 1 * ones(1,npcs-1)];
    knot = [0 cumsum(intervals)]; %times of waypoints

    [soln, free] = pp_waypoints(knot, degree, continuity, x, 'monomial');
    fprintf('%d free dimensions\n', size(free, 2));

    pp = polyvec2pp(knot, dim, soln);
    %pp2csv(pp, strcat(name,".csv"));
 
    %return: (duration, x, y, z, yaw)
    [breaks, coefs, npieces, order, dim] = unmkpp(pp);
	assert(dim == 4);
	coefs = reshape(coefs, 4, npieces, order);
    duration = nan(npieces, 1);
    dx = nan(npieces,degree+1);
    dy = nan(npieces,degree+1);    
    dz = nan(npieces,degree+1);    
    dyaw = nan(npieces,degree+1);

    for piece=1:npieces
		duration(piece,:)= breaks(piece+1) - breaks(piece);
			%add x, y, z, yaw to respective variables
        dx(piece,:) = flipud(squeeze(coefs(1,piece,:)));
        dy(piece,:) = flipud(squeeze(coefs(2,piece,:)));
        dz(piece,:) = flipud(squeeze(coefs(3,piece,:)));
        dyaw(piece,:) = flipud(squeeze(coefs(4,piece,:)));
    end
    
    tx = transpose(dx);
    ty = transpose(dy);
    tz = transpose(dz);
    tyaw = transpose(dyaw);
    
    fx = num2cell(tx(:));
    fy = num2cell(ty(:));
    fz = num2cell(tz(:));
    fyaw = num2cell(tyaw(:));
    duration = num2cell(duration);
    
end
