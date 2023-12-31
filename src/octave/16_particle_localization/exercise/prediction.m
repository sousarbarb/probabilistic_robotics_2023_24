% this function computes the prediction step by applying the motion model to each particle
% inputs:
%   samples: the set of particles
%   transition: current transition

% outputs:
%   samples: the particle after the motion model

function samples = prediction(samples, transition)

	dim_particles = size(samples,2);
	
	u = transition.v;
        %it returns u = [ux, uy, utheta]. simply not consider uy
	u_x = u(1);
	u_theta = u(3);
	
	a1 = abs(u_x);
	a2 = abs(u_theta);
	%apply transition to every particle
	for i=1:dim_particles
		%sample noise here and apply
		noise_x = (rand() - 0.5)*a1;
		noise_theta = (rand() - 0.5)*a2;

		u_input = [ u_x + noise_x , 0.0 , u_theta + noise_theta ]';
		
		samples(:,i) = motion_model( samples(:,i) , u_input );
	end
endfunction
