#generate a bunch of sample points

source "./icp_3d.m"

n_points=100;
P_world=(rand(3,n_points)-0.5)*10;
inv_scaling = 0.5;

#ideal position of world w.r.t robot
x_true=[10, 0.2, 0.5, -pi/2, pi/2, -pi, log(inv_scaling)]';
X_true=v2s(x_true);

#compute the measurements by mapping them in the observer frame 
P_world_hom=ones(4, n_points);
P_world_hom(1:3, :)=P_world;
Z_hom=X_true*P_world_hom;
Z=Z_hom(1:3,:).*Z_hom(4,:);

p_fix = P_world_hom(:,1)
hom_p = Z_hom(:, 1)
p = Z(:, 1)

noise_sigma=1;
# add some white noise
# Z(1:3,:) += normrnd(0, noise_sigma, 3, n_points);

iterations=100;
damping=0.6; # damping factor

chi_stats=zeros(1,iterations);
inliers_stats=zeros(1,iterations);

# test with a good initial guess
#x_guess=x_true+[-4,0.7,-0.7,0.4,0.4,0.4,0.1]'; #good initial guess
x_guess=[0,0,0,0,0,0,0]'; #bad initial guess
[X_result, chi_stats, inliers_stats] = doICP(x_guess, P_world, Z, iterations, damping, 1e6);


disp("Stats:")
X_guess = v2s(x_guess)
X_true = v2s(x_true)
X_result
initial_error = chi_stats(1, 1)
final_error = chi_stats(1, iterations)

t=figure(1);
title('good guess plain implementation');
xlabel('iteration');
ylabel('error (less is better)');
hold on
plot(log(chi_stats(1,:)+1),"-b",'LineWidth',3)
hold off
