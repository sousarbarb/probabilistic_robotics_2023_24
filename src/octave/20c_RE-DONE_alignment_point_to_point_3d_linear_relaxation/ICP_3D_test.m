#generate a bunch of sample points

source "./icp_3d.m"

n_points=100;
P_world=(rand(3,n_points)-0.5)*10;

#ideal position of world w.r.t robot
x_true=[0, 0, 0, pi/2, pi/6, pi]';
printf("The target Transform is:\n");
X_true=v2t(x_true)

#compute the measurements by mapping them in the observer frame
P_world_hom=ones(4, n_points);
P_world_hom(1:3, :)=P_world;
Z_hom=X_true*P_world_hom;
Z=Z_hom(1:3,:);

#generate 3 noise matrices 3 levels of noise
N1=0.1*(rand(3,n_points)-.5);
N2=0.5*(rand(3,n_points)-.5);
N3=2.0*(rand(3,n_points)-.5);


# define  a good initial guess
x_guess=x_true+[0.5,0.5,0.5,0.1,0.1,0.1]';
printf("Using as initial guess: \n");
X_guess=v2t(x_guess)
R_guess = X_guess(1:3, 1:3);
guess = reshape(R_guess', 1, 9);
guess = [guess,  X_guess(1:3, 4)'];

printf("\n****************\n");
printf("\nExp #1: no noise\n");
[R, t] = doICP(guess, P_world, Z)
printf("\n****************\n");
printf("\nExp #2: low noise\n");
[R, t] = doICP(guess, P_world, Z+N1)
printf("\n****************\n");
printf("\nExp #3: middle noise\n");
[R, t] = doICP(guess, P_world, Z+N2)
printf("\n****************\n");
printf("\nExp #4: high noise\n");
[R, t] = doICP(guess, P_world, Z+N3)
printf("\n****************\n");



#pause();
