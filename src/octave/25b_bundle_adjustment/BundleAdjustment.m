#generate a bunch of sample points

source "./ba_utils.m"
num_landmarks=150;
num_poses=5;
world_size=20;

# landmarks in a matrix, one per column
P_world=(rand(landmark_dim, num_landmarks)-0.5)*world_size;


# poses in an array of 4x4 homogeneous transform matrices
XR_true=zeros(4,4,num_poses);
XL_true=P_world;

# initialize 1st pose
XR_true(:,:,1)=eye(4);

# scaling coefficient for uniform random pose generation
# adjusts the translation to cover world_size
# adjusts the rotation to span the three angles;
rand_scale=eye(6);
rand_scale(1:3,1:3)*=(0.5*world_size);
rand_scale(4:6,4:6)*=pi;

for (pose_num=2:num_poses)
    xr=rand(6,1)-0.5;
    Xr=v2t(rand_scale*xr);
    XR_true(:,:,pose_num)=Xr;
endfor;

K=[100,0,320;
   0, 100,240;
   0, 0, 1];

# check if all the landmarks are visible
[_, _, observations_for_landmark] = computeMeasurementAndAssociations(XR_true, XL_true, K);

#select the subset of landmarks that are observable from at least two cameras
XL_true = XL_true(:,observations_for_landmark>2);
num_landmarks = size(XL_true,2);
# compute associations on those landmarks
[associations, Z, _] = computeMeasurementAndAssociations(XR_true, XL_true, K);

noise = 1.;
Z += noise * randn(2,size(Z,2));
# apply a perturbation to each ideal pose (construct the estimation problem)
pert_deviation=0.2;
pert_scale=eye(6)*pert_deviation;
XR_guess=XR_true;
XL_guess=XL_true;

for (pose_num=2:num_poses)
    xr=rand(6,1)-0.5;
    dXr=v2t(pert_scale*xr);
    XR_guess(:,:,pose_num)=dXr*XR_guess(:,:,pose_num);
endfor;

pert_deviation_landmark = 1.0;
#apply a perturbation to each landmark
dXl=(rand(landmark_dim, num_landmarks)-0.5)*pert_deviation_landmark;
XL_guess+=dXl;

num_iterations = 30;
damping = 1.5;
kernel_threshold = 10.0;
[XR, XL, chi_stats, num_inliers]=doBundleAdjustment(XR_guess, XL_guess, Z, K,
							associations, 
							num_poses, 
							num_landmarks, 
							num_iterations, 
							damping, 
							kernel_threshold);
pause()
close all
