source "../tools/utilities/geometry_helpers_3d.m"

%(minimal) size of pose and landmarks
global pose_dim=6;
global landmark_dim=3;


# retrieves the index in the perturbation vector, that corresponds to
# a certain pose
# input:
#   pose_index:     the index of the pose for which we want to compute the
#                   index
#   num_poses:      number of pose variables in the state
#   num_landmarks:  number of pose variables in the state
# output:
#   v_idx: the index of the sub-vector corrsponding to 
#          pose_index, in the array of perturbations  (-1 if error)
function v_idx=poseMatrixIndex(pose_index, num_poses, num_landmarks)
  global pose_dim;
  global landmark_dim;

  if (pose_index>num_poses)
    v_idx=-1;
    return;
  endif;
  v_idx=1+(pose_index-1)*pose_dim;
endfunction;


# retrieves the index in the perturbation vector, that corresponds to
# a certain landmark
# input:
#   landmark_index:     the index of the landmark for which we want to compute the
#                   index
#   num_poses:      number of pose variables in the state
#   num_landmarks:  number of pose variables in the state
# output:
#   v_idx: the index of the perturnation corrsponding to the
#           landmark_index, in the array of perturbations
function v_idx=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks)
  global pose_dim;
  global landmark_dim;
  if (landmark_index>num_landmarks)
    v_idx=-1;
    return;
  endif;
  v_idx=1 + (num_poses)*pose_dim + (landmark_index-1) * landmark_dim;
endfunction;

function pimg = omniCameraPointToImg(pc, K, xi)
  ps = pc/norm(pc);
  inv_depth = 1/(ps(3) + xi);
  ps(1:2) *= inv_depth;
  ps(3) = 1;
  pimg_homo = K * ps;
  pimg = pimg_homo(1:2);
endfunction

function [is_behind, z_hat] = omniProject(Xr, Xl, K, xi)
  p_camera_frame = Xr(1:3,1:3) * Xl + Xr(1:3,4);
  is_behind = true;
  z_hat = [-1;-1];
  if(p_camera_frame(3) > 0)
    z_hat = omniCameraPointToImg(p_camera_frame, K, xi);
    is_behind = false;
  endif
endfunction


function [associations, Z, seen_landmarks] = computeMeasurementAndAssociations(XR, XL, K, xi)
  num_poses=size(XR,3);
  num_landmarks=size(XL,2);
  associations = zeros(2,0);
  Z = zeros(2,0);
  seen_landmarks = zeros(num_landmarks,1);
  for (pose_num=1:num_poses)
      Xr=XR(:,:,pose_num);
      R = Xr(1:3,1:3);
      t = Xr(1:3,4);
      for (landmark_num=1:num_landmarks)
        Xl=XL(:,landmark_num);
        p = (R*Xl + t);
        if(p(3) < 0)
          continue;
        endif
        z_proj = omniCameraPointToImg(p, K, xi)
        associations(:,end+1)=[pose_num,landmark_num]';
        # binary array that take track of how many times each landmark have been observed
        seen_landmarks(landmark_num) += 1;
        Z(:,end+1)= z_proj;
      endfor;
  endfor
endfunction

# error and jacobian of a measured landmark
# input:
#   Xr: the robot pose (4x4 homogeneous matrix)
#   Xl: the landmark pose (3x1 vector, 3d pose in world frame)
#   z:  measured position of landmark
# output:
#   e: 2x1 is the difference between prediction and measurement
#   Jr: 2x6 derivative w.r.t a the error and a perturbation on the
#       pose
#   Jl: 2x3 derivative w.r.t a the error and a perturbation on the
#       landmark
function [is_behind,e,Jr,Jl]=errorAndJacobian(Xr, Xl, z, K, xi)
   R=Xr(1:3,1:3);
   t=Xr(1:3,4);
   [is_behind,z_hat]=omniProject(Xr,Xl,K,xi); #prediction
   e = zeros(2,1);
   Jl = zeros(2,3);
   Jr = zeros(2,6);
   if (is_behind)
    return
   endif
   % projecting point  
   p_camera_frame = R * Xl + t;
   p_sphere = p_camera_frame/norm(p_camera_frame);
   inverse_z = 1 / (p_sphere(3) + xi); 
   inverse_square_z = inverse_z*inverse_z; 
   p_sphere(1:2) *= inverse_z;
   p_sphere(3) = 1;
   p_projected_homo = K * p_sphere;
   p_projected = p_projected_homo(1:2);
   
   % calculating error  
   e=z_hat-z;
   
   % jacobian of the projective part  
   J_proj = [inverse_z, 0, -p_sphere(1) * inverse_square_z; 
             0, inverse_z, -p_sphere(2) * inverse_square_z];
   
   % jacobian of the norm, projection to unit sphere
   n = norm(p_camera_frame);
   n2 = n*n;
   in3 = 1/(n2*n);
   J_norm = -p_camera_frame*p_camera_frame' + eye(3) * n2;
   J_norm *= in3; 
   
   % jcip for jacobian wrt to robot  
   Jicp = J_icp(p_camera_frame);
   
   Jr = K(1:2, 1:2) * J_proj * J_norm * Jicp;
   Jl = K(1:2, 1:2) * J_proj * J_norm * R; 
   
endfunction;


# implementation of the boxplus
# applies a perturbation to a set of landmarks and robot poses
# input:
#   XR: the robot poses (4x4xnum_poses: array of homogeneous matrices)
#   XL: the landmark pose (3xnum_landmarks matrix of landmarks)
#   num_poses: number of poses in XR (added for consistency)
#   num_landmarks: number of landmarks in XL (added for consistency)
#   dx: the perturbation vector of appropriate dimensions
#       the poses come first, then the landmarks
# output:
#   XR: the robot poses obtained by applying the perturbation
#   XL: the landmarks obtained by applying the perturbation
function [XR, XL]=boxPlus(XR, XL, num_poses, num_landmarks, dx)
  global pose_dim;
  global landmark_dim;
  for(pose_index=1:num_poses)
    pose_matrix_index=poseMatrixIndex(pose_index, num_poses, num_landmarks);
    dxr=dx(pose_matrix_index:pose_matrix_index+pose_dim-1);
    XR(:,:,pose_index)=v2t(dxr)*XR(:,:,pose_index);
  endfor;
  for(landmark_index=1:num_landmarks)
    landmark_matrix_index=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);
    dxl=dx(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
    XL(:,landmark_index)+=dxl;
  endfor;
endfunction;

# implementation of the optimization loop with robust kernel
# applies a perturbation to a set of landmarks and robot poses
# input:
#   XR: the initial robot poses (4x4xnum_poses: array of homogeneous matrices)
#   XL: the initial landmark estimates (3xnum_landmarks matrix of landmarks)
#   Z:  the measurements (3xnum_measurements)
#   associations: 2xnum_measurements. 
#                 associations(:,k)=[p_idx,l_idx]' means the kth measurement
#                 refers to an observation made from pose p_idx, that
#                 observed landmark l_idx
#   num_poses: number of poses in XR (added for consistency)
#   num_landmarks: number of landmarks in XL (added for consistency)
#   num_iterations: the number of iterations of least squares
#   damping:      damping factor (in case system not spd)
#   kernel_threshod: robust kernel threshold

# output:
#   XR: the robot poses after optimization
#   XL: the landmarks after optimization
#   chi_stats: array 1:num_iterations, containing evolution of chi2
#   num_inliers: array 1:num_iterations, containing evolution of inliers
function [XR, XL, chi_stats, num_inliers]=doBundleAdjustment(XR, XL, Z, K, xi,
							associations, 
							num_poses, 
							num_landmarks, 
							num_iterations, 
							damping, 
							kernel_threshold)
  global pose_dim;
  global landmark_dim;

  chi_stats=zeros(1,num_iterations);
  num_inliers=zeros(1,num_iterations);
  # size of the linear system
  system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
  for (iteration=1:num_iterations)
    H=zeros(system_size, system_size);
    b=zeros(system_size,1);
    chi_stats(iteration)=0;
    for (measurement_num=1:size(Z,2))
      pose_index=associations(1,measurement_num);
      landmark_index=associations(2,measurement_num);
      z=Z(:,measurement_num);
      Xr=XR(:,:,pose_index);
      Xl=XL(:,landmark_index);
      [is_behind,e,Jr,Jl] = errorAndJacobian(Xr, Xl, z, K, xi);
      if (is_behind)
        continue;
      endif
      chi=e'*e;
      # apply the robust kernel as a weight to the linear system
      w = 1;
      if (chi>kernel_threshold)
      	w = 1/kernel_threshold;
      	chi=kernel_threshold;
      else
      	num_inliers(iteration)++;
      endif;
      chi_stats(iteration)+=chi;
    
      omega = w * eye(2);
      Hrr = Jr' * omega * Jr;
      Hrl = Jr' * omega * Jl;
      Hll = Jl' * omega * Jl;
      br = Jr' * omega * e;
      bl = Jl' * omega * e;

      pose_matrix_index=poseMatrixIndex(pose_index, num_poses, num_landmarks);
      landmark_matrix_index=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);

      H(pose_matrix_index:pose_matrix_index+pose_dim-1,
	pose_matrix_index:pose_matrix_index+pose_dim-1)+=Hrr;

      H(pose_matrix_index:pose_matrix_index+pose_dim-1,
	landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Hrl;

      H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
	landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Hll;

      H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
	pose_matrix_index:pose_matrix_index+pose_dim-1)+=Hrl';

      b(pose_matrix_index:pose_matrix_index+pose_dim-1)+=br;
      b(landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=bl;

    endfor
    size_H=size(H,1)
    rank_H=rank(H)
    H+=eye(system_size)*damping;
    dx=zeros(system_size,1);
    
    % we solve the linear system, blocking the first pose
    % this corresponds to "remove" from H and b the locks
    % of the 1st pose, while solving the system

    dx(pose_dim+1:end)=-(H(pose_dim+1:end,pose_dim+1:end)\b(pose_dim+1:end,1));
    [XR, XL]=boxPlus(XR,XL,num_poses, num_landmarks, dx);
  endfor

  % visualize the sparsity pattern of the approximate hessian
  visual_H = ones(system_size,system_size);
  for r=1:system_size
    for c=1:system_size
      if(abs(H(r,c)) > 0)
        visual_H(r,c) = 0;
      endif
    endfor
  endfor
  imshow(visual_H)
endfunction

function i = plotState(XL, XL_guess, XL_gt)
#plot landmarks
hold on;
plot3(XL(1,:),XL(2,:),XL(3,:),'b*',"linewidth",2);
hold on;
plot3(XL_guess(1,:),XL_guess(2,:),XL_guess(3,:),'ro',"linewidth",2);
hold on;
plot3(XL_gt(1,:),XL_gt(2,:),XL_gt(3,:),'g*',"linewidth",2);
hold on;
legend("estimate","initial guess","ground truth")
i = 1;
endfunction
