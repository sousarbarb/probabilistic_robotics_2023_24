source "epipolar.m"
function [P1, P2]=makePointPairs(X, n_points)
  P1=(rand(3,n_points)-0.5)*5;
  P2=X(1:3,1:3)*P1 + repmat(X(1:3,4),1,n_points);
endfunction;

function X = generateTransform()
  R=rand(3,3);
  [U,S,V]=svd(R);
  if (det(U)<0)
    U(:,1)=-U(:,1);
  endif;
  X=[U, rand(3,1);
     0, 0, 0, 1];
endfunction;

function testTriangulatePoint(X,n_points)
  K=[100, 0,   160;
     0,   100, 120;
     0,   0,    1];

  #generate two set of points 2nd set transformed by x
  [P1,P2]=makePointPairs(X,n_points);
  #project into image
  [P1_img, is_outside1]=projectPoints(K,P1);
  [P2_img, is_outside2]=projectPoints(K,P2);
  [P1_img, P2_img]=pruneProjectedPointPairs(P1_img, is_outside1,
                                            P2_img, is_outside2);
  #inverse transform
  iX=inv(X);
  #inverse camera matrix
  iK=inv(K);
  #inverse rotation * inverse camera matix
  iRiK=iX(1:3,1:3)*iK;

  #express the points in camera coordinates
  #rotate the direction vector of P2 in world frame
  P1_cam=iK*[P1_img; ones(1,n_points)];
  P2_cam=iRiK*[P2_img; ones(1,n_points)];

  for (i=1:n_points)
    p1_cam=P1_cam(:,i);
    p2_cam=P2_cam(:,i);
    [success, p, e]=triangulatePoint(iX(1:3,4),p1_cam, p2_cam);
    if (success==true)
      disp(norm(P1(:,i)-p));
    endif;
  endfor;
endfunction;

function testTriangulatePoints(X,n_points)
  K=[100, 0,   160;
     0,   100, 120;
     0,   0,    1];

  #generate two set of points 2nd set transformed by x
  [P1,P2]=makePointPairs(X,n_points);
  #project into image
  P1_img=projectPoints(K,P1);
  P2_img=projectPoints(K,P2);
  [n_points, P]=triangulatePoints(K,X,P1_img, P2_img);
  n_points
endfunction;


function testFundamental(X,n_points)
  K=[100, 0,   160;
     0,   100, 120;
     0,   0,    1];

  #get ground truth
  F_gt=transform2fundamental(K,X);

  #make world
  [P1,P2]=makePointPairs(X,n_points);
  P1_proj=projectPoints(K,P1);
  P2_proj=projectPoints(K,P2);
  
  #estimate fundamental (w/o outliers)
  F=estimateFundamental(P1_proj,P2_proj)
  disp("fund ratio");
  F_gt./F

  # extract essential
  E=K'*F*K
  
  disp("transform original");
  X
  
  #extract transforms from essential
  disp("transforms recovered");
  [X1,X2]=essential2transform(E)
  
  disp("transforms ratio");
  X./X1
  X./X2
endfunction;

function testTransform(X,n_points, noise, use_preconditioning=false)
  K=[100, 0,   320;
     0,   100, 240;
     0,   0,    1];

  noise_sigma=1;
  #get ground truth
  F=transform2fundamental(K,X);

  #make world
  [P1,P2]=makePointPairs(X,n_points);
  [P1_img, is_outside1]=projectPoints(K,P1);
  [P2_img, is_outside2]=projectPoints(K,P2);
  disp("input points: ");
  size(P1,2)
  [P1_img, P2_img]=pruneProjectedPointPairs(P1_img, is_outside1,
                                            P2_img, is_outside2);
  P1_img+=normrnd(0, noise_sigma, size(P1_img));
  P2_img+=normrnd(0, noise_sigma, size(P2_img));
  
  disp("common points: ");
  size(P1_img,2)
  X_est=estimateTransform(K,P1_img,P2_img, use_preconditioning);
  disp("transform original");
  X

  disp("transform estimated");
  X_est

  disp("t ratio")
  X(1:3,4)./X_est(1:3,4)
endfunction;
