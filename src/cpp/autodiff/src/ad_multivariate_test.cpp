#include "ad_geometry.h"
#include <iostream>
#include <Eigen/StdVector>
#include "ad_multivariate.h"
#include "ad_geometry.h"

using namespace std;
using namespace AD;
using namespace Eigen;

class ProjectPoint: public MultivariateFunction_<9, 3>{
public:

  // copied from base class
  using BaseType=MultivariateFunction_<9, 3>;

  template <typename Scalar>
  using InputVectorType = typename BaseType::InputVectorType<Scalar>;

  template <typename Scalar>
  using OutputVectorType = typename BaseType::OutputVectorType<Scalar>;


  // basic things
  template <typename Scalar>
  inline OutputVectorType<Scalar> operator()(const InputVectorType<Scalar> & input) const{
    const Eigen::Matrix<Scalar, 6,1> robot_pose=input.template head<6>();
    const Eigen::Matrix<Scalar, 3,1> point=input.template tail<3>();
    Isometry3<Scalar> robot_pose_matrix=v2t<Scalar>(robot_pose);
    return robot_pose_matrix*point;
  }
};


int main(int argc, char** argv){
  ProjectPoint project;

  Eigen::Matrix<float, 6,1> robot_pose;
  robot_pose << 0,0,0,0,0,0;
    
  Eigen::Matrix<float, 3,1> point;
  point << 1,2,3;


  // construct input
  // 1st 6 elements:  pose
  // next 3 elements: point
  ProjectPoint::InputVectorType<float> input;
  input.block<6,1>(0,0)=robot_pose;
  input.block<3,1>(6,0)=point;

  // call the plain function
  Eigen::Matrix<float, 3,1> output = project(input);
 
  auto jacobian = adJacobian(project, input);

  cerr << "output: " << endl;
  cerr << output.transpose() << endl;
  cerr << "jacobian: " << endl;
  cerr << jacobian << endl;
}
