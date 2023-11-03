#include "ad.h"
#include <Eigen/Core>

namespace AD {

  // generic stub for multivariate function
  template <int InputSize_, int OutputSize_>
  class MultivariateFunction_{
  public:
    static constexpr int InputSize=InputSize_;
    static constexpr int OutputSize=OutputSize_;

    template <typename Scalar>
    using InputVectorType = Eigen::Matrix<Scalar, InputSize, 1>;

    template <typename Scalar>
    using OutputVectorType = Eigen::Matrix<Scalar, OutputSize, 1>;

    // this must be defined in the derived classes
    template <typename Scalar>
    OutputVectorType<Scalar> operator()(const InputVectorType<Scalar>& in) const;
  };


  
  // function that computes the jacobian
  // call it as adJacobian(function, value);
  
  template <typename FunctionType, typename Scalar=float>
  
  Eigen::Matrix<Scalar, FunctionType::OutputSize, FunctionType::InputSize>

  adJacobian (const FunctionType & fn,
              const typename FunctionType::template InputVectorType<Scalar>& input)
  {
    static constexpr int InputSize = FunctionType::InputSize;
    static constexpr int OutputSize = FunctionType::OutputSize;
    
    using JacobianType = Eigen::Matrix<Scalar, OutputSize, InputSize>;
    using DualValue = DualValue_<Scalar>;
    using InputVectorTypeAD =  Eigen::Matrix<DualValue, InputSize, 1>;
    using OutputVectorTypeAD = Eigen::Matrix<DualValue, OutputSize, 1>;

    // populate the input vector
    InputVectorTypeAD in;
    for (int c=0;  c<InputSize; ++c)
      in(c,0)=DualValue(input(c,0),0);

    JacobianType j;
    for (int c=0; c<InputSize; ++c) {
      in(c,0).derivative = 1;
      auto out_col=fn(in);
      for (int r=0; r<OutputSize; ++r)
        j(r,c)=out_col(r,0).derivative;
      in(c,0).derivative = 0;
    }
    return j;
  }

  
}
