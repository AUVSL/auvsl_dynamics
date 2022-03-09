#include <cppad/cppad.hpp>
#include <cppad/utility/to_string.hpp>
#include <Eigen/Dense>

#include "forward_dynamics.h"
#include "model_constants.h"

using Jackal::rcg::Scalar;

class AdjointMethod {
public:
  AdjointMethod();
  ~AdjointMethod();
  
  void getGradient(Eigen::Matrix<float,Eigen::Dynamic,1>& values);
  void performAD();
  void setODE(void (*function)(Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&), int dim_state, int dim_params);
  void setLossFunction(void (*function)(Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&));
  
  void augmentedODE(Eigen::Matrix<float,Eigen::Dynamic,1> &zt, //this is going to have to be recorded from the forward pass
                                   Eigen::Matrix<float,Eigen::Dynamic,1> &W1, //a(t)
                                   Eigen::Matrix<float,Eigen::Dynamic,1> &W2, //idk?
                                   Eigen::Matrix<float,Eigen::Dynamic,1> &W1d,//-a_dot(t)
                                   Eigen::Matrix<float,Eigen::Dynamic,1> &W2d //-a(t)*df/dtheta
                                   );
  //private:
  void (*ode_f_)(Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&);
  void (*loss_f_)(Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&);
  
  CppAD::ADFun<float> *df_dz_fun_;
  CppAD::ADFun<float> *df_dtheta_fun_;
  CppAD::ADFun<float> *loss_fun_;
  
  int dim_state_;
  int dim_params_;
  
  std::vector<Eigen::Matrix<float,Eigen::Dynamic,1>> z_history_;
  Eigen::Matrix<float,Eigen::Dynamic,1> theta_;
};
