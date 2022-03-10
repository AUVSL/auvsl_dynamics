#include <Eigen/Dense>




class ODESystem{
  ODESystem(){};
  virtual ~ODESystem(){};
  
  virtual void setTimestep();
  
  template <typename MatrixType>
  virtual void ode(MatrixType &X, MatrixType &Xd, MatrixType &theta) = 0;
  virtual void forwardRecord() = 0; //forwardPropagate Solver and record states
  
  
};
