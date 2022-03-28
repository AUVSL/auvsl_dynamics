#include <string>

#include "forward_dynamics.h"
#include "model_constants.h"

#pragma once

//Abstract Base Class.
class Model{
public:
  Model(){};
  virtual ~Model(){};

  virtual void initState() = 0;
  virtual void initState(Scalar *start_state) = 0;
  virtual void initStateCOM(Scalar *start_state) = 0;
  virtual void step(Scalar vl, Scalar vr) = 0;
  virtual void settle() = 0;
  virtual void getState(Scalar *state) = 0;
  
  virtual unsigned getStateDim() const = 0;
  virtual unsigned getControlDim() const = 0;
  
private:
  

};
