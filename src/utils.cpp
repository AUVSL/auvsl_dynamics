#include "utils.h"



Eigen::Matrix<Scalar,4,1> calcQuatDot(Eigen::Matrix<Scalar,4,1> orientation, Eigen::Matrix<Scalar,3,1> ang_vel_body){
  Eigen::Matrix<Scalar,4,3> m;
  m(0, 0) =  orientation[3];   m(0, 1) = -orientation[2];   m(0, 2) =  orientation[1];
  m(1, 0) =  orientation[2];   m(1, 1) =  orientation[3];   m(1, 2) = -orientation[0];
  m(2, 0) = -orientation[1];   m(2, 1) =  orientation[0];   m(2, 2) =  orientation[3];
  m(3, 0) = -orientation[0];   m(3, 1) = -orientation[1];   m(3, 2) = -orientation[2];
  
  return .5 * m * ang_vel_body;
}



//https://github.com/rbdl/rbdl/blob/master/include/rbdl/Quaternion.h
//Say quaternion represents the orientation of Frame B wtr to Frame A
//Then this returns a matrix representation of the rotational
//displacement from A to B.
//The rotation that converts a frame coincident with A to frame B
Eigen::Matrix<Scalar,3,3> toMatrixRotation(Eigen::Matrix<Scalar,4,1> quaternion) {
  Scalar x = quaternion[0];
  Scalar y = quaternion[1];
  Scalar z = quaternion[2];
  Scalar w = quaternion[3];
  return toMatrixRotation(x,y,z,w);
}

Eigen::Matrix<Scalar,3,3> toMatrixRotation(Scalar x, Scalar y, Scalar z, Scalar w) {
  Eigen::Matrix<Scalar,3,3> rot;
  rot <<
    1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y,
    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x,
    2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y;
  return rot;
}
