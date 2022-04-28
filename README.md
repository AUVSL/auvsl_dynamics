# auvsl_dynamics
This repo contains code related to vehicle dynamics written by Justin Yurkanin (yurkanin321@gmail.com)
The different branches refer to different experiments.
float_model uses Robcogen templated with float scalar type and is fastest and should be used for path planning and anything serious.
bekker_descent uses an auto-diff scalar type for doing gradient descent to identify bekker parameters.
hybrid_network is similar and trains a physics based neural ODE.
2d_neural_ode is a test branch for training 2D neural ode's for vehicle dynamics.
adjoint_method is another test branch to see if the adjoint sensitivity method works better for neural ODE's than just directly calculating the parameter/loss gradient with auto diff.

This repo requires CppAD for auto-diff, libeigen3, and Robcogen for forward dynamics.

For a more in depth description of what all this code does, see my thesis.
