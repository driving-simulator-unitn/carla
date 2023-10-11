/**
 * \file SingleTrackModel.cpp
 * \brief Source file for the single track model
*/

#include "SingleTrackModel.h"

SingleTrackModel::SingleTrackModel()
{
  this->_parameters.a   = 1.3;
  this->_parameters.b   = 1.3;
  this->_parameters.M   = 1000;
  this->_parameters.Nr  = 5000;
  this->_parameters.Nf  = 5000;
  this->_parameters.Iz  = 1200;
  this->_parameters.KLr = 3.0 * 24;
  this->_parameters.KLf = 3.0 * 20;
  this->_parameters.Dyr = 3.0 * 1.2;
  this->_parameters.Dyf = 3.0 * 1.2;
  this->_error          = 0;
  this->_error_dot      = 0;
  this->_error_int      = 0;
  this->_error_prev     = 0;
  this->_tau            = 0.04;
  this->_Kp             = 1.0;
  this->_Ki             = 0.0;
  this->_Kd             = 0.0;
  this->_A_max          = 3.1;
  this->_A_min          = -3.1;
}

SingleTrackModel::SingleTrackModel(Parameters &params)
{
  this->_parameters = params;
  this->_error      = 0;
  this->_error_dot  = 0;
  this->_error_int  = 0;
  this->_error_prev = 0;
  this->_tau        = 0.04;
  this->_Kp         = 1.0;
  this->_Ki         = 1.0;
  this->_Kd         = 0.0;
  this->_A_max      = 2.0;
  this->_A_min      = -2.0;
}

void SingleTrackModel::ode(const std::vector<double> & X,
                      const std::vector<double> & U,
                      std::vector<double> & XDOT)
{
  // Get the parameters
  double M  = _parameters.M;
  double a  = _parameters.a;
  double b  = _parameters.b;
  double Iz = _parameters.Iz;

  // Unpack the states and inputs
  double u     = X.at(0);
  double v     = X.at(1);
  double w     = X.at(2);
  double theta = X.at(5);
  double Sr    = U.at(0);
  double delta = U.at(1);

  // Compute the tyre forces
  std::vector<double> tyre_results;
  tyre_model(X, U, tyre_results);

  double Fr = tyre_results.at(0);
  double Ff = tyre_results.at(1);

  // Compute the derivatives
  double u_dot   = -(-w * M * v + sin(delta) * Ff - Sr) / M;
  double v_dot   = (-w * M * u + cos(delta) * Ff + Fr) / M;
  double w_dot   = (a * cos(delta) * Ff - b * Fr) / Iz;
  double x_dot   = u * cos(theta) - v * sin(theta);
  double y_dot   = u * sin(theta) + v * cos(theta);
  double psi_dot = w;

  XDOT = {u_dot, v_dot, w_dot, x_dot, y_dot, psi_dot};
}

void SingleTrackModel::tyre_model(const std::vector<double> & X,
                             const std::vector<double> & U,
                             std::vector<double> & results)
{
  // Get the parameters
  double a   = _parameters.a;
  double b   = _parameters.b;
  double Nr  = _parameters.Nr;
  double Nf  = _parameters.Nf;
  double KLr = _parameters.KLr;
  double KLf = _parameters.KLf;
  double Dyr = _parameters.Dyr;
  double Dyf = _parameters.Dyf;

  // Unpack the states and inputs
  double u     = X.at(0);
  double v     = X.at(1);
  double w     = X.at(2);
  double delta = U.at(1);

  // Compute the tyre forces
  // Handle the case where u = 0
  if (abs(u) < 1e-6)
  {
    u = 1e-6;
  }
  double lambda_R = -atan((-w * b + v) / u);
  double lambda_F = -atan((-sin(delta) * u + cos(delta) * w * a +
                            cos(delta) * v) /
                          (cos(delta) * u + sin(delta) * w * a +
                            sin(delta) * v));

  double Fr = KLr * lambda_R * Nr / sqrt(1.0 + pow((KLr * lambda_R), 2) /
              pow(Dyr, 2));
  double Ff = KLf * lambda_F * Nf / sqrt(1.0 + pow((KLf * lambda_F), 2) /
              pow(Dyf, 2));

  results = {Fr, Ff};
}

void SingleTrackModel::step(const std::vector<double> & X0,
                       const std::vector<double> & U0,
                       double dt,
                       std::vector<double> &X1)
{
  // Compute the derivative of the state vector
  std::vector<double> XDOT;
  ode(X0, U0, XDOT);

  // Compute the next state vector
  for (unsigned i = 0; i < X0.size(); i++)
  {
    X1[i] = X0[i] + XDOT.at(i) * dt;
  }
}

void SingleTrackModel::compute_controls(const std::vector<double> & X,
                                   std::vector<double> & U,
                                   double u_ref, double kappa)
{
  // Get the parameters
  double a = _parameters.a;
  double b = _parameters.b;
  double M = _parameters.M;

  // Unpack the states
  double u = X.at(0);

  // Compute the traction force
  this->_error     = (u_ref - u) / this->_tau;
  this->_error_int = this->_error_int + this->_error;
  double a_tilde   = this->_Kp * this->_error     +
                     this->_Ki * this->_error_int +
                     this->_Kd * this->_error_dot;

  double Sr      = a_tilde * M;
  double Sr_clip = std::min(std::max(Sr, this->_A_min * M), this->_A_max * M);

  U.at(0) = Sr_clip;
  U.at(1) = atan(kappa * (a + b));
}

double SingleTrackModel::compute_acceleration(const std::vector<double> & X,
                                         const std::vector<double> & U)
{
  // Get the parameters
  double M = _parameters.M;

  // Unpack the states and inputs
  double v     = X.at(1);
  double w     = X.at(2);
  double Sr    = U.at(0);
  double delta = U.at(1);

  // Compute the tyre forces
  std::vector<double> tyre_results;
  tyre_model(X, U, tyre_results);

  double Ff = tyre_results.at(1);

  // Compute the acceleration
  return -(-w * M * v + sin(delta) * Ff - Sr) / M;
}