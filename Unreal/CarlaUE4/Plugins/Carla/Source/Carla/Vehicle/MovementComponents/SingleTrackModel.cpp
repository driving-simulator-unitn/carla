// ██████╗ ███████╗ ██████╗ ██╗███╗   ██╗
// ██╔══██╗██╔════╝██╔════╝ ██║████╗  ██║
// ██████╔╝█████╗  ██║  ███╗██║██╔██╗ ██║
// ██╔══██╗██╔══╝  ██║   ██║██║██║╚██╗██║
// ██████╔╝███████╗╚██████╔╝██║██║ ╚████║
// ╚═════╝ ╚══════╝ ╚═════╝ ╚═╝╚═╝  ╚═══╝
// #UNITN_MODIFICATIONS

#include "SingleTrackModel.h"
#include <cmath>

SingleTrackModel::SingleTrackModel() {}

void SingleTrackModel::initialize(std::map<std::string, std::string> &parameters)
{
  // Initialize the parameters
  if (parameters.find("a") != parameters.end())
  {
    this->a = std::stod(parameters["a"]);
  }
  if (parameters.find("b") != parameters.end())
  {
    this->b = std::stod(parameters["b"]);
  }
  if (parameters.find("M") != parameters.end())
  {
    this->M = std::stod(parameters["M"]);
  }
  if (parameters.find("Nr") != parameters.end())
  {
    this->Nr = std::stod(parameters["Nr"]);
  }
  if (parameters.find("Nf") != parameters.end())
  {
    this->Nf = std::stod(parameters["Nf"]);
  }
  if (parameters.find("Iz") != parameters.end())
  {
    this->Iz = std::stod(parameters["Iz"]);
  }
  if (parameters.find("KLr") != parameters.end())
  {
    this->KLr = std::stod(parameters["KLr"]);
  }
  if (parameters.find("KLf") != parameters.end())
  {
    this->KLf = std::stod(parameters["KLf"]);
  }
  if (parameters.find("Dyr") != parameters.end())
  {
    this->Dyr = std::stod(parameters["Dyr"]);
  }
  if (parameters.find("Dyf") != parameters.end())
  {
    this->Dyf = std::stod(parameters["Dyf"]);
  }
  if (parameters.find("tau_H") != parameters.end())
  {
    this->tau_H = std::stod(parameters["tau_H"]);
  }
  if (parameters.find("kd") != parameters.end())
  {
    this->kd = std::stod(parameters["kd"]);
  }
  if (parameters.find("error") != parameters.end())
  {
    this->_error = std::stod(parameters["error"]);
  }
  if (parameters.find("error_dot") != parameters.end())
  {
    this->error_dot = std::stod(parameters["error_dot"]);
  }
  if (parameters.find("error_int") != parameters.end())
  {
    this->error_int = std::stod(parameters["error_int"]);
  }
  if (parameters.find("error_prev") != parameters.end())
  {
    this->error_prev = std::stod(parameters["error_prev"]);
  }
  if (parameters.find("tau") != parameters.end())
  {
    this->tau = std::stod(parameters["tau"]);
  }
  if (parameters.find("Kp") != parameters.end())
  {
    this->Kp = std::stod(parameters["Kp"]);
  }
  if (parameters.find("Ki") != parameters.end())
  {
    this->Ki = std::stod(parameters["Ki"]);
  }
  if (parameters.find("Kd") != parameters.end())
  {
    this->Kd = std::stod(parameters["Kd"]);
  }
  if (parameters.find("A_max") != parameters.end())
  {
    this->A_max = std::stod(parameters["A_max"]);
  }
  if (parameters.find("A_min") != parameters.end())
  {
    this->A_min = std::stod(parameters["A_min"]);
  }
  if (parameters.find("dt") != parameters.end())
  {
    this->dt = std::stod(parameters["dt"]);
  }
}

void SingleTrackModel::step(double dt)
{
  // Compute the derivative of the state vector
  ode(this->X, this->U, this->XDOT);

  // Compute the next state vector
  for (int i = 0; i < this->X.size(); i++)
  {
     this->X[i] += this->XDOT[i] * dt;
  }
}

void SingleTrackModel::terminate()
{
  // Nothing to do here
}

void SingleTrackModel::set_control(FVehicleControl &control)
{
  // Convert the pedal control
  double Sr = model.get_parameters().M * 9.81 * (VehicleControl.Throttle - VehicleControl.Brake);

  // Handle reverse mode and stop
  double eps = 1e-6;
  if (!VehicleControl.bReverse)
  {
    // Avoid going backwards
    if (Sr < 0 && this->X[0] < eps)
    {
      Sr = 0;
    }
  }
  else
  {
    // Avoid going forward
    Sr = -Sr;
    if (Sr > 0 && this->X[0] > -eps)
    {
      Sr = 0;
    }
  }

  this->U[0] = Sr;
  this->U[1] = VehicleControl.Steer * this->DEGTORAD * this->tau_H;
}

void SingleTrackModel::set_location(FVector &location)
{
  this->X[3] = location.X * this->CMTOM;
  this->X[4] = location.Y * this->CMTOM;
}

void SingleTrackModel::set_rotation(FRotator &rotation)
{
  this->X[5] = rotation.Yaw * this->DEGTORAD;
}

void SingleTrackModel::get_location(FVector &location)
{
  location.X = this->X[3] * this->MTOCM;
  location.Y = this->X[4] * this->MTOCM;
}

void SingleTrackModel::get_rotation(FRotator &rotation)
{
  rotation.Yaw = this->X[5] * this->RADTODEG;
}

void SingleTrackModel::get_current_gear(int32 &gear)
{
  // We do not have gears in this model, return 0
  gear = 0;
}

void SingleTrackModel::get_front_wheel_angles(std::pair<float, float> &angles)
{
  // We combine the front wheel angles into a single value in this model
  angles.first  = this->U[1] * this->RADTODEG;
  angles.second = angles.first;
}

void SingleTrackModel::ode(const std::vector<double> &X,
                           const std::vector<double> &U,
                           std::vector<double> &XDOT)
{
  // Compute the tyre forces
  tyre_model(X, U, this->tyre_forces);

  // Compute the derivatives
  this->XDOT[0] = -(-this->X[2] * this->M * this->X[1] + sin(this->X[1]) * this->tyre_forces[1] - this->U[0] + kd * this->X[0] * this->X[0]) / this->M;
  this->XDOT[1] = (-this->X[2] * this->M * this->X[0] + cos(this->X[1]) * this->tyre_forces[1] + this->tyre_forces[0]) / this->M;
  this->XDOT[2] = (a * cos(this->X[1]) * this->tyre_forces[1] - b * this->tyre_forces[0]) / this->Iz;
  this->XDOT[3] = this->X[0] * cos(this->X[5]) - this->X[1] * sin(this->X[5]);
  this->XDOT[4] = this->X[0] * sin(this->X[5]) + this->X[1] * cos(this->X[5]);
  this->XDOT[5] = this->X[2];
}

void SingleTrackModel::tyre_model(const std::vector<double> &X,
                                  const std::vector<double> &U,
                                  std::vector<double> &results)
{
  // Handle the case where u = 0
  double eps = 1e-6;

  // Compute the tyre forces
  double lambda_R = -atan((-this->X[2] * this->b + this->X[1]) / (this->X[0] + eps));
  double lambda_F = -atan((-sin(this->U[1]) * this->X[0] + cos(this->U[1]) * this->X[2] * this->a + cos(this->U[1]) * this->X[1]) / (cos(this->U[1]) * (this->X[0] + eps) + sin(this->U[1]) * this->X[2] * this->a + sin(this->U[1]) * this->X[1]));

  double Fr = this->KLr * lambda_R * this->Nr / sqrt(1.0 + pow((this->KLr * lambda_R), 2) / pow(this->Dyr, 2));
  double Ff = this->KLf * lambda_F * this->Nf / sqrt(1.0 + pow((this->KLf * lambda_F), 2) / pow(this->Dyf, 2));

  results[0] = Fr;
  results[1] = Ff;
}

// ███████╗███╗   ██╗██████╗
// ██╔════╝████╗  ██║██╔══██╗
// █████╗  ██╔██╗ ██║██║  ██║
// ██╔══╝  ██║╚██╗██║██║  ██║
// ███████╗██║ ╚████║██████╔╝
// ╚══════╝╚═╝  ╚═══╝╚═════╝