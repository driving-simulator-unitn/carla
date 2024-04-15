// ██████╗ ███████╗ ██████╗ ██╗███╗   ██╗
// ██╔══██╗██╔════╝██╔════╝ ██║████╗  ██║
// ██████╔╝█████╗  ██║  ███╗██║██╔██╗ ██║
// ██╔══██╗██╔══╝  ██║   ██║██║██║╚██╗██║
// ██████╔╝███████╗╚██████╔╝██║██║ ╚████║
// ╚═════╝ ╚══════╝ ╚═════╝ ╚═╝╚═╝  ╚═══╝
// #UNITN_MODIFICATIONS

#pragma once

#include "VehicleModelInterface.h"

#include <iostream>
#include <vector>

/**
 * \class SingleTrackModel
 * \brief Class for the single track model
 */
class SingleTrackModel : public VehicleModelInterface
{
private:
  // Keep all parameters private
  double a = 1.3;
  double b = 1.3;
  double M = 1000;
  double Nr = 5000;
  double Nf = 5000;
  double Iz = 1200;
  double KLr = 3.0 * 24;
  double KLf = 3.0 * 20;
  double Dyr = 3.0 * 1.2;
  double Dyf = 3.0 * 1.2;
  double tau_H = 1.0 / 4.0;
  double kd = 0.25;
  double error = 0;
  double error_dot = 0;
  double error_int = 0;
  double error_prev = 0;
  double tau = 0.04;
  double Kp = 1.0;
  double Ki = 1.0;
  double Kd = 0.0;
  double A_max = 2.0;
  double A_min = -2.0;
  double dt = 0.001;

  // State vector, X = [u, v, w, x, y, psi]
  std::vector<double> X = {0, 0, 0, 0, 0, 0};

  // Input vector, U = [Sr, delta]
  std::vector<double> U = {0, 0};

  // Derivative of the state vector,
  // XDOT = [u_dot, v_dot, w_dot, x_dot, y_dot, psi_dot]
  std::vector<double> XDOT = {0, 0, 0, 0, 0, 0};

  // Tyre forces, [Fr, Ff]
  std::vector<double> tyre_forces = {0, 0};

public:
  /**
   * \brief Constructor
   */
  SingleTrackModel();

   /**
   * \brief Initialize the vehicle model with the given parameters
   *
   * \param parameters The parameters of the vehicle model expressed as a map of
   * string keys and string values
   */
  void initialize(std::map<std::string, std::string> &parameters);

  /**
   * \brief Advance one step, you must support substepping as `dt` could vary
   *
   * \param dt The time step
   */
  void step(double dt);

  /**
   * \brief Get the current state of the vehicle model
   *
   * \return The current state of the vehicle model
   */
  void terminate();

  /**
   * \brief Set the control of the vehicle model
   *
   * \param control The control of the vehicle model
   */
  void set_control(FVehicleControl &control);

  /**
   * \brief Set the location of the vehicle model
   *
   * \param location The location of the vehicle model
   */
  void set_location(FVector &location);

  /**
   * \brief Set the rotation of the vehicle model
   *
   * \param rotation The rotation of the vehicle model
   */
  void set_rotation(FRotator &rotation);

  /**
   * \brief Get the location of the vehicle model
   *
   * \param location The location of the vehicle model
   */
  void get_location(FVector &location);

  /**
   * \brief Get the rotation of the vehicle model
   *
   * \param rotation The rotation of the vehicle model
   */
  void get_rotation(FRotator &rotation);

  /**
   * \brief Get the current gear of the vehicle model
   *
   * \param gear The current gear of the vehicle model
   */
  void get_current_gear(int32 &gear);

  /**
   * \brief Get the front wheel angles of the vehicle model
   *
   * \param angles The front wheel angles of the vehicle model, [left, right]
   */
  void get_front_wheel_angles(std::pair<float, float> &angles_in_deg);

private:
  /**
   * \brief System ode
   * \param[in] X The state vector, X = [u, v, w, x, y, psi]
   * \param[in] U The input vector, U = [Sr, delta]
   * \param[out] XDOT The derivative of the state vector, XDOT = [u_dot, v_dot, w_dot, x_dot, y_dot, psi_dot]
   * Given the state vector X and the input vector U, compute the derivative
   * of the state vector XDOT.
   */
  void ode(const std::vector<double> &X,
           const std::vector<double> &U,
           std::vector<double> &XDOT);

  /**
   * \brief Tyre model
   * \param[in] X The state vector, X = [u, v, w, x, y, psi]
   * \param[in] U The input vector, U = [Sr, delta]
   * \param[out] results The tyre forces, results = [Fr, Ff]
   * Given the state vector X and the input vector U, compute the tyre forces.
   */
  void tyre_model(const std::vector<double> &X,
                  const std::vector<double> &U,
                  std::vector<double> &results);
};

// ███████╗███╗   ██╗██████╗
// ██╔════╝████╗  ██║██╔══██╗
// █████╗  ██╔██╗ ██║██║  ██║
// ██╔══╝  ██║╚██╗██║██║  ██║
// ███████╗██║ ╚████║██████╔╝
// ╚══════╝╚═╝  ╚═══╝╚═════╝