/**
 * \file SingleTrackModel.h
 * \brief Header file for the single track model
*/

#ifndef SINGLE_TRACK_H
#define SINGLE_TRACK_H

#include <iostream> // to access the standard library

/**
 * \brief Structure for the parameters of the vehicle
*/
struct Parameters
{
  double M, a, b, Iz, Nr, Nf, KLr, KLf, Dyr, Dyf;
};

/**
 * \class SingleTrackModel
 * \brief Class for the single track model
*/
class SingleTrackModel
{
  private:

    // Keep all parameters private
    Parameters _parameters;
    double _error      = 0;
    double _error_dot  = 0;
    double _error_int  = 0;
    double _error_prev = 0;
    double _tau        = 0.04;
    double _Kp         = 1.0;
    double _Ki         = 1.0;
    double _Kd         = 0.0;
    double _A_max      = 2.0;
    double _A_min      = -2.0;

  public:

    /**
     * \brief Constructor with default parameters
     * Default parameters are set.
    */
    SingleTrackModel();

    /**
     * \brief Constructor without default parametersZ
     * \param parameters The parameters of the vehicle
     * The parameters of the vehicle are set.
    */
    SingleTrackModel(Parameters & parameters);

    /**
     * \brief Set the parameters
     * \param parameters The parameters of the vehicle
     * The parameters of the vehicle are set.
    */
    void set_parameters(Parameters & parameters)
    {
      this->_parameters = parameters;
    }

    /**
     * \brief Get the parameters
     * \return The parameters of the vehicle
     * The parameters of the vehicle are returned.
    */
    Parameters get_parameters()
    {
      return _parameters;
    }

    /**
     * \brief Set the parameter tau
     * \param tau The parameter tau
     * The parameter tau is set.
    */
    void set_tau(double tau)
    {
      this->_tau = tau;
    }

    /**
     * \brief Set the parameters Kp, Ki, Kd
     * \param Kp The proportional gain
     * \param Ki The integral gain
     * \param Kd The derivative gain
     * The parameters Kp, Ki, Kd are set.
    */
    void set_Kp_Ki_Kd(double Kp, double Ki, double Kd)
    {
      this->_Kp = Kp;
      this->_Ki = Ki;
      this->_Kd = Kd;
    }

    /**
     * \brief Set the parameters A_max and A_min
     * \param amin The minimum acceleration
     * \param amax The maximum acceleration
     * The parameters A_max and A_min are set.
    */
    void set_A_min_max(double amin, double amax)
    {
      this->_A_max = amax;
      this->_A_min = amin;
    }

    /**
     * \brief System ode
     * \param[in] X The state vector, X = [u, v, w, x, y, psi]
     * \param[in] U The input vector, U = [Sr, delta]
     * \param[out] XDOT The derivative of the state vector, XDOT = [u_dot, v_dot, w_dot, x_dot, y_dot, psi_dot]
     * Given the state vector X and the input vector U, compute the derivative
     * of the state vector XDOT.
    */
    void ode(const std::vector<double> & X,
             const std::vector<double> & U,
             std::vector<double> & XDOT);

    /**
     * \brief System ode
     * \param[in] X The state vector, X = [u, v, w, x, y, psi]
     * \param[in] U The input vector, U = [Sr, delta]
     * \return XDOT The derivative of the state vector, XDOT = [u_dot, v_dot, w_dot, x_dot, y_dot, psi_dot]
     * Given the state vector X and the input vector U, compute the derivative
     * of the state vector XDOT.
    */
    std::vector<double> ode(const std::vector<double> & X,
                            const std::vector<double> & U)
    {
      std::vector<double> XDOT(6);
      ode(X, U, XDOT);

      return XDOT;
    }

    /**
     * \brief Tyre model
     * \param[in] X The state vector, X = [u, v, w, x, y, psi]
     * \param[in] U The input vector, U = [Sr, delta]
     * \param[out] results The tyre forces, results = [Fr, Ff]
     * Given the state vector X and the input vector U, compute the tyre forces.
    */
    void tyre_model(const std::vector<double> & X,
                    const std::vector<double> & U,
                    std::vector<double> & results);

    /**
     * \brief Tyre model
     * \param[in] X The state vector, X = [u, v, w, x, y, psi]
     * \param[in] U The input vector, U = [Sr, delta]
     * \return results The tyre forces, results = [Fr, Ff]
     * Given the state vector X and the input vector U, compute the tyre forces.
    */
    std::vector<double> tyre_model(const std::vector<double> & X,
                                   const std::vector<double> & U)
    {
      std::vector<double> results(2);
      tyre_model(X, U, results);

      return results;
    }

    /**
     * \brief Advance one step
     * \param[in] X0 The initial state vector, X0 = [u0, v0, w0, x0, y0, psi0]
     * \param[in] U0 The initial input vector, U0 = [Sr0, delta0]
     * \param[in] dt The time step
     * \param[out] X1 The next state vector, X1 = [u1, v1, w1, x1, y1, psi1]
     * Given the initial state vector X0 and the initial input vector U0,
     * compute the state vector X1 after a time step dt.
    */
    void step(const std::vector<double> & X0,
              const std::vector<double> & U0,
              double dt,
              std::vector<double> &X1);

    /**
     * \brief Advance one step
     * \param[in] X0 The initial state vector, X0 = [u0, v0, w0, x0, y0, psi0]
     * \param[in] U0 The initial input vector, U0 = [Sr0, delta0]
     * \param[in] dt The time step
     * \return X1 The next state vector, X1 = [u1, v1, w1, x1, y1, psi1]
     * Given the initial state vector X0 and the initial input vector U0,
     * compute the state vector X1 after a time step dt.
    */
    std::vector<double> step(const std::vector<double> & X0,
                             const std::vector<double> & U0,
                             double dt)
    {
      std::vector<double> X1(6);
      step(X0, U0, dt, X1);

      return X1;
    }

    /**
     * \brief Compute the controls
     * \param[in] X The state vector, X = [u, v, w, x, y, psi]
     * \param[out] U The input vector, U = [Sr, delta]
     * \param[in] u_ref The reference speed
     * \param[in] kappa The curvature
     * Given the state vector X, compute the input vector U.
    */
    void compute_controls(const std::vector<double> & X,
                          std::vector<double> & U,
                          double u_ref, double kappa);

    /**
     * \brief Compute the controls
     * \param[in] X The state vector, X = [u, v, w, x, y, psi]
     * \param[in] u_ref The reference speed
     * \param[in] kappa The curvature
     * \return U The input vector, U = [Sr, delta]
     * Given the state vector X, compute the input vector U.
    */
    std::vector<double> compute_controls(const std::vector<double> & X,
                                         double u_ref, double kappa)
    {
      std::vector<double> U(2);
      compute_controls(X, U, u_ref, kappa);

      return U;
    }

    /**
     * \brief Compute the acceleration
     * \param X The state vector, X = [u, v, w, x, y, psi]
     * \param U The input vector, U = [Sr, delta]
     * \return The acceleration
     * Given the state vector X and the input vector U, compute the acceleration
     * of the vehicle.
    */
    double compute_acceleration(const std::vector<double> & X,
                                const std::vector<double> & U);
};

#endif