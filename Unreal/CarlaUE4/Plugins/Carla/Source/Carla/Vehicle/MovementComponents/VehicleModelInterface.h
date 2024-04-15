// ██████╗ ███████╗ ██████╗ ██╗███╗   ██╗
// ██╔══██╗██╔════╝██╔════╝ ██║████╗  ██║
// ██████╔╝█████╗  ██║  ███╗██║██╔██╗ ██║
// ██╔══██╗██╔══╝  ██║   ██║██║██║╚██╗██║
// ██████╔╝███████╗╚██████╔╝██║██║ ╚████║
// ╚═════╝ ╚══════╝ ╚═════╝ ╚═╝╚═╝  ╚═══╝
// #UNITN_MODIFICATIONS

#pragma once

// Carla includes
#include "Carla/Vehicle/VehicleControl.h"

// Standard includes
#include <utility>
#include <map>
#include <string>
#define _USE_MATH_DEFINES // enable M_PI on windows
#include <math.h>

/**
 * \class VehicleModelInterface
 * \brief Interface for internal vehicle models
 */
class VehicleModelInterface
{
public:
  /**
   * \brief Initialize the vehicle model with the given parameters
   *
   * \param parameters The parameters of the vehicle model expressed as a map of
   * string keys and string values
   */
  virtual void initialize(std::map<std::string, std::string> &parameters) = 0;

  /**
   * \brief Advance one step, you must support substepping as `dt` could vary
   *
   * \param dt The time step
   */
  virtual void step(double dt) = 0;

  /**
   * \brief Get the current state of the vehicle model
   *
   * \return The current state of the vehicle model
   */
  virtual void terminate() = 0;

  /**
   * \brief Set the control of the vehicle model
   *
   * \param control The control of the vehicle model
   */
  virtual void set_control(FVehicleControl &control) = 0;

  /**
   * \brief Set the location of the vehicle model
   *
   * \param location The location of the vehicle model, in cm
   */
  virtual void set_location(FVector &location) = 0;

  /**
   * \brief Set the rotation of the vehicle model
   *
   * \param rotation The rotation of the vehicle model, in degrees
   */
  virtual void set_rotation(FRotator &rotation) = 0;

  /**
   * \brief Get the location of the vehicle model
   *
   * \param location The location of the vehicle model, in cm
   */
  virtual void get_location(FVector &location) = 0;

  /**
   * \brief Get the rotation of the vehicle model
   *
   * \param rotation The rotation of the vehicle model, in degrees
   */
  virtual void get_rotation(FRotator &rotation) = 0;

  /**
   * \brief Get the current gear of the vehicle model
   *
   * \param gear The current gear of the vehicle model
   */
  virtual void get_current_gear(int32 &gear) = 0;

  /**
   * \brief Get the front wheel angles of the vehicle model
   *
   * \param angles The front wheel angles of the vehicle model, in degrees
   * [left, right]
   */
  virtual void get_front_wheel_angles(std::pair<float, float> &angles) = 0;

protected:
  // UE4 conversions
  const double CMTOM = 0.01;
  const double MTOCM = 100;
  const double DEGTORAD = M_PI / 180.0;
  const double RADTODEG = 180.0 / M_PI;
};

// ███████╗███╗   ██╗██████╗
// ██╔════╝████╗  ██║██╔══██╗
// █████╗  ██╔██╗ ██║██║  ██║
// ██╔══╝  ██║╚██╗██║██║  ██║
// ███████╗██║ ╚████║██████╔╝
// ╚══════╝╚═╝  ╚═══╝╚═════╝