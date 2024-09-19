// Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

/*
* This file is a custom movement component which implements an asynchronous
* communication between the Carla simulator and any external physics engine.
* The communication is done through ZMQ sockets. The pattern used is PUSH-PULL,
* one socket to push data to the physics engine and another one to pull data
* from the physics engine.
*/

#include "ZMQMovementComponent.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/MovementComponents/DefaultMovementComponent.h"

#include "compiler/disable-ue4-macros.h"
#include <carla/rpc/String.h>
#include <carla/geom/Transform.h>
#include <carla/geom/Location.h>
#include <carla/geom/Rotation.h>
#include "compiler/enable-ue4-macros.h"
#include "Carla/Util/RayTracer.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaGameInstance.h"
#include "Carla/Game/CarlaStatics.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include <chrono>

void UZMQMovementComponent::CreateZMQMovementComponent(
  ACarlaWheeledVehicle* Vehicle,
  FString front_endpoint,
  FString back_endpoint,
  bool attach_spectator,
  FTransform spectator_transform
)
{
  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: CreateZMQMovementComponent - In"));

  // Create the movement component
  UZMQMovementComponent* ZMQMovementComponent = NewObject<UZMQMovementComponent>(Vehicle);

  // Save original vehicle states
  ZMQMovementComponent->location    = Vehicle->GetActorLocation();
  ZMQMovementComponent->orientation = Vehicle->GetActorRotation();
  ZMQMovementComponent->gear        = Vehicle->GetVehicleCurrentGear();
  ZMQMovementComponent->velocity    = Vehicle->GetVelocity();

  // Initialize the ZMQ context
  ZMQMovementComponent->context = zmq_ctx_new();

  // Initialize the ZMQ frontend socket
  ZMQMovementComponent->frontend   = zmq_socket(ZMQMovementComponent->context, ZMQ_PUB);
  ZMQMovementComponent->front_endpoint = TCHAR_TO_UTF8(*front_endpoint);

  // Initialize the ZMQ backend socket
  ZMQMovementComponent->backend   = zmq_socket(ZMQMovementComponent->context, ZMQ_SUB);
  ZMQMovementComponent->back_endpoint = TCHAR_TO_UTF8(*back_endpoint);

  // Set PUB socket to not linger
  int linger = 0;
  zmq_setsockopt(ZMQMovementComponent->frontend, ZMQ_LINGER, &linger, sizeof(linger));

  // Set the SUB socket receive timeout
  int rcvtimeo = 1000; // 1 s
  zmq_setsockopt(ZMQMovementComponent->backend, ZMQ_RCVTIMEO, &rcvtimeo, sizeof(rcvtimeo));

  // Assign the movement component to the vehicle
  Vehicle->SetCarlaMovementComponent(ZMQMovementComponent);
  ZMQMovementComponent->RegisterComponent();

  if (attach_spectator)
  {
    // Get the current episode
    auto *World = Vehicle->GetWorld();
    if (World == nullptr)
    {
      UE_LOG(LogCarla, Error, TEXT("ZMQ Physics: could not get world."));
    }

    UCarlaGameInstance *GameInstance = UCarlaStatics::GetGameInstance(World);
    if (GameInstance == nullptr)
    {
      UE_LOG(LogCarla, Error, TEXT("ZMQ Physics: could not get game instance."));
    }

    auto *Episode = GameInstance->GetCarlaEpisode();
    if (Episode == nullptr)
    {
      UE_LOG(LogCarla, Error, TEXT("ZMQ Physics: could not get episode."));
    }

    // Get the spectator
    ZMQMovementComponent->spectator = Episode->GetSpectatorPawn();
    if (ZMQMovementComponent->spectator == nullptr)
    {
      UE_LOG(LogCarla, Error, TEXT("ZMQ Physics: could not get episode."));
    }

    // Save the spectator matrix
    ZMQMovementComponent->spectator_matrix = carla::geom::Transform(spectator_transform).GetMatrix();
  }

  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: CreateZMQMovementComponent - Out"));
}

void UZMQMovementComponent::BeginPlay()
{
  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: BeginPlay - In"));

  // Call the base class
  Super::BeginPlay();

  // Disable UE4 physics
  DisableUE4VehiclePhysics();

  // Connect the frontend socket to the frontend endpoint
  if (zmq_connect(this->frontend, this->front_endpoint.c_str()) == -1)
  {
    UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: ZMQ frontend socket could not connect to the frontend endpoint, reason %s"), zmq_strerror(errno));
  }

  // Connect the backend socket to the backend endpoint
  if (zmq_connect(this->backend, this->back_endpoint.c_str()) == -1)
  {
    UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: ZMQ backend socket could not connect to the backend endpoint, reason %s"), zmq_strerror(errno));
  }

  /**
   * In theory, one should load the topics from the topics.toml file provided with
   * the interfaces. In practice, that is difficult because the binary is intalled
   * via Unreal, meaning we would need to again modifiy the CarlaBuild.cs file as we
   * did with ZeroMQ. We could do that, but I see diminishing returns as if the topic
   * changes, the interface changed too. This means that we would have to compile and
   * export the package again, so we might as well change the topic manually too.
   * This could be a future improvement, for now, we leave it like this.
  */

  // Subscribe to the topics
  std::string egovehicle_topic = "DrivingSimulator.EgoVehicle.IEgoVehicle";
  zmq_setsockopt(this->backend, ZMQ_SUBSCRIBE, egovehicle_topic.c_str(), egovehicle_topic.size());

  // Start the receive queue
  this->recv_queue.start(&this->backend);

  // Set the PUB topic
  this->topic = "DrivingSimulator.EnvironmentInteractions.IEnvironmentInteractions";

  // Change running status
  this->status = true;

  // Set callbacks to react to collisions
  CarlaVehicle->OnActorHit.AddDynamic(
    this,
    &UZMQMovementComponent::OnVehicleHit
  );
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.AddDynamic(
    this,
    &UZMQMovementComponent::OnVehicleOverlap
  );
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
    ECollisionChannel::ECC_WorldStatic,
    ECollisionResponse::ECR_Overlap
  );

  // Activate audio
  CarlaVehicle->SetVolume(1.f);

  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: BeginPlay - Out"));
}

void UZMQMovementComponent::ProcessControl(FVehicleControl &Control) {}

// On each tick we do the following:
// 1. Broadcast the low level data (e.g., ego-vehicle pose, environment information)
// 2. Receive the next vehicle state from the physics engine
// 3. Update the vehicle state
// 4. Update the spectator position if the user asked to attach it to the vehicle
void UZMQMovementComponent::TickComponent(
  float DeltaTime,
  ELevelTick TickType,
  FActorComponentTickFunction* ThisTickFunction
)
{
  // Receive the ego-vehicle state
  if (this->recv_queue.msg_recv(this->msg, ZMQ_DONTWAIT))
  {
    this->egovehicle = flatbuffers::GetRoot<DrivingSimulator::EgoVehicle::IEgoVehicle>(zmq_msg_data(this->msg->at(1)));

    if (this->egovehicle->ecu() != nullptr)
    {
      // Save the gear
      this->gear = (int32) this->egovehicle->ecu()->ecu_gear();
    }

    if (nullptr == this->egovehicle->chassis())
    {
      return;
    }

    // If the ego-vehicle is sending a custom reference frame we will use that instead of
    // the chassis one
    double x, y, z, pitch, yaw, roll = 0.0;
    bool got_rf    = false;
    if (nullptr != this->egovehicle->sensors())
    {
      if (nullptr != this->egovehicle->sensors()->rf())
      {
        got_rf = nullptr != this->egovehicle->sensors()->rf()->ref_frame_custom();
      }
    }
    if (got_rf)
    {
      // The RF is saved in column-major format as a RHS system
      // We want to obtain the angles which correspond to a Z-Y-X rotation
      x         = this->egovehicle->sensors()->rf()->ref_frame_custom()->data()->Get(12);
      y         = this->egovehicle->sensors()->rf()->ref_frame_custom()->data()->Get(13);
      z         = this->egovehicle->sensors()->rf()->ref_frame_custom()->data()->Get(14);
      pitch     = -std::asin(this->egovehicle->sensors()->rf()->ref_frame_custom()->data()->Get(2));
      yaw       = std::atan2(this->egovehicle->sensors()->rf()->ref_frame_custom()->data()->Get(1), this->egovehicle->sensors()->rf()->ref_frame_custom()->data()->Get(0));
      roll      = std::atan2(this->egovehicle->sensors()->rf()->ref_frame_custom()->data()->Get(6), this->egovehicle->sensors()->rf()->ref_frame_custom()->data()->Get(10));
    }
    else
    {
      x     = this->egovehicle->chassis()->x();
      y     = this->egovehicle->chassis()->y();
      z     = this->egovehicle->chassis()->z();
      pitch = this->egovehicle->chassis()->mu();
      yaw   = this->egovehicle->chassis()->psi();
      roll  = this->egovehicle->chassis()->phi();
    }

    // Save the pose
    this->location.X        =  x     * this->MTOCM;
    this->location.Y        = -y     * this->MTOCM;
    this->location.Z        =  z     * this->MTOCM;
    this->orientation.Pitch = -pitch * this->RADTODEG;
    this->orientation.Yaw   = -yaw   * this->RADTODEG;
    this->orientation.Roll  =  roll  * this->RADTODEG;

    // Save the velocity
    this->velocity.X =  this->egovehicle->chassis()->u() * this->MTOCM;
    this->velocity.Y = -this->egovehicle->chassis()->v() * this->MTOCM;
    this->velocity.Z =  this->egovehicle->chassis()->w() * this->MTOCM;

    // Update the vehicle pose
    CarlaVehicle->SetActorLocation(this->location);
    CarlaVehicle->SetActorRotation(this->orientation);

    // Move the spectator to the correct pose
    if (this->spectator != nullptr)
    {
      // Get the actor transformation matrix
      this->vehicle_matrix = carla::geom::Transform(
        carla::geom::Location(this->location),
        carla::geom::Rotation(this->orientation)
      ).GetMatrix();

      // Perform matrix multiplication
      // Our matrix is column-major, carla matrices are row major
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          this->spectator_resulting_matrix[i * 4 + j] = 0.0f;
          for (int k = 0; k < 4; k++)
          {
            this->spectator_resulting_matrix[i * 4 + j] += vehicle_matrix[i * 4 + k] * this->spectator_matrix[k * 4 + j];
          }
        }
      }

      // Convert to a Carla Transform
      carla::geom::Transform result_transform(
        carla::geom::Location(
          this->spectator_resulting_matrix[3],
          this->spectator_resulting_matrix[7],
          this->spectator_resulting_matrix[11]
        ),
        carla::geom::Rotation(
          std::asinf(this->spectator_resulting_matrix[8])                                         * this->RADTODEG,
          std::atan2f(this->spectator_resulting_matrix[4], this->spectator_resulting_matrix[0])   * this->RADTODEG,
          std::atan2f(-this->spectator_resulting_matrix[9], this->spectator_resulting_matrix[10]) * this->RADTODEG
        )
      );

      // Move the spectator
      this->spectator->SetActorTransform(FTransform(result_transform));
    }
  }

  // Broadcast the environment information
  this->get_environment(this->egovehicle);
  this->send_environment();

  // Update the audio
  float zmq_rpm = this->velocity.X / (42*this->MTOCM)*5000;
  UE_LOG(LogCarla, Warning, TEXT("velocity: %f [cm/s]"), this->velocity.X);
  CarlaVehicle->TickSounds(DeltaTime, zmq_rpm);
}

void UZMQMovementComponent::get_environment(DrivingSimulator::EgoVehicle::IEgoVehicle const *egovehicle)
{
  // Clear the builder
  this->builder.Clear();

  // Get transformation matrix of the vehicle in a RHS system with XYZ rotation
  double transform[16];
  this->get_vehicle_rhs_matrix(transform);
  DrivingSimulator::EnvironmentInteractions::TransformationMatrix transformation_matrix(transform);

  // Prepare the pois offsets
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<DrivingSimulator::EnvironmentInteractions::WheelContactPoints>>> wheels_contact_points_offset = 0;
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<DrivingSimulator::EnvironmentInteractions::ContactPoint>>> pois_contact_points_offset = 0;

  // Check if we got wheels
  bool got_wheels = true;
  if (nullptr == egovehicle)
  {
    got_wheels = false;
  }
  else if (nullptr == egovehicle->wheels())
  {
    got_wheels = false;
  }
  else if (nullptr == egovehicle->wheels()->wheels())
  {
    got_wheels = false;
  }
  else if (0 == egovehicle->wheels()->wheels()->size())
  {
    got_wheels = false;
  }

  // Check if we got sensor POIs
  bool got_sensor_pois = false;
  if (nullptr == egovehicle->sensors())
  {
    got_sensor_pois = false;
  }
  else if (nullptr == egovehicle->sensors()->pois())
  {
    got_sensor_pois = false;
  }
  else if (0 == egovehicle->sensors()->pois()->size())
  {
    got_sensor_pois = false;
  }

  // Maximum distance to search for terrain properties in [m]
  double const max_distance = 10.0;

  // Raycast points
  FVector start_location = {};
  FVector end_location   = {};

  // Prepare hit result
  FHitResult hit = {};
  FCollisionQueryParams collision_query_params = {};
  bool got_hit = false;

  // Ignore CarlaVehicle
  collision_query_params.AddIgnoredActor(CarlaVehicle);

  // Get the material back
  collision_query_params.bReturnPhysicalMaterial = true;

  // For each wheel of the ego vehicle, compute the contact points
  if (got_wheels)
  {
    // Extract the number of wheels
    size_t wheels_number = egovehicle->wheels()->wheels()->size();

    // Prepare the wheels_contact_points vector
    std::vector<flatbuffers::Offset<DrivingSimulator::EnvironmentInteractions::WheelContactPoints>> wheels_contact_points;
    wheels_contact_points.reserve(wheels_number);

    // Prepare the contact points vector that will be re-used for each wheel
    std::vector<flatbuffers::Offset<DrivingSimulator::EnvironmentInteractions::ContactPoint>> wheel_contact_points;


    DrivingSimulator::EgoVehicle::Wheels::Wheel const *wheel   = nullptr;
    DrivingSimulator::EgoVehicle::Wheels::POI const *wheel_poi = nullptr;
    size_t wheel_pois_number                                   = 0;
    for (size_t i = 0; i < wheels_number; i++)
    {
      wheel = egovehicle->wheels()->wheels()->Get(i);

      // Make sure we have Points of Interests (POIs)
      if (nullptr == wheel->terrain_output())
      {
        continue;
      }
      if (nullptr == wheel->terrain_output()->pois())
      {
        continue;
      }
      if (0 == wheel->terrain_output()->pois()->size())
      {
        continue;
      }

      wheel_pois_number = wheel->terrain_output()->pois()->size();
      wheel_contact_points.clear();
      wheel_contact_points.reserve(wheel_pois_number);

      for (size_t j = 0; j < wheel_pois_number; j++)
      {
        wheel_poi = wheel->terrain_output()->pois()->Get(j);

        if (nullptr == wheel_poi->transform())
        {
          // Add an emtpy collision since the order matters
          wheel_contact_points.push_back(this->create_contact_point(false, hit));
          continue;
        }

        if (!this->check_transform(wheel_poi->transform()->data()->data()))
        {
          // Add an emtpy collision since the order matters
          wheel_contact_points.push_back(this->create_contact_point(false, hit));
          continue;
        }

        // Compute the contact point
        got_hit = this->compute_contact_point(
          wheel_poi->transform()->data()->data(),
          start_location,
          end_location,
          max_distance,
          hit,
          collision_query_params
        );

        // Fill the contact point table
        wheel_contact_points.push_back(this->create_contact_point(got_hit, hit));
      }

      // Fill the wheel contact points table
      auto contact_points_offset = this->builder.CreateVector(wheel_contact_points);
      wheels_contact_points.push_back(
        DrivingSimulator::EnvironmentInteractions::CreateWheelContactPoints(
          this->builder,
          contact_points_offset
        )
      );
    }

/*     // Front left
    if (nullptr != egovehicle->chassis()->rfw_fix_fl())
    {
      // Compute the contact point
      got_hit = this->compute_contact_point(
        egovehicle->chassis()->rfw_fix_fl()->data()->data(),
        start_location,
        end_location,
        max_distance,
        hit,
        collision_query_params
      );

      // Fill the contact point table
      contact_points.push_back(this->create_contact_point(got_hit, hit));
    }

    // Front right
    if (nullptr != egovehicle->chassis()->rfw_fix_fr())
    {
      // Compute the contact point
      got_hit = this->compute_contact_point(
        egovehicle->chassis()->rfw_fix_fr()->data()->data(),
        start_location,
        end_location,
        max_distance,
        hit,
        collision_query_params
      );

      // Fill the contact point table
      contact_points.push_back(this->create_contact_point(got_hit, hit));
    }

    // Rear left
    if (nullptr != egovehicle->chassis()->rfw_fix_rl())
    {
      // Compute the contact point
      got_hit = this->compute_contact_point(
        egovehicle->chassis()->rfw_fix_rl()->data()->data(),
        start_location,
        end_location,
        max_distance,
        hit,
        collision_query_params
      );

      // Fill the contact point table
      contact_points.push_back(this->create_contact_point(got_hit, hit));
    }

    // Rear right
    if (nullptr != egovehicle->chassis()->rfw_fix_rr())
    {
      // Compute the contact point
      got_hit = this->compute_contact_point(
        egovehicle->chassis()->rfw_fix_rr()->data()->data(),
        start_location,
        end_location,
        max_distance,
        hit,
        collision_query_params
      );

      // Fill the contact point table
      contact_points.push_back(this->create_contact_point(got_hit, hit));
    }
 */

    // Fill in the offset
    wheels_contact_points_offset = this->builder.CreateVector(wheels_contact_points);
  }

  // For each POI, compute the contact points
  if (got_sensor_pois)
  {
    // Extract the number of sensor pois
    size_t sensor_pois_number = egovehicle->sensors()->pois()->size();

    // Prepare the pois_contact_points vector
    std::vector<flatbuffers::Offset<DrivingSimulator::EnvironmentInteractions::ContactPoint>> sensor_pois_contact_points;
    sensor_pois_contact_points.reserve(sensor_pois_number);


    DrivingSimulator::EgoVehicle::Sensors::POI const *sensor_poi = nullptr;
    for (size_t i = 0; i < sensor_pois_number; i++)
    {
      sensor_poi = egovehicle->sensors()->pois()->Get(i);

      if (nullptr == sensor_poi->transform())
      {
        // Add an emtpy collision since the order matters
        sensor_pois_contact_points.push_back(this->create_contact_point(false, hit));
        continue;
      }

      if (!this->check_transform(sensor_poi->transform()->data()->data()))
      {
        // Add an emtpy collision since the order matters
        sensor_pois_contact_points.push_back(this->create_contact_point(false, hit));
        continue;
      }

      // Compute the contact point
      got_hit = this->compute_contact_point(
        sensor_poi->transform()->data()->data(),
        start_location,
        end_location,
        max_distance,
        hit,
        collision_query_params
      );

      // Fill the contact point table
      sensor_pois_contact_points.push_back(this->create_contact_point(got_hit, hit));
    }

    // Fill in the offset
    auto sensor_pois_contact_points_offset = this->builder.CreateVector(sensor_pois_contact_points);
  }

  // Fill the environment interaction table
  auto environment_builder = DrivingSimulator::EnvironmentInteractions::IEnvironmentInteractionsBuilder(this->builder);
  environment_builder.add_running(this->status);
  environment_builder.add_transform(&transformation_matrix);
  environment_builder.add_wheels_contact_points(wheels_contact_points_offset);
  environment_builder.add_pois_contact_points(pois_contact_points_offset);
  auto environment_offset = environment_builder.Finish();
  this->builder.Finish(environment_offset);
}

// Transform is assumed colum-major
bool UZMQMovementComponent::check_transform(double const transform[16])
{
  double tolerance = 1e-10;

  // First off, the last row has to be [0, 0, 0, 1]
  for (size_t i = 0; i < 3; i++)
  {
    if (std::abs(transform[i + 3 * (i + 1)]) > tolerance)
    {
      return false;
    }
  }
  if (std::abs(transform[15] - 1.0) > tolerance)
  {
    return false;
  }

  // Then, the rotation matrix has to respect R * R^T = I
  double elem = 0.0;
  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = 0; j < 3; j++)
    {
      elem = 0.0;
      for (size_t k = 0; k < 3; k++)
      {
       elem += transform[i + k * 4] * transform[j + k * 4];
      }

      if (i == j)
      {
        elem -= 1.0;
      }

      if (std::abs(elem) > tolerance)
      {
        return false;
      }
    }
  }

  return true;
}

bool UZMQMovementComponent::compute_contact_point(
  double const transform[16], // assumed column-major
  FVector &start_location,
  FVector &end_location,
  double max_distance,
  FHitResult &hit,
  FCollisionQueryParams const &collision_query_params
)
{
  // Get the end point in global coordinates
  double end_point_loc[4]  = {0.0, 0.0, -max_distance, 1.0};
  double end_point_glob[4] = {0.0, 0.0,  0.0,          1.0};
  this->transform_point(transform, end_point_loc, end_point_glob);

  // Fill the start and end locations
  start_location.X =  transform[12]     * this->MTOCM;
  start_location.Y = -transform[13]     * this->MTOCM;
  start_location.Z =  transform[14]     * this->MTOCM;
  end_location.X   =  end_point_glob[0] * this->MTOCM;
  end_location.Y   = -end_point_glob[1] * this->MTOCM;
  end_location.Z   =  end_point_glob[2] * this->MTOCM;

  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: compute_contact_point - start [%f, %f, %f], end [%f, %f, %f]"),
  start_location.X, start_location.Y, start_location.Z,
  end_location.X, end_location.Y, end_location.Z);

  // Raycast to get the environment properties
  // ref: https://docs.unrealengine.com/4.26/en-US/API/Runtime/Engine/Engine/FHitResult/
  bool got_hit = CarlaVehicle->GetWorld()->LineTraceSingleByChannel(
      hit,
      start_location,
      end_location,
      ECC_GameTraceChannel2, // camera (any collision)
      collision_query_params,
      FCollisionResponseParams()
  );

  return got_hit;
}

flatbuffers::Offset<DrivingSimulator::EnvironmentInteractions::ContactPoint> UZMQMovementComponent::create_contact_point(
  bool got_hit,
  FHitResult const &hit
)
{
  if (got_hit)
  {
    // Material
    auto material_builder = DrivingSimulator::EnvironmentInteractions::MaterialBuilder(this->builder);
    material_builder.add_density((hit.PhysMaterial)->Density);
    material_builder.add_friction((hit.PhysMaterial)->Friction);
    material_builder.add_restitution((hit.PhysMaterial)->Restitution);
    auto material_offset = material_builder.Finish();

    // Contact point
    auto contact_point_builder = DrivingSimulator::EnvironmentInteractions::ContactPointBuilder(this->builder);
    contact_point_builder.add_hit(got_hit);
    contact_point_builder.add_blocking_hit(hit.bBlockingHit);
    contact_point_builder.add_start_penetrating(hit.bStartPenetrating);
    contact_point_builder.add_distance(hit.Distance * this->CMTOM);
    DrivingSimulator::EnvironmentInteractions::Vec3 impact_normal(hit.ImpactNormal.X * this->CMTOM, -hit.ImpactNormal.Y * this->CMTOM, hit.ImpactNormal.Z * this->CMTOM);
    contact_point_builder.add_impact_normal(&impact_normal);
    DrivingSimulator::EnvironmentInteractions::Vec3 impact_point(hit.ImpactPoint.X * this->CMTOM, -hit.ImpactPoint.Y * this->CMTOM, hit.ImpactPoint.Z * this->CMTOM);
    contact_point_builder.add_impact_point(&impact_point);
    DrivingSimulator::EnvironmentInteractions::Vec3 vec_location(hit.Location.X * this->CMTOM, -hit.Location.Y * this->CMTOM, hit.Location.Z * this->CMTOM);
    contact_point_builder.add_location(&vec_location);
    DrivingSimulator::EnvironmentInteractions::Vec3 normal(hit.Normal.X * this->CMTOM, -hit.Normal.Y * this->CMTOM, hit.Normal.Z * this->CMTOM);
    contact_point_builder.add_normal(&normal);
    contact_point_builder.add_penetration_depth(hit.PenetrationDepth * this->CMTOM);
    contact_point_builder.add_material(material_offset);
    return contact_point_builder.Finish();
  }

  return DrivingSimulator::EnvironmentInteractions::CreateContactPoint(this->builder, got_hit);
}

void UZMQMovementComponent::send_environment()
{
 // Get the current timestamp in milliseconds as a string
 this->timestamp_str = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

 // Convert the cycle_count to string
 this->cycle_count_str = std::to_string(this->cycle_count);

 // Send the data
 zmq_send(this->frontend, this->topic.c_str(), this->topic.size(), ZMQ_SNDMORE);
 zmq_send(this->frontend, this->builder.GetBufferPointer(), this->builder.GetSize(), ZMQ_SNDMORE);
 zmq_send(this->frontend, this->timestamp_str.c_str(), this->timestamp_str.size(), ZMQ_SNDMORE);
 zmq_send(this->frontend, this->cycle_count_str.c_str(), this->cycle_count_str.size(), 0);

 // Increment the cylce count
 this->cycle_count++;
}

void UZMQMovementComponent::transform_point(double const transform[16], double const P0[4], double P1[4])
{
  // The transform is assumed column-major
 for (int i = 0; i < 4; i++)
 {
   P1[i] = 0.0;
   for (int j = 0; j < 4; j++)
   {
     P1[i] += transform[i + j * 4] * P0[j];
   }
 }
}

void UZMQMovementComponent::get_vehicle_rhs_matrix(double matrix[16])
{
  // Get transformation matrix of the vehicle in a RHS system with XYZ rotation
  // The matrix is saved in column-major order
  double x     =  this->location.X        * this->CMTOM;
  double y     = -this->location.Y        * this->CMTOM;
  double z     =  this->location.Z        * this->CMTOM;
  double roll  =  this->orientation.Roll  * this->DEGTORAD;
  double pitch = -this->orientation.Pitch * this->DEGTORAD;
  double yaw   = -this->orientation.Yaw   * this->DEGTORAD;
  double cr    = std::cos(roll);
  double sr    = std::sin(roll);
  double cp    = std::cos(pitch);
  double sp    = std::sin(pitch);
  double cy    = std::cos(yaw);
  double sy    = std::sin(yaw);
  //  matrix = {
  //      cy * cp, cy * sp * sr - sy * cr, sy * sr + cy * sp * cr, x,
  //      sy * cp, cy * cr + sy * sp * sr, sy * sp * cr - cy * sr, y,
  //     -sp,       cp * sr,               cp * cr,                z,
  //      0.0,     0.0,                    0.0,                    1.0
  //  };
  matrix[0]  = cy * cp;
  matrix[1]  = sy * cp;
  matrix[2]  = -sp;
  matrix[3]  = 0.0;

  matrix[4]  = cy * sp * sr - sy * cr;
  matrix[5]  = cy * cr + sy * sp * sr;
  matrix[6]  = cp * sr;
  matrix[7]  = 0.0;

  matrix[8]  = sy * sr + cy * sp * cr;
  matrix[9]  = sy * sp * cr - cy * sr;
  matrix[10] = cp * cr;
  matrix[11] = 0.0;

  matrix[12] = x;
  matrix[13] = y;
  matrix[14] = z;
  matrix[15] = 1.0;
}

FVector UZMQMovementComponent::GetVelocity() const
{
  return this->velocity;
}

int32 UZMQMovementComponent::GetVehicleCurrentGear() const
{
  return this->gear;
}

float UZMQMovementComponent::GetVehicleForwardSpeed() const
{
  return this->velocity.X;
}

void UZMQMovementComponent::close_zmq()
{
  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: close_zmq - In"));

  // Stop the queue
  this->recv_queue.stop(&this->backend);

  // Close the ZMQ sockets
  if (zmq_close(this->frontend) == -1)
  {
    UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: ZMQ frontend socket could not be closed, reason %s"), zmq_strerror(errno));
  }
  if (zmq_close(this->backend) == -1)

  {
    UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: ZMQ backend socket could not be closed, reason %s"), zmq_strerror(errno));
  }

  // Destroy the ZMQ context
  if (zmq_ctx_destroy(this->context) == -1)
  {
    UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: ZMQ context could not be destroyed, reason %s"), zmq_strerror(errno));
  }

  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: close_zmq - Out"));
}

void UZMQMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: EndPlay - In"));

  if(!CarlaVehicle)
  {
    return;
  }

  // Reset callbacks to react to collisions
  CarlaVehicle->OnActorHit.RemoveDynamic(
    this,
    &UZMQMovementComponent::OnVehicleHit
  );
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(
    this,
    &UZMQMovementComponent::OnVehicleOverlap
  );
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
    ECollisionChannel::ECC_WorldStatic,
    ECollisionResponse::ECR_Block
  );

  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: EndPlay - Out"));
}

void UZMQMovementComponent::DisableSpecialPhysics()
{
  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: DisableSpecialPhysics - In"));

  DisableZMQPhysics();

  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: DisableSpecialPhysics - Out"));
}

void UZMQMovementComponent::DisableZMQPhysics()
{
  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: DisableZMQPhysics - In"));

  // Send the last environment message
  this->status = false;
  this->get_environment(this->egovehicle);
  this->send_environment();

  // Close ZMQ communication
  this->close_zmq();

  // Remove the tick component
  this->SetComponentTickEnabled(false);

  // Enable UE4 physics
  EnableUE4VehiclePhysics(true);

  // Reset callbacks to react to collisions
  CarlaVehicle->OnActorHit.RemoveDynamic(
    this,
    &UZMQMovementComponent::OnVehicleHit
    );
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(
    this,
    &UZMQMovementComponent::OnVehicleOverlap
  );
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
    ECollisionChannel::ECC_WorldStatic,
    ECollisionResponse::ECR_Block
  );

  // Reset the movement component to the default one
  UDefaultMovementComponent::CreateDefaultMovementComponent(CarlaVehicle);

  // Disable sound
  CarlaVehicle->SetVolume(0.f);

  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: DisableZMQPhysics - Out"));
}

void UZMQMovementComponent::OnVehicleHit(
  AActor *Actor,
  AActor *OtherActor,
  FVector NormalImpulse,
  const FHitResult &Hit
)
{
  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: OnVehicleHit - In"));

  DisableZMQPhysics();

  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: OnVehicleHit - Out"));
}

void UZMQMovementComponent::OnVehicleOverlap(
  UPrimitiveComponent* OverlappedComponent,
  AActor* OtherActor,
  UPrimitiveComponent* OtherComp,
  int32 OtherBodyIndex,
  bool bFromSweep,
  const FHitResult & SweepResult
)
{
  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: OnVehicleOverlap - In"));

  if (OtherComp->GetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldDynamic) ==
      ECollisionResponse::ECR_Block)
  {
    DisableZMQPhysics();
  }

  UE_LOG(LogCarla, Log, TEXT("ZMQ Physics: OnVehicleOverlap - Out"));
}
