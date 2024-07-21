// Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

/*
* This file is a custom movement component which implements a synchronous
* communication between the Carla simulator and any external physics engine.
* The communication is done through ZMQ sockets.
*/

#pragma once

#include <vector>

#include "BaseCarlaMovementComponent.h"
#include "Carla/Vehicle/VehicleControl.h"

#include "compiler/disable-ue4-macros.h"
#include "compiler/enable-ue4-macros.h"

#include "zmq.h"
#include "interfaces/egovehicle_generated.h"
#include "interfaces/terrain_generated.h"
#include "sim_utils/zmq_recv_queue.hpp"

#define _USE_MATH_DEFINES // enable M_PI on windows
#include <math.h>

#include "ZMQMovementComponent.generated.h"

UCLASS(Blueprintable, meta=(BlueprintSpawnableComponent) )
class CARLA_API UZMQMovementComponent : public UBaseCarlaMovementComponent
{
  GENERATED_BODY()

  // ZMQ context
  void *context = nullptr;

  // ZMQ broker frontend
  void *frontend = nullptr;
  std::string front_endpoint = "";
  std::string topic = "";

  // ZMQ broker backend
  void *backend = nullptr;
  std::string back_endpoint = "";

  // ZMQ receive queue
  zmq_extensions::ZMQRecvQueue<stl_extensions::FIFO> recv_queue{1, 10};

  // ZMQ message
  std::shared_ptr<zmq_extensions::ZMQMessage> msg = nullptr;

  // FlatBuffers builder
  flatbuffers::FlatBufferBuilder builder{1024};

  // FlatBuffers ego-vehicle table
  DrivingSimulator::EgoVehicle::IEgoVehicle const *egovehicle = nullptr;

  // Running status
  bool status = false;

  // Cycle counter
  size_t cycle_count = 0;
  std::string cycle_count_str = "";

  // Timestamp
  std::string timestamp_str = "";

  // UE4 conversions
  const double CMTOM    = 0.01;
  const double MTOCM    = 100;
  const double DEGTORAD = M_PI / 180.0;
  const double RADTODEG = 180.0 / M_PI;

  // Vehicle states
  FVector location{0, 0, 0};
  FRotator orientation{0, 0, 0};
  int32 gear = 0;
  FVector velocity{0, 0, 0};
  std::array<float, 16> vehicle_matrix{};

  // Spectator
  APawn *spectator = nullptr;
  std::array<float, 16> spectator_matrix{};
  std::array<float, 16> spectator_resulting_matrix{};

public:

  static void CreateZMQMovementComponent(
    ACarlaWheeledVehicle* Vehicle,
    FString frontend_endpoint,
    FString backend_endpoint,
    bool attach_spectator,
    FTransform spectator_transform
  );

  virtual void BeginPlay() override;

  void ProcessControl(FVehicleControl &Control) override;

  void TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction
  ) override;

  virtual FVector GetVelocity() const override;

  virtual int32 GetVehicleCurrentGear() const override;

  virtual float GetVehicleForwardSpeed() const override;

  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

  virtual void DisableSpecialPhysics() override;

private:

  void get_terrain(DrivingSimulator::EgoVehicle::IEgoVehicle const *egovehicle);

  void send_terrain();

  // Transform a point given a transformation matrix
  // - P0 is the point to transform
  // - P1 is the transformed point
  void transform_point(double const transform[16], double const P0[4], double P1[4]);

  void get_vehicle_rhs_matrix(double matrix[16]);

  bool compute_contact_point(
    double const transform[16],
    FVector &start_location,
    FVector &end_location,
    double max_distance,
    FHitResult &hit,
    FCollisionQueryParams const &collision_query_params
  );

  flatbuffers::Offset<DrivingSimulator::Terrain::ContactPoint> create_contact_point(
    bool got_hit,
    FHitResult const &hit
  );

  void close_zmq();

  void DisableZMQPhysics();

  UFUNCTION()
  void OnVehicleHit(
    AActor *Actor,
    AActor *OtherActor,
    FVector NormalImpulse,
    const FHitResult &Hit
  );

  UFUNCTION()
  void OnVehicleOverlap(
    UPrimitiveComponent* OverlappedComponent,
    AActor* OtherActor,
    UPrimitiveComponent* OtherComp,
    int32 OtherBodyIndex,
    bool bFromSweep,
    const FHitResult & SweepResult
  );
};
