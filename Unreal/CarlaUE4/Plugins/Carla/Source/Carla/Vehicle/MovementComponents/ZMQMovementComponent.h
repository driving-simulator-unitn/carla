// Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

/*
* This file is a custom movement component which implements an asynchronous
* communication between the Carla simulator and any external physics engine.
* The communication is done through ZMQ sockets. The pattern used is
* DEALER-ROUTER, the asynchronous version of REQ-REP.
*/

#pragma once

#include <vector>

#include "BaseCarlaMovementComponent.h"
#include "Carla/Vehicle/VehicleControl.h"

#include "compiler/disable-ue4-macros.h"
#include "compiler/enable-ue4-macros.h"

// Include zmqpp
#define ZMQ_BUILD_DRAFT_API 1
#include "zmq.h"

#include "ZMQMovementComponent.generated.h"

UCLASS(Blueprintable, meta=(BlueprintSpawnableComponent) )
class CARLA_API UZMQMovementComponent : public UBaseCarlaMovementComponent
{
  GENERATED_BODY()

  // ZMQ context
  void *context;

  // ZMQ synchronization socket to synchronize the physics engine
  void *sync_socket;
  std::string sync_endpoint;

  // ZMQ push socket to send data to the physics engine
  void *push_socket;
  std::string push_endpoint;

  // ZMQ pull socket to receive data from the physics engine
  void *pull_socket;
  std::string pull_endpoint;

  // UE4 conversions
  const double CMTOM    = 0.01;
  const double MTOCM    = 100;
  const double DEGTORAD = M_PI/180.0;
  const double RADTODEG = 180.0/M_PI;

  // Position and orientation of the vehicle
  FVector location;
  FRotator orientation;

public:

  static void CreateZMQMovementComponent(
    ACarlaWheeledVehicle* Vehicle,
    FString sync_endpoint,
    FString push_endpoint,
    FString pull_endpoint
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

private:

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
