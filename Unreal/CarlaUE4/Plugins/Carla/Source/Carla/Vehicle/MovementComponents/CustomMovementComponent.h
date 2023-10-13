// Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <vector>

#include "BaseCarlaMovementComponent.h"
#include "Carla/Vehicle/VehicleControl.h"

#include "compiler/disable-ue4-macros.h"
#include "compiler/enable-ue4-macros.h"

// Includes for UDP communication
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

// Includes for SingleTrackModel
#include "SingleTrackModel.h"

#include "CustomMovementComponent.generated.h"

UCLASS(Blueprintable, meta=(BlueprintSpawnableComponent) )
class CARLA_API UCustomMovementComponent : public UBaseCarlaMovementComponent
{
  GENERATED_BODY()

  FVehicleControl VehicleControl;

  // UDP socket
  struct sockaddr_in dest_addr;
  int sockfd;

  // SingleTrackModel states
  std::vector<double> X0;
  std::vector<double> X1;
  FRotator original_orientation;

  // SingleTrackModel
  SingleTrackModel model;

public:

  static void CreateCustomMovementComponent(
      ACarlaWheeledVehicle* Vehicle,
      FString UDPip,
      int UDPPort);

  virtual void BeginPlay() override;

  void InitializeCustomVehicle();

  void ProcessControl(FVehicleControl &Control) override;

  void TickComponent(float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction* ThisTickFunction) override;

  virtual FVector GetVelocity() const override;

  virtual int32 GetVehicleCurrentGear() const override;

  virtual float GetVehicleForwardSpeed() const override;

  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:

  void DisableCustomPhysics();

  UFUNCTION()
  void OnVehicleHit(AActor *Actor,
      AActor *OtherActor,
      FVector NormalImpulse,
      const FHitResult &Hit);

  // On car mesh overlap, only works when carsim is enabled
  // (this event triggers when overlapping with static environment)
  UFUNCTION()
  void OnVehicleOverlap(UPrimitiveComponent* OverlappedComponent,
      AActor* OtherActor,
      UPrimitiveComponent* OtherComp,
      int32 OtherBodyIndex,
      bool bFromSweep,
      const FHitResult & SweepResult);
};
