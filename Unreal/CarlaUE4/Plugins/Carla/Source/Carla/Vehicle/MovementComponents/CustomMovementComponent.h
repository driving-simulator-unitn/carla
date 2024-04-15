// ██████╗ ███████╗ ██████╗ ██╗███╗   ██╗
// ██╔══██╗██╔════╝██╔════╝ ██║████╗  ██║
// ██████╔╝█████╗  ██║  ███╗██║██╔██╗ ██║
// ██╔══██╗██╔══╝  ██║   ██║██║██║╚██╗██║
// ██████╔╝███████╗╚██████╔╝██║██║ ╚████║
// ╚═════╝ ╚══════╝ ╚═════╝ ╚═╝╚═╝  ╚═══╝
// #UNITN_MODIFICATIONS

/*

This class handles the usage of internal vehicle models for the simulation. If
you want to add a new vehicle model, simply create one inheriting from the
`VehicleModelInterface.h` and add it to the map of usable vehicle models.

*/

#pragma once

#include "BaseCarlaMovementComponent.h"
#include "Carla/Vehicle/VehicleControl.h"

#include "compiler/disable-ue4-macros.h"
#include "compiler/enable-ue4-macros.h"

#include <vector>
#include <map>
#include <string>

// Include the various vehicle models
#include "VehicleModelInterface.h"
#include "SingleTrackModel.h"

#define _USE_MATH_DEFINES // enable M_PI on windows
#include <math.h>

#include "CustomMovementComponent.generated.h"

UCLASS(Blueprintable, meta = (BlueprintSpawnableComponent))
class CARLA_API UCustomMovementComponent : public UBaseCarlaMovementComponent
{
  GENERATED_BODY()

private:
  // Vehicle controls
  FVehicleControl VehicleControl;

  // Vehicle pose
  FVector location;
  FRotator orientation;

  // Models map
  std::map<std::string, VehicleModelInterface *> VehicleModels;

  // Chosen model
  std::string ChosentModel = "SingleTrackModel";

public:
  static void CreateCustomMovementComponent(ACarlaWheeledVehicle *Vehicle);

  virtual void BeginPlay() override;

  void InitializeCustomVehicle();

  void ProcessControl(FVehicleControl &Control) override;

  void TickComponent(float DeltaTime,
                     ELevelTick TickType,
                     FActorComponentTickFunction *ThisTickFunction) override;

  virtual FVector GetVelocity() const override;

  virtual int32 GetVehicleCurrentGear() const override;

  virtual float GetVehicleForwardSpeed() const override;

  std::pair<bool, FHitResult> GetTerrainProperties(const FVector &Location) const;

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
  void OnVehicleOverlap(UPrimitiveComponent *OverlappedComponent,
                        AActor *OtherActor,
                        UPrimitiveComponent *OtherComp,
                        int32 OtherBodyIndex,
                        bool bFromSweep,
                        const FHitResult &SweepResult);
};

// ███████╗███╗   ██╗██████╗
// ██╔════╝████╗  ██║██╔══██╗
// █████╗  ██╔██╗ ██║██║  ██║
// ██╔══╝  ██║╚██╗██║██║  ██║
// ███████╗██║ ╚████║██████╔╝
// ╚══════╝╚═╝  ╚═══╝╚═════╝