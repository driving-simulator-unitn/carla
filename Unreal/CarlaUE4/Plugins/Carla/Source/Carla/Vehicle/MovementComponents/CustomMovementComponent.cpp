// ██████╗ ███████╗ ██████╗ ██╗███╗   ██╗
// ██╔══██╗██╔════╝██╔════╝ ██║████╗  ██║
// ██████╔╝█████╗  ██║  ███╗██║██╔██╗ ██║
// ██╔══██╗██╔══╝  ██║   ██║██║██║╚██╗██║
// ██████╔╝███████╗╚██████╔╝██║██║ ╚████║
// ╚═════╝ ╚══════╝ ╚═════╝ ╚═╝╚═╝  ╚═══╝
// #UNITN_MODIFICATIONS

#include "CustomMovementComponent.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/MovementComponents/DefaultMovementComponent.h"

#include "compiler/disable-ue4-macros.h"
#include <carla/rpc/String.h>

#include "compiler/enable-ue4-macros.h"
#include "Carla/Util/RayTracer.h"

void UCustomMovementComponent::CreateCustomMovementComponent(ACarlaWheeledVehicle *Vehicle)
{

  UCustomMovementComponent *CustomMovementComponent = NewObject<UCustomMovementComponent>(Vehicle);

  // Get the initial vehicle pose
  CustomMovementComponent->location = Vehicle->GetActorLocation();
  CustomMovementComponent->orientation = Vehicle->GetActorRotation();

  // Set the vehicle movement component
  Vehicle->SetCarlaMovementComponent(CustomMovementComponent);
  CustomMovementComponent->RegisterComponent();
}

void UCustomMovementComponent::BeginPlay()
{
  Super::BeginPlay();

  DisableUE4VehiclePhysics();

  InitializeCustomVehicle();

  CarlaVehicle->OnActorHit.AddDynamic(
      this, &UCustomMovementComponent::OnVehicleHit);
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.AddDynamic(
      this, &UCustomMovementComponent::OnVehicleOverlap);
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Overlap);
}

void UCustomMovementComponent::InitializeCustomVehicle()
{
  // Create the custom vehicle
  model = SingleTrackModel();
}

void UCustomMovementComponent::ProcessControl(FVehicleControl &Control) {}

void UCustomMovementComponent::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction *ThisTickFunction)
{
  // Get vehicle control
  VehicleControl = CarlaVehicle->GetVehicleControl();

  // Create control vector
  double throttle = VehicleControl.Throttle;
  double brake = VehicleControl.Brake;
  double steer = VehicleControl.Steer;
  double Sr = model.get_parameters().M * 9.81 * (throttle - brake);

  // Handle reverse mode and stop
  double eps = 1e-6;
  if (!VehicleControl.bReverse)
  {
    // Avoid going backwards
    if (Sr < 0 && CarlaVehicle->GetVelocity().X * CMTOM < eps)
    {
      Sr = 0;
    }
  }
  else
  {
    // Avoid going forward
    Sr = -Sr;
    if (Sr > 0 && CarlaVehicle->GetVelocity().X * CMTOM > -eps)
    {
      Sr = 0;
    }
  }

  std::vector<double> U = {Sr, steer * model.get_parameters().tau_H};

  // Do a step in SingleTrackModel
  double DeltaTimeDouble = (double)DeltaTime;
  double DeltaTimeRemainder = DeltaTimeDouble - floor(DeltaTimeDouble / model.get_dt()) * model.get_dt();
  for (double t = 0;
       t < DeltaTimeDouble - DeltaTimeRemainder;
       t += model.get_dt())
  {
    model.step(X0, U, model.get_dt(), X1);
    X0 = X1;
  }
  model.step(X0, U, DeltaTimeRemainder, X1);

  // Get the terrain properties and log them
  std::pair<bool, FHitResult> TerrainProperties = GetTerrainProperties(CarlaVehicle->GetActorLocation());
  if (TerrainProperties.first)
  {
    // Log the location
    UE_LOG(LogCarla, Log, TEXT("Location: %f, %f, %f"),
           TerrainProperties.second.Location.X,
           TerrainProperties.second.Location.Y,
           TerrainProperties.second.Location.Z);

    // Log the prenetration depth
    UE_LOG(LogCarla, Log, TEXT("Penetration depth: %f"), TerrainProperties.second.PenetrationDepth);

    // Log the normal
    UE_LOG(LogCarla, Log, TEXT("Normal: %f, %f, %f"),
           TerrainProperties.second.Normal.X,
           TerrainProperties.second.Normal.Y,
           TerrainProperties.second.Normal.Z);
  }

  // Update state
  X0 = X1;

  // Update vehicle location
  CarlaVehicle->SetActorLocation(FVector(X1[3] * MTOCM, X1[4] * MTOCM, 0));

  // Update vehicle rotation
  CarlaVehicle->SetActorRotation(FRotator(original_orientation.Pitch, X1[5] * RADTODEG, original_orientation.Roll));
}

FVector UCustomMovementComponent::GetVelocity() const
{
  if (CarlaVehicle)
  {
    return FVector(X0[0] * MTOCM, X0[1] * MTOCM, 0);
  }
  return FVector(0, 0, 0);
}

int32 UCustomMovementComponent::GetVehicleCurrentGear() const
{
  return 0;
}

float UCustomMovementComponent::GetVehicleForwardSpeed() const
{
  return 0.f;
}

std::pair<bool, FHitResult> UCustomMovementComponent::GetTerrainProperties(
    const FVector &Location) const
{
  // Maximum distance to search for terrain properties
  const double MaxDistance = 1000000;

  // Raycast downwards
  FVector StartLocation = Location;
  FVector EndLocation = Location + FVector(0, 0, -1) * MaxDistance; // search downwards

  // Prepare hit result
  FHitResult Hit;
  FCollisionQueryParams CollisionQueryParams;

  // Ignore CarlaVehicle
  CollisionQueryParams.AddIgnoredActor(CarlaVehicle);

  // Raycast
  bool bDidHit = CarlaVehicle->GetWorld()->LineTraceSingleByChannel(
      Hit,
      StartLocation,
      EndLocation,
      ECC_GameTraceChannel2, // camera (any collision)
      CollisionQueryParams,
      FCollisionResponseParams());

  // Return hit result
  return std::make_pair(bDidHit, Hit);
}

void UCustomMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  if (!CarlaVehicle)
  {
    return;
  }
  // reset callbacks to react to collisions
  CarlaVehicle->OnActorHit.RemoveDynamic(
      this, &UCustomMovementComponent::OnVehicleHit);
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(
      this, &UCustomMovementComponent::OnVehicleOverlap);
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
}

void UCustomMovementComponent::DisableCustomPhysics()
{
  this->SetComponentTickEnabled(false);
  EnableUE4VehiclePhysics(true);
  CarlaVehicle->OnActorHit.RemoveDynamic(this, &UCustomMovementComponent::OnVehicleHit);
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.RemoveDynamic(
      this, &UCustomMovementComponent::OnVehicleOverlap);
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
  UDefaultMovementComponent::CreateDefaultMovementComponent(CarlaVehicle);
  carla::log_warning("Custom physics does not support collisions yet, reverting to default PhysX physics.");
}

void UCustomMovementComponent::OnVehicleHit(AActor *Actor,
                                            AActor *OtherActor,
                                            FVector NormalImpulse,
                                            const FHitResult &Hit)
{
  DisableCustomPhysics();
}

// On car mesh overlap, only works when carsim is enabled
// (this event triggers when overlapping with static environment)

void UCustomMovementComponent::OnVehicleOverlap(
    UPrimitiveComponent *OverlappedComponent,
    AActor *OtherActor,
    UPrimitiveComponent *OtherComp,
    int32 OtherBodyIndex,
    bool bFromSweep,
    const FHitResult &SweepResult)
{
  if (OtherComp->GetCollisionResponseToChannel(
          ECollisionChannel::ECC_WorldDynamic) ==
      ECollisionResponse::ECR_Block)
  {
    DisableCustomPhysics();
  }
}

// ███████╗███╗   ██╗██████╗
// ██╔════╝████╗  ██║██╔══██╗
// █████╗  ██╔██╗ ██║██║  ██║
// ██╔══╝  ██║╚██╗██║██║  ██║
// ███████╗██║ ╚████║██████╔╝
// ╚══════╝╚═╝  ╚═══╝╚═════╝