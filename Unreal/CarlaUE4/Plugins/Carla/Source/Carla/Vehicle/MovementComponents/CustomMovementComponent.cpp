// Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
// Copyright (c) 2019 Intel Corporation
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CustomMovementComponent.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"
#include "Carla/Vehicle/MovementComponents/DefaultMovementComponent.h"

#include "compiler/disable-ue4-macros.h"
#include <carla/rpc/String.h>

#include "compiler/enable-ue4-macros.h"
#include "Carla/Util/RayTracer.h"

// Includes for UDP communication
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

// Includes for SingleTrackModel
#include "SingleTrackModel.h"

// UDP socket
struct sockaddr_in dest_addr;
int sockfd;

// Initial condition
static std::vector<double> X0;
static std::vector<double> X1;
SingleTrackModel model;

void UCustomMovementComponent::CreateCustomMovementComponent(
    ACarlaWheeledVehicle* Vehicle,
    uint64_t MaxSubsteps,
    float MaxSubstepDeltaTime,
    FString UDPip,
    int UDPport)
{

  UCustomMovementComponent* CustomMovementComponent = NewObject<UCustomMovementComponent>(Vehicle);

  // Save original velocity and location
  FVector original_velocity = Vehicle->GetVelocity();
  FVector original_location = Vehicle->GetActorLocation();
  // Set initial condition
  X0 = {(original_velocity.X)*0.01 , (original_velocity.Y)*0.01 , 0, (original_location.X)*0.01, (original_location.Y)*0.01 , 0};
  X1 = X0;

  CustomMovementComponent->MaxSubsteps = MaxSubsteps;
  CustomMovementComponent->MaxSubstepDeltaTime = MaxSubstepDeltaTime;
  Vehicle->SetCarlaMovementComponent(CustomMovementComponent);
  CustomMovementComponent->RegisterComponent();

  // Create UDP socket
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
  {
    UE_LOG(LogCarla, Error, TEXT("ERROR opening socket"));
    return;
  }

  // Set destination address and port
  std::memset(&dest_addr, 0, sizeof(dest_addr));
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(UDPport);
  dest_addr.sin_addr.s_addr = inet_addr(std::string(TCHAR_TO_UTF8(*UDPip)).c_str()); 

  // Print initialization message
  std::string output = "Throttle,Steer,Brake,HandBrake,Reverse,ManualGearShift,Gear";

  int num_bytes = sendto(sockfd, output.c_str(), output.length(), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (num_bytes < 0)
  {
    UE_LOG(LogCarla, Error, TEXT("ERROR sending data"));
    return;
  }
}

constexpr double CMTOM = 0.01;
std::vector<double> UE4LocationToCustom(const FVector& Location)
{
  std::vector<double> CustomLocation = {Location.X, -Location.Y, Location.Z};
  for (double& value : CustomLocation)
  {
    value *= CMTOM;
  }
  return CustomLocation;
}

constexpr double MTOCM = 100;
FVector CustomToUE4Location(const std::vector<double>& Location)
{
  return MTOCM * FVector(Location[0], -Location[1], Location[2]);
}

std::vector<double> UE4DirectionToCustom(const FVector& Location)
{
  std::vector<double> CustomDirection = {Location.X, -Location.Y, Location.Z};
  return CustomDirection;

}
FVector CustomToUE4Direction(const std::vector<double>& Location)
{
  return FVector(Location[0], -Location[1], Location[2]);
}

std::vector<double> UE4QuatToCustom(const FQuat& Quat)
{
  std::vector<double> CustomQuat = {Quat.W, -Quat.X, Quat.Y, -Quat.Z};
  return CustomQuat;
}

FQuat CustomToUE4Quat(const std::vector<double>& quat)
{

  return FQuat(-quat[0], quat[1], -quat[2], quat[3]);
}


void UCustomMovementComponent::BeginPlay()
{
  Super::BeginPlay();

  DisableUE4VehiclePhysics();

  InitializeCustomVehicle();


  // TODO: check

  CarlaVehicle->OnActorHit.AddDynamic(
      this, &UCustomMovementComponent::OnVehicleHit);
  CarlaVehicle->GetMesh()->OnComponentBeginOverlap.AddDynamic(
      this, &UCustomMovementComponent::OnVehicleOverlap);
  CarlaVehicle->GetMesh()->SetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Overlap);
}

void UCustomMovementComponent::InitializeCustomVehicle()
{
  // Initial location with small offset to prevent falling through the ground
  FVector VehicleLocation = CarlaVehicle->GetActorLocation() + FVector(0,0,25);
  FQuat VehicleRotation = CarlaVehicle->GetActorRotation().Quaternion();
  std::vector<double> CustomLocation = UE4LocationToCustom(VehicleLocation);
  std::vector<double> CustomRotation = UE4QuatToCustom(VehicleRotation);

  // Create the custom vehicle
  model = SingleTrackModel();
}

void UCustomMovementComponent::ProcessControl(FVehicleControl &Control)
{
 
}

void UCustomMovementComponent::TickComponent(float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction* ThisTickFunction)
{

  // Retrive vehicle controls
  FVehicleControl last_controls = CarlaVehicle->GetVehicleControl();
  float throttle = last_controls.Throttle;
  float steer = last_controls.Steer;
  float brake = last_controls.Brake;
  bool hand_brake = last_controls.bHandBrake;
  bool reverse = last_controls.bReverse;
  bool manual_gear_shift = last_controls.bManualGearShift;
  int32 gear = last_controls.Gear;
  
  // Send data over UDP
  std::string output = std::to_string(throttle) + "," + std::to_string(steer) + "," + std::to_string(brake) + "," + std::to_string(hand_brake) + "," + std::to_string(reverse) + "," + std::to_string(manual_gear_shift) + "," + std::to_string(gear);

  int num_bytes = sendto(sockfd, output.c_str(), output.length(), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (num_bytes < 0)
  {
    UE_LOG(LogCarla, Error, TEXT("ERROR sending data"));
    return;
  }
  
  // Do a step in SingleTrackModel
  std::vector<double> U0 = {(throttle-brake)*100, steer};
  model.step(X0, U0, DeltaTime, X1);

  // Update vehicle location and rotation
  CarlaVehicle->SetActorLocation(FVector(X1[3]*100, X1[4]*100, 25));
  
  // Update state
  X0 = X1;
}

FVector UCustomMovementComponent::GetVelocity() const
{
  if (CarlaVehicle){
      return FVector(X0[0]*100, X0[1]*100, 0);
  }
  return FVector(0,0,0);
}

int32 UCustomMovementComponent::GetVehicleCurrentGear() const
{
  return 0;
}

float UCustomMovementComponent::GetVehicleForwardSpeed() const
{
  return 0.f;
}

void UCustomMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  if(!CarlaVehicle)
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
  carla::log_warning("Chrono physics does not support collisions yet, reverting to default PhysX physics.");

  // Close the socket
  close(sockfd);
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
    UPrimitiveComponent* OverlappedComponent,
    AActor* OtherActor,
    UPrimitiveComponent* OtherComp,
    int32 OtherBodyIndex,
    bool bFromSweep,
    const FHitResult & SweepResult)
{
  if (OtherComp->GetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldDynamic) ==
      ECollisionResponse::ECR_Block)
  {
    DisableCustomPhysics();
  }
}
