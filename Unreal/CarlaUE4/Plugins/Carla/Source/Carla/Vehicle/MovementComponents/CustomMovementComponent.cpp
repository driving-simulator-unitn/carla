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

// Conversions
constexpr double CMTOM = 0.01;
constexpr double MTOCM = 100;
constexpr double DEGTORAD = M_PI/180.0;
constexpr double RADTODEG = 180.0/M_PI;

void UCustomMovementComponent::CreateCustomMovementComponent(
    ACarlaWheeledVehicle* Vehicle,
    FString UDPip,
    int UDPport)
{

  UCustomMovementComponent* CustomMovementComponent = NewObject<UCustomMovementComponent>(Vehicle);

  // Save original velocity and location
  FVector original_velocity     = Vehicle->GetVelocity();
  FVector original_location     = Vehicle->GetActorLocation();
  CustomMovementComponent->original_orientation = Vehicle->GetActorRotation();

  // UE_LOG(LogCarla, Warning, TEXT("Original orientation: %f, %f, %f"), original_orientation.Roll, original_orientation.Pitch, original_orientation.Yaw);

  // Set initial condition
  CustomMovementComponent->X0 = {
    (original_velocity.X) * CMTOM,                                 // u
    (original_velocity.Y) * CMTOM,                                 // v
    0,                                                             // omega
    (original_location.X) * CMTOM,                                 // x
    (original_location.Y) * CMTOM,                                 // y
    (CustomMovementComponent->original_orientation.Yaw) * DEGTORAD // yaw
  };
  CustomMovementComponent->X1 = CustomMovementComponent->X0;

  Vehicle->SetCarlaMovementComponent(CustomMovementComponent);
  CustomMovementComponent->RegisterComponent();

  // Create UDP socket
  CustomMovementComponent->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (CustomMovementComponent->sockfd < 0)
  {
    UE_LOG(LogCarla, Error, TEXT("ERROR opening socket"));
    return;
  }

  // Set destination address and port
  std::memset(&(CustomMovementComponent->dest_addr), 0, sizeof(CustomMovementComponent->dest_addr));
  CustomMovementComponent->dest_addr.sin_family      = AF_INET;
  CustomMovementComponent->dest_addr.sin_port        = htons(UDPport);
  CustomMovementComponent->dest_addr.sin_addr.s_addr = inet_addr(std::string(TCHAR_TO_UTF8(*UDPip)).c_str()); 

  // Print initialization message
  std::string output = "Throttle,Steer,Brake,HandBrake,Reverse,ManualGearShift,Gear";
  int num_bytes = sendto(
    CustomMovementComponent->sockfd, 
    output.c_str(), 
    output.length(), 
    0, 
    (struct sockaddr *) &(CustomMovementComponent->dest_addr), 
    sizeof(CustomMovementComponent->dest_addr)
  );
  if (num_bytes < 0)
  {
    UE_LOG(LogCarla, Error, TEXT("ERROR sending data"));
    return;
  }
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

void UCustomMovementComponent::ProcessControl(FVehicleControl &Control)
{
  // Retrive vehicle controls
  float throttle         = Control.Throttle;
  float steer            = Control.Steer;
  float brake            = Control.Brake;
  bool hand_brake        = Control.bHandBrake;
  bool reverse           = Control.bReverse;
  bool manual_gear_shift = Control.bManualGearShift;
  int32 gear             = Control.Gear;
  
  // Send data over UDP
  std::string output = std::to_string(throttle)          + "," + 
                       std::to_string(steer)             + "," + 
                       std::to_string(brake)             + "," + 
                       std::to_string(hand_brake)        + "," + 
                       std::to_string(reverse)           + "," + 
                       std::to_string(manual_gear_shift) + "," + 
                       std::to_string(gear);

  int num_bytes = sendto(
    sockfd, 
    output.c_str(), 
    output.length(), 
    0, 
    (struct sockaddr *) &dest_addr, 
    sizeof(dest_addr)
  );
  if (num_bytes < 0)
  {
    UE_LOG(LogCarla, Error, TEXT("ERROR sending data"));
    return;
  }
}

void UCustomMovementComponent::TickComponent(float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction* ThisTickFunction)
{
  // Send data over UDP
  VehicleControl = CarlaVehicle->GetVehicleControl();
  ProcessControl(VehicleControl);

  // Create control vector
  double throttle       = VehicleControl.Throttle;
  double brake          = VehicleControl.Brake;
  double steer          = VehicleControl.Steer;
  double Sr             = model.get_parameters().M * 9.81 * (throttle - brake);
  std::vector<double> U = {Sr, steer / 4};

  // Do a step in SingleTrackModel
  UE_LOG(LogCarla, Warning, TEXT("DeltaTime: %f"), DeltaTime);
  model.step(X0, U, DeltaTime, X1);

  // Update vehicle location and rotation
  CarlaVehicle->SetActorLocation(FVector(X1[3]*MTOCM, X1[4]*MTOCM, 0));
  CarlaVehicle->SetActorRotation(FRotator(original_orientation.Pitch, X1[5]*RADTODEG, original_orientation.Roll));
  
  // Update state
  X0 = X1;
}

FVector UCustomMovementComponent::GetVelocity() const
{
  if (CarlaVehicle){
    return FVector(X0[0]*MTOCM, X0[1]*MTOCM, 0);
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
