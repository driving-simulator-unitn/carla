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
#include "compiler/enable-ue4-macros.h"
#include "Carla/Util/RayTracer.h"

void UZMQMovementComponent::CreateZMQMovementComponent(
  ACarlaWheeledVehicle* Vehicle,
  FString Endpoint
)
{
  // Create the movement component
  UZMQMovementComponent* ZMQMovementComponent = NewObject<UZMQMovementComponent>(Vehicle);

  // Save original location and orientation
  ZMQMovementComponent->location    = Vehicle->GetActorLocation();
  ZMQMovementComponent->orientation = Vehicle->GetActorRotation();

  // Convert the location and orientation to m and rad
  ZMQMovementComponent->location.X        *= ZMQMovementComponent->CMTOM;
  ZMQMovementComponent->location.Y        *= ZMQMovementComponent->CMTOM;
  ZMQMovementComponent->location.Z        *= ZMQMovementComponent->CMTOM;
  ZMQMovementComponent->orientation.Pitch *= ZMQMovementComponent->DEGTORAD;
  ZMQMovementComponent->orientation.Yaw   *= ZMQMovementComponent->DEGTORAD;
  ZMQMovementComponent->orientation.Roll  *= ZMQMovementComponent->DEGTORAD;

  std::string tmp_push = "tcp://*:5557";
  std::string tmp_pull = "tcp://localhost:5558";

  // Initialize the ZMQ context
  ZMQMovementComponent->context = zmq_ctx_new();

  // Initialize the ZMQ push socket
  ZMQMovementComponent->push_socket   = zmq_socket(ZMQMovementComponent->context, ZMQ_PUSH);
  ZMQMovementComponent->push_endpoint = tmp_push;

  // Initialize the ZMQ pull socket
  ZMQMovementComponent->pull_socket   = zmq_socket(ZMQMovementComponent->context, ZMQ_PULL);
  ZMQMovementComponent->pull_endpoint = tmp_pull;

  // Set both sockets to keep only the last message in the queue
  int conflate = 1;
  zmq_setsockopt(ZMQMovementComponent->push_socket, ZMQ_CONFLATE, &conflate, sizeof(conflate));
  zmq_setsockopt(ZMQMovementComponent->pull_socket, ZMQ_CONFLATE, &conflate, sizeof(conflate));

  // Assign the movement component to the vehicle
  Vehicle->SetCarlaMovementComponent(ZMQMovementComponent);
  ZMQMovementComponent->RegisterComponent();
}

void UZMQMovementComponent::BeginPlay()
{
  // Call the base class
  Super::BeginPlay();

  // Disable UE4 physics
  DisableUE4VehiclePhysics();

  // Bind the push socket to the push endpoint
  int rc = zmq_bind(push_socket, push_endpoint.c_str());
  if (rc != 0) {
    carla::log_error("ZMQ push socket could not be bound to the push endpoint.");
  }

  // Connect the pull socket to the pull endpoint
  rc = zmq_connect(pull_socket, pull_endpoint.c_str());
  if (rc != 0) {
    carla::log_error("ZMQ pull socket could not be connected to the pull endpoint.");
  }

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
}

void UZMQMovementComponent::ProcessControl(FVehicleControl &Control) {}

// On each tick we do the following:
// 1. Send the current vehicle state to the physics engine
// 2. Receive the next vehicle state from the physics engine
// 3. Update the vehicle state
void UZMQMovementComponent::TickComponent(
  float DeltaTime,
  ELevelTick TickType,
  FActorComponentTickFunction* ThisTickFunction
)
{
  // Send the current vehicle state to the physics engine
  std::string message = std::to_string(location.X)        + "," +
                        std::to_string(location.Y)        + "," +
                        std::to_string(location.Z)        + "," +
                        std::to_string(orientation.Pitch) + "," +
                        std::to_string(orientation.Yaw)   + "," +
                        std::to_string(orientation.Roll);
  zmq_send(push_socket, message.c_str(), message.size(), ZMQ_DONTWAIT);

  // Receive the next vehicle state from the physics engine
  char buffer[1024];
  int size = zmq_recv(pull_socket, buffer, sizeof(buffer), ZMQ_DONTWAIT);

  if (size != -1) {
    // Add a null terminator to the end of the message (mandatory!)
    buffer[size] = '\0';

    // Unpack the message
    std::string response(buffer);
    std::string delimiter = ",";
    size_t pos = 0;
    std::string token;
    std::vector<std::string> tokens;

    while ((pos = response.find(delimiter)) != std::string::npos) {
      token = response.substr(0, pos);
      tokens.push_back(token);
      response.erase(0, pos + delimiter.length());
    }
    // Add the last token to the vector
    tokens.push_back(response);

    // Update the vehicle location and orientation
    location.X        = std::stod(tokens[0]);
    location.Y        = std::stod(tokens[1]);
    location.Z        = std::stod(tokens[2]);
    orientation.Pitch = std::stod(tokens[3]);
    orientation.Yaw   = std::stod(tokens[4]);
    orientation.Roll  = std::stod(tokens[5]);

    // Reset the buffer
    memset(buffer, 0, sizeof(buffer));
  }

  // Update the vehicle state
  FVector new_location = FVector(
    location.X * MTOCM,
    location.Y * MTOCM,
    location.Z * MTOCM
  );
  FRotator new_orientation = FRotator(
    orientation.Pitch * RADTODEG,
    orientation.Yaw   * RADTODEG,
    orientation.Roll  * RADTODEG
  );

  CarlaVehicle->SetActorLocation(new_location);
  CarlaVehicle->SetActorRotation(new_orientation);
}

FVector UZMQMovementComponent::GetVelocity() const
{
  if (CarlaVehicle){
    return FVector(0,0,0);
  }
  return FVector(0,0,0);
}

int32 UZMQMovementComponent::GetVehicleCurrentGear() const
{
  return 0;
}

float UZMQMovementComponent::GetVehicleForwardSpeed() const
{
  return 0.f;
}

void UZMQMovementComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  if(!CarlaVehicle)
  {
    return;
  }

  // Free the ZMQ sockets and context
  zmq_close(push_socket);
  zmq_close(pull_socket);
  zmq_ctx_destroy(context);

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
}


void UZMQMovementComponent::DisableZMQPhysics()
{
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

  // Warn the user that collisions are not supported yet
  carla::log_warning("ZMQ physics does not support collisions yet, reverting to default PhysX physics.");
}

void UZMQMovementComponent::OnVehicleHit(
  AActor *Actor,
  AActor *OtherActor,
  FVector NormalImpulse,
  const FHitResult &Hit
)
{
  DisableZMQPhysics();
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
  if (OtherComp->GetCollisionResponseToChannel(
      ECollisionChannel::ECC_WorldDynamic) ==
      ECollisionResponse::ECR_Block)
  {
    DisableZMQPhysics();
  }
}
