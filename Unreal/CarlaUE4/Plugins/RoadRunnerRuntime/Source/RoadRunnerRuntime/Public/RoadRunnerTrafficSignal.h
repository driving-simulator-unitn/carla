// Copyright 2020 The MathWorks, Inc.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "Components/ActorComponent.h"
#include "Components/MeshComponent.h"
#include "RoadRunnerTrafficJunction.h"
#include "RoadRunnerTrafficSignal.generated.h"

class URoadRunnerTrafficController;
class USCS_Node;

////////////////////////////////////////////////////////////////////////////////
// Handles single signal to be controlled separately
UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class ROADRUNNERRUNTIME_API URoadRunnerTrafficSignal : public USceneComponent
{
	GENERATED_BODY()

public:

	// Holds list of Signal states
	UPROPERTY(EditAnywhere, Category = "Traffic")
	TArray<FSignalState> SignalStates;

	// Sets default values for this component's properties
	URoadRunnerTrafficSignal();

	UPROPERTY(EditAnywhere, Category = "Traffic")
	FString Id;

	UPROPERTY(EditAnywhere, Category = "Traffic")
	FString AssetId;

	UPROPERTY()
	float Timer = 0.0f;

	UPROPERTY()
	int Config = 0;

	UPROPERTY()
	bool Blinking = false;

	UPROPERTY()
	bool IsLegacy = false;

#if WITH_EDITOR
	void BlueprintInit(FString signalId, URoadRunnerTrafficController* controller, FString assetId, const TMap<FString, USCS_Node*>& blueprintMap);
	void Init(FString signalId, URoadRunnerTrafficController* controller, FString assetId);
	void CreateInstance(const FLightBulbState& lightBulbState, FComponentReference& ref, USceneComponent* descendent, const FString& suffix);
	void BPCreateInstance(const FLightBulbState& lightBulbState, FComponentReference& ref, FString& ComponentName, const FSignalState& signalState, const TMap<FString, USCS_Node*>& uuidToComponentMap, const FString& suffix);
#endif
	void AddSignalState(const FSignalState& signalState);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void SetConfiguration(int index);

	bool ErrorFlag;
};
