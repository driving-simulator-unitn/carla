// Copyright 2020 The MathWorks, Inc.

#include "RoadRunnerTrafficSignal.h"
#include "RoadRunnerTrafficController.h"
#include "RoadRunnerTrafficUtil.h"
#include "Components/MeshComponent.h"
#include "RoadRunnerRuntimeLog.h"

////////////////////////////////////////////////////////////////////////////////

URoadRunnerTrafficSignal::URoadRunnerTrafficSignal()
{
	PrimaryComponentTick.bCanEverTick = true;
}

////////////////////////////////////////////////////////////////////////////////

#if WITH_EDITOR
void URoadRunnerTrafficSignal::BlueprintInit(FString signalId, URoadRunnerTrafficController* controller, FString assetId, const TMap<FString, USCS_Node*>& uuidToComponentMap)
{
	//blueprint init
	Id = signalId;
	AssetId = assetId;

	const auto& signalAsset = controller->GetSignalAsset(AssetId);
	// Convert signal asset to signal state
	int index = 0;

	for (const auto& config : signalAsset.SignalConfigurations)
	{
		FSignalState signalState;
		signalState.Id = Id;
		signalState.SignalAssetId = AssetId;
		signalState.Configuration = index;

		// Loop over light bulb states and set up the light instance states
		for (const auto& lightBulbState : config.LightBulbStates)
		{
			if (!uuidToComponentMap.Contains(signalState.Id))
			{
				UE_LOG(RoadRunnerTraffic, Warning, TEXT("Signal %s not found inside this blueprint."), *signalState.Id);
				continue;
			}

			FLightInstanceState lightInstanceState;
			lightInstanceState.State = lightBulbState.State;

			if (!IsLegacy) {
				BPCreateInstance(lightBulbState, lightInstanceState.OnRef, lightInstanceState.OnComponentName, signalState, uuidToComponentMap, "_on");
				BPCreateInstance(lightBulbState, lightInstanceState.OffRef, lightInstanceState.OffComponentName, signalState, uuidToComponentMap, "_off");
			}
			else {
				BPCreateInstance(lightBulbState, lightInstanceState.LegacyRef, lightInstanceState.LegacyComponentName, signalState, uuidToComponentMap, "");
			}
			signalState.LightInstanceStates.Add(lightInstanceState);

		}
		AddSignalState(signalState);
		index++;
	}
}

////////////////////////////////////////////////////////////////////////////////

void URoadRunnerTrafficSignal::Init(FString signalId, URoadRunnerTrafficController* controller, FString assetId)
{
	Id = signalId;
	AssetId = assetId;

	const auto& signalAsset = controller->GetSignalAsset(AssetId);
	// Convert signal asset to signal state
	int index = 0;

	for (const auto& config : signalAsset.SignalConfigurations)
	{
		FSignalState signalState;
		signalState.Id = Id;
		signalState.SignalAssetId = AssetId;
		signalState.Configuration = index;

		// Loop over light bulb states and set up the light instance states
		for (const auto& lightBulbState : config.LightBulbStates)
		{
			// Search here since we need to find the actor
			auto rootComp = GetOwner()->GetRootComponent();
			TArray< USceneComponent* > descendents;
			rootComp->GetChildrenComponents(true, descendents);

			FLightInstanceState lightInstanceState;
			lightInstanceState.State = lightBulbState.State;

			for (const auto& descendent : descendents)
			{
				FString actorName = descendent->GetOwner()->GetActorLabel();
				
				if (!IsLegacy) {
					CreateInstance(lightBulbState, lightInstanceState.OnRef, descendent, "_on");
					CreateInstance(lightBulbState, lightInstanceState.OffRef, descendent, "_off");
				}
				else {
					CreateInstance(lightBulbState, lightInstanceState.LegacyRef, descendent, "");			
				}

				auto meshComp = Cast<UStaticMeshComponent>(descendent);

				if (!meshComp)
					continue; 

				// Check static mesh's owner name
			
			}
			signalState.LightInstanceStates.Add(lightInstanceState);

		}
		AddSignalState(signalState);
		index++;
	}


}


////////////////////////////////////////////////////////////////////////////////

void URoadRunnerTrafficSignal::BPCreateInstance (const FLightBulbState& lightBulbState, FComponentReference& compRef, FString& compName, const FSignalState & signalState, const TMap<FString, USCS_Node*>&uuidToComponentMap, const FString & suffix)
{
	FString name = lightBulbState.Name + suffix;
	compName = FRoadRunnerTrafficUtil::FindByNamePrefix(uuidToComponentMap[signalState.Id], name);
	compRef.ComponentProperty = FName(*compName);
	compRef.OtherActor = GetOwner();
};

////////////////////////////////////////////////////////////////////////////////

void URoadRunnerTrafficSignal::CreateInstance(const FLightBulbState & lightBulbState, FComponentReference& compRef, USceneComponent* descendent, const FString & suffix)
{
	FString actorName = descendent->GetOwner()->GetActorLabel();
	FString regex = FString(TEXT("^"));
	regex += lightBulbState.Name;
	regex += suffix;
	regex += FString(TEXT(".*"));
	FRegexPattern pattern(regex);
	FRegexMatcher matcher(pattern, actorName);
	// Check if has match
	if (matcher.FindNext())
	{
		compRef.ComponentProperty = FName("StaticMeshComponent");
		compRef.OtherActor = descendent->GetOwner();
	}
	
};

#endif

////////////////////////////////////////////////////////////////////////////////

void URoadRunnerTrafficSignal::AddSignalState(const FSignalState & signalState)
{
	SignalStates.Add(signalState);
}

////////////////////////////////////////////////////////////////////////////////
// Called when the game starts
void URoadRunnerTrafficSignal::BeginPlay()
{
	Super::BeginPlay();
}

////////////////////////////////////////////////////////////////////////////////
// Called every frame
void URoadRunnerTrafficSignal::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	Timer += DeltaTime;
	if (Timer > 10000) Timer = 0;

	if (Blinking) {
		for (auto& lightInstanceState : SignalStates[Config].LightInstanceStates)
		{
			if (lightInstanceState.OnMeshComponent && lightInstanceState.OffMeshComponent)
			{
				if (lightInstanceState.State == EnumBulbState::eBlinking) {
					bool state = std::fmod(Timer, 1.0) < 0.5f;
					lightInstanceState.OnMeshComponent->SetVisibility(state);
					lightInstanceState.OffMeshComponent->SetVisibility(!state);
				}
			}
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

void URoadRunnerTrafficSignal::SetConfiguration(int index)
{
	if (index >= SignalStates.Num())
	{
		UE_LOG(RoadRunnerTraffic, Warning, TEXT("Invalid configuration %d for signal %s."), index, *Id);
		return;
	}

	Config = index;
	Blinking = false;

	// Set lightbulb state
	for (auto& lightInstanceState : SignalStates[index].LightInstanceStates)
	{
		// Find mesh component
		if (!lightInstanceState.OnMeshComponent)
		{
			// Legacy backup
			if (lightInstanceState.OnRef.OtherActor == nullptr)
			{
				lightInstanceState.OnRef.OtherActor = GetOwner();
			}
			lightInstanceState.OnMeshComponent = Cast<UMeshComponent>(lightInstanceState.OnRef.GetComponent(nullptr));
		}
		if (!lightInstanceState.OffMeshComponent) {
			// Legacy backup
			if (lightInstanceState.OffRef.OtherActor == nullptr)
			{
				lightInstanceState.OffRef.OtherActor = GetOwner();
			}
			lightInstanceState.OffMeshComponent = Cast<UMeshComponent>(lightInstanceState.OffRef.GetComponent(nullptr));
		}
		if (!lightInstanceState.LegacyMeshComponent) {
			// Legacy backup
			if (lightInstanceState.LegacyRef.OtherActor == nullptr)
			{
				lightInstanceState.LegacyRef.OtherActor = GetOwner();
			}
			lightInstanceState.LegacyMeshComponent = Cast<UMeshComponent>(lightInstanceState.LegacyRef.GetComponent(nullptr));
		}

		if (lightInstanceState.OnMeshComponent && lightInstanceState.OffMeshComponent)
		{
			bool state = false;
			if (lightInstanceState.State == EnumBulbState::eOff) state = false;
			else if (lightInstanceState.State == EnumBulbState::eOn) state = true;
			else if (lightInstanceState.State == EnumBulbState::eBlinking) {
				Blinking = true;
				state = std::fmod(Timer, 1.0) < 0.5f;
			}
			else UE_LOG(RoadRunnerTraffic, Error, TEXT("unknown state"));
			lightInstanceState.OnMeshComponent->SetVisibility(state);
			lightInstanceState.OffMeshComponent->SetVisibility(!state);

		}
		else if (lightInstanceState.LegacyMeshComponent) {
			bool state = false;
			if (lightInstanceState.State == EnumBulbState::eOn) state = true;
			lightInstanceState.LegacyMeshComponent->SetVisibility(state);
		}
		else
		{
			// Only print error once per instance
			if (!ErrorFlag)
			{
				UE_LOG(RoadRunnerTraffic, Error, TEXT("Light State not set up properly in %s."), *GetName());
				ErrorFlag = true;
			}
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
