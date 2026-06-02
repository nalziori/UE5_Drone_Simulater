#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Components/BoxComponent.h"
#include "InputActionValue.h"
#include "Blueprint/UserWidget.h"
#include "DroneActor.generated.h"

USTRUCT()
struct FPIDController
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere) float Kp = 5.0f;
    UPROPERTY(EditAnywhere) float Ki = 0.1f;
    UPROPERTY(EditAnywhere) float Kd = 2.0f;

    float Integral = 0.f;
    float PrevError = 0.f;

    float Update(float Error, float DeltaTime)
    {
        Integral += Error * DeltaTime;
        float Derivative = (Error - PrevError) / DeltaTime;
        PrevError = Error;
        return Kp * Error + Ki * Integral + Kd * Derivative;
    }

    void Reset()
    {
        Integral = 0.f;
        PrevError = 0.f;
    }
};

UCLASS()
class DRONE_SIMULATER_API ADroneActor : public APawn
{
    GENERATED_BODY()

public:
    ADroneActor();

protected:
    virtual void BeginPlay() override;

public:
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Mesh")
    class UStaticMeshComponent* DroneMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Physics")
    class UBoxComponent* PhysicsRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    class USpringArmComponent* SpringArm;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    class UCameraComponent* ThirdPersonCamera;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    class UCameraComponent* FPVCamera;

    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void ToggleCamera(const FInputActionValue& Value);

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputMappingContext* DroneInputMappingContext;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_Throttle;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_Pitch;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_Roll;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_Yaw;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_Hover;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_CameraToggle;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|HUD")
    TSubclassOf<UUserWidget> HUDWidgetClass;

    UPROPERTY()
    UUserWidget* HUDWidget;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Drone|Rotor")
    USceneComponent* RotorPivot1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Drone|Rotor")
    USceneComponent* RotorPivot2;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Drone|Rotor")
    USceneComponent* RotorPivot3;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Drone|Rotor")
    USceneComponent* RotorPivot4;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* Rotor1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* Rotor2;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* Rotor3;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* Rotor4;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    float RotorSpeed = 100.f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* RotorShaft1; 

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* RotorShaft2; 

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* RotorShaft3; 

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* RotorShaft4;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* RotorRing1;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* RotorRing2;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* RotorRing3;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Rotor")
    class UStaticMeshComponent* RotorRing4;

    void HandleThrottle(const FInputActionValue& Value);
    void HandlePitch(const FInputActionValue& Value);
    void HandleRoll(const FInputActionValue& Value);
    void HandleYaw(const FInputActionValue& Value);
    void HandleHover(const FInputActionValue& Value);
    void ResetPitch(const FInputActionValue& Value);
    void ResetRoll(const FInputActionValue& Value);

private:
    bool bIsFPV = false;
    bool bHoverMode = false;
    float TargetAltitude = 0.f;

    FPIDController PitchPID;
    FPIDController RollPID;
    FPIDController AltitudePID;

    float InputPitchAxis = 0.f;
    float InputRollAxis  = 0.f;

    float RotorAngle = 0.f;
};