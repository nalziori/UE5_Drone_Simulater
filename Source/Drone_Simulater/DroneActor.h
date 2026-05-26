#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Components/BoxComponent.h"
#include "InputActionValue.h"
#include "DroneActor.generated.h"


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

    // 드론 본체 메시
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Mesh")
    class UStaticMeshComponent* DroneMesh;

    // 물리 시뮬레이션용 (Chaos Physics)
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Physics")
    class UBoxComponent* PhysicsRoot;

    // 3인칭 카메라용 스프링암
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    class USpringArmComponent* SpringArm;

    // 3인칭 카메라
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    class UCameraComponent* ThirdPersonCamera;

    // FPV 카메라
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    class UCameraComponent* FPVCamera;

    // 카메라 전환 함수
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void ToggleCamera();

    // Input Mapping Context
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputMappingContext* DroneInputMappingContext;

    // Input Actions
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_Throttle;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_Pitch;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_Roll;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Drone|Input")
    class UInputAction* IA_Yaw;

    // 입력 처리 함수
    void HandleThrottle(const FInputActionValue& Value);
    void HandlePitch(const FInputActionValue& Value);
    void HandleRoll(const FInputActionValue& Value);
    void HandleYaw(const FInputActionValue& Value);

private:
    bool bIsFPV = false;
};