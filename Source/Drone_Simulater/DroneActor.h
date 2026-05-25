#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
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
    class UStaticMeshComponent* PhysicsRoot;

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

private:
    bool bIsFPV = false;
};