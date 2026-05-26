#include "DroneActor.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/BoxComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "GameFramework/PlayerController.h"

ADroneActor::ADroneActor()
{
    PrimaryActorTick.bCanEverTick = true;
	AutoPossessPlayer = EAutoReceiveInput::Player0;

    // 물리 루트 컴포넌트 (Chaos Physics 적용 대상)
	PhysicsRoot = CreateDefaultSubobject<UBoxComponent>(TEXT("PhysicsRoot"));
    SetRootComponent(PhysicsRoot);
    PhysicsRoot->SetSimulatePhysics(true);
    PhysicsRoot->SetEnableGravity(true);
	PhysicsRoot->SetBoxExtent(FVector(50.f, 50.f, 20.f));
    PhysicsRoot->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    PhysicsRoot->SetCollisionProfileName(TEXT("BlockAll"));

    // 드론 본체 메시 (PhysicsRoot에 부착)
    DroneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneMesh"));
    DroneMesh->SetupAttachment(PhysicsRoot);
    DroneMesh->SetSimulatePhysics(false);

    // 스프링암 (3인칭 카메라 거리 조절)
    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(PhysicsRoot);
    SpringArm->TargetArmLength = 300.f;
    SpringArm->SetRelativeLocation(FVector(0.f, 0.f, 50.f));
    SpringArm->bUsePawnControlRotation = false;

    // 3인칭 카메라
    ThirdPersonCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ThirdPersonCamera"));
    ThirdPersonCamera->SetupAttachment(SpringArm);
    ThirdPersonCamera->SetActive(true);

    // FPV 카메라 (드론 앞쪽에 부착)
    FPVCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FPVCamera"));
    FPVCamera->SetupAttachment(PhysicsRoot);
    FPVCamera->SetRelativeLocation(FVector(20.f, 0.f, 0.f));
    FPVCamera->SetActive(false);


}

void ADroneActor::BeginPlay()
{
    Super::BeginPlay();

    PhysicsRoot->SetMassOverrideInKg(NAME_None, 1.5f, true);

    if (APlayerController* PC = Cast<APlayerController>(GetController()))
    {
        if (UEnhancedInputLocalPlayerSubsystem* Subsystem =
            ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PC->GetLocalPlayer()))
        {
            Subsystem->AddMappingContext(DroneInputMappingContext, 0);
        }
    }
}

void ADroneActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void ADroneActor::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    if (UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(PlayerInputComponent))
    {
        EIC->BindAction(IA_Throttle, ETriggerEvent::Triggered, this, &ADroneActor::HandleThrottle);
        EIC->BindAction(IA_Pitch,    ETriggerEvent::Triggered, this, &ADroneActor::HandlePitch);
        EIC->BindAction(IA_Roll,     ETriggerEvent::Triggered, this, &ADroneActor::HandleRoll);
        EIC->BindAction(IA_Yaw,      ETriggerEvent::Triggered, this, &ADroneActor::HandleYaw);
    }
}

void ADroneActor::HandleThrottle(const FInputActionValue& Value)
{
    float Axis = Value.Get<float>();
    PhysicsRoot->AddForce(FVector(0.f, 0.f, Axis * 10000.f));
}

void ADroneActor::HandlePitch(const FInputActionValue& Value)
{
    float Axis = Value.Get<float>();
    PhysicsRoot->AddTorqueInDegrees(FVector(0.f, Axis * 50000.f, 0.f));
}

void ADroneActor::HandleRoll(const FInputActionValue& Value)
{
    float Axis = Value.Get<float>();
    PhysicsRoot->AddTorqueInDegrees(FVector(Axis * 50000.f, 0.f, 0.f));
}

void ADroneActor::HandleYaw(const FInputActionValue& Value)
{
    float Axis = Value.Get<float>();
    PhysicsRoot->AddTorqueInDegrees(FVector(0.f, 0.f, Axis * 50000.f));
}

void ADroneActor::ToggleCamera()
{
    bIsFPV = !bIsFPV;
    ThirdPersonCamera->SetActive(!bIsFPV);
    FPVCamera->SetActive(bIsFPV);
}
