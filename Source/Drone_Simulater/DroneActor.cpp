#include "DroneActor.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/BoxComponent.h"
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
	PhysicsRoot->SetBoxExtent(FVector(50.f, 50.f, 20.f)); // 이 줄 추가


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

	//auto possess player 0
	APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (PC)
    {
        PC->Possess(this);
    }
}

void ADroneActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
}

void ADroneActor::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void ADroneActor::ToggleCamera()
{
    bIsFPV = !bIsFPV;
    ThirdPersonCamera->SetActive(!bIsFPV);
    FPVCamera->SetActive(bIsFPV);
}
