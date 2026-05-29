#include "DroneActor.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/BoxComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Components/TextBlock.h"
#include "GameFramework/PlayerController.h"

ADroneActor::ADroneActor()
{
    PrimaryActorTick.bCanEverTick = true;
    AutoPossessPlayer = EAutoReceiveInput::Player0;

    PhysicsRoot = CreateDefaultSubobject<UBoxComponent>(TEXT("PhysicsRoot"));
    SetRootComponent(PhysicsRoot);
    PhysicsRoot->SetSimulatePhysics(true);
    PhysicsRoot->SetEnableGravity(true);
    PhysicsRoot->SetBoxExtent(FVector(50.f, 50.f, 20.f));
    PhysicsRoot->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    PhysicsRoot->SetCollisionProfileName(TEXT("BlockAll"));

    DroneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneMesh"));
    DroneMesh->SetupAttachment(PhysicsRoot);
    DroneMesh->SetSimulatePhysics(false);
    DroneMesh->SetRelativeLocation(FVector(0.f, 0.f, 0.f));
    DroneMesh->SetRelativeRotation(FRotator(0.f, 0.f, 0.f));

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(PhysicsRoot);
    SpringArm->TargetArmLength = 300.f;
    SpringArm->SetRelativeLocation(FVector(0.f, 0.f, 50.f));
    SpringArm->bUsePawnControlRotation = false;

    ThirdPersonCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ThirdPersonCamera"));
    ThirdPersonCamera->SetupAttachment(SpringArm);
    ThirdPersonCamera->SetActive(true);

    FPVCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FPVCamera"));
    FPVCamera->SetupAttachment(PhysicsRoot);
    FPVCamera->SetRelativeLocation(FVector(20.f, 0.f, 0.f));
    FPVCamera->SetActive(false);
}

void ADroneActor::BeginPlay()
{
    Super::BeginPlay();

    PitchPID.Kp = 1.5f;  PitchPID.Ki = 0.0f;  PitchPID.Kd = 4.0f;
    RollPID.Kp  = 1.5f;  RollPID.Ki  = 0.0f;  RollPID.Kd  = 4.0f;
    AltitudePID.Kp = 10.0f; AltitudePID.Ki = 0.1f; AltitudePID.Kd = 4.0f;

    PhysicsRoot->SetMassOverrideInKg(NAME_None, 1.5f, true);
    PhysicsRoot->SetCenterOfMass(FVector(0.f, 0.f, 0.f));
    PhysicsRoot->SetLinearDamping(0.3f);
    PhysicsRoot->SetAngularDamping(4.0f);

    if (APlayerController* PC = Cast<APlayerController>(GetController()))
    {
        if (UEnhancedInputLocalPlayerSubsystem* Subsystem =
            ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PC->GetLocalPlayer()))
        {
            Subsystem->AddMappingContext(DroneInputMappingContext, 0);
        }
    }

    if (HUDWidgetClass)
    {
        HUDWidget = CreateWidget<UUserWidget>(GetWorld(), HUDWidgetClass);
        if (HUDWidget)
        {
            HUDWidget->AddToViewport();
        }
    }
}

void ADroneActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // 드론 로컬 축
    FVector DroneForward = PhysicsRoot->GetForwardVector();
    FVector DroneRight   = PhysicsRoot->GetRightVector();
    FVector DroneUp      = PhysicsRoot->GetUpVector();
    FVector WorldUp      = FVector::UpVector;

    // 로컬 기준 현재 기울기
    float LocalPitchAngle = FMath::RadiansToDegrees(
        FMath::Asin(FMath::Clamp(FVector::DotProduct(DroneForward, -WorldUp), -1.f, 1.f)));
    float LocalRollAngle = FMath::RadiansToDegrees(
        FMath::Asin(FMath::Clamp(FVector::DotProduct(DroneRight, -WorldUp), -1.f, 1.f)));

    UE_LOG(LogTemp, Warning, TEXT("=== 드론 초기 방향 ==="));
    UE_LOG(LogTemp, Warning, TEXT("Forward: %s"), *PhysicsRoot->GetForwardVector().ToString());
    UE_LOG(LogTemp, Warning, TEXT("Right: %s"), *PhysicsRoot->GetRightVector().ToString());
    UE_LOG(LogTemp, Warning, TEXT("Up: %s"), *PhysicsRoot->GetUpVector().ToString());    
    UE_LOG(LogTemp, Warning, TEXT("DroneForward: %s"), *DroneForward.ToString());
    UE_LOG(LogTemp, Warning, TEXT("LocalPitch: %.1f / LocalRoll: %.1f"), LocalPitchAngle, LocalRollAngle);
    UE_LOG(LogTemp, Warning, TEXT("InputPitch: %.1f / InputRoll: %.1f"), InputPitchAxis, InputRollAxis);

    // if (GEngine)
    // {
    //     GEngine->AddOnScreenDebugMessage(1, 0.f, FColor::Red,
    //         FString::Printf(TEXT("DroneForward: %s"), *DroneForward.ToString()));
    //     GEngine->AddOnScreenDebugMessage(2, 0.f, FColor::Green,
    //         FString::Printf(TEXT("LocalPitch: %.1f / LocalRoll: %.1f"), LocalPitchAngle, LocalRollAngle));
    //     GEngine->AddOnScreenDebugMessage(3, 0.f, FColor::Yellow,
    //         FString::Printf(TEXT("InputPitch: %.1f / InputRoll: %.1f"), InputPitchAxis, InputRollAxis));
    // }

    if (FMath::Abs(LocalPitchAngle) < 85.f && FMath::Abs(LocalRollAngle) < 85.f)
    {
        float TargetPitch = InputPitchAxis * 30.f;
        float TargetRoll  = InputRollAxis  * 30.f;

        float PitchError = TargetPitch - LocalPitchAngle;
        float RollError  = TargetRoll  - LocalRollAngle;

        float PitchCorrection = PitchPID.Update(PitchError, DeltaTime);
        float RollCorrection  = RollPID.Update(RollError, DeltaTime);

        // 드론 로컬 축 기준으로 토크 적용
        FVector Torque = DroneForward * (-RollCorrection * 3000.f)
                       + DroneRight * (PitchCorrection * 3000.f);
        PhysicsRoot->AddTorqueInDegrees(Torque);
    }
    else
    {
        PitchPID.Reset();
        RollPID.Reset();

        // 뒤집혔을 때 수평 복구
        FVector RotationAxis = FVector::CrossProduct(DroneUp, WorldUp);
        float RotationAmount = FVector::DotProduct(DroneUp, WorldUp);

        if (RotationAxis.SizeSquared() > 0.01f)
        {
            PhysicsRoot->AddTorqueInRadians(
                RotationAxis.GetSafeNormal() * (1.f - RotationAmount) * 500000.f);
        }
    }

    if (bHoverMode)
    {
        float CurrentAltitude = GetActorLocation().Z;
        float AltitudeError = TargetAltitude - CurrentAltitude;
        float AltitudeCorrection = AltitudePID.Update(AltitudeError, DeltaTime);
        PhysicsRoot->AddForce(FVector(0.f, 0.f, AltitudeCorrection * 100.f));
    }

    if (HUDWidget)
    {
        float Altitude = GetActorLocation().Z / 100.f;
        UTextBlock* AltitudeText = Cast<UTextBlock>(HUDWidget->GetWidgetFromName(TEXT("Text_Altitude")));
        if (AltitudeText)
            AltitudeText->SetText(FText::FromString(FString::Printf(TEXT("고도: %.1fm"), Altitude)));

        float Speed = PhysicsRoot->GetPhysicsLinearVelocity().Size() / 100.f;
        UTextBlock* SpeedText = Cast<UTextBlock>(HUDWidget->GetWidgetFromName(TEXT("Text_Speed")));
        if (SpeedText)
            SpeedText->SetText(FText::FromString(FString::Printf(TEXT("속도: %.1fm/s"), Speed)));

        UTextBlock* HoverText = Cast<UTextBlock>(HUDWidget->GetWidgetFromName(TEXT("Text_HoverMode")));
        if (HoverText)
            HoverText->SetText(FText::FromString(bHoverMode ? TEXT("호버: ON") : TEXT("호버: OFF")));
    }
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
        EIC->BindAction(IA_Hover,    ETriggerEvent::Started, this, &ADroneActor::HandleHover);
        EIC->BindAction(IA_Pitch,    ETriggerEvent::Completed, this, &ADroneActor::ResetPitch);
        EIC->BindAction(IA_Roll,     ETriggerEvent::Completed, this, &ADroneActor::ResetRoll);
        EIC->BindAction(IA_CameraToggle, ETriggerEvent::Started, this, &ADroneActor::ToggleCamera);
    }

    
}

void ADroneActor::HandleThrottle(const FInputActionValue& Value)
{
    float Axis = Value.Get<float>();
    FVector UpVector = PhysicsRoot->GetUpVector();
    PhysicsRoot->AddForce(UpVector * Axis * 10000.f);
}

void ADroneActor::HandlePitch(const FInputActionValue& Value)
{
    InputPitchAxis = Value.Get<float>();
    //GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Red, FString::Printf(TEXT("Pitch Input: %f"), Axis));
    //PhysicsRoot->AddTorqueInDegrees(FVector(0.f, Axis * 1000000.f, 0.f));
}

void ADroneActor::HandleRoll(const FInputActionValue& Value)
{
    InputRollAxis = Value.Get<float>();
    //GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Green, FString::Printf(TEXT("Roll Input: %f"), Axis));
    //PhysicsRoot->AddTorqueInDegrees(FVector(Axis * -1000000.f, 0.f, 0.f));
}

void ADroneActor::HandleYaw(const FInputActionValue& Value)
{
    float Axis = Value.Get<float>();
    FVector DroneUp = PhysicsRoot->GetUpVector();
    PhysicsRoot->AddTorqueInRadians(DroneUp * Axis * 20000.f);
}

void ADroneActor::HandleHover(const FInputActionValue& Value)
{
    bHoverMode = !bHoverMode;
    if (bHoverMode)
    {
        TargetAltitude = GetActorLocation().Z;
        GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Cyan,
            FString::Printf(TEXT("호버링 ON - 고도: %.1f"), TargetAltitude));
    }
    else
    {
        GEngine->AddOnScreenDebugMessage(-1, 2.f, FColor::Yellow, TEXT("호버링 OFF"));
    }
}

void ADroneActor::ToggleCamera(const FInputActionValue& Value)
{
    bIsFPV = !bIsFPV;
    ThirdPersonCamera->SetActive(!bIsFPV);
    FPVCamera->SetActive(bIsFPV);
}

void ADroneActor::ResetPitch(const FInputActionValue& Value) { InputPitchAxis = 0.f; }
void ADroneActor::ResetRoll(const FInputActionValue& Value)  { InputRollAxis  = 0.f; }
