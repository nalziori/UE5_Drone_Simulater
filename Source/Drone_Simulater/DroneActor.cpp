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

    PitchPID.Kp = 2.0f;  PitchPID.Ki = 0.0f;  PitchPID.Kd = 5.0f;
    RollPID.Kp  = 2.0f;  RollPID.Ki  = 0.0f;  RollPID.Kd  = 5.0f;
    AltitudePID.Kp = 8.0f; AltitudePID.Ki = 0.2f; AltitudePID.Kd = 3.0f;

    PhysicsRoot->SetMassOverrideInKg(NAME_None, 1.5f, true);
    PhysicsRoot->SetCenterOfMass(FVector(0.f, 0.f, 0.f));
    PhysicsRoot->SetLinearDamping(0.5f);
    PhysicsRoot->SetAngularDamping(5.0f);

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

    FRotator CurrentRotation = PhysicsRoot->GetComponentRotation();

    if (FMath::Abs(CurrentRotation.Pitch) < 90.f && FMath::Abs(CurrentRotation.Roll) < 90.f)
    {
        float TargetPitch = InputPitchAxis * -30.f;
        float TargetRoll  = InputRollAxis  *  30.f;

        float PitchError = TargetPitch - CurrentRotation.Pitch;
        float RollError  = TargetRoll  - CurrentRotation.Roll;

        float PitchCorrection = PitchPID.Update(PitchError, DeltaTime);
        float RollCorrection  = RollPID.Update(RollError, DeltaTime);

        FVector LocalTorque = PhysicsRoot->GetComponentTransform().TransformVector(
            FVector(-RollCorrection * 5000.f, -PitchCorrection * 5000.f, 0.f));
        PhysicsRoot->AddTorqueInDegrees(LocalTorque);

        // PhysicsRoot->AddTorqueInDegrees(
        //     FVector(-RollCorrection * 5000.f, -PitchCorrection * 5000.f, 0.f),
        //     NAME_None, 
        //     false);
    }
    else
    {
        PitchPID.Reset();
        RollPID.Reset();

        float RecoveryPitch  = -CurrentRotation.Pitch * 0.1f;
        float RecoveryRoll = -CurrentRotation.Roll * 0.1f;
        PhysicsRoot->AddTorqueInDegrees(FVector(-RecoveryRoll * 5000.f, -RecoveryPitch * 5000.f, 0.f));
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
        
        float Altitude = GetActorLocation().Z / 100.f; // cm → m
        UTextBlock* AltitudeText = Cast<UTextBlock>(HUDWidget->GetWidgetFromName(TEXT("Text_Altitude")));
        if (AltitudeText)
            AltitudeText->SetText(FText::FromString(FString::Printf(TEXT("고도: %.1fm"), Altitude)));

        
        float Speed = PhysicsRoot->GetPhysicsLinearVelocity().Size() / 100.f; // cm/s → m/s
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
    //GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Blue, FString::Printf(TEXT("Yaw Input: %f"), Axis));
    PhysicsRoot->AddTorqueInRadians(FVector(0.f, 0.f, Axis * 20000.f));
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
