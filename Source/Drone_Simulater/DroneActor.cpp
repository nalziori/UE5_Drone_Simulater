#include "DroneActor.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/BoxComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Components/TextBlock.h"
#include "GameFramework/PlayerController.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"

ADroneActor::ADroneActor()
{
    PrimaryActorTick.bCanEverTick = true;
    AutoPossessPlayer = EAutoReceiveInput::Player0;

    PhysicsRoot = CreateDefaultSubobject<UBoxComponent>(TEXT("PhysicsRoot"));
    SetRootComponent(PhysicsRoot);
    // Motion comes from FlightCore; the box is kinematic and only used for sweeps against the level.
    PhysicsRoot->SetSimulatePhysics(false);
    PhysicsRoot->SetBoxExtent(FVector(50.f, 50.f, 20.f));
    PhysicsRoot->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    PhysicsRoot->SetCollisionProfileName(TEXT("BlockAll"));

    DroneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneMesh"));
    DroneMesh->SetupAttachment(PhysicsRoot);
    DroneMesh->SetSimulatePhysics(false);
 
    // Rotor layout:
    // - RotorPivotN is the spin axis and must be placed at the propeller center.
    // - RotorN is the only child of RotorPivotN, so only the propeller rotates.
    // - RotorRingN and RotorShaftN are fixed visual parts attached to DroneMesh.
    RotorPivot1 = CreateDefaultSubobject<USceneComponent>(TEXT("RotorPivot1"));
    RotorPivot1->SetupAttachment(DroneMesh);
    RotorPivot1->SetRelativeLocation(RotorPivotOffset1);
    Rotor1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Rotor1"));
    Rotor1->SetupAttachment(RotorPivot1);
    Rotor1->SetSimulatePhysics(false);
    RotorShaft1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorShaft1"));
    RotorShaft1->SetupAttachment(DroneMesh);
    RotorShaft1->SetSimulatePhysics(false);
    RotorRing1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorRing1"));
    RotorRing1->SetupAttachment(DroneMesh);
    RotorRing1->SetSimulatePhysics(false);

    RotorPivot2 = CreateDefaultSubobject<USceneComponent>(TEXT("RotorPivot2"));
    RotorPivot2->SetupAttachment(DroneMesh);
    RotorPivot2->SetRelativeLocation(RotorPivotOffset2);
    Rotor2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Rotor2"));
    Rotor2->SetupAttachment(RotorPivot2);
    Rotor2->SetSimulatePhysics(false);
    RotorShaft2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorShaft2"));
    RotorShaft2->SetupAttachment(DroneMesh);
    RotorShaft2->SetSimulatePhysics(false);
    RotorRing2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorRing2"));
    RotorRing2->SetupAttachment(DroneMesh);
    RotorRing2->SetSimulatePhysics(false);

    RotorPivot3 = CreateDefaultSubobject<USceneComponent>(TEXT("RotorPivot3"));
    RotorPivot3->SetupAttachment(DroneMesh);
    RotorPivot3->SetRelativeLocation(RotorPivotOffset3);
    Rotor3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Rotor3"));
    Rotor3->SetupAttachment(RotorPivot3);
    Rotor3->SetSimulatePhysics(false);
    RotorShaft3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorShaft3"));
    RotorShaft3->SetupAttachment(DroneMesh);
    RotorShaft3->SetSimulatePhysics(false);
    RotorRing3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorRing3"));
    RotorRing3->SetupAttachment(DroneMesh);
    RotorRing3->SetSimulatePhysics(false);

    RotorPivot4 = CreateDefaultSubobject<USceneComponent>(TEXT("RotorPivot4"));
    RotorPivot4->SetupAttachment(DroneMesh);
    RotorPivot4->SetRelativeLocation(RotorPivotOffset4);
    Rotor4 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Rotor4"));
    Rotor4->SetupAttachment(RotorPivot4);
    Rotor4->SetSimulatePhysics(false);
    RotorShaft4 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorShaft4"));
    RotorShaft4->SetupAttachment(DroneMesh);
    RotorShaft4->SetSimulatePhysics(false);
    RotorRing4 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorRing4"));
    RotorRing4->SetupAttachment(DroneMesh);
    RotorRing4->SetSimulatePhysics(false);

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
    // UCameraComponent defaults bAutoActivate to true, which would re-activate this on
    // registration regardless of the SetActive(false) below; must be turned off explicitly
    // so only one camera is active at BeginPlay.
    FPVCamera->SetAutoActivate(false);
    FPVCamera->SetActive(false);
}

namespace
{
    constexpr double PhysicsDt = 0.001;   // 1 kHz rigid-body integration
    constexpr int ControlDivider = 4;     // 250 Hz controller
    constexpr double MaxFrameDt = 0.1;    // drop sim time after hitches instead of spiralling
    constexpr double CmPerM = 100.0;

    FVector ToUE(const fc::Vec3& V) { const fc::Vec3 L = fc::toLeftHanded(V); return FVector(L.x, L.y, L.z); }
    fc::Vec3 FromUE(const FVector& V) { return fc::fromLeftHanded(fc::Vec3{V.X, V.Y, V.Z}); }
    FQuat ToUE(const fc::Quat& Q) { const fc::Quat L = fc::toLeftHanded(Q); return FQuat(L.x, L.y, L.z, L.w); }
    fc::Quat FromUE(const FQuat& Q) { return fc::fromLeftHanded(fc::Quat{Q.W, Q.X, Q.Y, Q.Z}); }
}

void ADroneActor::BeginPlay()
{
    Super::BeginPlay();

    // Spawn point is the sim origin and ground level; keep the spawn heading.
    OriginUE = GetActorLocation();
    Quad.s = fc::QuadState{};
    Quad.s.att = FromUE(FQuat(FRotator(0.f, GetActorRotation().Yaw, 0.f)));
    Ctrl.mode = fc::FlightMode::Angle;
    Ctrl.reset(Quad.s);
    Safety.lim.geofenceRadius = GeofenceRadiusM;
    Safety.lim.ceiling = CeilingM;
    Safety.reset(Quad.s.pos);
    RunPreflightCheck();

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

    if (bRecordFlightLog)
    {
        FlightLog.Add(TEXT("t,x,y,z,vx,vy,vz,roll,pitch,yaw,w1,w2,w3,w4,mode,safety"));
    }
}

// Same FlightCore, flown headlessly before the real session: the reference mission (10 m square at 5 m)
// under the configured mean wind with randomized turbulence, sensor noise and model error.
// Uses FlightCore's nominal parameters, which is also what this actor flies.
void ADroneActor::RunPreflightCheck()
{
    if (PreflightRuns <= 0)
    {
        return;
    }
    const double Wind = WindMps.Size();
    int32 Passed = 0;
    for (int32 i = 0; i < PreflightRuns; ++i)
    {
        Passed += fc::virtualFlight(Wind, 1000u + static_cast<uint32>(i)).pass ? 1 : 0;
    }
    const bool bGo = Passed == PreflightRuns;
    PreflightResult = FString::Printf(TEXT("사전검증 %s: 기준 임무 %d/%d 통과 (평균풍 %.1f m/s)"),
        bGo ? TEXT("PASS") : TEXT("FAIL"), Passed, PreflightRuns, Wind);
    UE_LOG(LogTemp, Log, TEXT("%s"), *PreflightResult);
}

void ADroneActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (bRecordFlightLog && FlightLog.Num() > 1)
    {
        const FString Dir = FPaths::ProjectSavedDir() / TEXT("FlightLogs");
        FPlatformFileManager::Get().GetPlatformFile().CreateDirectoryTree(*Dir);
        const FString Path = Dir / FString::Printf(TEXT("flight_%s.csv"), *FDateTime::Now().ToString());
        FFileHelper::SaveStringArrayToFile(FlightLog, *Path);
        UE_LOG(LogTemp, Log, TEXT("Flight log written: %s (%d rows)"), *Path, FlightLog.Num() - 1);
    }
    Super::EndPlay(EndPlayReason);
}

void ADroneActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    Quad.wind = FromUE(WindMps);

    Accumulator += FMath::Min<double>(DeltaTime, MaxFrameDt);
    while (Accumulator >= PhysicsDt)
    {
        StepSimulation(PhysicsDt);
        Accumulator -= PhysicsDt;
    }

    ApplyStateToActor();
    UpdateRotors(DeltaTime);
    UpdateHUD();
}

void ADroneActor::StepSimulation(double Dt)
{
    if (SimSteps % ControlDivider == 0)
    {
        // No GPS model in UE: position is always valid here.
        fc::PilotInput CmdInput = Pilot;
        if (bEnableSafetyMonitor)
        {
            Safety.update(Quad.s, true, Ctrl.saturated, Dt * ControlDivider);
            Safety.command(Ctrl, CmdInput, Quad.s, true);
        }
        Ctrl.update(Quad.p, Quad.s, CmdInput, Dt * ControlDivider, OmegaCmd);
        if (Safety.motorsOff)
        {
            for (double& Omega : OmegaCmd)
            {
                Omega = 0;
            }
        }
        if (Safety.action != LoggedSafetyAction)
        {
            LoggedSafetyAction = Safety.action;
            UE_LOG(LogTemp, Warning, TEXT("Safety %s (%s) at t=%.2f s"),
                ANSI_TO_TCHAR(fc::actionName(Safety.action)), ANSI_TO_TCHAR(Safety.reason), SimTime);
        }
    }
    Quad.step(OmegaCmd, Dt);
    SimTime += Dt;
    ++SimSteps;

    if (bRecordFlightLog && SimSteps % 20 == 0)   // 50 Hz
    {
        const fc::Vec3 E = Quad.s.att.toEuler();
        const fc::QuadState& S = Quad.s;
        FlightLog.Add(FString::Printf(TEXT("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%d,%d"),
            SimTime, S.pos.x, S.pos.y, S.pos.z, S.vel.x, S.vel.y, S.vel.z,
            E.x / fc::kDeg, E.y / fc::kDeg, E.z / fc::kDeg,
            S.omega[0], S.omega[1], S.omega[2], S.omega[3], static_cast<int32>(Ctrl.mode), static_cast<int32>(Safety.action)));
    }
}

void ADroneActor::ApplyStateToActor()
{
    const FVector Target = OriginUE + ToUE(Quad.s.pos) * CmPerM;
    FHitResult Hit;
    SetActorLocationAndRotation(Target, ToUE(Quad.s.att), true, &Hit);

    if (Hit.bBlockingHit)
    {
        // Level geometry wins: snap the sim to where the sweep stopped and remove velocity into the surface.
        // ponytail: no bounce or crash damage; add a contact model if collisions need to matter.
        Quad.s.pos = FromUE(GetActorLocation() - OriginUE) / CmPerM;
        const fc::Vec3 N = FromUE(Hit.ImpactNormal);
        const double Into = Quad.s.vel.dot(N);
        if (Into < 0)
        {
            Quad.s.vel = Quad.s.vel - N * Into;
        }
    }
}

void ADroneActor::UpdateRotors(float DeltaTime)
{
    // RotorPivotN should sit on FlightCore motor N: 1 front-left, 2 front-right, 3 rear-right, 4 rear-left.
    USceneComponent* Pivots[4] = {RotorPivot1, RotorPivot2, RotorPivot3, RotorPivot4};
    const double HoverOmega = FMath::Sqrt(Quad.p.hoverThrustPerMotor() / Quad.p.kThrust);
    for (int32 i = 0; i < 4; ++i)
    {
        const double Visual = RotorSpeed * Quad.s.omega[i] / HoverOmega;
        RotorAngle[i] = static_cast<float>(FMath::Fmod(RotorAngle[i] + fc::Quadrotor::kSpin[i] * Visual * DeltaTime, 360.0));
        if (Pivots[i])
        {
            Pivots[i]->SetRelativeRotation(FRotator(0.f, 0.f, RotorAngle[i]));
        }
    }
}

void ADroneActor::UpdateHUD()
{
    if (!HUDWidget)
    {
        return;
    }
    if (UTextBlock* AltitudeText = Cast<UTextBlock>(HUDWidget->GetWidgetFromName(TEXT("Text_Altitude"))))
    {
        AltitudeText->SetText(FText::FromString(FString::Printf(TEXT("고도: %.1fm"), Quad.s.pos.z)));
    }
    if (UTextBlock* SpeedText = Cast<UTextBlock>(HUDWidget->GetWidgetFromName(TEXT("Text_Speed"))))
    {
        SpeedText->SetText(FText::FromString(FString::Printf(TEXT("속도: %.1fm/s"), Quad.s.vel.norm())));
    }
    if (UTextBlock* HoverText = Cast<UTextBlock>(HUDWidget->GetWidgetFromName(TEXT("Text_HoverMode"))))
    {
        const bool bHold = Ctrl.mode == fc::FlightMode::Position;
        HoverText->SetText(FText::FromString(bHold ? TEXT("모드: 위치 고정") : TEXT("모드: 자세 (고도 유지)")));
    }
    // Optional TextBlock: shows the pre-flight result until the monitor triggers, then the action and reason.
    if (UTextBlock* SafetyText = Cast<UTextBlock>(HUDWidget->GetWidgetFromName(TEXT("Text_Safety"))))
    {
        const FString Line = Safety.action == fc::SafetyAction::None ? PreflightResult
            : FString::Printf(TEXT("안전: %s (%s)"), ANSI_TO_TCHAR(fc::actionName(Safety.action)), ANSI_TO_TCHAR(Safety.reason));
        SafetyText->SetText(FText::FromString(Line));
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
        EIC->BindAction(IA_Hover,    ETriggerEvent::Started,   this, &ADroneActor::HandleHover);
        EIC->BindAction(IA_Throttle, ETriggerEvent::Completed, this, &ADroneActor::ResetThrottle);
        EIC->BindAction(IA_Pitch,    ETriggerEvent::Completed, this, &ADroneActor::ResetPitch);
        EIC->BindAction(IA_Roll,     ETriggerEvent::Completed, this, &ADroneActor::ResetRoll);
        EIC->BindAction(IA_Yaw,      ETriggerEvent::Completed, this, &ADroneActor::ResetYaw);
        EIC->BindAction(IA_CameraToggle, ETriggerEvent::Started, this, &ADroneActor::ToggleCamera);
    }
}

// Sticks: throttle = climb rate, pitch/roll = attitude angle (+pitch nose down, +roll right), yaw = turn rate.
void ADroneActor::HandleThrottle(const FInputActionValue& Value) { Pilot.throttle = FMath::Clamp(Value.Get<float>(), -1.f, 1.f); }
void ADroneActor::HandlePitch(const FInputActionValue& Value)    { Pilot.pitch = FMath::Clamp(Value.Get<float>(), -1.f, 1.f); }
void ADroneActor::HandleRoll(const FInputActionValue& Value)     { Pilot.roll = FMath::Clamp(Value.Get<float>(), -1.f, 1.f); }
// UE yaw + is a right turn; FlightCore yaw + is a left turn (right-handed).
void ADroneActor::HandleYaw(const FInputActionValue& Value)      { Pilot.yaw = -FMath::Clamp(Value.Get<float>(), -1.f, 1.f); }

// Hover key toggles position hold (FlightCore Position mode) <-> manual attitude flight.
void ADroneActor::HandleHover(const FInputActionValue& Value)
{
    if (Safety.action != fc::SafetyAction::None)
    {
        return;   // the monitor owns the mode once it has triggered
    }
    const bool bToHold = Ctrl.mode == fc::FlightMode::Angle;
    Ctrl.setMode(bToHold ? fc::FlightMode::Position : fc::FlightMode::Angle, Quad.s);
}

void ADroneActor::ToggleCamera(const FInputActionValue& Value)
{
    bIsFPV = !bIsFPV;
    ThirdPersonCamera->SetActive(!bIsFPV);
    FPVCamera->SetActive(bIsFPV);
}

void ADroneActor::ResetPitch(const FInputActionValue& Value)    { Pilot.pitch = 0; }
void ADroneActor::ResetRoll(const FInputActionValue& Value)     { Pilot.roll = 0; }
void ADroneActor::ResetThrottle(const FInputActionValue& Value) { Pilot.throttle = 0; }
void ADroneActor::ResetYaw(const FInputActionValue& Value)      { Pilot.yaw = 0; }
