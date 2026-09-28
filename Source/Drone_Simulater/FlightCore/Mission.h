// Headless flight harness shared by Tools/FlightSim and the UE pre-flight check:
// plant + controller + safety monitor + a model of how the real world differs from the nominal model,
// and a reference mission with pass criteria.
#pragma once
#include "Safety.h"

#include <algorithm>
#include <functional>
#include <random>
#include <vector>

namespace fc {

// Everything the real flight may do differently from the nominal model. Defaults = ideal.
struct Conditions {
    Vec3 wind;                                        // mean wind, world, m/s
    double turbSigma = 0, turbTau = 2.0;              // 1st-order Gauss-Markov turbulence per axis (m/s, s)
    double gyroNoise = 0, posNoise = 0, velNoise = 0; // 1 sigma
    Vec3 gyroBias;                                    // rad/s
    double posLossStart = 1e9, posLossEnd = 1e9;      // horizontal position/velocity frozen in [start, end)
    double massScale = 1, inertiaScale = 1, motorTauScale = 1;
    double motorEff[4] = {1, 1, 1, 1};
    int faultMotor = -1;                              // this motor's efficiency drops to faultEff at faultTime
    double faultEff = 1, faultTime = 0;
};

struct Sample {
    double t;
    QuadState s;
    Quat attSp;
    Vec3 rateSp;
    SafetyAction safety;
};

struct Harness {
    Quadrotor quad;
    Controller ctrl;
    SafetyMonitor safety;
    bool safetyEnabled = false;
    QuadParams nominal;              // what the controller believes; the plant may differ (applyConditions)
    Conditions cond;
    PilotInput input;
    double t = 0;
    double physDt = 0.001;
    int ctrlDivider = 4;             // controller at 250 Hz
    std::mt19937 rng{42};            // sensor noise
    std::mt19937 envRng{7};          // turbulence
    Vec3 turb;
    std::vector<Sample> log;
    bool recordLog = true;
    double omegaCmd[4] = {0, 0, 0, 0};
    long steps = 0;
    QuadState lastFix;

    void applyConditions() {
        quad.p = nominal;
        quad.p.mass *= cond.massScale;
        quad.p.inertia = quad.p.inertia * cond.inertiaScale;
        quad.p.motorTau *= cond.motorTauScale;
        for (int i = 0; i < 4; ++i) quad.motorEff[i] = cond.motorEff[i];
    }

    bool posValid() const { return !(t >= cond.posLossStart && t < cond.posLossEnd); }

    QuadState measure() {
        QuadState m = quad.s;
        std::normal_distribution<double> n(0.0, 1.0);
        m.rate += Vec3{n(rng), n(rng), n(rng)} * cond.gyroNoise + cond.gyroBias;
        m.pos += Vec3{n(rng), n(rng), n(rng)} * cond.posNoise;
        m.vel += Vec3{n(rng), n(rng), n(rng)} * cond.velNoise;
        if (posValid()) {
            lastFix = m;
        } else {  // baro still gives z / vz
            m.pos.x = lastFix.pos.x; m.pos.y = lastFix.pos.y;
            m.vel.x = lastFix.vel.x; m.vel.y = lastFix.vel.y;
        }
        return m;
    }

    void run(double duration, const std::function<void(Harness&)>& perStep = nullptr) {
        double end = t + duration;
        while (t < end - 1e-9) {
            if (perStep) perStep(*this);
            if (cond.faultMotor >= 0 && t >= cond.faultTime) quad.motorEff[cond.faultMotor] = cond.faultEff;
            if (cond.turbSigma > 0) {
                std::normal_distribution<double> n(0.0, 1.0);
                double k = physDt / cond.turbTau, s = cond.turbSigma * std::sqrt(2 * k);
                turb = turb * (1 - k) + Vec3{n(envRng), n(envRng), n(envRng)} * s;
            }
            quad.wind = cond.wind + turb;
            if (steps % ctrlDivider == 0) {
                double dtc = physDt * ctrlDivider;
                QuadState m = measure();
                PilotInput in = input;
                if (safetyEnabled) {
                    safety.update(m, posValid(), ctrl.saturated, dtc);
                    safety.command(ctrl, in, m, posValid());
                }
                ctrl.update(nominal, m, in, dtc, omegaCmd);
                if (safetyEnabled && safety.motorsOff)
                    for (double& o : omegaCmd) o = 0;
            }
            quad.step(omegaCmd, physDt);
            t += physDt;
            ++steps;
            if (recordLog && steps % 10 == 0) log.push_back({t, quad.s, ctrl.attSetpoint, ctrl.rateSetpoint, safety.action});
        }
    }

    // Start hovering at `alt` in Position mode (motors spun up, controller settled), t reset to 0.
    void startHover(double alt) {
        quad.s = QuadState{};
        quad.s.pos = {0, 0, alt};
        quad.s.onGround = false;
        double w = std::sqrt(quad.p.hoverThrustPerMotor() / quad.p.kThrust);
        for (double& o : quad.s.omega) o = w;
        for (double& o : omegaCmd) o = w;
        ctrl.mode = FlightMode::Position;
        ctrl.reset(quad.s);
        bool keep = safetyEnabled;
        safetyEnabled = false;
        run(2.0);
        safetyEnabled = keep;
        safety.reset({0, 0, 0});
        log.clear();
        t = 0;
    }
};

// Reference mission: 10 m square at 5 m. Pass criteria were fixed before any campaign was run.
struct MissionCriteria {
    double reachPos = 0.5, reachVel = 0.5;   // waypoint reached
    double legTimeout = 15;                  // s per 10 m leg
    double maxPathDev = 1.0;                 // m from the planned straight segment
    double maxAltErr = 0.5;                  // m
};

struct MissionResult {
    bool pass = false, allReached = true;
    double maxPathDev = 0, maxAltErr = 0, maxTilt = 0, missionTime = 0;
    SafetyAction safety = SafetyAction::None;
};

inline double distToSegment(const Vec3& p, const Vec3& a, const Vec3& b) {
    Vec3 ab = b - a;
    double u = clampd((p - a).dot(ab) / ab.dot(ab), 0.0, 1.0);
    return (p - (a + ab * u)).norm();
}

inline MissionResult flySquare(Harness& h, const MissionCriteria& crit = {}) {
    const Vec3 wps[] = {{0, 0, 5}, {10, 0, 5}, {10, 10, 5}, {0, 10, 5}, {0, 0, 5}};
    MissionResult r;
    h.startHover(5);
    for (int leg = 1; leg < 5 && r.allReached && h.safety.action == SafetyAction::None; ++leg) {
        const Vec3 a = wps[leg - 1], b = wps[leg];
        h.ctrl.posSetpoint = b;
        double start = h.t;
        bool reached = false;
        while (!reached && h.t - start < crit.legTimeout && h.safety.action == SafetyAction::None) {
            h.run(0.02, [&](Harness& s) {
                const QuadState& q = s.quad.s;
                r.maxPathDev = std::max(r.maxPathDev, distToSegment(q.pos, a, b));
                r.maxAltErr = std::max(r.maxAltErr, std::abs(q.pos.z - 5));
                r.maxTilt = std::max(r.maxTilt, std::acos(clampd(q.att.rotate({0, 0, 1}).z, -1.0, 1.0)));
            });
            reached = (h.quad.s.pos - b).norm() < crit.reachPos && h.quad.s.vel.norm() < crit.reachVel;
        }
        r.allReached = reached;
    }
    r.missionTime = h.t;
    r.safety = h.safety.action;
    r.pass = r.allReached && r.safety == SafetyAction::None && r.maxPathDev < crit.maxPathDev && r.maxAltErr < crit.maxAltErr;
    return r;
}

// Randomized "real world" for one virtual flight. Ranges are assumptions, not measured data:
// wind direction, 20 % turbulence intensity, sensor noise, and model error the controller does not know.
inline Conditions sampleConditions(std::mt19937& rng, double windSpeed) {
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    auto pick = [&](double lo, double hi) { return lo + (hi - lo) * u01(rng); };
    Conditions c;
    double dir = pick(0, 2 * kPi);
    c.wind = {windSpeed * std::cos(dir), windSpeed * std::sin(dir), 0};
    c.turbSigma = 0.2 * windSpeed;
    c.gyroNoise = pick(0, 0.03);
    c.posNoise = pick(0, 0.05);
    c.velNoise = pick(0, 0.1);
    c.massScale = pick(0.9, 1.1);
    c.inertiaScale = pick(0.85, 1.15);
    c.motorTauScale = pick(0.8, 1.2);
    for (double& e : c.motorEff) e = pick(0.95, 1.05);
    return c;
}

// One reproducible virtual flight of the reference mission with the safety monitor active.
inline MissionResult virtualFlight(double windSpeed, unsigned seed, Harness* keep = nullptr) {
    std::mt19937 rng(seed);
    Harness h;
    h.cond = sampleConditions(rng, windSpeed);
    h.rng.seed(seed + 1);
    h.envRng.seed(seed + 2);
    h.applyConditions();
    h.safetyEnabled = true;
    h.safety.lim.floorAlt = 2.0;
    h.recordLog = keep != nullptr;
    MissionResult r = flySquare(h);
    if (keep) *keep = h;
    return r;
}

}  // namespace fc
