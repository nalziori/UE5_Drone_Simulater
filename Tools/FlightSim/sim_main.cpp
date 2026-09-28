// Headless verification of FlightCore: unit checks + flight scenarios with pass/fail criteria.
// Build (any C++17 compiler):  c++ -std=c++17 -O2 -I ../../Source/Drone_Simulater sim_main.cpp -o flightsim
// Run:  ./flightsim [outdir]   -> writes one CSV per scenario, prints metrics, exit code 1 on any failure.
#include "FlightCore/Controller.h"

#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace fc;

static int gFailures = 0;
static void expect(bool ok, const char* what, double value, const char* unit) {
    std::printf("  [%s] %-44s %10.3f %s\n", ok ? "PASS" : "FAIL", what, value, unit);
    if (!ok) ++gFailures;
}

struct Sample {
    double t;
    QuadState s;
    Quat attSp;
    Vec3 rateSp;
};

struct Sim {
    Quadrotor quad;
    Controller ctrl;
    PilotInput input;
    double t = 0;
    double physDt = 0.001;
    int ctrlDivider = 4;            // controller at 250 Hz
    double gyroNoise = 0, posNoise = 0, velNoise = 0;
    std::mt19937 rng{42};
    std::vector<Sample> log;
    double omegaCmd[4] = {0, 0, 0, 0};
    long steps = 0;

    QuadState measure() {
        QuadState m = quad.s;
        std::normal_distribution<double> n(0.0, 1.0);
        m.rate += Vec3{n(rng), n(rng), n(rng)} * gyroNoise;
        m.pos += Vec3{n(rng), n(rng), n(rng)} * posNoise;
        m.vel += Vec3{n(rng), n(rng), n(rng)} * velNoise;
        return m;
    }

    void run(double duration, const std::function<void(Sim&)>& perStep = nullptr) {
        double end = t + duration;
        while (t < end - 1e-9) {
            if (perStep) perStep(*this);
            if (steps % ctrlDivider == 0)
                ctrl.update(quad.p, measure(), input, physDt * ctrlDivider, omegaCmd);
            quad.step(omegaCmd, physDt);
            t += physDt;
            ++steps;
            if (steps % 10 == 0) log.push_back({t, quad.s, ctrl.attSetpoint, ctrl.rateSetpoint});
        }
    }

    void writeCsv(const std::string& path) const {
        FILE* f = std::fopen(path.c_str(), "w");
        if (!f) { std::printf("  (could not write %s)\n", path.c_str()); return; }
        std::fprintf(f, "t,x,y,z,vx,vy,vz,roll,pitch,yaw,roll_sp,pitch_sp,yaw_sp,p,q,r,p_sp,q_sp,r_sp,w1,w2,w3,w4\n");
        for (const auto& l : log) {
            Vec3 e = l.s.att.toEuler(), es = l.attSp.toEuler();
            std::fprintf(f, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f\n",
                         l.t, l.s.pos.x, l.s.pos.y, l.s.pos.z, l.s.vel.x, l.s.vel.y, l.s.vel.z,
                         e.x / kDeg, e.y / kDeg, e.z / kDeg, es.x / kDeg, es.y / kDeg, es.z / kDeg,
                         l.s.rate.x / kDeg, l.s.rate.y / kDeg, l.s.rate.z / kDeg,
                         l.rateSp.x / kDeg, l.rateSp.y / kDeg, l.rateSp.z / kDeg,
                         l.s.omega[0], l.s.omega[1], l.s.omega[2], l.s.omega[3]);
        }
        std::fclose(f);
    }

    // Start hovering at `alt` in Position mode (motors spun up, controller settled).
    void startHover(double alt) {
        quad.s = QuadState{};
        quad.s.pos = {0, 0, alt};
        quad.s.onGround = false;
        double w = std::sqrt(quad.p.hoverThrustPerMotor() / quad.p.kThrust);
        for (double& o : quad.s.omega) o = w;
        for (double& o : omegaCmd) o = w;
        ctrl.mode = FlightMode::Position;
        ctrl.reset(quad.s);
        run(2.0);
        log.clear();
        t = 0;
    }
};

// Time after which |signal - target| stays within band (from samples at/after t0).
static double settlingTime(const std::vector<Sample>& log, double t0, double band,
                           const std::function<double(const Sample&)>& err) {
    double last = t0;
    for (const auto& l : log)
        if (l.t >= t0 && std::abs(err(l)) > band) last = l.t;
    return last - t0;
}

static double near(double a, double b) { return std::abs(a - b); }

// ---------------------------------------------------------------------------------------------
static void unitChecks() {
    std::printf("\n[0] Unit checks\n");
    QuadParams p;

    // Mixer must be the exact inverse of the motor geometry (when not saturated).
    Vec3 tau{0.05, -0.08, 0.01};
    double thrust = 16.0, w[4];
    bool sat = Controller::mix(p, thrust, tau, w);
    Quadrotor q; q.p = p;
    for (int i = 0; i < 4; ++i) q.s.omega[i] = w[i];
    Vec3 tauBack; double thrustBack = 0;
    for (int i = 0; i < 4; ++i) {
        double t = q.motorThrust(i);
        thrustBack += t;
        tauBack += q.motorPos(i).cross(Vec3{0, 0, t});
        tauBack.z += -Quadrotor::kSpin[i] * p.kTorque * w[i] * w[i];
    }
    double mixErr = (tauBack - tau).norm() + near(thrustBack, thrust);
    expect(!sat && mixErr < 1e-9, "mixer inverse error", mixErr, "");

    // Euler round trip
    Quat e = Quat::fromEuler(0.3, -0.2, 2.5);
    Vec3 eb = e.toEuler();
    double eulerErr = near(eb.x, 0.3) + near(eb.y, -0.2) + near(eb.z, 2.5);
    expect(eulerErr < 1e-9, "euler round trip error", eulerErr, "rad");

    // Positive pitch = nose down, positive roll = right side down (matches UE stick conventions).
    Vec3 nose = Quat::fromEuler(0, 10 * kDeg, 0).rotate({1, 0, 0});
    Vec3 left = Quat::fromEuler(10 * kDeg, 0, 0).rotate({0, 1, 0});
    expect(nose.z < 0 && left.z > 0, "sign: +pitch nose-down, +roll right-down", nose.z, "");

    // Left-handed conversion must commute with rotation: L(q v) == L(q) L(v)
    Quat qa = Quat::fromEuler(0.4, 0.7, -1.9);
    Vec3 v{1.0, 2.0, -0.5};
    double lhErr = (toLeftHanded(qa.rotate(v)) - toLeftHanded(qa).rotate(toLeftHanded(v))).norm();
    expect(lhErr < 1e-12, "left-handed conversion consistency", lhErr, "");

    // Unreal yaw +90 turns right: right-handed yaw -90 must map to a UE rotation taking +X to +Y.
    Vec3 ueFwd = toLeftHanded(Quat::fromEuler(0, 0, -90 * kDeg)).rotate({1, 0, 0});
    expect(near(ueFwd.y, 1.0) < 1e-9, "RH yaw -90 == UE yaw +90 (faces UE +Y)", ueFwd.y, "");
}

static void scenarioTakeoff(const std::string& out) {
    std::printf("\n[1] Takeoff from ground to 5 m (Position mode)\n");
    Sim sim;
    sim.ctrl.mode = FlightMode::Position;
    sim.ctrl.reset(sim.quad.s);
    sim.ctrl.posSetpoint = {0, 0, 5};
    sim.run(12.0);
    sim.writeCsv(out + "/01_takeoff.csv");

    double peak = 0, t90 = -1;
    for (const auto& l : sim.log) {
        peak = std::max(peak, l.s.pos.z);
        if (t90 < 0 && l.s.pos.z >= 4.5) t90 = l.t;
    }
    double finalErr = near(sim.quad.s.pos.z, 5.0);
    double ts = settlingTime(sim.log, 0, 0.05, [](const Sample& l) { return l.s.pos.z - 5.0; });
    expect(t90 > 0 && t90 < 4.0, "time to 90% altitude", t90, "s");
    expect(peak - 5.0 < 0.25, "altitude overshoot", peak - 5.0, "m");
    expect(ts < 7.0, "settling time (+-5 cm)", ts, "s");
    expect(finalErr < 0.02, "final altitude error", finalErr, "m");
}

static void scenarioRollStep(const std::string& out) {
    std::printf("\n[2] Roll step 0 -> 20 deg (Angle mode, attitude loop response)\n");
    Sim sim;
    sim.startHover(10);
    sim.ctrl.setMode(FlightMode::Angle, sim.quad.s);
    sim.run(0.5);
    sim.input.roll = 20.0 / 30.0;  // 20 deg of the 30 deg full-stick range
    double t0 = sim.t;
    sim.run(2.0);
    sim.writeCsv(out + "/02_roll_step.csv");

    double peak = 0, tRise = -1;
    for (const auto& l : sim.log) {
        if (l.t < t0) continue;
        double r = l.s.att.toEuler().x / kDeg;
        peak = std::max(peak, r);
        if (tRise < 0 && r >= 18.0) tRise = l.t - t0;
    }
    double ts = settlingTime(sim.log, t0, 1.0, [](const Sample& l) { return l.s.att.toEuler().x / kDeg - 20.0; });
    expect(tRise > 0 && tRise < 0.4, "rise time to 90%", tRise, "s");
    expect(peak - 20.0 < 3.0, "overshoot", peak - 20.0, "deg");
    expect(ts < 1.0, "settling time (+-1 deg)", ts, "s");
}

static void scenarioGust(const std::string& out) {
    std::printf("\n[3] Gust rejection: 6 N lateral push for 0.5 s during position hold\n");
    Sim sim;
    sim.startHover(5);
    sim.run(1.0);
    double t0 = sim.t;
    sim.run(6.0, [t0](Sim& s) { s.quad.extForce = (s.t >= t0 && s.t < t0 + 0.5) ? Vec3{0, 6, 0} : Vec3{}; });
    sim.writeCsv(out + "/03_gust.csv");

    double maxDev = 0;
    for (const auto& l : sim.log) maxDev = std::max(maxDev, Vec3{l.s.pos.x, l.s.pos.y, l.s.pos.z - 5}.norm());
    double rec = settlingTime(sim.log, t0, 0.1, [](const Sample& l) { return Vec3{l.s.pos.x, l.s.pos.y, l.s.pos.z - 5}.norm(); });
    expect(maxDev < 0.8, "max position deviation", maxDev, "m");
    expect(rec < 4.0, "recovery time (<10 cm)", rec, "s");
}

static void scenarioWind(const std::string& out) {
    std::printf("\n[4] Steady 5 m/s crosswind, position hold (integrator removes offset)\n");
    Sim sim;
    sim.startHover(5);
    sim.quad.wind = {0, 5, 0};
    sim.run(15.0);
    sim.writeCsv(out + "/04_wind.csv");
    double err = Vec3{sim.quad.s.pos.x, sim.quad.s.pos.y, sim.quad.s.pos.z - 5}.norm();
    double tilt = std::acos(sim.quad.s.att.rotate({0, 0, 1}).z) / kDeg;
    expect(err < 0.05, "steady-state position error", err, "m");
    expect(tilt > 1.0, "holds by leaning into the wind (tilt)", tilt, "deg");
}

static void scenarioWaypoints(const std::string& out) {
    std::printf("\n[5] Waypoint mission: 10 m square at 5 m altitude\n");
    Sim sim;
    sim.startHover(5);
    const Vec3 wps[] = {{10, 0, 5}, {10, 10, 5}, {0, 10, 5}, {0, 0, 5}};
    double total = 0, worstArrive = 0;
    bool allReached = true;
    for (const Vec3& wp : wps) {
        sim.ctrl.posSetpoint = wp;
        double start = sim.t, arrived = -1;
        sim.run(12.0, [&](Sim& s) {
            if (arrived < 0 && (s.quad.s.pos - wp).norm() < 0.3 && s.quad.s.vel.norm() < 0.3) arrived = s.t - start;
        });
        if (arrived < 0) allReached = false;
        worstArrive = std::max(worstArrive, arrived);
        total += arrived;
    }
    sim.writeCsv(out + "/05_waypoints.csv");
    double maxAltErr = 0;
    for (const auto& l : sim.log) maxAltErr = std::max(maxAltErr, near(l.s.pos.z, 5));
    expect(allReached, "all 4 waypoints reached (<0.3 m, <0.3 m/s)", allReached ? 1 : 0, "");
    expect(worstArrive < 8.0, "slowest leg (10 m)", worstArrive, "s");
    expect(maxAltErr < 0.3, "max altitude error during mission", maxAltErr, "m");
    std::printf("  total mission time %.2f s\n", total);
}

static void scenarioNoise(const std::string& out) {
    std::printf("\n[6] Sensor noise: gyro 0.02 rad/s, position 2 cm, velocity 5 cm/s (1 sigma)\n");
    Sim sim;
    sim.gyroNoise = 0.02; sim.posNoise = 0.02; sim.velNoise = 0.05;
    sim.startHover(5);
    sim.run(20.0);
    sim.writeCsv(out + "/06_noise.csv");
    double sumSq = 0, maxTilt = 0;
    int n = 0;
    for (const auto& l : sim.log) {
        if (l.t < 5) continue;
        double e = Vec3{l.s.pos.x, l.s.pos.y, l.s.pos.z - 5}.norm();
        sumSq += e * e; ++n;
        maxTilt = std::max(maxTilt, std::acos(clampd(l.s.att.rotate({0, 0, 1}).z, -1, 1)) / kDeg);
    }
    double rms = std::sqrt(sumSq / n);
    expect(rms < 0.1, "hover position RMS error", rms, "m");
    expect(maxTilt < 5.0, "max tilt jitter", maxTilt, "deg");
}

int main(int argc, char** argv) {
    std::string out = argc > 1 ? argv[1] : ".";
    unitChecks();
    scenarioTakeoff(out);
    scenarioRollStep(out);
    scenarioGust(out);
    scenarioWind(out);
    scenarioWaypoints(out);
    scenarioNoise(out);
    std::printf("\n%s: %d failure(s)\n", gFailures ? "FAILED" : "ALL PASSED", gFailures);
    return gFailures ? 1 : 0;
}
