// Cascaded flight controller: position -> velocity -> attitude -> body rate -> mixer.
//
// Modes
//   Angle    : pilot sets roll/pitch angle, yaw rate and climb rate (sticks centered = hold altitude, level).
//   Position : holds / flies to a world position setpoint (waypoints, position hold).
#pragma once
#include "Quadrotor.h"

namespace fc {

struct Pid3 {
    Vec3 kp, ki, kd;
    Vec3 integ, prevMeas;
    double integLimit = 1e9;
    bool first = true;

    // Derivative on measurement (no kick on setpoint steps). Integrator frozen while saturated.
    Vec3 update(const Vec3& sp, const Vec3& meas, double dt, bool saturated) {
        Vec3 err = sp - meas;
        if (!saturated) {
            integ += err * dt;
            integ = {clampd(integ.x, -integLimit, integLimit), clampd(integ.y, -integLimit, integLimit),
                     clampd(integ.z, -integLimit, integLimit)};
        }
        Vec3 dmeas = first ? Vec3{} : (meas - prevMeas) / dt;
        prevMeas = meas;
        first = false;
        return kp.cwise(err) + ki.cwise(integ) - kd.cwise(dmeas);
    }
    void reset() { integ = {}; first = true; }
};

enum class FlightMode { Angle, Position };

struct PilotInput {       // all in [-1, 1]
    double roll = 0, pitch = 0, yaw = 0, throttle = 0;
};

struct Gains {
    double attP = 7.0;                    // 1/s, attitude error -> rate setpoint
    Vec3 rateP{0.35, 0.35, 0.25};         // N m / (rad/s)
    Vec3 rateI{0.15, 0.15, 0.05};
    Vec3 rateD{0.004, 0.004, 0.0};
    double posP = 1.2;                    // 1/s
    Vec3 velP{2.5, 2.5, 4.0};             // 1/s -> m/s^2
    Vec3 velI{0.8, 0.8, 1.5};
    Vec3 velD{0.0, 0.0, 0.0};
};

struct Limits {
    double maxTilt = 35 * kDeg;
    double maxAngleCmd = 30 * kDeg;     // full stick in Angle mode
    double maxYawRate = 120 * kDeg;
    double maxClimb = 3.0;              // m/s
    double maxHorizSpeed = 6.0;         // m/s
    double maxRate = 220 * kDeg;
};

class Controller {
public:
    Gains g;
    Limits lim;
    FlightMode mode = FlightMode::Angle;
    Vec3 posSetpoint;
    double yawSetpoint = 0;

    // Diagnostics for logging / HUD
    Quat attSetpoint;
    Vec3 rateSetpoint;
    double collective = 0;
    bool saturated = false;

    void reset(const QuadState& st) {
        ratePid.reset(); velPid.reset();
        posSetpoint = st.pos;
        yawSetpoint = st.att.toEuler().z;
    }

    void setMode(FlightMode m, const QuadState& st) {
        if (m == mode) return;
        mode = m;
        velPid.reset();
        posSetpoint = st.pos;
    }

    // Writes motor speed commands (rad/s). Pilot yaw > 0 turns left (CCW, right-handed).
    // `meas` may be a noisy copy of the true state.
    void update(const QuadParams& p, const QuadState& meas, const PilotInput& in, double dt, double omegaOut[4]) {
        initGains();
        Vec3 accSp;  // desired world acceleration (without gravity)
        if (mode == FlightMode::Position) {
            Vec3 velSp = (posSetpoint - meas.pos) * g.posP;
            Vec3 horiz = clampNorm(Vec3{velSp.x, velSp.y, 0}, lim.maxHorizSpeed);
            velSp = {horiz.x, horiz.y, clampd(velSp.z, -lim.maxClimb, lim.maxClimb)};
            accSp = velPid.update(velSp, meas.vel, dt, saturated);
            thrustVectorToAttitude(p, accSp);
        } else {
            yawSetpoint = wrapPi(yawSetpoint + in.yaw * lim.maxYawRate * dt);
            Vec3 velSp{meas.vel.x, meas.vel.y, in.throttle * lim.maxClimb};
            Vec3 a = velPid.update(velSp, meas.vel, dt, saturated);
            double roll = in.roll * lim.maxAngleCmd, pitch = in.pitch * lim.maxAngleCmd;
            attSetpoint = Quat::fromEuler(roll, pitch, yawSetpoint);
            double tilt = meas.att.rotate(Vec3{0, 0, 1}).z;  // measured cos(tilt)
            collective = p.mass * (p.gravity + a.z) / (tilt > 0.5 ? tilt : 0.5);
        }

        // Attitude: quaternion error in body frame -> rate setpoint
        Quat qe = meas.att.conj() * attSetpoint;
        if (qe.w < 0) qe = Quat{-qe.w, -qe.x, -qe.y, -qe.z};
        rateSetpoint = Vec3{qe.x, qe.y, qe.z} * (2.0 * g.attP);
        rateSetpoint = {clampd(rateSetpoint.x, -lim.maxRate, lim.maxRate),
                        clampd(rateSetpoint.y, -lim.maxRate, lim.maxRate),
                        clampd(rateSetpoint.z, -lim.maxRate, lim.maxRate)};

        // Body rate loop -> torque, plus gyroscopic feed-forward
        Vec3 torque = ratePid.update(rateSetpoint, meas.rate, dt, saturated);
        torque += meas.rate.cross(p.inertia.cwise(meas.rate));

        saturated = mix(p, collective, torque, omegaOut);
    }

    // Thrust/torque -> per-motor speed. Returns true if any motor hit a limit.
    static bool mix(const QuadParams& p, double thrust, const Vec3& tau, double omegaOut[4]) {
        double d = p.motorOffset(), c = p.kTorque / p.kThrust;
        double f[4] = {
            thrust / 4 + tau.x / (4 * d) - tau.y / (4 * d) - tau.z / (4 * c),
            thrust / 4 - tau.x / (4 * d) - tau.y / (4 * d) + tau.z / (4 * c),
            thrust / 4 - tau.x / (4 * d) + tau.y / (4 * d) - tau.z / (4 * c),
            thrust / 4 + tau.x / (4 * d) + tau.y / (4 * d) + tau.z / (4 * c),
        };
        // ponytail: plain per-motor clamp; add yaw-first desaturation if aggressive maneuvers need it.
        bool sat = false;
        double fMax = p.maxMotorThrust();
        for (int i = 0; i < 4; ++i) {
            if (f[i] < 0 || f[i] > fMax) sat = true;
            omegaOut[i] = std::sqrt(clampd(f[i], 0.0, fMax) / p.kThrust);
        }
        return sat;
    }

private:
    Pid3 ratePid, velPid;
    bool gainsSet = false;

    void initGains() {
        if (gainsSet) return;
        ratePid.kp = g.rateP; ratePid.ki = g.rateI; ratePid.kd = g.rateD; ratePid.integLimit = 0.3;
        velPid.kp = g.velP; velPid.ki = g.velI; velPid.kd = g.velD; velPid.integLimit = 12.0;  // Ki*limit = 4.8 m/s^2: must exceed steady wind drag (5 m/s wind ~ 1.7 m/s^2)
        gainsSet = true;
    }

    static double wrapPi(double a) {
        while (a > kPi) a -= 2 * kPi;
        while (a < -kPi) a += 2 * kPi;
        return a;
    }

    // Desired acceleration -> tilted thrust vector (limited to maxTilt) + yaw -> attitude setpoint.
    void thrustVectorToAttitude(const QuadParams& p, Vec3 accSp) {
        Vec3 f = (accSp + Vec3{0, 0, p.gravity}) * p.mass;
        if (f.z < 0.1 * p.mass * p.gravity) f.z = 0.1 * p.mass * p.gravity;
        double horiz = std::sqrt(f.x * f.x + f.y * f.y), maxH = f.z * std::tan(lim.maxTilt);
        if (horiz > maxH) { f.x *= maxH / horiz; f.y *= maxH / horiz; }
        Vec3 zb = f.normalized();
        Vec3 xc{std::cos(yawSetpoint), std::sin(yawSetpoint), 0};
        Vec3 yb = zb.cross(xc).normalized();
        Vec3 xb = yb.cross(zb);
        attSetpoint = Quat::fromAxes(xb, yb, zb);
        collective = f.norm();
    }
};

}  // namespace fc
