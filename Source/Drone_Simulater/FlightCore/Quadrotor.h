// 6-DOF quadrotor rigid-body model, X configuration.
//
//        front (+x)
//     M1 (CCW)  M2 (CW)
//          \   /
//           [ ]          +y = left
//          /   \
//     M4 (CW)   M3 (CCW)
//
// Each motor: first-order lag on angular speed, thrust T = kT*w^2, reaction torque Q = kQ*w^2.
#pragma once
#include "FcMath.h"

namespace fc {

struct QuadParams {
    double mass = 1.5;                        // kg
    double armLength = 0.225;                 // m, center to motor
    Vec3 inertia{0.0213, 0.0213, 0.0395};     // kg m^2, principal axes
    double kThrust = 1.0e-5;                  // N / (rad/s)^2
    double kTorque = 1.6e-7;                  // N m / (rad/s)^2
    double motorTau = 0.03;                   // s, motor speed time constant
    double omegaMax = 1100.0;                 // rad/s
    double dragLinear = 0.25;                 // N / (m/s)
    double dragQuadratic = 0.05;              // N / (m/s)^2
    double angularDrag = 0.002;               // N m / (rad/s)
    double gravity = 9.81;

    double maxMotorThrust() const { return kThrust * omegaMax * omegaMax; }
    double hoverThrustPerMotor() const { return mass * gravity / 4.0; }
    // Motor offsets along body x/y for an X frame.
    double motorOffset() const { return armLength / std::sqrt(2.0); }
};

struct QuadState {
    Vec3 pos, vel;        // world
    Quat att;             // body -> world
    Vec3 rate;            // body angular rate, rad/s
    double omega[4] = {0, 0, 0, 0};  // motor speeds, rad/s
    bool onGround = true;
};

class Quadrotor {
public:
    QuadParams p;
    QuadState s;
    Vec3 wind;            // world wind velocity, m/s
    Vec3 extForce;        // world disturbance force, N (e.g. gust pulse)

    // +1 = CCW seen from above (body reaction torque is -z).
    static constexpr int kSpin[4] = {+1, -1, +1, -1};

    Vec3 motorPos(int i) const {
        double d = p.motorOffset();
        static const double sx[4] = {+1, +1, -1, -1};
        static const double sy[4] = {+1, -1, -1, +1};
        return {sx[i] * d, sy[i] * d, 0};
    }

    double motorThrust(int i) const { return p.kThrust * s.omega[i] * s.omega[i]; }

    // Advance by dt with commanded motor speeds (rad/s). Semi-implicit Euler; keep dt <= 2 ms.
    void step(const double omegaCmd[4], double dt) {
        // Motors
        for (int i = 0; i < 4; ++i) {
            double target = clampd(omegaCmd[i], 0.0, p.omegaMax);
            s.omega[i] += (target - s.omega[i]) * (dt / (p.motorTau + dt));
        }

        // Body-frame forces and torques
        Vec3 torque;
        double thrust = 0;
        for (int i = 0; i < 4; ++i) {
            double t = motorThrust(i);
            thrust += t;
            torque += motorPos(i).cross(Vec3{0, 0, t});
            torque.z += -kSpin[i] * p.kTorque * s.omega[i] * s.omega[i];
        }
        torque += s.rate * (-p.angularDrag);

        // Translational dynamics (world)
        Vec3 airVel = s.vel - wind;
        double speed = airVel.norm();
        Vec3 drag = airVel * (-(p.dragLinear + p.dragQuadratic * speed));
        Vec3 force = s.att.rotate(Vec3{0, 0, thrust}) + drag + extForce + Vec3{0, 0, -p.mass * p.gravity};
        Vec3 acc = force / p.mass;

        // Rotational dynamics (body): I w_dot = tau - w x I w
        Vec3 Iw = p.inertia.cwise(s.rate);
        Vec3 wdot = torque - s.rate.cross(Iw);
        wdot = {wdot.x / p.inertia.x, wdot.y / p.inertia.y, wdot.z / p.inertia.z};

        s.vel += acc * dt;
        s.rate += wdot * dt;
        s.pos += s.vel * dt;
        Quat dq{0, s.rate.x, s.rate.y, s.rate.z};
        Quat qd = s.att * dq;
        s.att = Quat{s.att.w + 0.5 * qd.w * dt, s.att.x + 0.5 * qd.x * dt,
                     s.att.y + 0.5 * qd.y * dt, s.att.z + 0.5 * qd.z * dt}.normalized();

        // Flat ground at z = 0: no penetration, friction kills horizontal slide and spin.
        // ponytail: ground is a plane; the UE layer handles real level geometry via sweep.
        s.onGround = false;
        if (s.pos.z <= 0.0) {
            s.pos.z = 0.0;
            if (s.vel.z < 0) s.vel.z = 0;
            s.vel.x *= 0.9; s.vel.y *= 0.9;
            s.rate = s.rate * 0.9;
            s.onGround = true;
        }
    }
};

}  // namespace fc
