// Runtime safety monitor, independent of the controller's internals.
// It reads only what sensors report (measured state, position-valid flag) and the mixer saturation flag,
// and overrides the controller's mode/setpoints: Hold -> (after holdTime) Land -> motors off on the ground.
// Loss of control stops the motors immediately (flight termination).
#pragma once
#include "Controller.h"

namespace fc {

enum class SafetyAction { None, Hold, Land, Terminate };  // ordered by severity; the monitor never steps down

inline const char* actionName(SafetyAction a) {
    switch (a) {
        case SafetyAction::Hold: return "HOLD";
        case SafetyAction::Land: return "LAND";
        case SafetyAction::Terminate: return "TERMINATE";
        default: return "OK";
    }
}

struct SafetyLimits {
    double tiltLimit = 45 * kDeg;         // controller clamps its setpoint at 35 deg; beyond 45 something is wrong
    double lossOfControlTilt = 70 * kDeg;
    double lossOfControlTime = 0.3;       // s above lossOfControlTilt -> stop motors
    double geofenceRadius = 30;           // m, horizontal distance from home
    double ceiling = 30;                  // m above home
    double floorAlt = 0;                  // m above home; 0 = off. Armed once the vehicle has been above it.
    double saturationTime = 1.0;          // s of continuous motor saturation
    double holdTime = 3.0;                // s of Hold before descending
    double landRate = 1.0;                // m/s
};

class SafetyMonitor {
public:
    SafetyLimits lim;
    Vec3 home;
    SafetyAction action = SafetyAction::None;
    const char* reason = "";
    double triggerTime = -1;              // monitor time (s) at the first trigger
    bool motorsOff = false;

    void reset(const Vec3& homePos) {
        SafetyLimits keep = lim;
        *this = SafetyMonitor{};
        lim = keep;
        home = homePos;
    }

    // Call at controller rate, before Controller::update.
    void update(const QuadState& meas, bool posValid, bool saturated, double dt) {
        elapsed += dt;
        double tilt = std::acos(clampd(meas.att.rotate({0, 0, 1}).z, -1.0, 1.0));
        tiltTimer = tilt > lim.lossOfControlTilt ? tiltTimer + dt : 0;
        satTimer = saturated ? satTimer + dt : 0;
        Vec3 d = meas.pos - home;
        if (lim.floorAlt > 0 && d.z > lim.floorAlt) floorArmed = true;

        if (tiltTimer > lim.lossOfControlTime) escalate(SafetyAction::Terminate, "loss of control");
        if (tilt > lim.tiltLimit) escalate(SafetyAction::Hold, "tilt limit");
        if (!posValid) escalate(SafetyAction::Land, "position lost");  // Hold needs position: descend directly
        if (posValid && std::sqrt(d.x * d.x + d.y * d.y) > lim.geofenceRadius) escalate(SafetyAction::Hold, "geofence");
        if (d.z > lim.ceiling) escalate(SafetyAction::Hold, "ceiling");
        if (floorArmed && d.z < lim.floorAlt) escalate(SafetyAction::Hold, "altitude floor");
        if (satTimer > lim.saturationTime) escalate(SafetyAction::Hold, "motor saturation");
        if (action == SafetyAction::Hold && elapsed - levelTime > lim.holdTime) escalate(SafetyAction::Land, reason);

        // Landed = no vertical motion for 1 s while commanded down (touchdown, or resting on level geometry).
        stillTimer = (action == SafetyAction::Land && std::abs(meas.vel.z) < 0.1) ? stillTimer + dt : 0;
        if (stillTimer > 1.0 || action == SafetyAction::Terminate) motorsOff = true;
    }

    // Overrides the controller and pilot input for the active action. Caller zeros motors if motorsOff.
    void command(Controller& c, PilotInput& in, const QuadState& meas, bool posValid) {
        if (action == SafetyAction::None || motorsOff) return;
        in = PilotInput{};
        if (action == SafetyAction::Hold) {
            if (!holdLatched) { c.setMode(FlightMode::Position, meas); c.posSetpoint = meas.pos; holdLatched = true; }
            return;
        }
        if (posValid) {  // Land straight down from the held (or current) point
            if (c.mode != FlightMode::Position) { c.setMode(FlightMode::Position, meas); c.posSetpoint = meas.pos; }
            c.posSetpoint.z = meas.pos.z - lim.landRate / c.g.posP;
        } else {         // No position: level attitude, climb-rate loop only; the wind carries the vehicle
            c.setMode(FlightMode::Angle, meas);
            in.throttle = -lim.landRate / c.lim.maxClimb;
        }
    }

private:
    double elapsed = 0, levelTime = 0, tiltTimer = 0, satTimer = 0, stillTimer = 0;
    bool floorArmed = false, holdLatched = false;

    void escalate(SafetyAction a, const char* why) {
        if (a <= action) return;
        if (action == SafetyAction::None) triggerTime = elapsed;
        action = a;
        reason = why;
        levelTime = elapsed;
    }
};

}  // namespace fc
