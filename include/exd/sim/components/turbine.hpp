#pragma once
// ─────────────────────────────────────────────────────────────────────
// TurbineSpec — ECS component carrying the parametric wind-turbine rotor
// design. Plain data only (POD): satisfies the exd::ecs::Component
// concept (trivially movable, trivially destructible, no owning
// pointers), so it can live on an entity in a hot per-frame view.
//
// Writers:   TurbineSystem's panel, OptimizationSystem (apply_best_design)
// Readers:   TurbineSystem (mesh rebuild + rotor spin)
// Entity:    the "Turbine" entity created by TurbineSystem.
// ─────────────────────────────────────────────────────────────────────
namespace exd::sim {

struct TurbineSpec {
    int   blade_count = 3;       // blades per rotor
    float radius      = 4.0f;    // blade tip radius              [m]
    float hub_radius  = 0.35f;   // root (hub) radius             [m]
    float axial_chord = 1.0f;    // LE→TE axial extent at root    [m]
    float tip_taper   = 0.55f;   // TE axial extent at tip / root [-]
    float pitch_deg   = 2.0f;    // collective pitch, any angle    [deg]
    float twist_deg   = 22.0f;   // root-to-tip twist ramp (can go negative) [deg]
    float thickness   = 0.12f;   // thickness-to-chord ratio      [t/c]
    float rpm         = 20.0f;   // visual rotor speed            [rpm]
    float yaw_deg     = 0.0f;    // rotor-plane yaw               [deg]

    // Rotor sense: with the default camber the convex side faces +theta, so
    // CCW spin (viewed from upwind +Z) extracts energy (turbine) and CW spin
    // pushes air (fan). Verify before changing defaults: see
    // BladeSection::stagger in extropian-geometry.
    int   rotor_sense  = 0;      // 0 = turbine (CCW from upwind), 1 = fan (CW)

    // Hub / center body (extropian-geometry HubShape).
    int   hub_shape   = 0;       // 0 Spinner, 1 Bullet, 2 Cylinder,
                                 // 3 Tapered, 4 FlatDisk
    float hub_nose    = 0.60f;   // hub extent forward of rotor plane [m]
    float hub_aft     = 0.40f;   // hub extent behind rotor plane     [m]
};

} // namespace exd::sim
