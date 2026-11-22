//! A body locked to a plane (no movement along z, no turning about x or y) used to lose all
//! its friction and slide as if on ice, because the locked tangent has no effective mass and
//! the coupled 2×2 friction solve zeroed both tangents when that matrix became singular. On a
//! tilted floor the matrix also missed the linear coupling between the tangents.

use rapier3d::prelude::*;

/// How far a unit cube resting on a floor tilted by `tilt` degrees (about z) slides after a
/// 3 m/s shove downhill.
fn slide(locked_to_plane: bool, model: FrictionModel, settle_first: bool, tilt: f32) -> f32 {
    let mut world = PhysicsWorld::new();
    world.integration_parameters.friction_model = model;

    let angle = tilt.to_radians();
    let up = Vector::new(-angle.sin(), angle.cos(), 0.0);
    let downhill = Vector::new(-angle.cos(), -angle.sin(), 0.0);
    let shove = downhill * 3.0;

    let ground = world.bodies.insert(
        RigidBodyBuilder::fixed()
            .translation(-up * 0.5)
            .rotation(Vector::new(0.0, 0.0, angle)),
    );
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(20.0, 0.5, 20.0).friction(0.5),
        ground,
        &mut world.bodies,
    );

    let mut body = RigidBodyBuilder::dynamic()
        .translation(up * 0.5)
        .rotation(Vector::new(0.0, 0.0, angle));
    if !settle_first {
        body = body.linvel(shove);
    }
    if locked_to_plane {
        body = body
            .enabled_translations(true, true, false)
            .enabled_rotations(false, false, true);
    }
    let handle = world.bodies.insert(body);
    world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5, 0.5).friction(0.5),
        handle,
        &mut world.bodies,
    );

    if settle_first {
        for _ in 0..30 {
            world.step();
        }
        world.bodies[handle].set_linvel(shove, true);
    }
    let start = world.bodies[handle].translation();
    for _ in 0..180 {
        world.step();
    }
    (world.bodies[handle].translation() - start).dot(downhill)
}

#[test]
fn plane_locked_body_keeps_its_friction() {
    // Report every case before judging, to see which ones break.
    let mut bad = Vec::new();
    for tilt in [0.0, 5.0] {
        for model in [FrictionModel::Simplified, FrictionModel::Coulomb] {
            for settle_first in [true, false] {
                let free = slide(false, model, settle_first, tilt);
                let locked = slide(true, model, settle_first, tilt);
                let case = format!("tilt {tilt}° {model:?} settle_first={settle_first}");
                println!("{case}: free {free:.2} m, locked {locked:.2} m");
                // v² / (2g(μ·cos − sin)) is 0.92 m flat and 1.1 m at 5°, both well below 2 m.
                if free >= 2.0 || locked >= 2.0 || (free - locked).abs() >= 0.25 {
                    bad.push(format!("{case}: free {free} m, locked {locked} m"));
                }
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}
