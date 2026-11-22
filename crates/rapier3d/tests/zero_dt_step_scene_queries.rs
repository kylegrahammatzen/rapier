//! A step with `dt <= 0.0` returned without putting back the broad-phase tree that a deferred
//! optimization pass had moved out, so every scene query missed every collider, including the
//! ones that did not move, until the next full step.

use rapier3d::prelude::*;

fn ray_down_at(world: &PhysicsWorld, x: f32, z: f32) -> Option<ColliderHandle> {
    let ray = Ray::new(Vector::new(x, 8.0, z), Vector::new(0.0, -1.0, 0.0));
    world
        .cast_ray(&ray, 20.0, true, QueryFilter::default())
        .map(|(handle, _)| handle)
}

fn zero_length_step(world: &mut PhysicsWorld) {
    let dt = world.integration_parameters.dt;
    world.integration_parameters.dt = 0.0;
    world.step();
    world.integration_parameters.dt = dt;
}

/// One moved collider out of two triggers the optimizer in the step that sees the move.
#[test]
fn zero_length_step_after_a_teleport_keeps_every_collider_queryable() {
    for body in [
        RigidBodyBuilder::fixed(),
        RigidBodyBuilder::kinematic_position_based(),
        RigidBodyBuilder::dynamic(),
    ] {
        let mut world = PhysicsWorld::new();
        let (moved_body, moved) = world.insert(body, ColliderBuilder::cuboid(0.5, 0.5, 0.5));
        let (_, untouched) = world.insert(
            RigidBodyBuilder::fixed().translation(Vector::new(-5.0, 0.0, 0.0)),
            ColliderBuilder::cuboid(0.5, 0.5, 0.5),
        );
        world.step();

        world.bodies[moved_body].set_translation(Vector::new(20.0, 0.0, 0.0), true);
        zero_length_step(&mut world);

        assert_eq!(
            ray_down_at(&world, 20.0, 0.0),
            Some(moved),
            "moved collider at its new pose"
        );
        assert_eq!(
            ray_down_at(&world, 0.0, 0.0),
            None,
            "moved collider gone from its old pose"
        );
        assert_eq!(
            ray_down_at(&world, -5.0, 0.0),
            Some(untouched),
            "untouched collider"
        );
    }
}

/// The optimizer can also run in a step with no user changes: every eighth broad-phase update,
/// once the in-place leaf updates since the last pass reach 1/64 of the leaves. Shift the frame
/// alignment so one of the zero-length steps lands on that update.
#[test]
fn consecutive_zero_length_steps_keep_every_collider_queryable() {
    const LEAVES: usize = 640;
    const BULK: usize = LEAVES / 4;
    const MOVED: usize = LEAVES / 32;

    for offset in 0..8 {
        for zero_length_steps in 1..=3 {
            let mut world = PhysicsWorld::new();
            let mut bodies = Vec::with_capacity(LEAVES);
            for i in 0..LEAVES {
                let position = Vector::new((i % 32) as f32 * 4.0, 0.0, (i / 32) as f32 * 4.0);
                let (body, _) = world.insert(
                    RigidBodyBuilder::fixed().translation(position),
                    ColliderBuilder::cuboid(0.5, 0.5, 0.5),
                );
                bodies.push(body);
            }
            world.step();
            for _ in 0..offset {
                world.step();
            }

            // Many user changes in one step: the next updates use in-place leaf updates, which
            // count toward the optimizer.
            for body in &bodies[..BULK] {
                let mut position = world.bodies[*body].translation();
                position.y = 0.25;
                world.bodies[*body].set_translation(position, true);
            }
            world.step();

            for body in &bodies[..MOVED] {
                let mut position = world.bodies[*body].translation();
                position.y = 0.5;
                world.bodies[*body].set_translation(position, true);
            }
            for _ in 0..zero_length_steps {
                zero_length_step(&mut world);
            }

            let last = world.bodies[bodies[LEAVES - 1]].translation();
            assert!(
                ray_down_at(&world, last.x, last.z).is_some(),
                "untouched collider missing after {zero_length_steps} zero-length step(s) \
                 at frame offset {offset}"
            );
        }
    }
}
