//! Joining the deferred BVH pass used to wait for another Rayon worker to pick up the queued
//! job, so a step run from a pool whose other workers were busy stalled until one of them
//! came free, and hung for good when that worker was waiting on the step itself.
// The pool API this drives is compiled out by `unsync-callbacks`.
#![cfg(all(feature = "parallel", not(feature = "unsync-callbacks")))]

use rapier3d::prelude::*;
use rapier3d::rayon;
use std::sync::mpsc;
use std::time::Duration;

/// The scene below runs in milliseconds when healthy, so this only has to separate
/// "running" from "wedged".
const TIMEOUT: Duration = Duration::from_secs(60);
const STEPS: usize = 64;
const LEAVES: usize = 640;
const MOVED: usize = LEAVES / 32;

/// A grid of fixed boxes far enough apart that nothing touches, so the narrow phase and
/// solver stay on the calling thread and only the deferred BVH pass needs a worker.
fn scene() -> (PhysicsWorld, Vec<RigidBodyHandle>) {
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
    (world, bodies)
}

/// Steps from inside a two-worker pool while the other worker is parked until the
/// stepping is done.
#[test]
fn step_completes_while_the_other_worker_is_busy() {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(2)
        .build()
        .unwrap();
    let (parked_tx, parked_rx) = mpsc::channel();
    let (release_tx, release_rx) = mpsc::channel::<()>();
    let (done_tx, done_rx) = mpsc::channel();

    std::thread::spawn(move || {
        pool.install(move || {
            rayon::spawn(move || {
                parked_tx.send(()).unwrap();
                let _ = release_rx.recv();
            });
            parked_rx
                .recv_timeout(Duration::from_secs(10))
                .expect("the second worker did not start");

            let (mut world, bodies) = scene();
            for step in 0..STEPS {
                // Moving a few boxes every step builds up the optimizer debt that makes
                // the broad-phase defer a pass.
                for body in &bodies[..MOVED] {
                    let mut position = world.bodies[*body].translation();
                    position.y = (step % 2) as f32 * 0.5;
                    world.bodies[*body].set_translation(position, true);
                }
                world.step();
            }
            let _ = done_tx.send(());
        });
    });

    let result = done_rx.recv_timeout(TIMEOUT);
    // Free the parked worker either way, so a stalled step can finish and exit.
    let _ = release_tx.send(());
    match result {
        Ok(()) => {}
        Err(mpsc::RecvTimeoutError::Timeout) => {
            panic!("step() made no progress for {TIMEOUT:?} while the pool's other worker was busy")
        }
        Err(mpsc::RecvTimeoutError::Disconnected) => panic!("stepping thread died"),
    }
}
