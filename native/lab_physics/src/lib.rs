// rapier_lab-0us: real NIF surface. A stateful physics world lives
// behind a ResourceArc<Mutex<World>>; the Elixir side calls world_new,
// add_body (ball | cuboid), apply_impulse, step, transforms — enough
// to drive one Filament viewport with real physics per tick.

use crossbeam::channel::{unbounded, Receiver, Sender};
use rapier3d::pipeline::ChannelEventCollector;
use rapier3d::prelude::*;
use rustler::{Atom, Encoder, Env, NifResult, ResourceArc, Term};
use std::sync::Mutex;

// ── Physics tuning ──────────────────────────────────────────────────────
//
// Rapier defaults are geared for large-scale physics: 0 damping, 1 kg/m³
// collider density (a shell-sized oblate weighs 45 mg at that density
// and jitters forever under numerical noise). We're modelling
// centimetre-scale dice and cowries, so pick values that match real
// plastic dice / shells and let rapier's built-in auto-sleep bring
// bodies to rest.

/// Density used for every dynamic collider — near the density of ABS
/// plastic (~1050 kg/m³), which puts a d6-sized cuboid at ~1-4 g and
/// an oblate cowrie at ~50 mg. Real-scale mass means damping actually
/// bites and sleep thresholds fire.
const COLLIDER_DENSITY: f32 = 1000.0;

/// Linear damping — bleeds off residual translational motion. Tuned
/// against telemetry: at 0.5 the contact solver's per-step energy
/// injection (from shells stacked against each other) held a
/// steady-state linear speed of ~50–180 mm/s. Damping increases
/// linearly cut that in inverse proportion, so 3.0 puts it under the
/// SLEEP_LINVEL floor.
const LINEAR_DAMPING: f32 = 3.0;

/// Angular damping — dice tumble but shouldn't spin forever. Tuned
/// against on-device telemetry (`MobRapier.Physics.body_states_in/1`):
/// at 10.0, shells resting flat on the ground sleep cleanly but shells
/// stacked on top of neighbours held a 2–5 rad/s residual and never
/// crossed the sleep threshold. 20.0 pushes stacked residuals under
/// SLEEP_ANGVEL too, at the small cost of dice looking a touch less
/// bouncy mid-roll.
const ANGULAR_DAMPING: f32 = 20.0;

// ── Auto-sleep thresholds ───────────────────────────────────────────────
//
// Rapier's built-in auto-sleep is the canonical way to say "this body has
// come to rest" — cheaper than reading transforms and diffing per tick,
// and it also stops the solver from touching the body, which kills the
// contact-driven micro-jitter two shells resting against each other
// otherwise exchange forever.
//
// Rapier's defaults (0.4 m/s linear, 0.5 rad/s angular, 2.0 s dwell) are
// tuned for room-scale bodies. Our shells are ~3 cm across and sit
// against each other with a low-amplitude jitter that exceeds 0.5 rad/s
// under the contact solver. Loosen the angular threshold, tighten the
// linear one to match the tabletop scale, and shorten dwell so a shell
// that briefly quiets down actually falls asleep instead of getting
// woken by the next stray contact event.

/// Linear velocity below which a body qualifies for sleep. Rapier
/// multiplies this by the world's length_unit (default 1 m), so a
/// value of 0.1 = 10 cm/s.
const SLEEP_LINVEL: f32 = 0.1;

/// Angular velocity below which a body qualifies for sleep, in rad/s.
/// Contact solver injects ~1 rad/s of residual jitter into shell-on-shell
/// resting contacts; 1.5 absorbs that without letting spinning dice
/// stop early.
const SLEEP_ANGVEL: f32 = 1.5;

/// Dwell time (seconds) both thresholds must hold before rapier puts the
/// body to sleep. Short so a briefly-quiet body actually sleeps before
/// the next contact wakes it.
const SLEEP_DWELL: f32 = 0.3;

/// Collider friction — plastic-on-wood-ish; enough that a resting
/// shell doesn't drift under gentle contact force from a neighbour.
const COLLIDER_FRICTION: f32 = 0.7;

/// Collider restitution — a small dice bounce, not a superball. Low
/// enough that stacked shells don't perpetually re-inject energy into
/// each other via the contact solver. Kept nonzero so the initial
/// shake still reads as a shake rather than a mud drop.
const COLLIDER_RESTITUTION: f32 = 0.05;

/// Build a dynamic rigid body pre-configured for our
/// centimetre-scale demos: translation set, damping applied so
/// rapier's auto-sleep can catch it once the shake is over.
fn build_dynamic_rb(x: f32, y: f32, z: f32) -> RigidBody {
    let mut rb = RigidBodyBuilder::dynamic()
        .translation(vector![x, y, z])
        .linear_damping(LINEAR_DAMPING)
        .angular_damping(ANGULAR_DAMPING)
        .build();
    // Tabletop-scale sleep thresholds — rapier's defaults are for
    // room-scale bodies; see SLEEP_* constants above for why we override.
    let activation = rb.activation_mut();
    activation.normalized_linear_threshold = SLEEP_LINVEL;
    activation.angular_threshold = SLEEP_ANGVEL;
    activation.time_until_sleep = SLEEP_DWELL;
    rb
}

/// Common tuning for every dynamic collider — density, friction,
/// restitution, and contact-event wiring.
fn tune_collider(builder: ColliderBuilder) -> ColliderBuilder {
    builder
        .density(COLLIDER_DENSITY)
        .friction(COLLIDER_FRICTION)
        .restitution(COLLIDER_RESTITUTION)
        .active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
        .contact_force_event_threshold(0.0)
}

/// Static colliders (walls, ground) still get friction and restitution
/// but no density (mass is irrelevant for fixed bodies).
fn tune_static_collider(builder: ColliderBuilder) -> ColliderBuilder {
    builder
        .friction(COLLIDER_FRICTION)
        .restitution(COLLIDER_RESTITUTION)
        .active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
        .contact_force_event_threshold(0.0)
}

mod atoms {
    rustler::atoms! { ok, started, stopped }
}

// Contact events surface (bead rapier_lab-bvr). We cap the buffer so a
// runaway sim can't OOM the BEAM; over-cap the oldest events are dropped
// silently and reported via `contacts_dropped` on the next drain.
const CONTACT_EVENT_CAP: usize = 1024;

pub struct WorldRes {
    inner: Mutex<World>,
}

#[rustler::resource_impl]
impl rustler::Resource for WorldRes {}

pub struct World {
    rigid_bodies: RigidBodySet,
    colliders: ColliderSet,
    islands: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    gravity: Vector<Real>,
    bodies: Vec<RigidBodyHandle>,
    // Bead rapier_lab-bvr — contact event channels. Kept alive for the
    // world's lifetime so cloned Senders drop when the world drops.
    collision_tx: Sender<CollisionEvent>,
    collision_rx: Receiver<CollisionEvent>,
    force_tx: Sender<ContactForceEvent>,
    force_rx: Receiver<ContactForceEvent>,
    // Events over CONTACT_EVENT_CAP are dropped; report the count so a
    // caller can spot buffer saturation instead of silently losing events.
    contacts_dropped: u32,
}

impl World {
    fn new() -> Self {
        let (collision_tx, collision_rx) = unbounded();
        let (force_tx, force_rx) = unbounded();
        World {
            rigid_bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            islands: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            pipeline: PhysicsPipeline::new(),
            integration_parameters: IntegrationParameters::default(),
            gravity: vector![0.0, -9.81, 0.0],
            bodies: Vec::new(),
            collision_tx,
            collision_rx,
            force_tx,
            force_rx,
            contacts_dropped: 0,
        }
    }

    fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;
        let hooks = ();
        // Bead rapier_lab-bvr: pass a fresh collector each step so the
        // pipeline can push collision + contact-force events into our
        // channels. Cheap: crossbeam Senders are Arc-cloned.
        let events = ChannelEventCollector::new(
            self.collision_tx.clone(),
            self.force_tx.clone(),
        );
        self.pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &hooks,
            &events,
        );
        // Cap the buffered events so a runaway sim doesn't OOM the BEAM.
        // Drain oldest first to keep the newest, which is what an agent
        // asserting on 'the most recent contact' expects.
        while self.collision_rx.len() > CONTACT_EVENT_CAP {
            let _ = self.collision_rx.try_recv();
            self.contacts_dropped = self.contacts_dropped.saturating_add(1);
        }
        while self.force_rx.len() > CONTACT_EVENT_CAP {
            let _ = self.force_rx.try_recv();
            self.contacts_dropped = self.contacts_dropped.saturating_add(1);
        }
    }

    // Look up the body_id (Elixir-side u32, the insertion index) for a
    // ColliderHandle. Static colliders parented to nothing report None —
    // they are the ground plane in the current demos.
    fn body_id_for_collider(&self, collider_handle: ColliderHandle) -> Option<u32> {
        let parent = self.colliders.get(collider_handle)?.parent()?;
        self.bodies
            .iter()
            .position(|h| *h == parent)
            .map(|i| i as u32)
    }
}

#[rustler::nif]
fn ping() -> Atom {
    atoms::ok()
}

/// Creates a new physics world with gravity (0, -9.81, 0) and a static
/// wide ground plane at y = 0. Returns a resource handle callers hold
/// onto for the lifetime of the world.
#[rustler::nif]
fn world_new() -> ResourceArc<WorldRes> {
    let mut world = World::new();

    // Static ground (100 x 0.2 x 100 m, top surface at y = 0).
    let ground = tune_static_collider(ColliderBuilder::cuboid(50.0, 0.1, 50.0))
        .translation(vector![0.0, -0.1, 0.0])
        .build();
    world.colliders.insert(ground);

    ResourceArc::new(WorldRes {
        inner: Mutex::new(world),
    })
}

/// Adds a dynamic ball of `radius` at world `(x, y, z)`. Returns the
/// body index (Elixir-side identifier — u32; kept in insertion order).
#[rustler::nif]
fn add_ball(res: ResourceArc<WorldRes>, x: f32, y: f32, z: f32, radius: f32) -> u32 {
    let mut world = res.inner.lock().unwrap();
    let World {
        rigid_bodies,
        colliders,
        bodies,
        ..
    } = &mut *world;

    let handle = rigid_bodies.insert(build_dynamic_rb(x, y, z));
    // Bead rapier_lab-bvr: emit collision + contact-force events so
    // agents can assert on 'ball landed' / 'first bounce' without diffing
    // transforms.
    let collider = tune_collider(ColliderBuilder::ball(radius)).build();
    colliders.insert_with_parent(collider, handle, rigid_bodies);
    bodies.push(handle);
    (bodies.len() - 1) as u32
}

/// Adds a fixed (static) cuboid — walls, tables, anything the physics
/// should NOT displace. No mass, no impulses, no gravity.
#[rustler::nif]
fn add_static_cuboid(
    res: ResourceArc<WorldRes>,
    x: f32,
    y: f32,
    z: f32,
    hx: f32,
    hy: f32,
    hz: f32,
) -> u32 {
    let mut world = res.inner.lock().unwrap();
    let World {
        rigid_bodies,
        colliders,
        bodies,
        ..
    } = &mut *world;

    let rb = RigidBodyBuilder::fixed()
        .translation(vector![x, y, z])
        .build();
    let handle = rigid_bodies.insert(rb);
    let collider = tune_static_collider(ColliderBuilder::cuboid(hx, hy, hz)).build();
    colliders.insert_with_parent(collider, handle, rigid_bodies);
    bodies.push(handle);
    (bodies.len() - 1) as u32
}

/// Vertex cloud sampling a triaxial ellipsoid surface (semi-axes a, b, c
/// along x/y/z), used by `add_oblate` to feed Rapier's convex-hull builder.
///
/// A `(lat_bands + 1) × lon_bands` UV-sphere parameterisation, then scaled
/// by the semi-axes. For an oblate-of-revolution (a == c) the collision
/// shape is symmetric about the y-axis with a shorter polar radius `b`,
/// which is what settles a cowrie-shell approximation flat-side-down under
/// gravity. 16 × 24 = 400 points is enough for a smooth hull without
/// making the SAT/GJK inner loop pay for it — the convex-hull builder
/// discards interior samples.
fn ellipsoid_hull_points(a: f32, b: f32, c: f32) -> Vec<Point<Real>> {
    const LAT_BANDS: u32 = 16;
    const LON_BANDS: u32 = 24;
    let mut pts = Vec::with_capacity(((LAT_BANDS + 1) * LON_BANDS) as usize);
    for i in 0..=LAT_BANDS {
        let theta = std::f32::consts::PI * (i as f32) / (LAT_BANDS as f32);
        let (st, ct) = theta.sin_cos();
        for j in 0..LON_BANDS {
            let phi = 2.0 * std::f32::consts::PI * (j as f32) / (LON_BANDS as f32);
            let (sp, cp) = phi.sin_cos();
            pts.push(point![a * st * cp, b * ct, c * st * sp]);
        }
    }
    pts
}

/// Adds a dynamic oblate-of-revolution ellipsoid at world `(x, y, z)`, with
/// `equatorial_r` on x/z and `polar_r` on y. The collider is the convex
/// hull of a low-poly ellipsoid mesh — Rapier ships no ellipsoid primitive
/// (bead rapier_lab-gmx), and hulls are the accurate option (a scaled
/// sphere loses the two stable orientations that make cowries actually
/// settle).
///
/// Panics only if the point cloud fails to form a convex hull (Rapier
/// returns `None`), which the fixed 16×24 vertex layout above cannot do
/// as long as both radii are positive and finite.
#[rustler::nif]
fn add_oblate(
    res: ResourceArc<WorldRes>,
    x: f32,
    y: f32,
    z: f32,
    equatorial_r: f32,
    polar_r: f32,
) -> u32 {
    let mut world = res.inner.lock().unwrap();
    let World {
        rigid_bodies,
        colliders,
        bodies,
        ..
    } = &mut *world;

    let handle = rigid_bodies.insert(build_dynamic_rb(x, y, z));
    let points = ellipsoid_hull_points(equatorial_r, polar_r, equatorial_r);
    let collider = tune_collider(
        ColliderBuilder::convex_hull(&points)
            .expect("ellipsoid_hull_points produces a valid convex hull"),
    )
    .build();
    colliders.insert_with_parent(collider, handle, rigid_bodies);
    bodies.push(handle);
    (bodies.len() - 1) as u32
}

/// Adds a dynamic dice-like convex polyhedron at world `(x, y, z)`, built
/// from the given vertex cloud (each `(x, y, z)` scaled by `scale` before
/// hulling). Rapier's `convex_hull` builder computes the actual polyhedron;
/// interior points are dropped, so the caller can pass extras without
/// hurting the collider shape.
///
/// Used for d10 (pentagonal trapezohedron, 12 vertices), d12 (regular
/// dodecahedron, 20 vertices), and d20 (regular icosahedron, 12 vertices) —
/// bead rapier_lab-ry2. The face-up decode lives in `RapierLab.Dice` and
/// uses precomputed face normals; the collider itself only cares about the
/// hull.
#[rustler::nif]
fn add_convex_hull(
    res: ResourceArc<WorldRes>,
    x: f32,
    y: f32,
    z: f32,
    points: Vec<(f32, f32, f32)>,
    scale: f32,
) -> u32 {
    let mut world = res.inner.lock().unwrap();
    let World {
        rigid_bodies,
        colliders,
        bodies,
        ..
    } = &mut *world;

    let scaled: Vec<Point<Real>> = points
        .into_iter()
        .map(|(px, py, pz)| point![px * scale, py * scale, pz * scale])
        .collect();

    let handle = rigid_bodies.insert(build_dynamic_rb(x, y, z));
    let collider = tune_collider(
        ColliderBuilder::convex_hull(&scaled)
            .expect("caller must pass a non-degenerate point cloud"),
    )
    .build();
    colliders.insert_with_parent(collider, handle, rigid_bodies);
    bodies.push(handle);
    (bodies.len() - 1) as u32
}

/// Adds a dynamic cuboid (half-extents hx, hy, hz) at world `(x, y, z)`.
#[rustler::nif]
fn add_cuboid(
    res: ResourceArc<WorldRes>,
    x: f32,
    y: f32,
    z: f32,
    hx: f32,
    hy: f32,
    hz: f32,
) -> u32 {
    let mut world = res.inner.lock().unwrap();
    let World {
        rigid_bodies,
        colliders,
        bodies,
        ..
    } = &mut *world;

    let handle = rigid_bodies.insert(build_dynamic_rb(x, y, z));
    // Same event-active config as add_ball (bead rapier_lab-bvr).
    let collider = tune_collider(ColliderBuilder::cuboid(hx, hy, hz)).build();
    colliders.insert_with_parent(collider, handle, rigid_bodies);
    bodies.push(handle);
    (bodies.len() - 1) as u32
}

/// Adds a linear-velocity impulse (m/s) to a body's centre of mass.
#[rustler::nif]
fn apply_impulse(
    res: ResourceArc<WorldRes>,
    body_id: u32,
    ix: f32,
    iy: f32,
    iz: f32,
) -> NifResult<Atom> {
    let mut world = res.inner.lock().unwrap();
    let Some(handle) = world.bodies.get(body_id as usize).copied() else {
        return Ok(atoms::ok());
    };
    if let Some(rb) = world.rigid_bodies.get_mut(handle) {
        rb.apply_impulse(vector![ix, iy, iz], true);
    }
    Ok(atoms::ok())
}

/// Applies an angular impulse to a body. Needed for dice + shell demos:
/// spinning objects need torque, not just linear force.
#[rustler::nif]
fn apply_torque_impulse(
    res: ResourceArc<WorldRes>,
    body_id: u32,
    tx: f32,
    ty: f32,
    tz: f32,
) -> NifResult<Atom> {
    let mut world = res.inner.lock().unwrap();
    let Some(handle) = world.bodies.get(body_id as usize).copied() else {
        return Ok(atoms::ok());
    };
    if let Some(rb) = world.rigid_bodies.get_mut(handle) {
        rb.apply_torque_impulse(vector![tx, ty, tz], true);
    }
    Ok(atoms::ok())
}

/// Steps the world forward by `dt` seconds.
#[rustler::nif]
fn step(res: ResourceArc<WorldRes>, dt: f32) -> Atom {
    res.inner.lock().unwrap().step(dt);
    atoms::ok()
}

/// Reads every body's `{id, {x, y, z}, {qx, qy, qz, qw}}`. Called each
/// frame to drive the scene.
#[rustler::nif]
fn transforms<'a>(env: Env<'a>, res: ResourceArc<WorldRes>) -> Term<'a> {
    let world = res.inner.lock().unwrap();
    let list: Vec<(u32, (f32, f32, f32), (f32, f32, f32, f32))> = world
        .bodies
        .iter()
        .enumerate()
        .filter_map(|(ix, handle)| {
            let rb = world.rigid_bodies.get(*handle)?;
            let t = rb.translation();
            let r = rb.rotation();
            Some((
                ix as u32,
                (t.x, t.y, t.z),
                (r.i, r.j, r.k, r.w),
            ))
        })
        .collect();
    list.encode(env)
}

/// Full per-body telemetry — the debug/observability surface for agents
/// and IEx. Encoded as an `%MobRapier.Physics.BodyState{}` struct so it
/// reads legibly in IEx and stays additive as new fields land.
#[derive(rustler::NifStruct)]
#[module = "MobRapier.Physics.BodyState"]
struct BodyState {
    id: u32,
    // Position of the body's centre in world coordinates, metres.
    pos: (f32, f32, f32),
    // Orientation quaternion {qx, qy, qz, qw}.
    quat: (f32, f32, f32, f32),
    // Linear velocity in world coordinates, m/s.
    linvel: (f32, f32, f32),
    // Angular velocity in world coordinates, rad/s.
    angvel: (f32, f32, f32),
    // |linvel| — one-glance "is it translating".
    speed: f32,
    // |angvel| — one-glance "is it spinning".
    ang_speed: f32,
    // Intrinsic Z-Y-X Tait-Bryan angles (yaw about world +Y, pitch about
    // world +Z after yaw, roll about world +X after pitch), radians.
    // Yaw / pitch / roll are the intuitive "which way is it facing / how
    // tilted is it" the settle-detection debugging cares about.
    euler: (f32, f32, f32),
    // Body-local +Y axis projected into world coords. For a die this is
    // the top face when euler == 0; for a cowrie, +Y = dorsal (convex)
    // side. Sign of the y component tells you convex-up vs concave-up
    // without recomputing per-shape face tables.
    up_axis: (f32, f32, f32),
    // Rapier's own auto-sleep verdict — the ground-truth "at rest" flag
    // the solver itself gates on. Prefer this over hand-tuned velocity
    // thresholds.
    sleeping: bool,
}

fn quat_to_euler(qx: f32, qy: f32, qz: f32, qw: f32) -> (f32, f32, f32) {
    // Intrinsic Z-Y-X (yaw around Y, pitch around Z, roll around X)
    // Tait-Bryan angles from a unit quaternion. Matches the aerospace
    // ZYX convention rotated for a Y-up world.
    let sin_pitch = 2.0 * (qw * qz - qx * qy);
    let pitch = if sin_pitch.abs() >= 1.0 {
        (std::f32::consts::FRAC_PI_2).copysign(sin_pitch)
    } else {
        sin_pitch.asin()
    };
    let yaw = (2.0 * (qw * qy + qx * qz)).atan2(1.0 - 2.0 * (qy * qy + qz * qz));
    let roll = (2.0 * (qw * qx + qy * qz)).atan2(1.0 - 2.0 * (qx * qx + qz * qz));
    (yaw, pitch, roll)
}

fn rotate_by_quat(v: (f32, f32, f32), q: (f32, f32, f32, f32)) -> (f32, f32, f32) {
    // v' = q v q⁻¹, expanded out.
    let (vx, vy, vz) = v;
    let (qx, qy, qz, qw) = q;
    let t2 = 2.0 * (qy * vz - qz * vy);
    let t3 = 2.0 * (qz * vx - qx * vz);
    let t4 = 2.0 * (qx * vy - qy * vx);
    (
        vx + qw * t2 + qy * t4 - qz * t3,
        vy + qw * t3 + qz * t2 - qx * t4,
        vz + qw * t4 + qx * t3 - qy * t2,
    )
}

/// Reads every body's full state — position, orientation, linvel, angvel,
/// derived speeds + Euler angles + body-local +Y in world, and rapier's
/// own `is_sleeping()` flag.
///
/// Prefer this over diffing two `transforms/1` calls to infer velocity:
/// it reads directly from rapier's `RigidBody::linvel()`, `angvel()`, and
/// `is_sleeping()`, which is the ground truth the auto-sleep + solver
/// themselves gate on. Also useful for face-up-style decoding without
/// per-shape tables — the up_axis field is just the body's +Y in world
/// coords, so its y component's sign says whether the die's top face is
/// facing up or down.
#[rustler::nif]
fn body_states<'a>(env: Env<'a>, res: ResourceArc<WorldRes>) -> Term<'a> {
    let world = res.inner.lock().unwrap();
    let list: Vec<BodyState> = world
        .bodies
        .iter()
        .enumerate()
        .filter_map(|(ix, handle)| {
            let rb = world.rigid_bodies.get(*handle)?;
            let t = rb.translation();
            let r = rb.rotation();
            let lv = rb.linvel();
            let av = rb.angvel();
            let quat = (r.i, r.j, r.k, r.w);
            let euler = quat_to_euler(quat.0, quat.1, quat.2, quat.3);
            let up_axis = rotate_by_quat((0.0, 1.0, 0.0), quat);
            let speed = (lv.x * lv.x + lv.y * lv.y + lv.z * lv.z).sqrt();
            let ang_speed = (av.x * av.x + av.y * av.y + av.z * av.z).sqrt();
            Some(BodyState {
                id: ix as u32,
                pos: (t.x, t.y, t.z),
                quat,
                linvel: (lv.x, lv.y, lv.z),
                angvel: (av.x, av.y, av.z),
                speed,
                ang_speed,
                euler,
                up_axis,
                sleeping: rb.is_sleeping(),
            })
        })
        .collect();
    list.encode(env)
}

/// Drains and returns every collision + contact-force event accumulated
/// since the last call.
///
/// Returns `{collisions, forces, dropped}`:
///
///   collisions :: [{body_a :: u32, body_b :: u32, :started | :stopped}]
///   forces     :: [{body_a, body_b, force_magnitude, {fx, fy, fz}}]
///   dropped    :: u32  # cap-overflow tally since last call
///
/// Bead rapier_lab-bvr. Colliders parented to nothing (the static ground)
/// report body_id = 4_294_967_295 (`u32::MAX`) so a caller can still filter
/// on 'body A hit the ground' without lookup gymnastics.
#[rustler::nif]
fn contacts<'a>(env: Env<'a>, res: ResourceArc<WorldRes>) -> Term<'a> {
    let mut world = res.inner.lock().unwrap();

    let mut collisions: Vec<(u32, u32, Atom)> = Vec::new();
    while let Ok(ev) = world.collision_rx.try_recv() {
        let a = world.body_id_for_collider(ev.collider1()).unwrap_or(u32::MAX);
        let b = world.body_id_for_collider(ev.collider2()).unwrap_or(u32::MAX);
        let kind = if ev.started() { atoms::started() } else { atoms::stopped() };
        collisions.push((a, b, kind));
    }

    let mut forces: Vec<(u32, u32, f32, (f32, f32, f32))> = Vec::new();
    while let Ok(ev) = world.force_rx.try_recv() {
        let a = world.body_id_for_collider(ev.collider1).unwrap_or(u32::MAX);
        let b = world.body_id_for_collider(ev.collider2).unwrap_or(u32::MAX);
        let f = ev.total_force;
        forces.push((a, b, ev.total_force_magnitude, (f.x, f.y, f.z)));
    }

    let dropped = world.contacts_dropped;
    world.contacts_dropped = 0;

    (collisions, forces, dropped).encode(env)
}

/// Legacy smoke — kept so MainScreen's mount check doesn't have to
/// change until the demo screen replaces it.
#[rustler::nif]
fn smoke_drop() -> f32 {
    let mut world = World::new();
    world
        .colliders
        .insert(ColliderBuilder::cuboid(50.0, 0.1, 50.0).build());

    let handle = {
        let World {
            rigid_bodies,
            colliders,
            ..
        } = &mut world;

        let h = rigid_bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(vector![0.0, 5.0, 0.0])
                .build(),
        );
        colliders.insert_with_parent(
            ColliderBuilder::ball(1.0).restitution(0.5).build(),
            h,
            rigid_bodies,
        );
        h
    };

    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }
    world.rigid_bodies[handle].translation().y
}

rustler::init!("Elixir.MobRapier.Physics");
