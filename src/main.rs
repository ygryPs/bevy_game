use bevy::prelude::*;

const FLOOR_Y: f32 = -360.0;
const PLAYER_RADIUS: f32 = 20.0;

fn main() {
    // When building for WASM, print panics to the browser console
    #[cfg(target_arch = "wasm32")]
    console_error_panic_hook::set_once();

    App::new()
        .add_plugins(DefaultPlugins)
        .add_startup_system(setup)
        .add_plugin(PhysicsPlugin)
        .add_system(player_system.before(integrator_before_system))
        .add_system(bevy::window::close_on_esc)
        .run();
}

struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems((
            integrator_before_system,
            gravity_system
                .after(integrator_before_system)
                .before(integrator_after_system),
            integrator_after_system,
            collision_system.after(integrator_after_system),
        ));
    }
}

#[derive(Component)]
struct PhysObj {
    mass: f32,
    vel: Vec2,
    acc: Vec2,
}

#[derive(Component)]
struct Gravity(f32);

impl Default for Gravity {
    fn default() -> Self {
        Gravity(2000.0)
    }
}

#[derive(Component)]
enum Collider {
    Ball {
        radius: f32,
        coef_of_restitution: f32,
        touching_ground: bool,
    },
}

#[derive(Component)]
struct Player {
    jump_impulse: f32,
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    // 2D orthographic camera
    commands.spawn(Camera2dBundle::default());

    // Player ball
    commands.spawn((
        ColorMesh2dBundle {
            mesh: meshes.add(shape::Circle::new(PLAYER_RADIUS).into()).into(),
            material: materials.add(Color::BLUE.into()),
            transform: Transform::from_xyz(0.0, 0.0, 0.0),
            ..default()
        },
        PhysObj {
            mass: 10.0,
            vel: Vec2::ZERO,
            acc: Vec2::ZERO,
        },
        Gravity::default(),
        Collider::Ball {
            radius: PLAYER_RADIUS,
            coef_of_restitution: 0.3,
            touching_ground: false,
        },
        Player {
            jump_impulse: 10_000.0,
        },
    ));
}

fn gravity_system(mut query: Query<(&mut PhysObj, &Gravity)>) {
    for (mut phys_obj, gravity) in &mut query {
        let a = Vec2::NEG_Y * gravity.0;
        phys_obj.acc += a;
    }
}

fn player_system(input: Res<Input<KeyCode>>, mut query: Query<(&Player, &mut PhysObj, &Collider)>) {
    let (
        player,
        mut phys_obj,
        Collider::Ball {
            touching_ground, ..
        },
    ) = query.single_mut();

    if input.pressed(KeyCode::Space) && *touching_ground {
        let dv = Vec2::Y * player.jump_impulse / phys_obj.mass;
        phys_obj.vel += dv;
    }
}

// The part of the integrator that runs before applying forces
fn integrator_before_system(time: Res<Time>, mut query: Query<(&mut Transform, &mut PhysObj)>) {
    let dt = time.delta_seconds();
    for (mut transform, mut phys_obj) in &mut query {
        integrate_before(dt, &mut transform, &mut phys_obj);
    }
}

// The part of the integrator that runs after applying forces
fn integrator_after_system(time: Res<Time>, mut query: Query<&mut PhysObj>) {
    let dt = time.delta_seconds();
    for mut phys_obj in &mut query {
        integrate_after(dt, &mut phys_obj);
    }
}

// The part of the integrator that runs before applying forces
fn integrate_before(dt: f32, transform: &mut Mut<Transform>, phys_obj: &mut Mut<PhysObj>) {
    let dv = 0.5 * phys_obj.acc * dt;
    phys_obj.vel += dv;
    let dx = phys_obj.vel * dt;
    transform.translation += dx.extend(0.0);
    // Functions that calculate acceleration simply add to it so it must be reset every iteration.
    phys_obj.acc = Vec2::ZERO;
}

// The part of the integrator that runs after applying forces
fn integrate_after(dt: f32, phys_obj: &mut Mut<PhysObj>) {
    let dv = 0.5 * phys_obj.acc * dt;
    phys_obj.vel += dv;
}

// Integrator for when acceleration is assumed constant (used in collision resolving)
fn integrate_simple(dt: f32, transform: &mut Mut<Transform>, phys_obj: &mut Mut<PhysObj>) {
    let dv = phys_obj.acc * dt;
    let dx = (phys_obj.vel + 0.5 * dv) * dt;
    transform.translation += dx.extend(0.0);
    phys_obj.vel += dv;
}

fn collision_system(mut query: Query<(&mut Transform, &mut PhysObj, &mut Collider)>) {
    for (mut transform, mut phys_obj, mut collider) in &mut query {
        match *collider {
            Collider::Ball {
                radius: r,
                ref mut touching_ground,
                ..
            } => {
                if transform.translation.y - r <= FLOOR_Y {
                    resolve_collision(&mut transform, &mut phys_obj, &mut collider);
                } else if *touching_ground {
                    *touching_ground = false;
                }
            }
        }
    }
}

fn resolve_collision(
    transform: &mut Mut<Transform>,
    phys_obj: &mut Mut<PhysObj>,
    collider: &mut Mut<Collider>,
) {
    match **collider {
        Collider::Ball {
            radius: r,
            touching_ground: true,
            ..
        } => {
            transform.translation.y = FLOOR_Y + r;
            phys_obj.vel.y = 0.0;
        }
        Collider::Ball {
            radius: r,
            coef_of_restitution,
            ref mut touching_ground,
        } => {
            bounce(transform, phys_obj, r, coef_of_restitution);
            *touching_ground = true;
        }
    }
}

fn bounce(
    transform: &mut Mut<Transform>,
    phys_obj: &mut Mut<PhysObj>,
    r: f32,
    coef_of_restitution: f32,
) {
    let collision_dt = ((transform.translation.y - r) - FLOOR_Y) / phys_obj.vel.y;
    assert!(collision_dt >= 0.0);
    integrate_simple(-collision_dt, transform, phys_obj);
    phys_obj.vel.y *= -coef_of_restitution;
    integrate_simple(collision_dt, transform, phys_obj);
}
