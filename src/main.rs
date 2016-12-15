//! Runs an nphysics integration demo

extern crate amethyst;
extern crate cgmath;
extern crate rand;
extern crate nalgebra;
extern crate ncollide;
extern crate nphysics3d;
extern crate obj;

use std::sync::{Mutex, Arc};

use amethyst::config::Element;
use amethyst::context::Context;
use amethyst::ecs::{Component, Join, Processor, RunArg, VecStorage, World};
use amethyst::engine::{Application, State, Trans};
use amethyst::processors::rendering::{RenderingProcessor, Renderable, Light, Camera, Projection};
use amethyst::processors::transform::{TransformProcessor, LocalTransform, Transform, Child, Init};
use amethyst::renderer::Light as RendererLight;
use amethyst::context::asset_manager::{Mesh, Texture};
use nalgebra::Vector3;
use ncollide::shape::{Ball as PhysicsBall, Plane};
use nphysics3d::object::{RigidBody, RigidBodyHandle};
use nphysics3d::world::World as PhysicsWorld;


struct BallFall;

struct PhysicsWorldRef {
    pub world: PhysicsWorld<f32>
}

impl PhysicsWorldRef {
    pub fn new(world: PhysicsWorld<f32>) -> PhysicsWorldRef {
        PhysicsWorldRef {
            world: world
        }
    }
}

impl Component for PhysicsWorldRef {
    type Storage = VecStorage<PhysicsWorldRef>;
}

unsafe impl Sync for PhysicsWorldRef { }
unsafe impl Send for PhysicsWorldRef { }

struct Ball {
    pub body: RigidBodyHandle<f32>,
}

impl Ball {
    pub fn new(body: RigidBodyHandle<f32>) -> Ball {
        Ball {
            body: body
        }
    }
}

unsafe impl Sync for Ball { }
unsafe impl Send for Ball { }

impl Component for Ball {
    type Storage = VecStorage<Ball>;
}

struct BallFallProcessor;

unsafe impl Sync for BallFallProcessor {  }

impl Processor<Arc<Mutex<Context>>> for BallFallProcessor {
    fn run(&mut self, arg: RunArg, ctx: Arc<Mutex<Context>>) {

        // Get all needed component storages and resources
        let ctx = ctx.lock().unwrap();
        let (mut balls,
             mut locals,
             mut worlds) = arg.fetch(|w| (
                 w.write::<Ball>(),
                 w.write::<LocalTransform>(),
                 w.write::<PhysicsWorldRef>()
        ));


        let delta_time = ctx.delta_time.subsec_nanos() as f32 / 1.0e9;

        for world in (&mut worlds).iter() {
            world.world.step(delta_time);
        }

        for (ball, local) in (&mut balls, &mut locals).iter() {
            let position = ball.body.borrow().position().translation;
            local.translation[0] = position[0];
            local.translation[1] = position[1];
            local.translation[2] = position[2];
        }
    }
}

impl State for BallFall {
    fn on_start(&mut self, ctx: &mut Context, world: &mut World) {
        let (w, h) = ctx.renderer.get_dimensions().unwrap();
        let aspect = w as f32 / h as f32;

        // Get an Orthographic projection
        let projection = Projection::Perspective {
            fov: 60.0,
            aspect: aspect,
            near: 1.0,
            far: 100.0,
        };

        world.add_resource::<Projection>(projection.clone());

        // Create a camera entity
        let eye =    [0., 50., 0.];
        let target = [0.,  0., 0.];
        let up =     [0.,  0., 1.];
        let mut camera = Camera::new(projection, eye, target, up);
        camera.activate();
        world.create_now()
            .with(camera)
            .build();


        let light = Light::new(RendererLight {
            color: [1.0, 1.0, 1.0, 1.0],
            radius: 5.0,
            center: [0.0, 5.0, 2.0],
            propagation_constant: 0.2,
            propagation_linear: 0.2,
            propagation_r_square: 0.6,
        });
        world.create_now()
            .with(light)
            .build();

        // Generate a square mesh
        ctx.asset_manager.register_asset::<Mesh>();
        ctx.asset_manager.register_asset::<Texture>();

        ctx.asset_manager.create_constant_texture("green", [0.1, 0.7, 0.1, 1.]);
        ctx.asset_manager.create_constant_texture("blue", [0.1, 0.1, 0.7, 1.]);
        ctx.asset_manager.gen_sphere("sphere", 16, 16);
        let sphere = Renderable::new("sphere", "blue", "green");

        // Set up physics world
        let mut physics_world_ref = PhysicsWorldRef::new(PhysicsWorld::new());
        physics_world_ref.world.set_gravity(Vector3::new(0.0, 0.0, -9.81));

        let mut rb = RigidBody::new_static(PhysicsBall::new(1.0), 0.3, 0.6);
        rb.append_translation(&Vector3::new(0.1, 0.0, -5.0));
        physics_world_ref.world.add_rigid_body(rb);

        let planes = vec![
            (Vector3::new( 1.0,  0.0, 0.0), Vector3::new(-15.0,   0.0,   0.0)),
            (Vector3::new(-1.0,  0.0, 0.0), Vector3::new( 15.0,   0.0,   0.0)),
            (Vector3::new( 0.0,  1.0, 0.0), Vector3::new(  0.0, -15.0,   0.0)),
            (Vector3::new( 0.0, -1.0, 0.0), Vector3::new(  0.0,  15.0,   0.0)),
            (Vector3::new( 0.0,  0.0, 1.0), Vector3::new(  0.0,   0.0, -15.0)),
        ];

        for plane in planes.iter() {
            let mut rb = RigidBody::new_static(Plane::new(plane.0), 0.3, 0.6);
            rb.append_translation(&plane.1);
            physics_world_ref.world.add_rigid_body(rb);
        }

        for i in 0..100 {
            let mut rb = RigidBody::new_dynamic(PhysicsBall::new(1.0), 1.0, 0.3, 0.6);
            rb.append_translation(&Vector3::new(rand::random::<f32>(), rand::random::<f32>(), i as f32 * 5.0));
            let handle = physics_world_ref.world.add_rigid_body(rb);

            let ball = Ball::new(handle);
            world.create_now()
                .with(sphere.clone())
                .with(ball)
                .with(LocalTransform::default())
                .with(Transform::default())
                .build();
        }

        world.create_now()
            .with(physics_world_ref)
            .build();
    }

    fn update(&mut self, ctx: &mut Context, _: &mut World) -> Trans {
        // Exit if user hits Escape or closes the window
        use amethyst::context::event::{EngineEvent, Event, VirtualKeyCode};
        let engine_events = ctx.broadcaster.read::<EngineEvent>();
        for engine_event in engine_events.iter() {
            match engine_event.payload {
                Event::KeyboardInput(_, _, Some(VirtualKeyCode::Escape)) => return Trans::Quit,
                Event::Closed => return Trans::Quit,
                _ => (),
            }
        }

        Trans::None
    }
}

fn main() {
    use amethyst::engine::Config;
    let path = format!("{}/resources/config.yml", env!("CARGO_MANIFEST_DIR"));
    let config = Config::from_file(path).unwrap();

    let mut ctx = Context::new(config.context_config);
    let rendering_processor = RenderingProcessor::new(config.renderer_config, &mut ctx);
    let mut game = Application::build(BallFall, ctx)
                   .with::<RenderingProcessor>(rendering_processor, "rendering_processor", 0)
                   .register::<Renderable>()
                   .register::<Light>()
                   .register::<Camera>()
                   .with::<BallFallProcessor>(BallFallProcessor, "ball_fall_processor", 1)
                   .register::<Ball>()
                   .register::<PhysicsWorldRef>()
                   .with::<TransformProcessor>(TransformProcessor::new(), "transform_processor", 2)
                   .register::<LocalTransform>()
                   .register::<Transform>()
                   .register::<Child>()
                   .register::<Init>()
                   .done();
    game.run();
}
