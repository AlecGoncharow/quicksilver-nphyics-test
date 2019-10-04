use quicksilver::{
    geom::{Circle, Rectangle, Vector},
    graphics::{Background::Col, Color},
    lifecycle::{run, Settings, State, Window},
    prelude::*,
    Result,
};

use nalgebra as na;

use na::Vector2;
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::object::{DefaultBodySet, DefaultColliderSet, RigidBody};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

use ncollide2d::shape::{Ball, Cuboid};

mod physics;
use physics::Physics;
use quicksilver::prelude::MouseButton;
use rand::Rng;

const WIDTH_LOCAL: f32 = 800.0;
const HEIGHT_LOCAL: f32 = 600.0;

struct DrawGeometry {
    physics: Physics,
    prev_pt: Vector2<f32>,
    rad: f32,
    mouse_down: bool,
}

// trusting this: https://github.com/rustsim/nphysics/blob/master/src_testbed/testbed.rs#L114

impl State for DrawGeometry {
    fn new() -> Result<DrawGeometry> {
        let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, 0.0));
        let geometrical_world = DefaultGeometricalWorld::new();

        let bodies = DefaultBodySet::new();
        let colliders = DefaultColliderSet::new();
        let constraints = DefaultJointConstraintSet::new();
        let forces = DefaultForceGeneratorSet::new();

        let mut physics = Physics {
            mechanical_world,
            geometrical_world,
            bodies,
            colliders,
            forces,
            constraints,
        };

        physics.add_wall(200.0, 200.0, 50.0, 50.0);
        physics.add_wall(500.0, 300.0, 50.0, 200.0);
        physics.add_wall(300.0, 400.0, 50.0, 100.0);
        physics.add_wall(600.0, 100.0, 100.0, 50.0);
        physics.add_wall(100.0, 500.0, 100.0, 50.0);

        // bottom
        physics.add_wall(WIDTH_LOCAL / 2.0, 10.0, WIDTH_LOCAL, 20.0);
        // top
        physics.add_wall(WIDTH_LOCAL / 2.0, HEIGHT_LOCAL - 10.0, WIDTH_LOCAL, 20.0);
        // left
        physics.add_wall(10.0, HEIGHT_LOCAL / 2.0, 20.0, HEIGHT_LOCAL);
        // right
        physics.add_wall(WIDTH_LOCAL - 10.0, HEIGHT_LOCAL / 2.0, 20.0, HEIGHT_LOCAL);

        // Load/create resources here: images, fonts, sounds, etc.
        Ok(DrawGeometry {
            physics,
            prev_pt: Vector2::new(0.0, 0.0),
            mouse_down: false,
            rad: 0.0,
        })
    }

    fn event(&mut self, event: &Event, window: &mut Window) -> Result<()> {
        match event {
            Event::MouseButton(button, state) => match button {
                MouseButton::Left => {
                    let mouse = window.mouse();
                    let x = mouse.pos().x;
                    let y = mouse.pos().y;
                    match state {
                        ButtonState::Pressed if !self.mouse_down => {
                            self.mouse_down = true;
                            let mut rng = rand::thread_rng();
                            let scale: f32 = rng.gen();
                            println!("Mouse button pressed: {:?}, x: {}, y: {}", button, x, y);
                            self.rad = 5.0 + (15.0 * scale);
                            self.prev_pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
                        }

                        ButtonState::Released if self.mouse_down => {
                            self.mouse_down = false;
                            println!("Mouse button released: {:?}, x: {}, y: {}", button, x, y);
                            let pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
                            let mut vel = pt - self.prev_pt;
                            vel *= 2.0;
                            self.physics
                                .add_ball(self.prev_pt.x, self.prev_pt.y, vel, self.rad);
                        }

                        _ => (),
                    }
                }
                _ => (),
            },
            _ => (),
        }
        Ok(())
    }

    fn update(&mut self, _window: &mut Window) -> Result<()> {
        self.physics.step();
        Ok(())
    }

    fn draw(&mut self, window: &mut Window) -> Result<()> {
        window.clear(Color::BLACK)?;

        // draw starting pos of ball if still dragging velocity
        if self.mouse_down {
            let vec = Vector2::new(self.prev_pt.x, HEIGHT_LOCAL - self.prev_pt.y);

            window.draw(&Circle::new((vec.x, vec.y), self.rad), Col(Color::CYAN));
        }

        for (handle, body) in self.physics.bodies.iter() {
            let down = body.downcast_ref::<RigidBody<f32>>().unwrap();
            let pos = down.position().translation.vector;
            //println!("{:?}", pos);
            let coll = self.physics.colliders.get(handle).unwrap();
            let is_ball = match coll.shape().downcast_ref::<Ball<f32>>() {
                None => false,
                _ => true,
            };

            let mut vec = Vector2::new(pos.x, HEIGHT_LOCAL - pos.y);
            if is_ball {
                let shp = coll.shape_handle().as_shape::<Ball<f32>>().unwrap();

                window.draw(
                    &Circle::new((vec.x, vec.y), shp.radius()),
                    Col(Color::WHITE),
                );
            } else {
                let box_coll = coll.shape().downcast_ref::<Cuboid<f32>>().unwrap();
                let half_ext = box_coll.half_extents();

                let x = vec.x - half_ext.x;
                let y = vec.y - half_ext.y;
                vec.x = x;
                vec.y = y;
                window.draw(
                    &Rectangle::new((vec.x, vec.y), (half_ext.x * 2.0, half_ext.y * 2.0)),
                    Col(Color::WHITE),
                );
            }
        }

        Ok(())
    }
}

fn main() {
    run::<DrawGeometry>("Draw Geometry", Vector::new(800, 600), Settings::default());
}
