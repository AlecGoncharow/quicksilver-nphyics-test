use quicksilver::{
    Future,
    geom::{Circle, Rectangle, Vector},
    combinators::result,
    graphics::{Background::Col, Color, Background::Img, Font, FontStyle, Image},
    lifecycle::{run, Settings, State, Window, Asset},
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

const WIDTH_LOCAL: f32 = 1200.0;
const HEIGHT_LOCAL: f32 = 900.0;

struct DrawGeometry {
    physics: Physics,
    prev_pt: Vector2<f32>,
    initial_rect_pt: Vector2<f32>,
    rad: f32,
    left_down: bool,
    right_down: bool,
    is_wall: bool,
    state_text: Asset<Image>,
}

impl DrawGeometry {
    fn get_rect(&self, x: f32, y: f32) -> (f32, f32, f32, f32) {
        let half_x = if self.initial_rect_pt.x < x {
            (x - self.initial_rect_pt.x)/2.0
        } else {
           (self.initial_rect_pt.x - x)/2.0
        };

        let center_x = if self.initial_rect_pt.x < x {
            self.initial_rect_pt.x + half_x
        } else {
            x + half_x
        };

        let half_y = if self.initial_rect_pt.y < y {
            (y - self.initial_rect_pt.y)/2.0
        } else {
            (self.initial_rect_pt.y - y)/2.0
        };

        let center_y = if self.initial_rect_pt.y < y {
            self.initial_rect_pt.y + half_y
        } else {
            y + half_y
        };

        return (center_x, center_y, half_x, half_y)
    }

}

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

        // bottom
        physics.add_wall(WIDTH_LOCAL / 2.0, 10.0, WIDTH_LOCAL, 20.0);
        // top
        physics.add_wall(WIDTH_LOCAL / 2.0, HEIGHT_LOCAL - 50.0, WIDTH_LOCAL, 20.0);
        // left
        physics.add_wall(10.0, HEIGHT_LOCAL / 2.0, 20.0, HEIGHT_LOCAL);
        // right
        physics.add_wall(WIDTH_LOCAL - 10.0, HEIGHT_LOCAL / 2.0, 20.0, HEIGHT_LOCAL);

        let state_text = Asset::new(Font::load("font.ttf")
            .and_then(|font| {
                let style = FontStyle::new(50.0, Color::WHITE);
                result(font.render("STATE: BALL", &style))
            }));

        // Load/create resources here: images, fonts, sounds, etc.
        Ok(DrawGeometry {
            physics,
            prev_pt: Vector2::new(0.0, 0.0),
            left_down: false,
            rad: 0.0,
            initial_rect_pt: Vector2::new(0.0, 0.0),
            right_down: false,
            is_wall: false,
            state_text,
        })
    }

    fn update(&mut self, _window: &mut Window) -> Result<()> {
        self.physics.step();
        Ok(())
    }

    fn event(&mut self, event: &Event, window: &mut Window) -> Result<()> {
        match event {
            Event::MouseButton(button, state) => match button {
                MouseButton::Left => {
                    let mouse = window.mouse();
                    let x = mouse.pos().x;
                    let y = mouse.pos().y;
                    match state {
                        ButtonState::Pressed if !self.left_down => {
                            self.left_down = true;
                            if self.is_wall {
                                self.initial_rect_pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
                            } else {
                                let mut rng = rand::thread_rng();
                                let scale: f32 = rng.gen();
                                //println!("Mouse button pressed: {:?}, x: {}, y: {}", button, x, y);
                                self.rad = 5.0 + (15.0 * scale);
                                self.prev_pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
                            }
                        }

                        ButtonState::Released if self.left_down => {
                            self.left_down = false;
                            if self.is_wall {
                                //println!("Mouse button released: {:?}, x: {}, y: {}", button, x, y);
                                let (center_x, center_y, half_x, half_y) = self.get_rect(x, HEIGHT_LOCAL - y);

                                self.physics.add_wall(center_x, center_y, half_x * 2.0, half_y * 2.0);
                            } else {

                                //println!("Mouse button released: {:?}, x: {}, y: {}", button, x, y);
                                let pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
                                let mut vel = pt - self.prev_pt;
                                vel *= 2.0;
                                self.physics
                                    .add_ball(self.prev_pt.x, self.prev_pt.y, vel, self.rad);
                            }
                        }

                        _ => (),
                    }
                }
                MouseButton::Right => {
                    let mouse = window.mouse();
                    let x = mouse.pos().x;
                    let y = mouse.pos().y;

                     match state {
                        ButtonState::Pressed if !self.right_down => {
                            self.right_down = true;
                            //println!("Mouse button pressed: {:?}, x: {}, y: {}", button, x, y);
                            self.initial_rect_pt = na::base::Vector2::new(x, HEIGHT_LOCAL - y);
                        }

                        ButtonState::Released if self.right_down => {
                            self.right_down = false;
                            //println!("Mouse button released: {:?}, x: {}, y: {}", button, x, y);
                            let (center_x, center_y, half_x, half_y) = self.get_rect(x, HEIGHT_LOCAL - y);

                            self.physics.add_wall(center_x, center_y, half_x * 2.0, half_y * 2.0);
                        }

                        _ => (),
                    }
                }
                _ => (),
            }
            Event::Key(key, _state) => match key {
                Key::W => {
                    self.is_wall = true;

                    self.state_text = Asset::new(Font::load("font.ttf")
                        .and_then(|font| {
                            let style = FontStyle::new(50.0, Color::WHITE);
                            result(font.render("STATE: WALL", &style))
                        }));
                }
                Key::B => {
                    self.is_wall = false;

                    self.state_text = Asset::new(Font::load("font.ttf")
                        .and_then(|font| {
                            let style = FontStyle::new(50.0, Color::WHITE);
                            result(font.render("STATE: BALL", &style))
                        }));
                }
                _ => ()
            }
            _ => (),
        }
        Ok(())
    }

    fn draw(&mut self, window: &mut Window) -> Result<()> {
        window.clear(Color::BLACK)?;

        self.state_text.execute(|image| {
            window.draw(&image.area().with_center((500, 20)), Img(&image));
            Ok(())
        })?;

        // draw starting pos of ball if still dragging velocity
        if self.left_down && !self.is_wall {
            let vec = Vector2::new(self.prev_pt.x, HEIGHT_LOCAL - self.prev_pt.y);

            window.draw(&Circle::new((vec.x, vec.y), self.rad), Col(Color::CYAN));
        }

        if self.right_down || (self.left_down && self.is_wall) {
            let mouse = window.mouse();
            let x = mouse.pos().x;
            let y = mouse.pos().y;

            let (center_x, center_y, half_x, half_y) = self.get_rect(x, HEIGHT_LOCAL - y);

            window.draw(
                &Rectangle::new((center_x - half_x, HEIGHT_LOCAL - center_y - half_y), (half_x * 2.0, half_y * 2.0)),
                Col(Color::CYAN),
            );

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
    run::<DrawGeometry>("Balls And Walls", Vector::new(WIDTH_LOCAL, HEIGHT_LOCAL), Settings::default());
}
