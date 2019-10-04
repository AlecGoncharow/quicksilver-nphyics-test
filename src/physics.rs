use nalgebra as na;

use na::base::Unit;
use na::Vector2;
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::math::Velocity;
use nphysics2d::object::{
    BodyPartHandle, BodyStatus, ColliderDesc, DefaultBodySet, DefaultColliderSet, RigidBody,
    RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

use ncollide2d::shape::{Ball, Cuboid, ShapeHandle};

// trusting this: https://github.com/rustsim/nphysics/blob/master/src_testbed/testbed.rs#L114
pub struct Physics {
    pub mechanical_world: DefaultMechanicalWorld<f32>,
    pub geometrical_world: DefaultGeometricalWorld<f32>,
    pub bodies: DefaultBodySet<f32>,
    pub colliders: DefaultColliderSet<f32>,
    pub forces: DefaultForceGeneratorSet<f32>,
    pub constraints: DefaultJointConstraintSet<f32>,
}

impl Physics {
    pub fn add_ball(&mut self, x: f32, y: f32, velocity: Vector2<f32>, radius: f32) {
        let rigid_body = RigidBodyDesc::new()
            .translation(Vector2::new(x, y))
            .mass(std::f32::consts::PI * radius.powf(2.0))
            //.angular_inertia(0.1)
            //.angular_damping(5.0)
            .velocity(Velocity::linear(velocity.x, velocity.y))
            .build();

        let rb_handle = self.bodies.insert(rigid_body);

        let shape = ShapeHandle::new(Ball::new(radius));

        let collider = ColliderDesc::new(shape)
            .ccd_enabled(false)
            //.density(5.0)
            .build(BodyPartHandle(rb_handle, 0));

        let _collider_handle = self.colliders.insert(collider);
    }

    pub fn add_wall(&mut self, x: f32, y: f32, width: f32, height: f32) {
        let rigid_body = RigidBodyDesc::new()
            .translation(Vector2::new(x, y))
            .status(BodyStatus::Static)
            .build();

        let rb_handle = self.bodies.insert(rigid_body);

        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(width / 2.0, height / 2.0)));

        let collider = ColliderDesc::new(shape)
            .ccd_enabled(false)
            //.density(1.0)
            .build(BodyPartHandle(rb_handle, 0));

        let _collider_handle = self.colliders.insert(collider);
    }

    pub fn step(&mut self) {
        for contact in self.geometrical_world.contact_events() {
            // this process adapted from https://ncollide.org/collision_detection_pipeline/
            // with a few changes in the API since then, logic for new velocities is the same
            use ncollide2d::pipeline::narrow_phase::ContactEvent;
            match contact {
                ContactEvent::Started(h1, h2) => {
                    if let Some((
                        _col_handle1,
                        collider1,
                        _col_handle2,
                        collider2,
                        _algorithm,
                        manifold,
                    )) = self
                        .geometrical_world
                        .contact_pair(&self.colliders, *h1, *h2, false)
                    {
                        // if either is a box, use normal, else requires some extra maths
                        if collider1.shape_handle().is::<Cuboid<f32>>()
                            || collider2.shape_handle().is::<Cuboid<f32>>()
                        {
                            let deep = match manifold.deepest_contact() {
                                Some(cont) => cont,
                                None => continue,
                            };
                            let normal: Unit<Vector2<f32>> = deep.contact.normal;
                            // totally sane, totally normal
                            {
                                let body1 = self
                                    .bodies
                                    .get_mut(collider1.body())
                                    .unwrap()
                                    .downcast_mut::<RigidBody<f32>>()
                                    .unwrap();

                                let vel1 = body1.velocity().linear.clone();
                                let new_v1 = vel1 - (2.0 * vel1.dot(&normal) * normal.into_inner());
                                body1.set_velocity(Velocity::linear(new_v1.x, new_v1.y));
                            }

                            // round 2
                            {
                                let body2 = self
                                    .bodies
                                    .get_mut(collider2.body())
                                    .unwrap()
                                    .downcast_mut::<RigidBody<f32>>()
                                    .unwrap();

                                let vel2 = body2.velocity().linear.clone();
                                let new_v2 = vel2 - (2.0 * vel2.dot(&normal) * normal.into_inner());
                                body2.set_velocity(Velocity::linear(new_v2.x, new_v2.y))
                            }
                        } else {
                            // adapted from https://stackoverflow.com/a/345863/11015039
                            let body1 = self
                                .bodies
                                .get(collider1.body())
                                .unwrap()
                                .downcast_ref::<RigidBody<f32>>()
                                .unwrap();

                            let body2 = self
                                .bodies
                                .get(collider2.body())
                                .unwrap()
                                .downcast_ref::<RigidBody<f32>>()
                                .unwrap();

                            let mut v1 = body1.velocity().linear.clone();
                            let mut v2 = body2.velocity().linear.clone();
                            let pos1 = body1.position().translation.vector;
                            let pos2 = body2.position().translation.vector;
                            let mass1: f32 = body1.augmented_mass().linear;
                            let mass2: f32 = body2.augmented_mass().linear;

                            let mut collision = pos1 - pos2;
                            let distance = collision.magnitude();

                            collision = collision / distance;

                            let v1ci = v1.dot(&collision);
                            let v2ci = v2.dot(&collision);

                            let v1cf =
                                (v1ci * (mass1 - mass2) + (2.0 * mass2 * v2ci)) / (mass1 + mass2);
                            let v2cf =
                                (v2ci * (mass2 - mass1) + (2.0 * mass1 * v1ci)) / (mass1 + mass2);

                            v1 += (v1cf - v1ci) * collision;
                            v2 += (v2cf - v2ci) * collision;

                            {
                                let body1 = self
                                    .bodies
                                    .get_mut(collider1.body())
                                    .unwrap()
                                    .downcast_mut::<RigidBody<f32>>()
                                    .unwrap();
                                body1.set_velocity(Velocity::linear(v1.x, v1.y));
                            }

                            {
                                let body2 = self
                                    .bodies
                                    .get_mut(collider2.body())
                                    .unwrap()
                                    .downcast_mut::<RigidBody<f32>>()
                                    .unwrap();
                                body2.set_velocity(Velocity::linear(v2.x, v2.y))
                            }
                        }
                    }
                }
                ContactEvent::Stopped(_, _) => {}
            }
        }

        self.mechanical_world.step(
            &mut self.geometrical_world,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.constraints,
            &mut self.forces,
        );
    }
}
