#include "integrator.h"

#include "configs.h"

void ExplicitEuler::integrate(const std::vector<Particles *> &particles, std::function<void(void)>) const {
  // TODO: Integrate velocity and acceleration
  //   1. Integrate velocity.
  //   2. Integrate acceleration.
  //   3. You should not compute position using acceleration. Since some part only update velocity. (e.g. impulse)
  // Note:
  //   1. You don't need the simulation function in explicit euler.
  //   2. You should do this first because it is very simple. Then you can chech your collision is correct or not.
  //   3. This can be done in 2 lines. (Hint: You can add / multiply all particles at once since it is a large matrix.)
  for (const auto &p : particles) {
    // (1) p point to cloth.particle  (2) p point to spheres.particle
    // Main idea is to update its velocity & position
    // formular: x(t+h) = x(t) + h * f(x,t) 
    p->position() += deltaTime * p->velocity();
    p->velocity() += deltaTime * p->acceleration();
  }
}

void ImplicitEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // Write code here!
  Eigen::Matrix4Xf cl_origin_position = particles[0]->position();
  Eigen::Matrix4Xf cl_origin_velocity = particles[0]->velocity();
  Eigen::Matrix4Xf cl_origin_acceleration = particles[0]->acceleration();

  Eigen::Matrix4Xf sp_origin_position = particles[1]->position();
  Eigen::Matrix4Xf sp_origin_velocity = particles[1]->velocity();
  Eigen::Matrix4Xf sp_origin_acceleration = particles[1]->acceleration();

  // 1. move the position&velocity of particle to future position
  particles[0]->position() += deltaTime * particles[0]->velocity();
  particles[1]->position() += deltaTime * particles[1]->velocity();
  particles[0]->velocity() += deltaTime * particles[0]->acceleration();
  particles[1]->velocity() += deltaTime * particles[1]->acceleration();

  // 2. modify the status of particle via simulateOneStep
  simulateOneStep();

  // 3. get  velocity_future acceleration_future
  Eigen::Matrix4Xf cl_future_velocity = particles[0]->velocity();
  Eigen::Matrix4Xf sp_future_velocity = particles[1]->velocity();
  Eigen::Matrix4Xf cl_future_acceleration = particles[0]->acceleration();
  Eigen::Matrix4Xf sp_future_acceleration = particles[1]->acceleration();

  // 4. Recover particles' status
  particles[0]->position() = cl_origin_position;
  particles[0]->velocity() = cl_origin_velocity;
  particles[0]->acceleration() = cl_origin_acceleration;

  particles[1]->position() = sp_origin_position;
  particles[1]->velocity() = sp_origin_velocity;
  particles[1]->acceleration() = sp_origin_acceleration;

  // 5. Refined position & velocity
  particles[0]->position() += deltaTime * cl_future_velocity;
  particles[0]->velocity() += deltaTime * cl_future_acceleration;

  particles[1]->position() += deltaTime * sp_future_velocity;
  particles[1]->velocity() += deltaTime * sp_future_acceleration;
}

void MidpointEuler::integrate(const std::vector<Particles *> &particles,
                              std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Integrate velocity and acceleration using explicit euler to get Xn+1.
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // (1) p point to cloth.particle  (2) p point to spheres.particle
  // Main idea is to update its velocity & position
  // formular: x(t+h) = x(t) + h * velocity_future

  // Main idea: How to get velocity_future?
  // 0. Save original particles' data. (position / velocity / acceleration)
  Eigen::Matrix4Xf cl_origin_position = particles[0]->position();
  Eigen::Matrix4Xf cl_origin_velocity = particles[0]->velocity();
  Eigen::Matrix4Xf cl_origin_acceleration = particles[0]->acceleration();

  Eigen::Matrix4Xf sp_origin_position = particles[1]->position();
  Eigen::Matrix4Xf sp_origin_velocity = particles[1]->velocity();
  Eigen::Matrix4Xf sp_origin_acceleration = particles[1]->acceleration();

  // 1. move the position&velocity of particle to future position
  particles[0]->position() += deltaTime / 2 * particles[0]->velocity();
  particles[1]->position() += deltaTime / 2 * particles[1]->velocity();
  particles[0]->velocity() += deltaTime / 2 * particles[0]->acceleration();
  particles[1]->velocity() += deltaTime / 2 * particles[1]->acceleration();

  // 2. modify the status of particle via simulateOneStep
  simulateOneStep();

  // 3. get  velocity_future acceleration_future
  Eigen::Matrix4Xf cl_future_velocity = particles[0]->velocity();
  Eigen::Matrix4Xf sp_future_velocity = particles[1]->velocity();
  Eigen::Matrix4Xf cl_future_acceleration = particles[0]->acceleration();
  Eigen::Matrix4Xf sp_future_acceleration = particles[1]->acceleration();

  // 4. Recover particles' status
  particles[0]->position() = cl_origin_position;
  particles[0]->velocity() = cl_origin_velocity;
  particles[0]->acceleration() = cl_origin_acceleration;

  particles[1]->position() = sp_origin_position;
  particles[1]->velocity() = sp_origin_velocity;
  particles[1]->acceleration() = sp_origin_acceleration;

  // 5. Refined position & velocity
  particles[0]->position() += deltaTime * cl_future_velocity;
  particles[0]->velocity() += deltaTime * cl_future_acceleration;

  particles[1]->position() += deltaTime * sp_future_velocity;
  particles[1]->velocity() += deltaTime * sp_future_acceleration;
}

void RungeKuttaFourth::integrate(const std::vector<Particles *> &particles,
                                 std::function<void(void)> simulateOneStep) const {
  // TODO: Integrate velocity and acceleration
  //   1. Backup original particles' data.
  //   2. Compute k1, k2, k3, k4
  //   3. Compute refined Xn+1 using (1.) and (2.).
  // Note:
  //   1. Use simulateOneStep with modified position and velocity to get Xn+1.

  // x(t+h) = x(t) + h * velocity
  // x(t+h) = x(t) + 1/6 (k1+ 2k2 + 2k3 +k4)
  // 1/6 (k1+ 2k2 + 2k3 +k4) => The main idea is to get the velocity / acceleration

  // 0. Save original particles' data. (position / velocity / acceleration)
  Eigen::Matrix4Xf cl_origin_position = particles[0]->position();
  Eigen::Matrix4Xf cl_origin_velocity = particles[0]->velocity();
  Eigen::Matrix4Xf cl_origin_acceleration = particles[0]->acceleration();

  Eigen::Matrix4Xf sp_origin_position = particles[1]->position();
  Eigen::Matrix4Xf sp_origin_velocity = particles[1]->velocity();
  Eigen::Matrix4Xf sp_origin_acceleration = particles[1]->acceleration();

  // 1. compute k1
  Eigen::Matrix4Xf cl_k1_position = deltaTime * particles[0]->velocity();
  Eigen::Matrix4Xf cl_k1_velocity = deltaTime * particles[0]->acceleration();
  Eigen::Matrix4Xf sp_k1_position = deltaTime * particles[1]->velocity();
  Eigen::Matrix4Xf sp_k1_velocity = deltaTime * particles[1]->acceleration();

  particles[0]->position() += cl_k1_position / 2;
  particles[0]->velocity() += cl_k1_velocity / 2;
  particles[1]->position() += sp_k1_position / 2;
  particles[1]->velocity() += sp_k1_velocity / 2;

  // simulateOneStep() to get new velocity & acceleration
  simulateOneStep();

  // 2. compute k2
  Eigen::Matrix4Xf cl_k2_position = deltaTime * particles[0]->velocity();
  Eigen::Matrix4Xf cl_k2_velocity = deltaTime * particles[0]->acceleration();
  Eigen::Matrix4Xf sp_k2_position = deltaTime * particles[1]->velocity();
  Eigen::Matrix4Xf sp_k2_velocity = deltaTime * particles[1]->acceleration();

  // 3. Recover particles' status & move
  particles[0]->position() = cl_origin_position;
  particles[0]->velocity() = cl_origin_velocity;
  particles[0]->acceleration() = cl_origin_acceleration;

  particles[1]->position() = sp_origin_position;
  particles[1]->velocity() = sp_origin_velocity;
  particles[1]->acceleration() = sp_origin_acceleration;

  particles[0]->position() += cl_k2_position / 2;
  particles[0]->velocity() += cl_k2_velocity / 2;
  particles[1]->position() += sp_k2_position / 2;
  particles[1]->velocity() += sp_k2_velocity / 2;

  // simulateOneStep() to get new velocity & acceleration
  simulateOneStep();

  // 4. compute k3
  Eigen::Matrix4Xf cl_k3_position = deltaTime * particles[0]->velocity();
  Eigen::Matrix4Xf cl_k3_velocity = deltaTime * particles[0]->acceleration();
  Eigen::Matrix4Xf sp_k3_position = deltaTime * particles[1]->velocity();
  Eigen::Matrix4Xf sp_k3_velocity = deltaTime * particles[1]->acceleration();

  // 5. Recover particles' status & move
  particles[0]->position() = cl_origin_position;
  particles[0]->velocity() = cl_origin_velocity;
  particles[0]->acceleration() = cl_origin_acceleration;

  particles[1]->position() = sp_origin_position;
  particles[1]->velocity() = sp_origin_velocity;
  particles[1]->acceleration() = sp_origin_acceleration;

  particles[0]->position() += cl_k3_position;
  particles[0]->velocity() += cl_k3_velocity;
  particles[1]->position() += sp_k3_position;
  particles[1]->velocity() += sp_k3_velocity;

  // simulateOneStep() to get new velocity & acceleration
  simulateOneStep();

  // 6. compute k4
  Eigen::Matrix4Xf cl_k4_position = deltaTime * particles[0]->velocity();
  Eigen::Matrix4Xf cl_k4_velocity = deltaTime * particles[0]->acceleration();
  Eigen::Matrix4Xf sp_k4_position = deltaTime * particles[1]->velocity();
  Eigen::Matrix4Xf sp_k4_velocity = deltaTime * particles[1]->acceleration();

  // 7. Recover particles' status & move
  particles[0]->position() = cl_origin_position;
  particles[0]->velocity() = cl_origin_velocity;
  particles[0]->acceleration() = cl_origin_acceleration;

  particles[1]->position() = sp_origin_position;
  particles[1]->velocity() = sp_origin_velocity;
  particles[1]->acceleration() = sp_origin_acceleration;

  // 8. Final Computation
  particles[0]->position() += (cl_k1_position + 2 * cl_k2_position + 2 * cl_k3_position + cl_k4_position) / 6;
  particles[0]->velocity() += (cl_k1_velocity + 2 * cl_k2_velocity + 2 * cl_k3_velocity + cl_k4_velocity) / 6;
  particles[1]->position() += (sp_k1_position + 2 * sp_k2_position + 2 * sp_k3_position + sp_k4_position) / 6;
  particles[1]->velocity() += (sp_k1_velocity + 2 * sp_k2_velocity + 2 * sp_k3_velocity + sp_k4_velocity) / 6;
}
