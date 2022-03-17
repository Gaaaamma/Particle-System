#include "cloth.h"
#include <Eigen/Geometry>
#include <iostream>
#include "configs.h"
#include "sphere.h"

Cloth::Cloth() : Shape(particlesPerEdge * particlesPerEdge, particleMass) {
  initializeVertex();
  initializeSpring();
}

void Cloth::draw(DrawType type) const {
  vao.bind();
  positionBuffer.load(0, 4 * particlesPerEdge * particlesPerEdge * sizeof(GLfloat), _particles.getPositionData());
  const ElementArrayBuffer* currentEBO = nullptr;
  switch (type) {
    case DrawType::PARTICLE: [[fallthrough]];
    case DrawType::FULL: currentEBO = &ebo; break;
    case DrawType::STRUCTURAL: currentEBO = &structuralSpring; break;
    case DrawType::SHEAR: currentEBO = &shearSpring; break;
    case DrawType::BEND: currentEBO = &bendSpring;
  }
  currentEBO->bind();
  GLsizei indexCount = static_cast<GLsizei>(currentEBO->size() / sizeof(GLuint));
  if (type == DrawType::FULL)
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
  else if (type == DrawType::PARTICLE)
    glDrawArrays(GL_POINTS, 0, particlesPerEdge * particlesPerEdge);
  else
    glDrawElements(GL_LINES, indexCount, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Cloth::initializeVertex() {
  float wStep = 2.0f * clothWidth / (particlesPerEdge - 1);
  float hStep = 2.0f * clothHeight / (particlesPerEdge - 1);

  int current = 0;
  for (int i = 0; i < particlesPerEdge; ++i) {
    for (int j = 0; j < particlesPerEdge; ++j) {
      _particles.position(current++) = Eigen::Vector4f(-clothWidth + j * wStep, 0, -clothHeight + i * hStep, 1);
    }
  }
  // Four corners will not move
  _particles.mass(0) = 0.0f;
  _particles.mass(particlesPerEdge - 1) = 0.0f;
  _particles.mass(particlesPerEdge * (particlesPerEdge - 1)) = 0.0f;
  _particles.mass(particlesPerEdge * particlesPerEdge - 1) = 0.0f;

  std::vector<GLuint> indices;
  indices.reserve((particlesPerEdge - 1) * (2 * particlesPerEdge + 1));
  for (int i = 0; i < particlesPerEdge - 1; ++i) {
    int offset = i * (particlesPerEdge);
    for (int j = 0; j < particlesPerEdge - 1; ++j) {
      indices.emplace_back(offset + j);
      indices.emplace_back(offset + j + particlesPerEdge);
      indices.emplace_back(offset + j + 1);

      indices.emplace_back(offset + j + 1);
      indices.emplace_back(offset + j + particlesPerEdge);
      indices.emplace_back(offset + j + particlesPerEdge + 1);
    }
  }

  int vboSize = particlesPerEdge * particlesPerEdge * sizeof(GLfloat);
  positionBuffer.allocate_load(vboSize * 4, _particles.getPositionData());
  normalBuffer.allocate(particlesPerEdge * particlesPerEdge * sizeof(float) * 4);

  ebo.allocate_load(indices.size() * sizeof(GLuint), indices.data());

  vao.bind();
  positionBuffer.bind();
  vao.enable(0);
  vao.setAttributePointer(0, 4, 4, 0);
  normalBuffer.bind();
  vao.enable(1);
  vao.setAttributePointer(1, 4, 4, 0);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Cloth::initializeSpring() {
  // TODO: Connect particles with springs.
  //   1. Compute spring length per type.
  //   2. Iterate the particles. Push spring objects into `_springs` vector
  // Note:
  //   1. The particles index:
  //   ===============================================
  //   0 1 2 3 ... particlesPerEdge
  //   particlesPerEdge + 1 ....
  //   ... ... particlesPerEdge * particlesPerEdge - 1
  //   ===============================================
  // Here is a simple example which connects the horizontal structrual springs.
  float structrualLength = (_particles.position(0) - _particles.position(1)).norm();
  
  // Handle STRUCTURE TYPE ->horizontal
  for (int i = 0; i < particlesPerEdge; ++i) {
    for (int j = 0; j < particlesPerEdge - 1; ++j) {
      int index = i * particlesPerEdge + j;
      _springs.emplace_back(index, index + 1, structrualLength, Spring::Type::STRUCTURAL);
    }
  }
  // Handle Vertical
  for (int i = 0; i < (particlesPerEdge * particlesPerEdge - particlesPerEdge); ++i) {
    // Each string is responsible for the particle below it.
    _springs.emplace_back(i, i + particlesPerEdge, structrualLength, Spring::Type::STRUCTURAL);
  }

  // Handle BEND TYPE -> Horizontal
  float bendLength = (_particles.position(0) - _particles.position(2)).norm();
  for (int i = 0; i < particlesPerEdge; ++i) {
    for (int j = 0; j < particlesPerEdge - 2; ++j) {
      int index = i * particlesPerEdge + j;
      _springs.emplace_back(index, index + 2, bendLength, Spring::Type::BEND);
    }
  }
  // Handle Vertical
  for (int i = 0; i < (particlesPerEdge * (particlesPerEdge - 2)); ++i) {
    // Each string is responsible for the particle below it.
    _springs.emplace_back(i, i + particlesPerEdge * 2, bendLength, Spring::Type::BEND);
  }

  // Handle SHEAR TYPE -> right & below
  float shearLength = (_particles.position(0) - _particles.position(particlesPerEdge+1)).norm();
  for (int i = 0; i < particlesPerEdge - 1; ++i) {
    for (int j = 0; j < particlesPerEdge - 1; ++j) {
      int index = i * particlesPerEdge + j;
      _springs.emplace_back(index, index + particlesPerEdge + 1, shearLength, Spring::Type::SHEAR);
    }
  }
  // Handle left & below
  for (int i = 0; i < particlesPerEdge - 1; ++i) {
    for (int j = 0; j < particlesPerEdge - 1; ++j) {
      int index = i * particlesPerEdge + j + 1;
      _springs.emplace_back(index, index + particlesPerEdge - 1, shearLength, Spring::Type::SHEAR);
    }
  }

  std::vector<GLuint> structrualIndices, shearIndices, bendIndices;
  for (const auto& spring : _springs) {
    switch (spring.type()) {
      case Spring::Type::STRUCTURAL:
        structrualIndices.emplace_back(spring.startParticleIndex());
        structrualIndices.emplace_back(spring.endParticleIndex());
        break;
      case Spring::Type::SHEAR:
        shearIndices.emplace_back(spring.startParticleIndex());
        shearIndices.emplace_back(spring.endParticleIndex());
        break;
      case Spring::Type::BEND:
        bendIndices.emplace_back(spring.startParticleIndex());
        bendIndices.emplace_back(spring.endParticleIndex());
        break;
    }
  }
  structuralSpring.allocate_load(structrualIndices.size() * sizeof(GLuint), structrualIndices.data());
  shearSpring.allocate_load(shearIndices.size() * sizeof(GLuint), shearIndices.data());
  bendSpring.allocate_load(bendIndices.size() * sizeof(GLuint), bendIndices.data());
}

void Cloth::computeSpringForce() {
  // TODO: Compute spring force and damper force for each spring.
  //   1. Read the start and end index from spring
  //   2. Use _particles.position(i) to get particle i's position.
  //   3. Modify particles' acceleration a = F / m;
  // Note:
  //   1. Use _particles.inverseMass(i) to get 1 / m can deal with m == 0. Which will returns 0.
  // Hint:
  //   1. Use a.norm() to get length of a.
  //   2. Use a.normalize() to normalize a inplace.
  //          a.normalized() will create a new vector.
  //   3. Use a.dot(b) to get dot product of a and b.
  for (const auto& spring : _springs) {
    // Write code here!
    // 1. Read something we need.
    float resetLength = spring.length();
    Eigen::Vector4f startParticlePosition = _particles.position(spring.startParticleIndex());
    Eigen::Vector4f endParticlePosition = _particles.position(spring.endParticleIndex());
    Eigen::Vector4f startParticleVelocity = _particles.velocity(spring.startParticleIndex());
    Eigen::Vector4f endParticleVelocity = _particles.velocity(spring.endParticleIndex());

    // 2. Calculate Force (Spring & Damper)
    // F = -Ks( |Xa-Xb| -L0 ) * ( Xa - Xb ) / |Xa -Xb|
    //     -Kd( (Va-Vb).(Xa-Xb) / |Xa - Xb| ) * ( Xa - Xb ) / |Xa -Xb|
    Eigen::Vector4f forceA =
        (-1) * springCoef * ( (startParticlePosition - endParticlePosition).norm() - resetLength) *
            (startParticlePosition - endParticlePosition).normalized() 
            - damperCoef *
            ( (startParticleVelocity - endParticleVelocity).dot(startParticlePosition - endParticlePosition) /
             (startParticlePosition - endParticlePosition).norm()) *
            (startParticlePosition - endParticlePosition).normalized();

    Eigen::Vector4f forceB =
        (-1) * springCoef * ((endParticlePosition - startParticlePosition).norm() - resetLength) *
            (endParticlePosition - startParticlePosition).normalized() 
            - damperCoef *
            ((endParticleVelocity - startParticleVelocity).dot(endParticlePosition - startParticlePosition) /
             (endParticlePosition - startParticlePosition).norm()) *
            (endParticlePosition - startParticlePosition).normalized();
    
    // 3. Modify acceleration
    _particles.acceleration(spring.startParticleIndex()) += forceA * _particles.inverseMass(spring.startParticleIndex());
    _particles.acceleration(spring.endParticleIndex()) += forceB * _particles.inverseMass(spring.endParticleIndex());
  }
}

void Cloth::collide(Shape* shape) { shape->collide(this); }
void Cloth::collide(Spheres* sphere) { sphere->collide(this); }

void Cloth::computeNormal() {
  static Eigen::Matrix<float, 4, particlesPerEdge * particlesPerEdge> normals;
  normals.setZero();
  for (int i = 0; i < particlesPerEdge - 1; ++i) {
    int offset = i * (particlesPerEdge);
    for (int j = 0; j < particlesPerEdge - 1; ++j) {
      Eigen::Vector4f v1 = _particles.position(offset + j) - _particles.position(offset + j + particlesPerEdge);
      Eigen::Vector4f v2 = _particles.position(offset + j + 1) - _particles.position(offset + j + particlesPerEdge);
      Eigen::Vector4f n1 = v2.cross3(v1);
      normals.col(offset + j) += n1;
      normals.col(offset + j + 1) += n1;
      normals.col(offset + j + particlesPerEdge) += n1;

      Eigen::Vector4f v3 =
          _particles.position(offset + j + particlesPerEdge + 1) - _particles.position(offset + j + particlesPerEdge);
      Eigen::Vector4f n2 = v3.cross3(v2);
      normals.col(offset + j + 1) += n2;
      normals.col(offset + j + particlesPerEdge) += n2;
      normals.col(offset + j + particlesPerEdge + 1) += n2;
    }
  }
  normals.colwise().normalize();
  normalBuffer.load(0, particlesPerEdge * particlesPerEdge * sizeof(float) * 4, normals.data());
}
