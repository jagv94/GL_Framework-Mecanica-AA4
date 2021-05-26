#include "RigidBody.h"

#pragma region forwardDeclarations
namespace Cube {
	extern void updateCube(const glm::mat4& _transform);
}

namespace Sphere {
	extern void updateSphere(glm::vec3 _pos, float _radius = 1.f);
}
#pragma endregion forwardDeclarations

#pragma region RigidBodySolver
void printBoxes(RigidBodySolver::BoundingBox boxes[]) {
	for (int i = 0; i < 5; i++) {
		std::cout << "B" << i << ": ";
		for (int j = 0; j < 50; j++) {
			float pos = j / 10.f;
			if (pos < boxes[i].x1 || pos > boxes[i].x2) {
				std::cout << " ";
			}
			else {
				std::cout << "#";
			}
		}
		std::cout << std::endl;
	}
}

void RigidBodySolver::updateState(Box* _box, glm::vec3 _force, glm::vec3 _torques, float _dt)
{
	RigidBody::State newState;

	newState.linearMomentum = _box->getState().linearMomentum + _dt * _force;
	newState.angularMomentum = _box->getState().angularMomentum + _dt * _torques;

	glm::vec3 velocity = newState.linearMomentum / _box->getMass();

	newState.com = _box->getState().com + _dt * velocity;

	glm::mat3 invertedInertiaTensor = glm::mat3_cast(_box->getState().rotation) * glm::inverse(_box->getInertiaTensor()) * glm::transpose(glm::mat3_cast(_box->getState().rotation));

	glm::vec3 angularVelocity = invertedInertiaTensor * newState.angularMomentum;

	glm::quat derRotation = (newState.angularMomentum * 0.5f) * _box->getState().rotation;

	glm::quat rotation = _box->getState().rotation + _dt * derRotation;

	newState.rotation = glm::normalize(rotation);

	_box->setState(newState);
}

void RigidBodySolver::updateState(Ball* _box, glm::vec3 _force, glm::vec3 _torques, float _dt)
{
}
#pragma endregion

#pragma region RigidBody
RigidBody::RigidBody(float _mass) : mass(_mass) {};

void RigidBody::initializeState(glm::vec3 initialPosition, glm::quat initialRotation, glm::vec3 linearSpeed, glm::vec3 angularSpeed) {
	// Initialize the state outside the constructor to use the virtual method getInitialInertiaTensor
	initialInertiaTensor = getInitialInertiaTensor(mass, 1.0f, 1.0f, 1.0f); //Se sobreescribe en las clases heredadas
	state = {
		initialPosition, //Posición del centro de masas (COM) (posición del objeto)
		initialRotation, //Rotación inicial
		mass * linearSpeed, //Momento lineal = masa * velocidad lineal (P = m * v)

		//Momento angular (L = I(t) * w(t)) //Momento angular = tensor de inercia actual * velocidad angular actual //Se debe sobreescribir con la función getInertiaTensor() en las clases heredadas
		angularSpeed * getInertiaTensor()
	};
}

RigidBody::State RigidBody::getState() { return state; }

void RigidBody::setState(RigidBody::State _newState) { 	state = _newState; }

RigidBody::State RigidBody::rollbackState() {
	// If the state is inconsistent, we can go back to the last correct state
	// (for example due to a collision)
	// Return the inconsistent state for cases where we want to use it
	State tmp = state;
	state = stableState;
	return tmp;
}

// Convert the state into a stable (correct) state
void RigidBody::commitState() { stableState = state; }

float RigidBody::getMass() { return mass; }

//Quaternion a matriz de rotación 3x3 q(t) -> R(t)
glm::mat3 RigidBody::getRotationMatrix() { return glm::mat3_cast(getState().rotation); }

glm::mat3 RigidBody::getInertiaTensor() {
	glm::mat3 rotationMat = getRotationMatrix();

	//Tensor inicial, rotaciones y rotación inversa
	return rotationMat * initialInertiaTensor * glm::transpose(rotationMat);
}
#pragma endregion

#pragma region Box
Box::Box(float width, float height, float depth, float mass)
	: RigidBody(mass), width(width), height(height), depth(depth) {};

float Box::getWidth()
{
	return width;
}

float Box::getHeight()
{
	return height;
}

float Box::getDepth()
{
	return depth;
}

void Box::draw() {
	RigidBody::State state = getState();
	glm::mat4 transform = glm::translate(glm::mat4(1.f), state.com) *
		glm::mat4_cast(state.rotation) *
		glm::scale(glm::mat4(1.f), glm::vec3(width, height, depth));
	Cube::updateCube(transform);
}

glm::mat3 Box::getRotationMatrix() {
	return glm::mat3_cast(getState().rotation);
}

glm::mat3 Box::getInitialInertiaTensor(float mass, float width, float height, float d) {
	// TODO implement
	return glm::mat3(
		(1.0f / 12.0f) * mass * (pow(height, 2) + pow(d, 2)), 0, 0,
		0, (1.0f / 12.0f) * mass * (pow(width, 2) + pow(d, 2)), 0,
		0, 0, (1.0f / 12.0f) * mass * (pow(width, 2) + pow(height, 2))
	);
}
#pragma endregion

#pragma region Ball
Ball::Ball(float radius, float mass) : RigidBody(mass), radius(radius) {};

void Ball::draw() {
	Sphere::updateSphere(getState().com, radius);
}

glm::mat3 Ball::getInitialInertiaTensor(float mass, float width, float height, float d) {
	// TODO implement
	return glm::mat3(1.f);
}
#pragma endregion