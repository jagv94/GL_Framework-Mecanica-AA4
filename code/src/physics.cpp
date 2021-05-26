#include "../RigidBody.h"
#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm/gtx/string_cast.hpp>

#pragma region ForwardDeclaration
namespace Cube
{
	extern void updateCube(const glm::mat4& transform);
}

namespace Sphere
{
	extern void updateSphere(const glm::vec3 pos, float radius = 1.0f);
}

extern bool renderCube;
extern bool renderSphere;

#pragma endregion

#pragma region RigidBody
#pragma region variables
//Box variables
Box* box;

//Other variables
RigidBodySolver solver;
const float G = 0.001f;
#pragma endregion

#pragma region utils
//eje, angulo -> quaternion
glm::quat getRotationQuaternion(glm::vec3 axis, float angle) {
	float w = cos(angle / 2);
	glm::vec3 v = sin(angle / 2) * axis;
	return glm::normalize(glm::quat(w, v));
}


glm::vec3 getGravityForce(RigidBody* r1, RigidBody* r2) {
	glm::vec3 direction = r2->getState().com - r1->getState().com;
	float distance = glm::length(direction);
	float magnitude = G * r1->getMass() * r2->getMass() / distance;
	return glm::normalize(direction) * magnitude;
}

float RandomFloat(float min, float max) {
	assert(max > min);
	float random = ((float)rand()) / (float)RAND_MAX;
	float range = max - min;
	return (random * range) + min;
}

int RandomInt(int min, int max) {
	assert(max > min);
	int random = ((int)rand()) / (int)RAND_MAX;
	int range = max - min;
	return (random * range) + min;
}

float Radians(float angle) {
	return 2 * 3.14f * (angle / 360);
}
#pragma endregion utils

struct BoundingBox {
	int id;
	float x1; // x1 < x2
	float x2;
};

enum class IntervalType {
	BEGIN, END
};

struct IntervalLimit {
	IntervalType type;
	int box;
	float* value; // <-- pointer so when we change the box we have the value updated
};

BoundingBox boxes[] =
{
	 BoundingBox{2, 1.2f, 2.0f},
	 BoundingBox{1, 0.8f, 1.3f},
	 BoundingBox{3, 3.0f, 3.7f},
	 BoundingBox{0, 0.0f, 1.0f},
	 BoundingBox{4, 3.5f, 4.0f}
};

void printBoxes() {
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

void InitIntervals(IntervalLimit intervalsLimits[])
{
	for (int i = 0; i < 5; i++) //	FOR EACH BOX...
	{
		intervalsLimits[2 * i] = IntervalLimit{ IntervalType::BEGIN, i, &(boxes[i].x1) };
		intervalsLimits[2 * i + 1] = IntervalLimit{ IntervalType::END, i, &(boxes[i].x2) };
	}
}

void Sort(IntervalLimit list[], int n)
{
	// Insertion sort
}

void Sweep(IntervalLimit list[], int n)
{
	bool active[5] = { false, false, false, false, false };

	// activar caixa 3 --> active[3] = true;
	// implementar sweep
	// print collisions
}

void CheckPotentialCollisions()
{
	IntervalLimit intervalsLimits[2 * 5];
	InitIntervals(intervalsLimits);

	// sort
	Sort(intervalsLimits, 10);
	// sweep
	Sweep(intervalsLimits, 10);

}

void SortAndSweep() {
	CheckPotentialCollisions();
	printBoxes();
}

bool CalculateBoundingBox(Box* _box)
{
	glm::vec3 vectors[8] = { glm::vec3(-1.0f, 1.0f, -1.0f), glm::vec3(1.0f, 1.0f, -1.0f), glm::vec3(1.0f, -1.0f, -1.0f), glm::vec3(1.0f, 1.0f, 1.0f),
	glm::vec3(1.0f, -1.0f, 1.0f), glm::vec3(-1.0f, -1.0f, 1.0f), glm::vec3(-1.0f, 1.0f, 1.0f), glm::vec3(-1.0f, -1.0f, -1.0f) };

	glm::vec3 boxPoints[8];

	for (int i = 0; i < 8; i++)
	{
		boxPoints[i] = glm::mat3_cast(_box->getState().rotation) * vectors[i] + _box->getState().com;
	}

	for (int i = 1; i < 8; i++)
	{
		if (boxPoints[0].x > boxPoints[i].x)
		{
			boxPoints[0].x = boxPoints[i].x;
		}
		if (boxPoints[0].y > boxPoints[i].y)
		{
			boxPoints[0].y = boxPoints[i].y;
		}
		if (boxPoints[0].z > boxPoints[i].z)
		{
			boxPoints[0].z = boxPoints[i].z;
		}
	}
	return (boxPoints[0].x <= -5.0f || boxPoints[0].x >= 5.0f || boxPoints[0].y <= 0.0f || boxPoints[0].y >= 10.0f || boxPoints[0].z <= -5.0f || boxPoints[0].z >= 5.0f);
}

glm::vec3 savePos[8];
glm::vec3 position;

void collisionCorrection(Box* _box, float _dt, int iteration, RigidBody::State _impulseState, glm::vec3 _vectors[8], float _positionInAxis, glm::vec3 _normal, int dtChecks) {
	glm::vec3 velocity = _box->getState().linearMomentum / _box->getMass();

	float newTime = _dt / 2;
	position = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 collisionPos[8];

	for (int j = 0; j < dtChecks; j++)
	{
		position = savePos[iteration] + newTime * velocity;
		if (_positionInAxis == -5.0f)
		{
			break;
		}
		else if (_positionInAxis < -5.0f)
		{
			newTime = newTime / 2;
		}
		else
		{
			newTime = newTime * 1.5f;
		}
	}

	collisionPos[iteration] = savePos[iteration] + newTime * velocity;
	_impulseState.com = (savePos[iteration] - _vectors[iteration]) + newTime * velocity;

	glm::mat3 invertedInertiaTensor = glm::mat3_cast(_box->getState().rotation) * glm::inverse(_box->getInertiaTensor()) * glm::transpose(glm::mat3_cast(_box->getState().rotation));

	glm::vec3 angularVelocity = invertedInertiaTensor * _box->getState().angularMomentum;

	float relativeVelocity = glm::dot(_normal, (velocity + glm::cross(angularVelocity, (collisionPos[iteration] - _impulseState.com))));

	if (relativeVelocity <= 0) {
		glm::vec3 j = -((1.0f + 1.0f) * relativeVelocity) / ((1.0f / _box->getMass()) + _normal * (invertedInertiaTensor * cross(cross((collisionPos[iteration] - _impulseState.com), _normal), (collisionPos[iteration] - _impulseState.com))));

		glm::vec3 impulse = j * _normal;

		_impulseState.linearMomentum = _box->getState().linearMomentum + impulse;
		_impulseState.angularMomentum = _box->getState().angularMomentum + cross((collisionPos[iteration] - _impulseState.com), impulse);
		_impulseState.com = _box->getState().com;
		_impulseState.rotation = _box->getState().rotation;

		_box->setState(_impulseState);
	}
}

void CalculateVelocityImpact(Box* _box, float _dt)
{
	int pointID = 0;
	glm::vec3 vectors[8] = { glm::vec3(-0.5f, 0.5f, -0.5f), glm::vec3(0.5f, 0.5f, -0.5f), glm::vec3(0.5f, -0.5f, -0.5f), glm::vec3(0.5f, 0.5f, 0.5f),
	glm::vec3(0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, 0.5f), glm::vec3(-0.5f, 0.5f, 0.5f), glm::vec3(-0.5f, -0.5f, -0.5f) };

	glm::vec3 boxPoints[8];
	RigidBody::State impulseState;
	glm::vec3 impulse = glm::vec3(0.f, 0.f, 0.f);

	for (int i = 0; i < 8; i++)
	{
		boxPoints[i] = glm::mat3_cast(_box->getState().rotation) * vectors[i] + _box->getState().com;
	}

	for (int i = 0; i < 8; i++)
	{
		if (boxPoints[i].x <= -5.0f)
		{
			collisionCorrection(_box, _dt, i, impulseState, vectors, position.x, glm::vec3(1.0f, 0.0f, 0.0f), 32);
		}
		if (boxPoints[i].x >= 5.0f)
		{
			collisionCorrection(_box, _dt, i, impulseState, vectors, position.x, glm::vec3(-1.0f, 0.0f, 0.0f), 32);
		}
		if (boxPoints[i].y <= 0.0f)
		{
			collisionCorrection(_box, _dt, i, impulseState, vectors, position.y, glm::vec3(0.0f, 1.0f, 0.0f), 32);
		}
		if (boxPoints[i].y >= 10.0f)
		{
			collisionCorrection(_box, _dt, i, impulseState, vectors, position.y, glm::vec3(0.0f, -1.0f, 0.0f), 32);
		}
		if (boxPoints[i].z <= -5.0f)
		{
			collisionCorrection(_box, _dt, i, impulseState, vectors, position.z, glm::vec3(0.0f, 0.0f, 1.0f), 32);
		}
		if (boxPoints[i].z >= 5.0f)
		{
			collisionCorrection(_box, _dt, i, impulseState, vectors, position.z, glm::vec3(0.0f, 0.0f, -1.0f), 32);
		}
		else
		{
			savePos[i] = boxPoints[pointID];
		}
	}
}
#pragma endregion

#pragma region main

void PhysicsInit() {
	srand(time(NULL));
	renderCube = true;

	glm::vec3 rotationAxis = glm::vec3(0.f, 1.f, 0.f);
	float angle = 3.14f / 4.f; // 45 degrees

	box = new Box(1.f, 1.f, 1.f, 1.f); //Anchura, altura, profundidad y masa
	glm::vec3 boxCom = glm::vec3(RandomFloat(-3, 3), RandomFloat(4, 8), RandomFloat(-3, 3)); //glm::vec3(4.0f, 5.0f, 0.0f); //Posición del centro de masas (posición del objeto)
	glm::quat rotation = getRotationQuaternion(glm::vec3(RandomInt(0.f, 1.f), RandomInt(0.f, 1.f), RandomInt(0.f, 1.f)), RandomFloat(Radians(-180), Radians(180))); //Quaternion de rotación (ejes, angulo)
	glm::vec3 boxLinearVelocity = glm::vec3(RandomFloat(-5.f, 5.f), RandomFloat(-5.f, 5.f), RandomFloat(-5.f, 5.f)); //Velocidad lineal
	glm::vec3 boxAngularVelocity = glm::vec3(RandomFloat(0.f, 50.f), RandomFloat(0.f, 50.f), RandomFloat(0.f, 50.f)); //Velocidad angular

	box->initializeState( //Inicializamos el estado al crear el cubo
		boxCom, rotation, boxLinearVelocity, boxAngularVelocity
	);
	printBoxes();
}

bool reset = false;
float timer = 0.f;
float timeLimit = 15.f;
void PhysicsUpdate(float dt) {
	timer += dt;
	glm::vec3 force; //masa * aceleración
	force = glm::vec3(0.0f, -9.81f, 0.0f);//getGravityForce(box, ball);
	
	glm::vec3 torques = glm::vec3(0.0f, 0.0f, 0.0f); //Variación del momento angular = derivada del tensor de inercia (dI(t)) * velocidad angular (w(t))
	
	solver.updateState(box, force, torques, dt);
	//solver.updateState(ball, -force, torques, dt);
	
	if (CalculateBoundingBox(box))
	{
		CalculateVelocityImpact(box, dt);
	}
	else
	{
		box->commitState();
	}
	box->draw();

	if (timer >= timeLimit || reset) {
		PhysicsInit();
		timer = 0.f;
		reset = false;
	}
}

void PhysicsCleanup() {
	delete box;
}
#pragma endregion

bool show_test_window = false;
void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS) - Time: %.0f", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate, ImGui::GetTime()); //FrameRate
		if (ImGui::Button("Reset")) {
			reset = true;
		}
	}
	ImGui::End();
}