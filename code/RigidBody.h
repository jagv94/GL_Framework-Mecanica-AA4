#include <glm\glm.hpp>
#include <glm\gtc\quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

#pragma region RigidBody
class RigidBody
{
public:
	struct State
	{
		glm::vec3 com; //Posición del centro de masas
		glm::quat rotation; //Rotación
		glm::vec3 linearMomentum; //Momento linear (velocidad linear)
		glm::vec3 angularMomentum; //Momento angular (velocidad angular)
	};

	RigidBody(float mass);
	void initializeState(glm::vec3 initialPosition, glm::quat initialRotation, glm::vec3 linearSpeed, glm::vec3 angularSpeed);

	State getState();
	void setState(State state);
	State rollbackState();
	void commitState();

	float getMass();
	glm::mat3 getInertiaTensor(); //Tensor de inercia actual //I(t)

	virtual void draw() = 0;

protected:
	//Quaternion a matriz de rotación 3x3 q(t) -> R(t)
	virtual glm::mat3 getRotationMatrix() = 0;
	virtual glm::mat3 getInitialInertiaTensor(float mass, float width, float height, float d) = 0; //Se sobreescribe en las clases heredadas

private:
	float mass;
	State stableState;
	State state; //Siempre será un estado válido
	glm::mat3 initialInertiaTensor; //Tensor de inercia inicial
};
#pragma endregion

#pragma region Box
class Box : public RigidBody
{
public:
	Box(float _width, float _height, float _depth, float _mass);
	float getWidth();
	float getHeight();
	float getDepth();
	virtual void draw() override;

protected:
	virtual glm::mat3 getRotationMatrix() override;
	virtual glm::mat3 getInitialInertiaTensor(float mass, float width, float height, float d) override;

private:
	float width, height, depth;
};
#pragma endregion

#pragma region Ball
class Ball : public RigidBody
{
public:
	Ball(float _radius, float _mass);
	virtual void draw() override;

protected:
	virtual glm::mat3 getInitialInertiaTensor(float mass, float width, float height, float d) override;

private:
	float radius;
};
#pragma endregion

#pragma region RigidBodySolver
class RigidBodySolver {
public:
	struct BoundingBox {
		int id;
		float x1;
		float x2;
	};

	enum class IntervalType {
		BEGIN, END
	};

	struct Interval {
		IntervalType type;
		int box;
		float* value; // <-- pointer so when we change the box we have the value updated
	};

	void updateState(Box* _box, glm::vec3 _force, glm::vec3 _torques, float _dt);
	void updateState(Ball* _box, glm::vec3 _force, glm::vec3 _torques, float _dt);
};
#pragma endregion