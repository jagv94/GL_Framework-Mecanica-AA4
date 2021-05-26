#include "particleSystem.h"

namespace LilSpheres {
    extern const int maxParticles;
    extern int firstParticleIdx;
    extern int particleCount;
    extern void updateParticles(int startIdx, int count, float* array_data);
}

ParticleSystem::ParticleSystem(int _numParticles) : maxParticles(_numParticles), currentParticles(0)
{
    positions = new glm::vec3[_numParticles];
    age = new float[_numParticles];
}

void ParticleSystem::UpdateParticle(int _idx, glm::vec3 _newPosition)
{
    positions[_idx] = _newPosition;
}

void ParticleSystem::UpdateLilSpheres()
{
    LilSpheres::firstParticleIdx = 0;
    LilSpheres::particleCount = currentParticles;
    float* first_float = &(positions[0].x);

    LilSpheres::updateParticles(0, maxParticles, first_float);
}

void ParticleSystem::spawnParticle(glm::vec3 position)
{

    UpdateParticle(currentParticles, position);
    age[currentParticles] = 0.f;
    currentParticles++;
}

void ParticleSystem::updateAge(float dt)
{
    for (int i = 0; i < currentParticles; i++)
    {
        age[i] += dt;
    }
}

template <typename T>

void shiftLeft(int count, T* arr, int positionsToShift)
{
    for (int i = 0; i < count - positionsToShift; i++)
    {
        arr[i] = arr[i + positionsToShift];
    }
}


void ParticleSystem::destroyOldParticles(float _maxAge)
{

    int positionToShift = 0;
    int currentAge = age[0];

    while (currentAge > _maxAge && positionToShift <= currentParticles)
    {
        positionToShift++;

        currentAge = age[positionToShift];
    }

    shiftLeft(currentParticles, positions, positionToShift);
    shiftLeft(currentParticles, age, positionToShift);

    currentParticles -= positionToShift;

}