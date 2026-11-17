#include <SDL3/SDL.h>
#include <SDL3/SDL_rect.h>
#include <cstddef>
#include <glm/geometric.hpp>
#include <strings.h>
#include <vector>
#include <iostream>
#include "physics.h"

#define GLM_ENABLE_EXPERIMENTAL true
#include <glm/gtx/perpendicular.hpp>
#include <glm/gtx/exterior_product.hpp>
#include <glm/gtx/norm.hpp>

using namespace std;

glm::vec2 f_ext(glm::vec2 &pos);
void dampVelocities(vector<Point> &points);
vector<Constraint> generateCollisionConstraints(Point &point);
void projectConstraints(vector<Constraint> &constraints, vector<Point> &points);

float distanceConstraint(vector<glm::vec2> array)
{
    if (array.size() != 2) {
        return 0;
    }
    return glm::length(array[0] - array[1]) - 50;
}

void initialise(vector<Point> &points, vector<Constraint> &constraints)
{
    Point point;
    point.pos = {25,25};
    point.vel = {0,0};
    point.invMass = 1;
    point.mass = 1;
    points.push_back(point);
    point.pos = {25,-25};
    point.vel = {0,0};
    point.invMass = 1;
    point.mass = 1;
    points.push_back(point);
    point.pos = {-25,-25};
    point.vel = {0,0};
    point.invMass = 1;
    point.mass = 1;
    points.push_back(point);
    point.pos = {-25,25};
    point.vel = {0,0};
    point.invMass = 1;
    point.mass = 1;
    points.push_back(point);

    Constraint constraint;
    for (int i = 0; i < 4; i++) {
        constraint.indices = {i, (i+1)%4};
        constraint.pfunc = &distanceConstraint;
        constraints.push_back(constraint);
    }
}

void step(vector<Point> &points, vector<Constraint> &constraints)
{
    // applying forces external to the system (gravity in this case)
    int test = 0;
    for (Point &point : points) {
        if (test)
            point.vel += dT * point.invMass * f_ext(point.pos);
        test = (test+1)%4;
    }

    dampVelocities(points);

    for (Point &point : points) {
        point.newPos.x = point.pos.x + dT * point.vel.x;
        point.newPos.y = point.pos.y + dT * point.vel.y;
    }

    vector<Constraint> collisionConstraints;
    vector<Constraint> temp;
    for (Point &point : points) {
        // temp = generateCollisionConstraints(point);
        for (Constraint &constraint : temp) {
            collisionConstraints.push_back(constraint);
        }
    }

    for (size_t i = 0; i < solverIterations; i++) {
        projectConstraints(constraints, points);
    }

    for (Point &point : points) {
        point.vel.x = (point.newPos.x - point.pos.x) / (dT);
        point.vel.y = (point.newPos.y - point.pos.y) / (dT);

        point.pos = point.newPos;
    }
}

glm::vec2 f_ext(glm::vec2 &pos)
{
    return {0,10}; // gravity
}

void dampVelocities(vector<Point> &points)
{
    glm::vec2 massPosSum = {0,0};
    glm::vec2 massVelSum = {0,0};
    float massSum = 0;
    for (Point &point : points) {
        massSum += point.mass;
        massPosSum += point.mass * point.pos;
        massVelSum += point.mass * point.vel;
    }
    glm::vec2 posCM = massPosSum / massSum;
    glm::vec2 velCM = massVelSum / massSum;

    glm::vec2 relative;
    float angularMomentum = 0;
    float momentInertia = 0;
    for (Point &point : points) {
        relative = point.pos - posCM;
        angularMomentum += glm::cross(relative, point.vel);
        momentInertia += glm::length2(relative);
    }
    float angularVel;
    if (momentInertia == 0) {
        angularVel = 0;
    } else {
        angularVel = angularMomentum / momentInertia;
    }

    glm::vec2 dVel;
    glm::vec2 invRel;
    for (Point &point : points) {
        relative = point.pos - posCM;
        invRel = {-relative.y, relative.x};
        dVel = velCM + angularVel * invRel - point.vel;
        point.vel += k_damping * dVel;
    }
}

vector<Constraint> generateCollisionConstraints(Point point)
{
    return {};
}

glm::vec2 gradientC(vector<glm::vec2> posArray, int index, Constraint constraint)
{
    // del_p_j C(p_1,...,p_n) ≈ (C(p_1,...,p_k+ε,...,p_n) - C(p_1,...,p_n))/ε
    vector<glm::vec2> posExtra = posArray;
    static float eps = 1e-4f;
    static glm::vec2 grad;
    float C = constraint.pfunc(posArray);

    posExtra[index].x += eps;
    float CplusX = constraint.pfunc(posExtra);
    grad.x = (CplusX - C) / eps;

    posExtra[index] += glm::vec2{-eps, eps};
    float CplusY = constraint.pfunc(posExtra);
    grad.y = (CplusY - C) / eps;
    return grad;
}

void projectConstraints(vector<Constraint> &constraints, vector<Point> &points)
{
    Point* point_i;
    float C;
    float sumWeightedGradSq;
    vector<glm::vec2> posArray;
    float scalingFactor;
    glm::vec2 gradC;
    for (Constraint &constraint : constraints) {
        sumWeightedGradSq = 0;
        posArray = {};
        for (int index : constraint.indices) {
            point_i = &points[index];
            posArray.push_back(point_i->newPos);
        }
        C = constraint.pfunc(posArray);
        for (int i = 0; i < constraint.indices.size(); i++) {
            point_i = &points[constraint.indices[i]];
            sumWeightedGradSq += point_i->invMass * glm::length2(gradientC(posArray, i, constraint));
        }
        for (int i = 0; i < constraint.indices.size(); i++) {
            point_i = &points[constraint.indices[i]];
            scalingFactor = C / sumWeightedGradSq;
            point_i->newPos -= scalingFactor * point_i->invMass * gradientC(posArray, i, constraint);
        }
    }
}
