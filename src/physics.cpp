#include <SDL3/SDL.h>
#include <SDL3/SDL_rect.h>
#include <cstddef>
#include <glm/exponential.hpp>
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

std::function<float(std::vector<glm::vec2>)> makeDistanceConstraint(float dist)
{
    return [dist](std::vector<glm::vec2> array) -> float {
        if (array.size() != 2) {
            return 0.0f;
        }
        return glm::length(array[0] - array[1]) - dist;
    };
}
std::function<float(std::vector<glm::vec2>)> makeBoundaryConstraint(float dist)
{
    return [dist](std::vector<glm::vec2> array) -> float {
        if (array.size() != 1) {
            return 0.0f;
        }
        return dist - glm::length(array[0]);
    };
}

void initialise(vector<Point> &points, vector<Constraint> &constraints)
{
    // TODO: get different objects going, after that, collisions
    Point point;
    // 4 points in square
    point.pos = {25,25};
    point.vel = {0,0};
    point.mass = 1;
    point.invMass = 1/point.mass;
    points.push_back(point);
    point.pos = {25,-25};
    point.vel = {0,0};
    point.mass = 1;
    point.invMass = 1/point.mass;
    points.push_back(point);
    point.pos = {-25,-25};
    point.vel = {0,0};
    point.mass = 1;
    point.invMass = 1/point.mass;
    points.push_back(point);
    point.pos = {-25,25};
    point.vel = {0,0};
    point.mass = 1;
    point.invMass = 1/point.mass;
    points.push_back(point);
    // control point
    point.pos = {200,0};
    point.vel = {0,0};
    point.mass = 1;
    point.invMass = 1/point.mass;
    points.push_back(point);
    
    Constraint constraint;
    float stiff = 1e-2;
    // hold in square
    for (int i = 0; i < 4; i++) {
        constraint.stiffness = stiff;
        constraint.indices = {i, (i+1)%4};
        constraint.pfunc = makeDistanceConstraint(50);
        constraint.cardinality = 2;
        constraint.equality = true;
        constraints.push_back(constraint);
    }
    constraint.stiffness = stiff;
    constraint.indices = {0, 2};
    constraint.pfunc = makeDistanceConstraint(glm::sqrt(5000));
    constraint.cardinality = 2;
    constraint.equality = true;
    constraints.push_back(constraint);
    constraint.stiffness = stiff;
    constraint.indices = {1, 3};
    constraint.pfunc = makeDistanceConstraint(glm::sqrt(5000));
    constraint.cardinality = 2;
    constraint.equality = true;
    constraints.push_back(constraint);

    // circle boundary
    for (int i = 0; i < 5; i++) {
        constraint.stiffness = 0.99;
        constraint.indices = {i};
        constraint.pfunc = makeBoundaryConstraint(100);
        constraint.cardinality = 1;
        constraint.equality = false;
        constraints.push_back(constraint);
    }
}

void step(vector<Point> &points, vector<Constraint> &constraints)
{
    // applying forces external to the system (gravity in this case)
    for (size_t i = 0; i < points.size(); i++) {
        points[i].vel += dT * points[i].invMass * f_ext(points[i].pos);
    }

    dampVelocities(points);

    for (Point &point : points) {
        point.newPos.x = point.pos.x + dT * point.vel.x;
        point.newPos.y = point.pos.y + dT * point.vel.y;
    }

    vector<Constraint> collisionConstraints;
    vector<Constraint> temp = {};
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

    posExtra[index] += glm::vec2{-eps, eps};
    float CplusY = constraint.pfunc(posExtra);

    return {(CplusX - C) / eps, (CplusY - C) / eps};
}

void projectConstraints(vector<Constraint> &constraints, vector<Point> &points)
{
    Point* point_i;
    float C;
    float sumWeightedGradSq;
    vector<glm::vec2> posArray;
    float scalingFactor;
    glm::vec2 correction;
    vector<glm::vec2> gradCs;

    for (Constraint &constraint : constraints) {
        sumWeightedGradSq = 0;
        posArray = {};
        gradCs = {};
        for (int index : constraint.indices) {
            point_i = &points[index];
            posArray.push_back(point_i->newPos);
        }
        C = constraint.pfunc(posArray);
        if (!constraint.equality && C >= 0) {
            continue;
        }
        for (int i = 0; i < constraint.cardinality; i++) {
            point_i = &points[constraint.indices[i]];
            gradCs.push_back(gradientC(posArray, i, constraint));
            sumWeightedGradSq += point_i->invMass * glm::length2(gradCs[i]);
        }
        for (int i = 0; i < constraint.cardinality; i++) {
            point_i = &points[constraint.indices[i]];
            scalingFactor = C / sumWeightedGradSq;
            correction = scalingFactor * point_i->invMass * gradCs[i];
            point_i->newPos -= correction * (float) (1 - glm::pow(1 - constraint.stiffness, 1 / (float) solverIterations));
        }
    }
}
