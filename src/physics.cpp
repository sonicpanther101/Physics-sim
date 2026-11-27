#include <SDL3/SDL.h>
#include <SDL3/SDL_rect.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <glm/exponential.hpp>
#include <glm/ext/vector_float2.hpp>
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
void generateCollisionConstraints(Point &point, size_t index, vector<Point> &points, vector<Constraint> &constraints);
void projectConstraints(vector<Constraint> &constraints, vector<Point> &points);

void addPoint(vector<Point> &points, glm::vec2 pos, glm::vec2 vel = {0,0}, float mass = 1, float invMass = 1);
void addConstraint(vector<Constraint> &constraints, vector<size_t> indices, size_t cardinality,
        function<float(vector<glm::vec2>)> pfunc, bool equality = true, float stiffness = 0.1);
void addSquare(vector<Object> &objects, glm::vec2 pos, float length = 50, float mass = 1,
        glm::vec2 vel = {0,0}, float stiffness = 1e-2);
void addDistanceConstraint(vector<Constraint> &constraints, vector<Point> &points,
        vector<size_t> indices, float stiffness = 1e-2);
void addKn(vector<Object> &objects, glm::vec2 pos, float radius = 25, size_t degree = 4, float mass = 1,
        glm::vec2 vel = {0,0}, float stiffness = 1e-2);

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

void initialisePhysics(vector<Object> &objects)
{
    // TODO: collisions

    addSquare(objects, {-30,0}, 50, 5);
    addSquare(objects, {20,20}, 50, 5);

    // addKn(objects, {50,-50}, 25, 6, 10);

    // control point
    Object object;
    addPoint(object.points, {200,0});
    objects.push_back(object);
    
    // circle boundary
    Constraint constraint;
    for (size_t i = 0; i < objects.size(); i++) {
        for (size_t j = 0; j < objects[i].points.size(); j++) {
            constraint.stiffness = 0.99;
            constraint.indices = {j};
            constraint.pfunc = makeBoundaryConstraint(100);
            constraint.cardinality = 1;
            constraint.equality = false;
            objects[i].constraints.push_back(constraint);
        }
    }
}

void step(vector<Object> &objects)
{
    for (Object &object : objects) {
        vector<Point>* points = &object.points;
        vector<Constraint>* constraints = &object.constraints;
        // applying forces external to the system (gravity in this case)
        for (size_t i = 0; i < points->size(); i++) {
            (*points)[i].vel += dT * (*points)[i].invMass * f_ext((*points)[i].pos);
        }

        dampVelocities(*points);

        for (Point &point : *points) {
            point.newPos.x = point.pos.x + dT * point.vel.x;
            point.newPos.y = point.pos.y + dT * point.vel.y;
        }

        size_t numConstraints = constraints->size();
        for (size_t i = 0; i < points->size(); i++) {
            for (Object &objectCompared: objects) {
                generateCollisionConstraints((*points)[i], i, objectCompared.points, *constraints);
            }
        }

        for (size_t i = 0; i < solverIterations; i++) {
            projectConstraints(*constraints, *points);
        }
        constraints->resize(numConstraints);

        for (Point &point : *points) {
            point.vel.x = (point.newPos.x - point.pos.x) / (dT);
            point.vel.y = (point.newPos.y - point.pos.y) / (dT);

            point.pos = point.newPos;
        }
    }
}

glm::vec2 f_ext(glm::vec2 &pos)
{
    return {0,-50}; // gravity
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

glm::vec2 gradientC(vector<glm::vec2> posArray, size_t index, Constraint constraint)
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
        for (size_t index : constraint.indices) {
            point_i = &points[index];
            posArray.push_back(point_i->newPos);
        }
        C = constraint.pfunc(posArray);
        if (!constraint.equality && C >= 0) {
            continue;
        }
        for (size_t i = 0; i < constraint.cardinality; i++) {
            point_i = &points[constraint.indices[i]];
            gradCs.push_back(gradientC(posArray, i, constraint));
            sumWeightedGradSq += point_i->invMass * glm::length2(gradCs[i]);
        }
        for (size_t i = 0; i < constraint.cardinality; i++) {
            point_i = &points[constraint.indices[i]];
            scalingFactor = C / sumWeightedGradSq;
            correction = scalingFactor * point_i->invMass * gradCs[i];
            point_i->newPos -= correction * (float) (1 - glm::pow(1 - constraint.stiffness, 1 / (float) solverIterations));
        }
    }
}

void addPoint(vector<Point> &points, glm::vec2 pos, glm::vec2 vel, float mass, float invMass)
{
    if (mass != 1 && invMass == 1)
        invMass = 1/mass;
    Point point = {
        .mass = mass,
        .invMass = invMass,
        .pos = pos,
        .vel = vel
    };
    points.push_back(point);
}

void addConstraint(vector<Constraint> &constraints, vector<size_t> indices, size_t cardinality, function<float(vector<glm::vec2>)> pfunc, bool equality, float stiffness)
{
    Constraint constraint = {
        .indices = indices,
        .cardinality = cardinality,
        .pfunc = pfunc,
        .equality = equality,
        .stiffness = stiffness
    };
    constraints.push_back(constraint);
}

void addSquare(vector<Object> &objects, glm::vec2 pos, float length, float mass, glm::vec2 vel, float stiffness)
{
    vector<Point> points = {};
    addPoint(points, glm::vec2{length/2,length/2} + pos, vel, mass/4);
    addPoint(points, glm::vec2{length/2,-length/2} + pos, vel, mass/4);
    addPoint(points, glm::vec2{-length/2,-length/2} + pos, vel, mass/4);
    addPoint(points, glm::vec2{-length/2,length/2} + pos, vel, mass/4);

    vector<Constraint> constraints;
    for (size_t i = 0; i < 4; i++) {
        addConstraint(constraints, {i, (i+1)%4}, 2, makeDistanceConstraint(length), true, stiffness);
    }
    addConstraint(constraints, {0, 2}, 2, makeDistanceConstraint(glm::sqrt(2*glm::pow(length,2))), true, stiffness);
    addConstraint(constraints, {1, 3}, 2, makeDistanceConstraint(glm::sqrt(2*glm::pow(length,2))), true, stiffness);
    Object object = {
        .points = points,
        .constraints = constraints
    };
    objects.push_back(object);
}

void addDistanceConstraint(vector<Constraint> &constraints, vector<Point> &points, vector<size_t> indices, float stiffness)
{
    Constraint constraint = {
        .indices = indices,
        .cardinality = 2,
        .pfunc = makeDistanceConstraint(glm::length(points[indices[0]].pos - points[indices[1]].pos)),
        .equality = true,
        .stiffness = stiffness
    };
    constraints.push_back(constraint);
}

void addObject(vector<Object> &objects, vector<Constraint> &constraints, vector<Point> &points) {
    Object object = {
        .points = points,
        .constraints = constraints
    };
    objects.push_back(object);
}

void addKn(vector<Object> &objects, glm::vec2 pos, float radius, size_t degree, float mass, glm::vec2 vel, float stiffness)
{
    vector<Point> points = {};
    float angle = 0;
    for (size_t i = 0; i < degree; i++) {
            addPoint(points, glm::vec2{radius*sinf(angle), radius*cosf(angle)} + pos, vel, mass/degree);
            angle += 2*SDL_PI_F/degree;
    }
    vector<Constraint> constraints;
    for (size_t i = 0; i < degree; i++) {
        for (size_t j = i+1; j < degree; j++) {
            addDistanceConstraint(constraints, points, {i, j}, stiffness);
        }
    }
    addObject(objects, constraints, points);
}

BoundingBox polygonBoundingBox(vector<Point> &points)
{
    vector<float> xs, ys;
    for (Point point : points) {
        xs.push_back(point.pos.x);
        ys.push_back(point.pos.y);
    }
    auto xMinMax = minmax_element(xs.begin(), xs.end());
    auto yMinMax = minmax_element(ys.begin(), ys.end());
    BoundingBox box = {
        .minX = *xMinMax.first,
        .minY = *yMinMax.first,
        .maxX = *xMinMax.second,
        .maxY = *yMinMax.second
    };
    return box;
}

void generateCollisionConstraints(Point &point, size_t index, vector<Point> &points, vector<Constraint> &constraints)
{
    // Quick precheck
    BoundingBox box = polygonBoundingBox(points);
    if (point.pos.x < box.minX || point.pos.x > box.maxX || point.pos.y < box.minY || point.pos.y > box.maxY)
        return;

    // Ray-cast test
    // points is an implicitly closed list of positions
    // a ray is cast from point pos to newPos 
    // TODO: make the ray start from outside bounding box and continue until it exits bounding box

    size_t n = points.size();
    size_t j = n - 1; // wraps edges
    glm::vec2* p1,* p2;
    float t,u;
    glm::vec2 edgeDiff, projDiff;
    float scalarProduct, dotProduct;
    glm::vec2 surfaceNormal, entryPoint;
    for (size_t i = 0; i < n; i++) {
        p1 = &(points[i].pos);
        p2 = &(points[j].pos);
        edgeDiff = glm::abs(*p1-*p2);
        projDiff = glm::abs(point.pos - point.newPos);
        scalarProduct = glm::length2(edgeDiff) * glm::length2(projDiff);
        dotProduct = glm::pow(glm::dot(edgeDiff, projDiff), 2);
        if (point.pos == point.newPos || scalarProduct - dotProduct < EPSILON) {
            continue;
        }
        t = ((point.pos.x - p1->x)          * (p1->y       - p2->y) - (point.pos.y - p1->y)          * (p1->x       - p2->x)) /
            ((point.pos.x - point.newPos.x) * (p1->y       - p2->y) - (point.pos.y - point.newPos.y) * (p1->x       - p2->x));
        u =-((point.pos.x - point.newPos.x) * (point.pos.y - p1->y) - (point.pos.y - point.newPos.y) * (point.pos.x - p1->x)) /
            ((point.pos.x - point.newPos.x) * (p1->y       - p2->y) - (point.pos.y - point.newPos.y) * (p1->x       - p2->x));
        if (t <= 1 && t >= 0 && u <= 1 && u >= 0) {
            surfaceNormal = glm::normalize(glm::vec2{-(p2->y - p1->y), p2->x - p1->x}); // rotated 90 clockwise, for clockwise ordering of points
            entryPoint = {
                ((point.pos.x * point.newPos.y - point.pos.y * point.newPos.x) * (p1->x - p2->x) - (point.pos.x - point.newPos.x) * (p1->x * p2->y - p1->y * p2->x)) /
                ((point.pos.x                  - point.newPos.x)               * (p1->y - p2->y) - (point.pos.y - point.newPos.y) * (p1->x         - p2->x)),
                ((point.pos.x * point.newPos.y - point.pos.y * point.newPos.x) * (p1->y - p2->y) - (point.pos.y - point.newPos.y) * (p1->x * p2->y - p1->y * p2->x)) /
                ((point.pos.x                  - point.newPos.x)               * (p1->y - p2->y) - (point.pos.y - point.newPos.y) * (p1->x         - p2->x))
            };
            addConstraint(constraints, {index}, 1,
                [surfaceNormal, entryPoint](std::vector<glm::vec2> array) -> float {
                    return glm::dot(array[0] - entryPoint, surfaceNormal);
                }, 
                false, 1
            );
        }
        j = i; // next edge
    }
}
