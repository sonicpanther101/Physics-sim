#pragma once
#include <glm/ext/vector_float2.hpp>
#include <vector>
#include <SDL3/SDL.h>
#include <glm/glm.hpp>

#define dT (float) 1/60
#define solverIterations 3
#define k_damping (float) 0.1

struct Point {
    float mass;
    float invMass;
    glm::vec2 pos;
    glm::vec2 newPos;
    glm::vec2 vel;
};

struct Constraint {
    std::vector<int> indices;
    int cardinality;
    std::function<float(std::vector<glm::vec2>)> pfunc;
    bool equality; // C_j(x_i_1, ..., x_i_n_j) = 0 as opposed to C_j() >= 0
    float stiffness; // in the range of [0,1]
};

struct Object {
    std::vector<Point> points;
    std::vector<Constraint> constraints;
};

void initialise(std::vector<Object> &objects);
void step(std::vector<Object> &objects);
