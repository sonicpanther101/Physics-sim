#pragma once
#include <glm/ext/vector_float2.hpp>
#include <vector>
#include <SDL3/SDL.h>
#include <glm/glm.hpp>

#define dT (float) 1/60
#define solverIterations 5
#define k_damping (float) 0.95

struct Point {
    float invMass;
    float mass;
    glm::vec2 pos;
    glm::vec2 newPos;
    glm::vec2 vel;
};

struct Constraint {
    int cardinality;
    float (*pfunc)(std::vector<glm::vec2> array);
    std::vector<int> indices;
    float stiffness; // in the range of [0,1]
    bool equality; // C_j(x_i_1, ..., x_i_n_j) = 0 as opposed to C_j() >= 0
};

void initialise(std::vector<Point> &points, std::vector<Constraint> &constraints);
void step(std::vector<Point> &points, std::vector<Constraint> &constraints);
