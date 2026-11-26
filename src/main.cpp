#include <SDL3/SDL_pixels.h>
#include <SDL3/SDL_rect.h>
#include <SDL3/SDL_render.h>
#include <SDL3/SDL_video.h>
#include <iostream>
#include <SDL3/SDL.h>
#include <set>
#include <vector>
#include "physics.h"

using namespace std;

struct SDLState
{
    SDL_Window *window;
    SDL_Renderer *renderer;
};

struct ColourSceme
{
    SDL_Color base;
    SDL_Color plain;
    SDL_Color crust;
    SDL_Color red;
    SDL_Color yellow;
    SDL_Color green;
    SDL_Color blue;
};

ColourSceme CATPPUCCIN = {
    .base   = {30 ,30 ,46 },
    .plain  = {205,214,244},
    .crust  = {17 ,17 ,27 },
    .red    = {243,139,168},
    .yellow = {249,226,175},
    .green  = {166,227,161},
    .blue   = {137,180,250}
};

int width = 800;
int height = 600;

int  initialiseRendering(SDLState &state);
void framePacing(bool shouldInitialise = false);
void cleanup(SDLState &state);
void drawCircle(glm::vec2 center, float radius, SDL_Color colour, SDLState &state);
void setColour(SDL_Color colour, SDLState &state);
void drawVertices(vector<Object> &objects, SDLState &state);
void drawDistanceConstraints(vector<Object> &objects, SDLState &state);
void drawBoundingBoxes(vector<Object> &objects, SDLState &state);

int main(int argc, char* argv[])
{
    // physics init
    vector<Object> objects;
    initialisePhysics(objects);

    SDLState state;
    if (initialiseRendering(state))
        return 1;

    framePacing(true);
    Uint64 frameCount = 0;

    // start sim loop
    bool running = true;
    while (running) {
        SDL_Event event{ 0 };
        while (SDL_PollEvent(&event)) {
            switch (event.type) {
                case SDL_EVENT_QUIT: {
                    running = false;
                    break;
                }
            }
        }

        // physics sim step
        if (frameCount % 2 == 0) {
            step(objects);
        }
        frameCount++;

        // perform drawing commands
        setColour(CATPPUCCIN.base, state);
        SDL_RenderClear(state.renderer);

        drawCircle(glm::vec2{0,0}, 100.0f, CATPPUCCIN.crust, state);

        drawBoundingBoxes(objects, state);

        drawVertices(objects, state);

        drawDistanceConstraints(objects, state);

        // swap buffers and present
        SDL_RenderPresent(state.renderer);

        framePacing();
    }

    cleanup(state);
    return 0;
}


int initialiseRendering(SDLState &state)
{
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "Error initialising SDL3", nullptr);
        return 1;
    }

    // create the window
    state.window = SDL_CreateWindow("Physics sim", width, height, 0);
    if (!state.window) {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "Error creating window", state.window);
        cleanup(state);
        return 1;
    }

    // create the renderer
    state.renderer = SDL_CreateRenderer(state.window, nullptr);
    if (!state.renderer) {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "Error creating renderer", state.window);
        cleanup(state);
        return 1;
    }

    return 0;
}

void framePacing(bool shouldInitialise)
{
    static const double targetFrameTime = 1.0 / 120.0; // 120 Hz render loop
    static const Uint64 perfFreq       = SDL_GetPerformanceFrequency();
    static const Uint64 frameTicks     = (Uint64)(targetFrameTime * (double)perfFreq);
    static Uint64 nextFrameTick        = SDL_GetPerformanceCounter();

    // slow-frame reporting
    static Uint64 reportWindowStart = SDL_GetTicks();
    static int    slowFrames        = 0;
    static double worstOverrunMs    = 0.0;

    if (shouldInitialise)
        return;

    // pace to the next 120 Hz deadline
    nextFrameTick += frameTicks; // absolute schedule, no drift
    Uint64 nowTick = SDL_GetPerformanceCounter();

    if (nowTick < nextFrameTick) {
        // got here early: sleep the remainder of the frame
        Uint64 sleepNs = (Uint64)((double)(nextFrameTick - nowTick) * 1e9 / (double)perfFreq);
        SDL_DelayPrecise(sleepNs);
    } else {
        // this frame took longer than 1/120 s, so we can't hold 120 Hz
        double overrunMs = (double)(nowTick - nextFrameTick) * 1000.0 / (double)perfFreq;
        ++slowFrames;
        if (overrunMs > worstOverrunMs) {
            worstOverrunMs = overrunMs;
        }

        Uint64 ticksNow = SDL_GetTicks();
        if (ticksNow - reportWindowStart >= 1000) {
            cout << "Warning: below 120 Hz - " << slowFrames
                 << " late frame(s) in the last " << (ticksNow - reportWindowStart)
                 << " ms (worst overrun " << worstOverrunMs << " ms)" << endl;
            reportWindowStart = ticksNow;
            slowFrames        = 0;
            worstOverrunMs    = 0.0;
        }

        // resync instead of trying to catch up; otherwise a recovered
        // loop would burst-frame and run physics faster than 60 Hz
        nextFrameTick = nowTick;
    }
}

void setColour(SDL_Color colour, SDLState &state)
{
    SDL_SetRenderDrawColor(state.renderer, colour.r, colour.g, colour.b, 255);
}

void cleanup(SDLState &state)
{
    SDL_DestroyRenderer(state.renderer);
    SDL_DestroyWindow(state.window);
    SDL_Quit();
}

SDL_FPoint screenPosition(glm::vec2 position)
{
    return {position.x + (float) width/2, (float) height/2 - position.y};
}

float screenX(float x)
{
    return x + (float) width/2;
}

float screenY(float y)
{
    return (float) height/2 - y;
}

void drawCircle(glm::vec2 center, float radius, SDL_Color colour, SDLState &state)
{
    constexpr int   segments = 32;                       // bump for larger radii
    constexpr float twoPi    = 2*SDL_PI_D;
    const SDL_FPoint screenCentre = screenPosition(center);

    SDL_Vertex verts[segments + 1];                      // 0 = centre, 1..segments = rim
    int        indices[segments * 3];

    const SDL_FColor c = {
        colour.r / 255.0f,
        colour.g / 255.0f,
        colour.b / 255.0f,
        1.0f
    };

    verts[0].position = screenCentre;
    verts[0].color    = c;
    verts[0].tex_coord = {0.0f, 0.0f};

    for (int i = 0; i < segments; ++i) {
        const float angle = twoPi * (float)i / (float)segments;
        verts[i + 1].position = {
            screenCentre.x + cosf(angle) * radius,
            screenCentre.y + sinf(angle) * radius
        };
        verts[i + 1].color     = c;
        verts[i + 1].tex_coord = {0.0f, 0.0f};
    }

    // fan -> triangle list: (0, i+1, i+2), wrapping the last rim vertex back to 1
    for (int i = 0; i < segments; ++i) {
        indices[i * 3 + 0] = 0;
        indices[i * 3 + 1] = i + 1;
        indices[i * 3 + 2] = (i + 1 < segments) ? i + 2 : 1;
    }

    SDL_RenderGeometry(state.renderer, nullptr, verts, segments + 1, indices, segments * 3);
}

void drawVertices(vector<Object> &objects, SDLState &state)
{
    for (Object &object : objects) {
        for (Point &point : object.points) {
            drawCircle(point.pos, 5.0f, CATPPUCCIN.blue, state);
        }
    }
}

void drawDistanceConstraints(vector<Object> &objects, SDLState &state)
{
    setColour(CATPPUCCIN.red, state);
    for (Object &object : objects) {
        for (Constraint &constraint : object.constraints) {
            if (constraint.cardinality == 2)
                SDL_RenderLine(state.renderer,
                    screenX(object.points[constraint.indices[0]].pos.x),
                    screenY(object.points[constraint.indices[0]].pos.y),
                    screenX(object.points[constraint.indices[1]].pos.x),
                    screenY(object.points[constraint.indices[1]].pos.y));
        }
    }
}

void drawBoundingBoxes(vector<Object> &objects, SDLState &state)
{
    const SDL_FColor boundingBoxColour = {
        CATPPUCCIN.yellow.r / 255.0f,
        CATPPUCCIN.yellow.g / 255.0f,
        CATPPUCCIN.yellow.b / 255.0f,
        1.0f
    };
    for (Object &object : objects) {
        printf("world point: (%f,%f) -> screen (%f,%f)\n",
               object.points[0].pos.x, object.points[0].pos.y,
               screenPosition(object.points[0].pos).x, screenPosition(object.points[0].pos).y);
        BoundingBox box = polygonBoundingBox(object.points);
        // and for the box:
        printf("box corner world (%f,%f) -> screen (%f,%f)\n",
               box.minX, box.maxY,
       screenPosition({box.minX, box.maxY}).x,
       screenPosition({box.minX, box.maxY}).y);
        SDL_Vertex vertices[4];
        for (SDL_Vertex &vertex : vertices) {
            vertex.color = boundingBoxColour;
        }
        vertices[0].position = screenPosition({box.minX, box.maxY});
        vertices[1].position = screenPosition({box.maxX, box.maxY});
        vertices[2].position = screenPosition({box.maxX, box.minY});
        vertices[3].position = screenPosition({box.minX, box.minY});
        int indices[] = {0,1,3, 1,2,3};
        SDL_RenderGeometry(state.renderer, nullptr, vertices, 4, indices, 6);
    }
}
