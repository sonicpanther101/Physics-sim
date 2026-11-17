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

void cleanup(SDLState &state);
void drawCircle(SDL_FPoint center, float radius, SDL_Color colour, SDLState &state);
void setColour(SDL_Color colour, SDLState &state);

int main(int argc, char* argv[])
{
    // physics init
    vector<Point> points;
    vector<Constraint> constraints;
    initialise(points, constraints);

    SDLState state;
    ColourSceme catppuccin = {
        .base   = {30 ,30 ,46 },
        .plain  = {205,214,244},
        .crust  = {17 ,17 ,27 },
        .red    = {243,139,168},
        .yellow = {249,226,175},
        .green  = {166,227,161},
        .blue   = {137,180,250}
    };

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", "Error initialising SDL3", nullptr);
        return 1;
    }

    // create the window
    int width = 800;
    int height = 600;
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

    // frame pacing setup
    const double targetFrameTime = 1.0 / 120.0; // 120 Hz render loop
    const Uint64 perfFreq       = SDL_GetPerformanceFrequency();
    const Uint64 frameTicks     = (Uint64)(targetFrameTime * (double)perfFreq);
    Uint64 nextFrameTick        = SDL_GetPerformanceCounter();
    Uint64 frameCount           = 0;

    // slow-frame reporting
    Uint64 reportWindowStart = SDL_GetTicks();
    int    slowFrames        = 0;
    double worstOverrunMs    = 0.0;

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
            step(points, constraints);
        }
        frameCount++;

        // perform drawing commands
        setColour(catppuccin.base, state);
        SDL_RenderClear(state.renderer);

        drawCircle({(float) width/2,(float) height/2}, 100.0f, catppuccin.crust, state);
        for (size_t i = 0; i < points.size(); i++) {
            drawCircle({points[i].pos.x+(float) width/2,points[i].pos.y+(float) height/2}, 5.0f, catppuccin.blue, state);
        }

        setColour(catppuccin.red, state);
        for (Constraint &constraint : constraints) {
            if (constraint.cardinality == 2)
                SDL_RenderLine(state.renderer, points[constraint.indices[0]].pos.x+(float) width/2, points[constraint.indices[0]].pos.y+(float) height/2, points[constraint.indices[1]].pos.x+(float) width/2, points[constraint.indices[1]].pos.y+(float) height/2);
        }

        // swap buffers and present
        SDL_RenderPresent(state.renderer);

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

    cleanup(state);
    return 0;
}

void drawCircle(SDL_FPoint center, float radius, SDL_Color colour, SDLState &state)
{
    constexpr int   segments = 32;                       // bump for larger radii
    constexpr float twoPi    = 2*SDL_PI_D;

    SDL_Vertex verts[segments + 1];                      // 0 = centre, 1..segments = rim
    int        indices[segments * 3];

    const SDL_FColor c = {
        colour.r / 255.0f,
        colour.g / 255.0f,
        colour.b / 255.0f,
        1.0f
    };

    verts[0].position = center;
    verts[0].color    = c;
    verts[0].tex_coord = {0.0f, 0.0f};

    for (int i = 0; i < segments; ++i) {
        const float angle = twoPi * (float)i / (float)segments;
        verts[i + 1].position = {
            center.x + cosf(angle) * radius,
            center.y + sinf(angle) * radius
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
