#include <SDL3/SDL_pixels.h>
#include <SDL3/SDL_rect.h>
#include <SDL3/SDL_render.h>
#include <SDL3/SDL_video.h>
#include <iostream>
#include <SDL3/SDL.h>
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
    SDL_Color red;
    SDL_Color yellow;
    SDL_Color green;
    SDL_Color blue;
};

void cleanup(SDLState &state);
void drawCircle(SDL_Point center, int radius, SDL_Color colour, SDLState &state);
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
        step(points, constraints);

        // perform drawing commands
        setColour(catppuccin.base, state);
        SDL_RenderClear(state.renderer);

        for (Point &point : points) {
            drawCircle({(int) point.pos.x+width/2,(int) point.pos.y+height/2}, 25.0f, catppuccin.blue, state);
        }

        // swap buffers and present
        SDL_RenderPresent(state.renderer);
    }

    cleanup(state);
    return 0;
}

void drawCircle(SDL_Point center, int radius, SDL_Color colour, SDLState &state)
{
    setColour(colour, state);
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w; // horizontal offset
            int dy = radius - h; // vertical offset
            if ((dx*dx + dy*dy) <= (radius * radius)) {
                SDL_RenderPoint(state.renderer, center.x + dx, center.y + dy);
            }
        }
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
