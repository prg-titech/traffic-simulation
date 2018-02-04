#include <stdio.h>
#include <SDL2/SDL2_gfxPrimitives.h>

#include "drawing.h"
#include "traffic.h"


Cell** free_cells;
int num_free_cells;

Cell** occupied_cells;
int num_occupied_cells;

SDL_Window* window;
SDL_Renderer* renderer;

double scale_factor_;

void init_gui(int num_cells, int size_x, int size_y, double scale_factor) {
  scale_factor_ = scale_factor;

  num_free_cells = 0;
  free_cells = new Cell*[num_cells];

  num_occupied_cells = 0;
  occupied_cells = new Cell*[num_cells];

  if (SDL_Init(SDL_INIT_VIDEO)) {
    printf("SDL_Init Error: %s", SDL_GetError());
    exit(1);
  }

  window = SDL_CreateWindow("SDL2_gfx test", 100, 100,
                            size_x, size_y, SDL_WINDOW_OPENGL); 
  if (window == NULL) { 
    printf("SDL_CreateWindow Error: %s", SDL_GetError());
    SDL_Quit();
    exit(2);
  }

  renderer = SDL_CreateRenderer(window, -1,
      SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (renderer == NULL) { 
    SDL_DestroyWindow(window);
    printf("SDL_CreateRenderer Error: %s", SDL_GetError());
    SDL_Quit();
    exit(3);
  }

  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
}

void update_gui() {
  SDL_Event e;
  if (SDL_PollEvent(&e))
  {
    if (e.type == SDL_QUIT) exit(0);
  }

  SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);

  for (; num_free_cells > 0; --num_free_cells) {
    filledCircleRGBA(renderer, free_cells[num_free_cells - 1]->x()*scale_factor_,
                     free_cells[num_free_cells - 1]->y()*scale_factor_, 3,
                     255, 255, 255, 255);
  }

  for (; num_occupied_cells > 0; --num_occupied_cells) {
    filledCircleRGBA(renderer, occupied_cells[num_occupied_cells - 1]->x()*scale_factor_,
                     occupied_cells[num_occupied_cells - 1]->y()*scale_factor_, 3,
                     255, 0, 0, 255);
  }

  SDL_RenderPresent(renderer);
  SDL_Delay(100);
}

void destroy_gui() {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

void draw_cell(Cell* cell) {
  if (cell->is_free()) {
    free_cells[num_free_cells++] = cell;
  } else {
    occupied_cells[num_occupied_cells++] = cell;
  }
}
