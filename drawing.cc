#include <algorithm>
#include <stdio.h>
#include <SDL2/SDL2_gfxPrimitives.h>

#include "drawing.h"
#include "traffic.h"


using namespace std;

Cell** cells_;
int num_cells_;

Cell** free_cells;
int num_free_cells;

Cell** occupied_cells;
int num_occupied_cells;

SDL_Window* window;
SDL_Renderer* renderer;

double scale_factor_;
double min_scale_factor_;
int size_x_;
int size_y_;

int mouse_pixel_x_ = 0;
int mouse_pixel_y_ = 0;
int mouse_world_x_ = 0;
int mouse_world_y_ = 0;

int origin_x_ = 0;
int origin_y_ = 0;

void redraw_everything() {
  update_gui();

  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
  SDL_RenderClear(renderer);
  for (int i = 0; i < num_cells_; ++i) {
    cells_[i]->draw();
  }
}

void init_gui(int num_cells, Cell** cells, int size_x, int size_y, double scale_factor) {
  min_scale_factor_ = scale_factor_ = scale_factor;
  size_x_ = size_x;
  size_y_ = size_y;

  num_cells_ = num_cells;
  cells_ = cells;

  num_free_cells = 0;
  free_cells = new Cell*[num_cells];

  num_occupied_cells = 0;
  occupied_cells = new Cell*[num_cells];

  if (SDL_Init(SDL_INIT_VIDEO)) {
    printf("SDL_Init Error: %s", SDL_GetError());
    exit(1);
  }

  window = SDL_CreateWindow("traffic_simulation", 100, 100,
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
  if (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_QUIT:
        exit(0);
        break;
      case SDL_MOUSEMOTION:
        mouse_pixel_x_ = e.motion.x;
        mouse_pixel_y_ = size_y_ - e.motion.y;
        mouse_world_x_ = origin_x_ + static_cast<double>(mouse_pixel_x_)/scale_factor_;
        mouse_world_y_ = origin_y_ + static_cast<double>(mouse_pixel_y_)/scale_factor_;
        break;
      case SDL_MOUSEWHEEL:
        if (e.wheel.y == 1) {
          // Scroll down (zoom out).
          if (scale_factor_ > min_scale_factor_) {
            origin_x_ -= 0.1*mouse_pixel_x_ / scale_factor_;
            origin_y_ -= 0.1*mouse_pixel_y_ / scale_factor_;
            scale_factor_ /= 1.1;
            redraw_everything();
          }
        } else if (e.wheel.y == -1) {
          // Scroll up (zoom in).
          origin_x_ += 0.1*mouse_pixel_x_/(1.1*scale_factor_);
          origin_y_ += 0.1*mouse_pixel_y_/(1.1*scale_factor_);
          scale_factor_ *= 1.1;
          redraw_everything();
        }
        break;
    }
  }

  SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);

  for (; num_free_cells > 0; --num_free_cells) {
    int pos_x = (free_cells[num_free_cells - 1]->x() - origin_x_)*scale_factor_;
    int pos_y = size_y_ - ((free_cells[num_free_cells - 1]->y() - origin_y_)*scale_factor_);

    if (pos_x >= 0 && pos_x < size_x_ && pos_y >= 0 && pos_y < size_y_)
    filledCircleRGBA(renderer, pos_x, pos_y, 2, 255, 255, 255, 255);
  }

  for (; num_occupied_cells > 0; --num_occupied_cells) {
    int pos_x = (occupied_cells[num_occupied_cells - 1]->x() - origin_x_)*scale_factor_;
    int pos_y = size_y_ - ((occupied_cells[num_occupied_cells - 1]->y() - origin_y_) *scale_factor_);

    if (pos_x >= 0 && pos_x < size_x_ && pos_y >= 0 && pos_y < size_y_)
    filledCircleRGBA(renderer, pos_x, pos_y, 2, 255, 0, 0, 255);
  }

  SDL_RenderPresent(renderer);
  //SDL_Delay(100);
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
