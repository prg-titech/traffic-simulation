#include <algorithm>
#include <stdio.h>
#include <SDL2/SDL2_gfxPrimitives.h>

#include "drawing.h"
#include "traffic.h"

using namespace std;


Renderer::Renderer(Simulation* simulation, int window_size_x,
                   int window_size_y, double scale_factor)
    : min_scale_factor_(scale_factor), scale_factor_(scale_factor),
      window_size_x_(window_size_x), window_size_y_(window_size_y),
      num_free_cells_(0), free_cells_(new Cell*[simulation->num_cells()]),
      num_occupied_cells_(0), simulation_(simulation),
      occupied_cells_(new Cell*[simulation->num_cells()]) {
  // Initialize renderer.
  if (SDL_Init(SDL_INIT_VIDEO)) {
    printf("SDL_Init Error: %s", SDL_GetError());
    exit(1);
  }

  window_ = SDL_CreateWindow("traffic_simulation", 100, 100,
                            window_size_x, window_size_y, SDL_WINDOW_OPENGL); 
  if (window_ == NULL) { 
    printf("SDL_CreateWindow Error: %s", SDL_GetError());
    SDL_Quit();
    exit(2);
  }

  renderer_ = SDL_CreateRenderer(window_, -1,
      SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (renderer_ == NULL) { 
    SDL_DestroyWindow(window_);
    printf("SDL_CreateRenderer Error: %s", SDL_GetError());
    SDL_Quit();
    exit(3);
  }

  // Draw black background.
  SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 0);
  SDL_RenderClear(renderer_);
  SDL_RenderPresent(renderer_);
}


Renderer::~Renderer() {
  // TODO: Use unique_ptr.
  delete[] free_cells_;
  delete[] occupied_cells_;

  SDL_DestroyRenderer(renderer_);
  SDL_DestroyWindow(window_);
  SDL_Quit();
}


void Renderer::redraw_everything() {
  num_free_cells_ = 0;
  num_occupied_cells_ = 0;

  SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 0);
  SDL_RenderClear(renderer_);

  auto& streets = simulation_->streets();
  // Draw streets.
  for (int i = 0; i < streets.size(); ++i) {
    int pos1_x = (get<0>(streets[i]) - origin_x_)*scale_factor_;
    int pos1_y = window_size_y_ - ((get<1>(streets[i]) - origin_y_)
                                   * scale_factor_);
    int pos2_x = (get<2>(streets[i]) - origin_x_)*scale_factor_;
    int pos2_y = window_size_y_ - ((get<3>(streets[i]) - origin_y_)
                                   * scale_factor_);

    if ((pos1_x >= 0 && pos1_x < window_size_x_ &&
        pos1_y >= 0 && pos1_y < window_size_y_) ||
        (pos2_x >= 0 && pos2_x < window_size_x_ &&
        pos2_y >= 0 && pos2_y < window_size_y_)) {
      thickLineRGBA(renderer_, pos1_x, pos1_y, pos2_x, pos2_y, 3,
                    255, 255, 255, 255);
    }
  }

  for (int i = 0; i < simulation_->num_cells(); ++i) {
    // Only draw occupied cells.
    if (!simulation_->cells()[i]->is_free()) {
      simulation_->cells()[i]->draw();
    }
  }
}


void Renderer::update_gui() {
  SDL_Event e;
  if (SDL_PollEvent(&e)) {
    switch (e.type) {
      case SDL_QUIT:
        exit(0);
        break;
      case SDL_MOUSEMOTION:
        mouse_pixel_x_ = e.motion.x;
        mouse_pixel_y_ = window_size_y_ - e.motion.y;
        mouse_world_x_ = origin_x_ +
                         static_cast<double>(mouse_pixel_x_)/scale_factor_;
        mouse_world_y_ = origin_y_ +
                         static_cast<double>(mouse_pixel_y_)/scale_factor_;
        break;
      case SDL_MOUSEWHEEL:
        if (e.wheel.y == 1) {
          // Scroll down (zoom out).
          if (scale_factor_ > min_scale_factor_) {
            origin_x_ -= 0.1*mouse_pixel_x_/scale_factor_;
            origin_y_ -= 0.1*mouse_pixel_y_/scale_factor_;
            scale_factor_ /= 1.1;
            redraw_everything();
            return;
          }
        } else if (e.wheel.y == -1) {
          // Scroll up (zoom in).
          scale_factor_ *= 1.1;
          origin_x_ += 0.1*mouse_pixel_x_/scale_factor_;
          origin_y_ += 0.1*mouse_pixel_y_/scale_factor_;
          redraw_everything();
          return;
        }
        break;
    }
  }

  SDL_SetRenderDrawColor(renderer_, 0, 0, 255, 255);

  for (; num_free_cells_ > 0; --num_free_cells_) {
    int pos_x = (free_cells_[num_free_cells_ - 1]->x() - origin_x_)
                * scale_factor_;
    int pos_y = window_size_y_ - ((free_cells_[num_free_cells_ - 1]->y()
                                   - origin_y_)
                                  * scale_factor_);

    if (pos_x >= 0 && pos_x < window_size_x_ &&
        pos_y >= 0 && pos_y < window_size_y_)
      filledCircleRGBA(renderer_, pos_x, pos_y, 2, 255, 255, 255, 255);
  }

  for (; num_occupied_cells_ > 0; --num_occupied_cells_) {
    int pos_x = (occupied_cells_[num_occupied_cells_ - 1]->x() - origin_x_)
                * scale_factor_;
    int pos_y = window_size_y_ - ((occupied_cells_[num_occupied_cells_ - 1]->y()
                                   - origin_y_)
                                  * scale_factor_);

    if (pos_x >= 0 && pos_x < window_size_x_ &&
        pos_y >= 0 && pos_y < window_size_y_)
    filledCircleRGBA(renderer_, pos_x, pos_y, 2, 255, 0, 0, 255);
  }

  SDL_RenderPresent(renderer_);
  //SDL_Delay(100);
}


void Renderer::draw_cell(Cell* cell) {
  if (cell->is_free()) {
    free_cells_[num_free_cells_++] = cell;
  } else {
    occupied_cells_[num_occupied_cells_++] = cell;
  }
}
