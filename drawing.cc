#include <algorithm>
#include <stdio.h>
#include <SDL2/SDL2_gfxPrimitives.h>

#include "drawing.h"
#include "traffic.h"

using namespace std;


bool gui_output = true;
bool quick_draw_streets = false;

Renderer::Renderer(Simulation* simulation, int window_size_x,
                   int window_size_y, double scale_factor)
    : min_scale_factor_(scale_factor), scale_factor_(scale_factor),
      window_size_x_(window_size_x), window_size_y_(window_size_y),
      num_free_cells_(0), num_occupied_cells_(0), simulation_(simulation),
      free_cells_(new const Cell*[simulation->cells().size()]),
      occupied_cells_(new const Cell*[simulation->cells().size()]) {

  if (!gui_output) return;

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

  if (!gui_output) return;
  SDL_DestroyRenderer(renderer_);
  SDL_DestroyWindow(window_);
  SDL_Quit();
}


uint32_t cell_color(Cell::Type type) {
  uint32_t s = 255-32*type;
  return 0xffff00ff + (s << 8);
}


void Renderer::redraw_everything() {
  num_free_cells_ = 0;
  num_occupied_cells_ = 0;

  if (!gui_output) return;
  SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 0);
  SDL_RenderClear(renderer_);

  auto& streets = simulation_->streets();
  int line_w = 4*scale_factor_/0.2;
  line_w = line_w == 0 ? 1 : line_w;

  // Draw streets.
  if (quick_draw_streets) {
    for (int i = 0; i < streets.size(); ++i) {
      int pos1_x = (streets[i]->first()->x() - origin_x_)*scale_factor_;
      int pos1_y = window_size_y_ - ((streets[i]->first()->y() - origin_y_)
                                     * scale_factor_);
      int pos2_x = (streets[i]->last()->x() - origin_x_)*scale_factor_;
      int pos2_y = window_size_y_ - ((streets[i]->last()->y() - origin_y_)
                                     * scale_factor_);

      if ((pos1_x >= 0 && pos1_x < window_size_x_ &&
          pos1_y >= 0 && pos1_y < window_size_y_) ||
          (pos2_x >= 0 && pos2_x < window_size_x_ &&
          pos2_y >= 0 && pos2_y < window_size_y_)) {
        thickLineColor(renderer_, pos1_x, pos1_y, pos2_x, pos2_y, line_w,
                       cell_color(streets[i]->type()));
      }
    }
  } else {
    // Draw all segments.
    Cell** cells = simulation_->cells().data();
    for (int i = 0; i < simulation_->cells().size(); ++i) {
      Cell* cell = cells[i];
      int pos1_x = (cell->x() - origin_x_)*scale_factor_;
      int pos1_y = window_size_y_ - ((cell->y() - origin_y_)
                                     * scale_factor_);

      for (int j = 0; j < cell->outgoing_cells().size(); ++j) {
        Cell* next_cell = cell->outgoing_cells()[j];
        int pos2_x = (next_cell->x() - origin_x_)*scale_factor_;
        int pos2_y = window_size_y_ - ((next_cell->y() - origin_y_)
                                       * scale_factor_);

        if (pos1_x > 0 && pos1_x < window_size_x_ &&
            pos1_y > 0 && pos1_y < window_size_y_ &&
            pos2_x > 0 && pos2_x < window_size_x_ &&
            pos2_y > 0 && pos2_y < window_size_y_) {
          thickLineColor(renderer_, pos1_x, pos1_y, pos2_x, pos2_y, 1,
                         cell_color(cell->type()));
        }
      }
    }
  }

  for (int i = 0; i < simulation_->cells().size(); ++i) {
    // Only draw occupied cells.
    if (!simulation_->cells()[i]->is_free()) {
      simulation_->cells()[i]->draw();
    }
  }
}


void Renderer::draw_updates() {
  SDL_SetRenderDrawColor(renderer_, 0, 0, 255, 255);
  double cell_w = 2*scale_factor_/0.2;

  for (; num_free_cells_ > 0; --num_free_cells_) {
    int pos_x = (free_cells_[num_free_cells_ - 1]->x() - origin_x_)
                * scale_factor_;
    int pos_y = window_size_y_ - ((free_cells_[num_free_cells_ - 1]->y()
                                   - origin_y_)
                                  * scale_factor_);

    if (pos_x >= 0 && pos_x < window_size_x_ &&
        pos_y >= 0 && pos_y < window_size_y_) {
      if (free_cells_[num_free_cells_ - 1]->is_sink()) {
        filledCircleRGBA(renderer_, pos_x, pos_y, cell_w*4,
                         0, 255, 0, 255);
      } else {
        filledCircleColor(renderer_, pos_x, pos_y, cell_w,
                          cell_color(free_cells_[num_free_cells_ - 1]->type()));
      }
    }
  }

  for (; num_occupied_cells_ > 0; --num_occupied_cells_) {
    int pos_x = (occupied_cells_[num_occupied_cells_ - 1]->x() - origin_x_)
                * scale_factor_;
    int pos_y = window_size_y_ - ((occupied_cells_[num_occupied_cells_ - 1]->y()
                                   - origin_y_)
                                  * scale_factor_);

    if (pos_x >= 0 && pos_x < window_size_x_ &&
        pos_y >= 0 && pos_y < window_size_y_) {
      if (occupied_cells_[num_occupied_cells_ - 1]->is_sink()) {
        // End of map.
        filledCircleRGBA(renderer_, pos_x, pos_y, cell_w, 20, 200, 85, 255);
      } else if (occupied_cells_[num_occupied_cells_ - 1]->car() == nullptr) {
        // Seems like this happens only when zooming...
        // printf("ERR!\n");
      } else if (!occupied_cells_[num_occupied_cells_ - 1]->car()->is_active()) {
        // Not active: GREEN
        filledCircleRGBA(renderer_, pos_x, pos_y, cell_w, 20, 200, 85, 255);
      } else if (occupied_cells_[num_occupied_cells_ - 1]->tag()
                 && Cell::kTurnLane) {
        // Turn lane: GREEN
        filledCircleRGBA(renderer_, pos_x, pos_y, cell_w, 20, 200, 85, 255);
      } else if (occupied_cells_[num_occupied_cells_ - 1]->car()->is_jammed()) {
        // Jammed (next cell not free): BLUE
        filledCircleRGBA(renderer_, pos_x, pos_y, cell_w, 20, 20, 220, 255);
      } else if (occupied_cells_[num_occupied_cells_ - 1]->max_velocity() == 0) {
        // Not allowed to move: GREENish
        filledCircleRGBA(renderer_, pos_x, pos_y, cell_w, 20, 100, 100, 255);
      } else if (occupied_cells_[num_occupied_cells_ - 1]->car()->path().size() > 0
          && occupied_cells_[num_occupied_cells_ - 1]->car()->path().front()->max_velocity() == 0) {
        // Next cell on path has velocity 0: LIGHT BLUE
        filledCircleRGBA(renderer_, pos_x, pos_y, cell_w, 20, 200, 200, 255);
      } else if (occupied_cells_[num_occupied_cells_ - 1]->car()->velocity() == 0) {
        // Not moving for some reason: YELLOW
        filledCircleRGBA(renderer_, pos_x, pos_y, cell_w, 200, 200, 0, 255);
      } else {
        // Normal: RED
        filledCircleRGBA(renderer_, pos_x, pos_y, cell_w, 255, 0, 0, 255);
      }
    }
  }
}


void Renderer::update_gui() {
  if (!gui_output) {
    num_free_cells_ = num_occupied_cells_ = 0;
    return;
  }

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

  draw_updates();
  SDL_RenderPresent(renderer_);
  //SDL_Delay(100);
}


void Renderer::draw_cell(const Cell& cell) {
  if (cell.is_free()) {
    free_cells_[num_free_cells_++] = &cell;
  } else {
    occupied_cells_[num_occupied_cells_++] = &cell;
  }
}
