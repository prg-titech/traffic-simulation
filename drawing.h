#ifndef DRAWING_H
#define DRAWING_H

#include <vector>

class SDL_Window;
class SDL_Renderer;
class Cell;
class Street;
class Simulation;

class Renderer {
 public:
  Renderer(Simulation* Simulation, int window_size_x, int window_size_y,
           double scale_factor);

  ~Renderer();

  void update_gui();

  void draw_cell(Cell* cell);

  void redraw_everything();

  void add_street(Street* street);

 private:

  void draw_updates();

  Simulation* simulation_;

  // A list of cells that became free in this iteration.
  Cell** free_cells_;
  int num_free_cells_;

  // A list of cells that became occupied in this iteration.
  Cell** occupied_cells_;
  int num_occupied_cells_;

  // SDL helper variables.
  SDL_Window* window_;
  SDL_Renderer* renderer_;

  // Scaling factor (higher = larger).
  double scale_factor_;

  // The minimum scaling factor.
  double min_scale_factor_;

  // X size of the window in pixels.
  int window_size_x_;
  // Y size of the window in pixels.
  int window_size_y_;

  // X coordinate of the mouse pointer in pixels.
  int mouse_pixel_x_ = 0;
  // Y coordinate of the mouse pointer in pixels.
  int mouse_pixel_y_ = 0;
  // X coordinate of the mouse pointer in world coordinates.
  int mouse_world_x_ = 0;
  // Y coordinate of the mouse pointer in world coordinates.
  int mouse_world_y_ = 0;

  // Upper left corner of the visible area in world coordinates.
  int origin_x_ = 0;
  int origin_y_ = 0;
};

#endif  // DRAWING_H
