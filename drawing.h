#ifndef DRAWING_H
#define DRAWING_H

class Cell;

void init_gui(int num_cells, int size_x, int size_y, double scale_factor);

void update_gui();

void destroy_gui();

void draw_cell(Cell* cell);

#endif  // DRAWING_H
