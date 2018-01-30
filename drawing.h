#ifndef DRAWING_H
#define DRAWING_H

class Cell;

void init_gui(int num_cells);

void update_gui();

void destroy_gui();

void draw_cell(Cell* cell);

#endif  // DRAWING_H
