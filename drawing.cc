#include <graphics.h>

#include "traffic.h"

int gd = DETECT, gm;

void init_gui() {
  initgraph(&gd, &gm, NULL);
}

void draw_cell(Cell* cell) {
  if (cell->is_free()) {
    setcolor(WHITE);
    fillellipse(cell->x(), cell->y(), 1, 1);
  } else {
    setcolor(RED);
    fillellipse(cell->x(), cell->y(), 1, 1);
  }
}
