// Size of cells
int cellSize = 10;

int Width = 50;
int Height = 19;

//directions

byte north = 1;
byte south = 2;
byte east = 3;
byte west = 4;
byte robotDirection = east;

// Colors for active/inactive cells
color alive = color(0, 200, 0);
color dead = color(0);

// Array of cells

int[][] cells; 
int[][] cellsBuffer; 
boolean[][] isStartOrEnd; 

// Pause
boolean pause = false;
void setup() {
  size (640, 360);
  cells = new int[Width][Height];
  //cellsBuffer = new int[Width/cellSize][height/cellSize];
  isStartOrEnd = new boolean[Width][Height];
  stroke(48);
  noSmooth();
  background(0); // Fill in black in case cells don't cover all the windows
  cells[1][1] = 1;
  cells[1][2] = 1;
  cells[1][3] = 1;
  cells[1][4] = 1;

  isStartOrEnd[1][1] = true;
  isStartOrEnd[1][4] = true;
}
void drawMap() {
  for (int x=0; x<Width; x++) {
    for (int y=0; y<Height; y++) {
      if (cells[x][y]==1) {
        fill(alive); // If alive
      } else {
        fill(dead); // If dead
      }
      rect (x*cellSize, y*cellSize, cellSize, cellSize);
    }
  }
}

void draw() {
  //iteration();
  solution(1000);
  drawMap();
}


void iteration() { // When the clock ticks

  // Visit each cell:
  for (int x=0; x<Width; x++) {
    for (int y=0; y<Height; y++) {
      // And visit all the neighbours of each cell
      int neighbours = 0; // We'll count the neighbours
      for (int xx=x-1; xx<=x+1; xx++) {
        for (int yy=y-1; yy<=y+1; yy++) {  
          if (((xx>=0)&&(xx<Width))&&((yy>=0)&&(yy<Height))) { // Make sure you are not out of bounds
            if (!((xx==x)&&(yy==y))) { // Make sure to to check against self
              if (xx == x || yy == y) {
                if (cells[xx][yy]==1) {
                  neighbours ++; // Check alive neighbours and count them
                }
              }
            } // End of if
          } // End of if
        } // End of yy loop
      } //End of xx loop
      // We've checked the neigbours: apply rules!
      if (cells[x][y]==1 && !isStartOrEnd[x][y]) { // The cell is alive: kill it if necessary
        if (neighbours < 2) {
          cells[x][y] = 0; // Die unless it has 2 or 3 neighbours
        }
      }
    } // End of y loop
  } // End of x loop
} // End of function
void solution(int step) {
  int x = 1;
  int y = 10;
  for (int i = 0; i < step; i++) {
    if (cells[x+1][y]==1) {
      cells[x][y]=2;
      x++;
    } else if (cells[x-1][y]==2) {
      cells[x][y]=1;
      x--;
    } else if (cells[x][y+1]==2) {      
      cells[x][y]=1;
      y++;
    } else if (cells[x][y-1]==2) {      
      cells[x][y]=1;
      y--;
    } else {
    }
  }
}