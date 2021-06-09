#include "Plotter.h"

double x; // global variables
Plotter p; // create plotter

void setup()
{
  p.Begin(); // start plotter
  
  p.AddTimeGraph( "Some title of a graph", 1500, "label for x", x ); // add any graphs you want
}

void loop()
{
  x = 10*sin( 2.0*PI*( millis() / 5000.0 ) ); // update your variables like usual

  p.Plot(); // plot all current data -- usually called within loop()
}
