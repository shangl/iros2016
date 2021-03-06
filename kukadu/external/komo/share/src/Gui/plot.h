/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#ifndef MT_plot_h
#define MT_plot_h

//===========================================================================

struct OpenGL;
struct Gaussian;
namespace MT {  template<class T> struct Array;  }

typedef unsigned int uint;
typedef MT::Array<double> arr;
typedef MT::Array<uint> uintA;
typedef MT::Array<Gaussian> GaussianA;
typedef MT::Array<Gaussian*> GaussianL;

//===========================================================================

typedef enum { opengl, xfig, gnupl } PlotMode;

struct PlotModule {
  struct sPlotModule *s;
  PlotMode mode;
  OpenGL *gl;
  bool light, grid, colors, drawBox, drawDots, perspective;
  uint thickLines;//display options
  PlotModule();
  ~PlotModule();
};
extern PlotModule plotModule;

//===========================================================================

void plotGnuplot();
void plotOpengl();
void plotOpengl(bool perspective, double xl=-1., double xh=1., double yl=-1., double yh=1., double zl=-1., double zh=1.);
void plot(bool wait=true, const char* txt=0);

void plotClear();
void plotFunction(const arr& f, double x0=0., double x1=0.);
void plotFunctionPoints(const arr& f, double x0=0., double x1=0.);
void plotFunctions(const arr& f, double x0=0., double x1=0.);
void plotFunction(const arr& x, const arr& f);
void plotFunctionPrecision(const arr& x, const arr& f, const arr& h, const arr& l);
void plotSurface(const arr& X);
void plotArray(const arr& X);
void plotPoint(double x, double y, double z);
void plotPoint(const arr& x);
void plotPoints(const arr& X);
void plotClearPoints();
void plotLine(const arr& X);
void plotPoints(const arr& X, const arr& Y);
void writeGnuplotFiles();
void plotCovariance(const arr& mean, const arr& cov);
void plotVectorField(const arr& X, const arr& dX);
void plotVectorField(arr& dX);
void plotMatrixFlow(uintA& M, double len);
void plotGaussians(const GaussianA& G);
void plotGaussians(const GaussianL& G);

void glDrawPlot(void *module);

#endif

