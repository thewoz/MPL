# MPL
###### Why more tools may do a better job than one multipurpose tool?

MPL is a Multi-Purpose Standard Library based on header files written in C++

## Files structures

###### Clustering Algorithms:
* clustering::byDistance - clustering algorthim based on distance
* clustering::NNDistance - clustering algorthim based on first near neighborhood distance
* clustering::kmeans - clustering algorthim that implement k-means


###### Configure:

* XXX

###### CPLEX:

* XXX

###### Debug:

* The file contains macros for debuging


###### FITS:

Based on the fitsio library and Astrometry

* fits::solveField - Used to solve the filed of an image
* cvFits - Class to open a fits and convert in a  OpenCV cv::Mat


###### Geometry:

XXX

###### Integration:

* Implementation of the Runge-Kutta algorithm

###### Interpolation:

* Implementation of the cubic spline algorithm
* Implementation of the Lagrange algorithm

###### Loader:

* XXX

###### Log:

* XXX

###### Matrix:

Some of the classes and functions wrap functions from other libraries for easy to use reasons

* Implementation of a Matrix class based on OpenCV cv::Mat with direct element access
* Implementation of a Vec class based on OpenCV cv::Mat with direct element access
* Implementation of a 3x3 matrix class based on OpenCV
* Implementation of a 4x4 matrix class based on OpenCV

###### Math:

* polySolve( ) - The funciton finds the real or complex roots of a polynomial equation
* solveCubic( ) - The funciton finds the real roots of a cubic equation
* eigen( ) - The funciton finds the eigenvalues and eigenvectors of a matrix 
* combinations() - Find the K of N elementes

###### Munkres:

Implementation of the Munkres algorithm

###### OpenCV:

XXX


###### Opt:

XXX

###### Params:

XXX

###### Profile:

XXX

###### Solver:

XXX

###### Stat:

XXX

###### Stdio:

XXX

###### Stdlib:

XXX

###### Utils:

XXX


###### Vision:

XXX

###### Web:

XXX

## Esempio solver CPLEX / Gurobi

È stato aggiunto un esempio in `src/solver_example.cpp`.

Compilazione con CPLEX:

```bash
g++ -std=c++17 -DMPL_ENABLE_CPLEX -I./include src/solver_example.cpp -o ~/bin/solver_cplex -I/Applications/CPLEX_Studio/cplex/include/ -I/Applications/CPLEX_Studio/concert/include/ -DIL_STD -L/Applications/CPLEX_Studio/cplex/lib/x86-64_osx/static_pic/ -L/Applications/CPLEX_Studio/concert/lib/x86-64_osx/static_pic/ -lilocplex -lcplex -lconcert -lm -lpthread -ld_classic -rpath /usr/local/lib/
```

Compilazione con Gurobi:

```bash
g++ -std=c++17 -DMPL_ENABLE_GUROBI -I./include src/solver_example.cpp -o ~/bin/solver_gurobi -I/Library/gurobi1301/macos_universal2/include -L/Library/gurobi1301/macos_universal2/lib  -lgurobi_c++ -lgurobi130 -ld_classic -rpath /usr/local/lib/ 
```

Esecuzione:

```bash
./bin/solver_cplex cplex hard
./bin/solver_gurobi gurobi soft
```
