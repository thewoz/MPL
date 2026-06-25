# MPL
###### Why more tools may do a better job than one multipurpose tool?

MPL is a Multi-Purpose Standard Library based on header files written in C++

## Usage

Run `make install` once: it symlinks `include/` to `/usr/local/include/mpl`, so
every header is reachable as `<mpl/<group>/<file>.hpp>`.

Include a single module:

```cpp
#include <mpl/math/geometry.hpp>
```

…or pull in the whole library through the umbrella header:

```cpp
#include <mpl/mpl.hpp>
```

`<mpl/mpl.hpp>` includes everything; modules that need heavy/optional third-party
backends can be excluded by defining a macro before the include
(`MPL_NO_CPLEX`, `MPL_NO_FITS`, `MPL_NO_SSH`, `MPL_NO_CURL`).

See [REFERENCE.md](REFERENCE.md) for a per-file list of the symbols each header provides.

## Files structure

Headers are grouped by theme under `include/`. Every header lives in the
`mpl::` namespace (the `mpl/<group>/<file>.hpp` include path mirrors the folder
layout). The two exceptions are `core/stdlib.hpp` and `core/bimap.hpp`,
which deliberately extend `namespace std`.

```
include/
├── core/         stdlib, log, opt, configure, profile, ssh, curl,
│                 bimap, graph, range, neighbors, stdio, loader,
│                 gnuplot, fits, params, algorithm
├── math/         utils, math, matrix, stat, histogram, kalman, fit, angles,
│                 rk4, cubicspline, lagrange, geometry, orientation,
│                 quaternion, kabsch, alphaShape, hull, mvee,
│                 multifit, clustering, cplex
└── vision/       camera, cameraSystem, convert, normalization, point4d,
                  reconstruction, reprojection, undistort, utils
```

###### core/
* `core/stdlib.hpp` - extensions to `std` (set operations, ...)
* `core/log.hpp` - logging (`mpl::log`)
* `core/opt.hpp` - command-line option parsing
* `core/configure.hpp` - configuration files
* `core/profile.hpp` - resource/profiling helpers
* `core/ssh.hpp` - SSH client wrapper (libssh)
* `core/curl.hpp` - libcurl wrapper (`mpl::web::curl`)
* `core/bimap.hpp` - bidirectional map (`std::bimap`)
* `core/graph.hpp` - graph data structure (`mpl::graph`)
* `core/range.hpp` - generic range (`mpl::range_t`)
* `core/neighbors.hpp` - nearest neighbors
* `core/stdio.hpp` - filesystem/IO helpers (`mpl::io`)
* `core/loader.hpp` - data loader
* `core/gnuplot.hpp` - gnuplot driver (gnuplot binary)
* `core/fits.hpp` - FITS support (cfitsio + Astrometry): `fits::solveField`, `cvFits`
* `core/params.hpp` - typed parameters
* `core/algorithm.hpp` - generic algorithm helper (`mpl::algorithm_t`)

###### math/
* `math/utils.hpp` - generic helpers (distances, min/max over containers, ...)
* `math/math.hpp` - `norm()`, `combinations()`, `polySolve()`, `solveCubic()`, `eigen()`, ...
* `math/matrix.hpp` - `Mat`/`Vec`/`Mat3`/`Mat4` wrappers over OpenCV `cv::Mat` with direct element access
* `math/stat.hpp` - running statistics (`mpl::stat_t`)
* `math/histogram.hpp` - histograms (`mpl::histo_t`, `mpl::histoInt_t`)
* `math/kalman.hpp` - Kalman filter (`mpl::kalman_t`)
* `math/fit.hpp` - curve fitting (`mpl::math::fit::parabola/linear/orear`)
* `math/angles.hpp` - degree/radian conversions (`mpl::math::angles`)
* `math/rk4.hpp` - Runge-Kutta integration (`mpl::numeric::integration`)
* `math/cubicspline.hpp` - cubic spline interpolation (`mpl::numeric::interpolation::cubicspline`)
* `math/lagrange.hpp` - Lagrange interpolation (`mpl::numeric::interpolation::lagrange`)
* `math/geometry.hpp` - barycenter, rotations, RTS, point/line/plane distances, ...
* `math/orientation.hpp`, `math/quaternion.hpp`
* `math/kabsch.hpp` - Kabsch best rigid transform
* `math/alphaShape.hpp` - 2D alpha shape
* `math/hull.hpp` - Andrew's monotone chain 2D convex hull
* `math/mvee.hpp` - minimum volume enclosing ellipsoid (`mpl::geometry::mvee`)
* `math/multifit.hpp` - non-linear least squares (OpenCV backend)
* `math/clustering.hpp` - `byDistance`, `NNDistance`, `kmeans`
* `math/cplex.hpp` - CPLEX wrapper
  * TODO: drop CPLEX in favor of GUROBI

###### vision/
* `vision/camera.hpp`, `vision/cameraSystem.hpp` - camera model and multi-camera systems
* `vision/convert.hpp`, `vision/normalization.hpp`, `vision/undistort.hpp`
* `vision/point4d.hpp` - homogeneous 4D point
* `vision/reconstruction.hpp`, `vision/reprojection.hpp`
* `vision/utils.hpp` - vision utilities (triangulation, ...)
