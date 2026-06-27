# MPL – API Reference

This page summarizes the main symbols available in the library headers.
All headers are header-only and live under the `include/` directory, grouped by
theme. Unless noted otherwise, every symbol lives in the `mpl` namespace and is
reachable as `<mpl/<group>/<file>.hpp>` (or all at once through the umbrella
header `<mpl/mpl.hpp>` — see the [README](README.md)).

Most files depend on OpenCV (`cv::Mat`, `cv::Point_`, …). Heavy/optional
third-party backends are noted per file and can be excluded with the
`MPL_NO_CPLEX`, `MPL_NO_FITS`, `MPL_NO_SSH`, `MPL_NO_CURL` macros.

## core

### `core/utils.hpp` (`mpl::utils`)

- `average(...)` – mean of a set of values/points.
- `median(...)` – median of a set of values/points.
- `defaultDistanceOp()` – default distance functor used by the helpers below.
- `distanceAvg(...)` – average distance between points.
- `distance(...)` – minimum distance: point↔set, set↔set, or from the barycenter (overloaded).
- `NNDistance(...)` – median minimum first-nearest-neighbor distance.

### `core/stdlib.hpp` (`namespace std`, deliberate extension)

- `normalize()` – normalize a container in place.
- `bool2str(...)` – boolean → `"true"`/`"false"`.
- `itoa(...)` – integer → string (overloaded).
- `decToHex()` – decimal → hexadecimal string.
- `ceilToSignificantFigures(...)` – round up to N significant figures.
- `parse(...)` – parse a string into a value (overloaded).
- `intersection` / `difference` / `equal` – set operations (in `std::util_set`).
- `isEqual()` – element-wise equality of two containers.
- `exec(...)` – run a shell command and capture its output.

### `core/log.hpp` (`mpl::log`)

- `init` / `flush` / `close` – logger lifecycle.
- `msn` / `warning` / `error` – emit messages by severity (`error` exits the program).
- `chapterStart` / `chapterEnd` – sectioned output.
- `increaseIndentation` / `decreaseIndentation` – indentation control.
- `setOutputOnScreen` / `getOutput` – output configuration.

### `core/opt.hpp` (`mpl::opt`)

Command-line option parser.

- `opt::param_t` – single option descriptor.
- `add()` – register an option.
- `init()` / `set()` – parse `argv` / set values.
- `get()` / `getInt()` / `getReal()` / `getList()` / `getListPairs()` / `getRange()` / `getPoint()` / `getSize()` / `getArgs()` – typed accessors.
- `isDefined()` / `isEqual()` / `find()` / `check()` – queries.
- `usage()` / `getInfo()` / `dump()` – help/diagnostics.

### `core/configure.hpp` (`mpl::configure`)

Configuration-file handling.

- `init` / `update` / `load` / `print` – lifecycle and I/O.
- `addKey` / `setParam` / `getParam` – build/populate config.
- `getInt` / `getDouble` / `getBool` / `getString` / `getRange` / `getList` – typed getters.
- `haveKey` / `isDefined` / `isEqual` – queries.

### `core/profile.hpp` (`mpl::profile`)

- `start` / `enlapse` / `done` – time profiling.
- `initMemory` / `logMemory` / `memUsage` / `memoryPeakGB` – memory profiling.
- `process_mem_usage()` – current/peak resident memory.
- `setOutputOnScreen` / `close` – output configuration.

### `core/ssh.hpp` (`mpl::ssh_t`) · **libssh**

- `ssh_t` – SSH client.
- `connect()` / `close()` / `available()` – connection lifecycle/status.
- `command()` – run a remote command.

### `core/curl.hpp` (`mpl::web::curl`) · **libcurl**

- `get()` – HTTP GET (overloaded: return body / write to file).

## container

### `container/bimap.hpp` (`namespace std`, deliberate extension)

Class `std::bimap<Left,Right>`:

- `insert` / `find_left` / `find_right` / `contains` – insert / lookup by either side / membership.
- `erase_left` / `erase_right` / `clear` – remove entries.
- `size` / `empty` – size queries; supports iteration.

### `container/graph.hpp` (`mpl::graph`)

- `graph::node_t` – node: `isRoot`/`isTail`, `addParents`/`addChildren`, `size`, `operator[]`.
- `graph_t` – graph container: `addNode`, `createTracks`, `buildTracksForward`, `save`.

### `container/range.hpp` (`mpl`)

- `range_t<T>` – generic numeric range.

## math

### `math/math.hpp` (`mpl::math`)

- `norm(...)` – norm / distance for points and vectors (overloaded).
- `combinations()` – all K-of-N combinations (optionally random, capped).
- `polySolve()` – real/complex roots of a polynomial (overloaded).
- `solveCubic()` – real roots of a cubic equation.
- `svd()` – singular value decomposition.
- `eigen()` – eigenvalues and eigenvectors of a matrix.
- `normal_pdf()` / `multivariate_normal_pdf()` – Gaussian PDFs.

### `math/matrix.hpp` (`mpl`)

Thin wrappers over `cv::Mat` with direct element access.

- `Mat` – generic matrix with `operator()`, `resize`, `operator=`.
- `Vec` – generic vector.
- `Mat3` – 3×3 double matrix.
- `Mat4` – 4×4 double matrix with `operator*`.
- `cv::mat::printMat` / `println` – pretty-printing helpers.

### `math/stat.hpp` (`mpl`)

- `stat_t` – running statistics (mean/variance/std, Welford-style `add`).

### `math/histogram.hpp` (`mpl`)

- `histoInt_t` – integer-binned histogram (`binning`, `print`).
- `histo_t` – floating-point histogram.

### `math/kalman.hpp` (`mpl`)

- `kalman_t` – Kalman filter: `init`, `predict`, `correct`, `drawPrediction`.

### `math/fit.hpp` (`mpl::math::fit`)

- `parabola()` – least-squares parabola fit (SVD).
- `linear()` – least-squares line fit; returns the residual error (overloaded for points or x/y).
- `orear()` – iterative weighted line fit (errors on both axes), returns error and energy.

### `math/angles.hpp` (`mpl::math::angles`)

- `radians(deg[,min,sec])` – degrees (optionally d/m/s) → radians.
- `degrees(rad)` – radians → degrees.
- `utils::deg2rad` / `utils::rad2deg` – conversion constants.

### `math/rk4.hpp` (`mpl::numeric::integration`)

- `rk4<N>(u0, dt, ut, f)` – one 4th-order Runge–Kutta step for an N-dimensional ODE system.

### `math/cubicspline.hpp` (`mpl::numeric::interpolation::cubicspline`)

- `fit(x, y)` – fit cubic-spline coefficients to sample points.
- `interpolate(x, coefficients)` – evaluate the spline at `x`.

### `math/lagrange.hpp` (`mpl::numeric::interpolation::lagrange`)

- `findCoefficients(x, y)` – Lagrange polynomial coefficients through the samples.
- `interpolate(x, coefficients)` – evaluate the polynomial at `x`.

## solver

### `solver/multifit.hpp` (`mpl::solver::multifit`)

- `data_t` / `params_t` – input data and fit parameters.
- `solve()` – non-linear least squares (function + Jacobian callbacks).
- `callback()` – per-iteration callback hook.

### `solver/cplex.hpp` (`mpl::cplex`) · **IBM CPLEX**

- `utils::data_t` – problem data.
- `minimizeSoft()` / `minimizeHard()` – minimum cover-set ILP (soft/hard constraints).

## cluster

### `cluster/clustering.hpp` (`mpl::clustering`)

- `connectedComponents()` – distance-based clustering.
- `connectedComponentsMedianFirstNN()` – clustering by median first-NN distance.
- `kmeans()` – k-means.
- `gmm()` – Gaussian mixture model.
- `mahalanobis()` – Mahalanobis distance.
- `nearestNeighbor()` – nearest-neighbor assignment.
- `dbscan()` – DBSCAN density clustering.

### `cluster/neighbors.hpp` (`mpl::neighbors`)

- `findNeighbors()` – neighbor search.
- `metric()` – metric (geometric) neighbors.
- `topological()` – topological neighbors.
- `dist_t` – distance record.

## geometry

### `geometry/geometry.hpp` (`mpl::geometry`)

- `barycenter()` – centroid of a point set.
- `dotAngle()` – angle between vectors via dot product.
- `computeRotationalMatrix()` – rotation matrix from Euler angles (point/vec/array overloads).
- `applyRTS()` – apply rotation+translation+scale to points.
- `rotate2D()` / `rotate()` – 2D/3D rotation of points.
- `isInside()` – point-in-region test.
- `distance(...)` – point↔point, point↔line (squared), point4D↔plane distances.
- `findBestRTS<dim>()` – best rigid+scale transform between two point sets.
- `minDist()` / `minDistLines()` – minimum distance between contours / lines.
- `rotationX/Y/Z()` – axis rotation matrices for a given angle.
- `getTranslationFromSphericalAngle()` – translation vector from spherical angles.
- `getAxes()` – principal axes of a point set.

### `geometry/orientation.hpp` (`mpl::geometry`)

- `orientation_t` – orientation result (axes/angle).
- `orientation::computationMethod` – enum selecting the estimation method.
- `orientation::utils::*` – `extractNonZeroPixels`, `createContourMask`, `normalizeAngle180`, `computeProjectedLengthsFromMask/Points`, `setAxesFromAngle`.

### `geometry/quaternion.hpp` (`mpl::geometry`)

- `quaternion` – quaternion (w,x,y,z / q4,q1,q2,q3); `getRotationMatrix`, `getAngles`, `print`/`println`.

### `geometry/kabsch.hpp` (`mpl::geometry::kabsch`)

- `info_t` / `info2D_t` / `info3D_t` – result holders (p0, R, T, S).
- `solve()` / `solve2D()` / `solve3D()` – Kabsch best rigid transform between point sets.

### `geometry/alphaShape.hpp` (`mpl::geometry`)

- `alphaShape()` – 2D alpha shape; returns the ordered boundary points.
- `utils::*` – grid hashing helpers (`gridKey_t`, `gridKeyHash_t`, …).

### `geometry/hull.hpp` (`mpl::geometry`)

- `hull(points)` – 2D convex hull (Andrew's monotone chain), CCW order.

### `geometry/mvee.hpp` (`mpl::geometry::mvee`)

- `fit()` – minimum volume enclosing ellipsoid (Khachiyan).
- `util::*` – `prepareInputPoints`, `invertSymmetricMatrix`, `computeMVEE`.

## io

### `io/stdio.hpp` (`mpl::io`)

Filesystem and path utilities (built on `std::filesystem`).

- `expandPath()` – expand `~`/relative paths (many overloads).
- `dirname` / `basename` / `extension` / `name` – path component extraction.
- `exists` / `isFile` / `isDirectory` – existence/type checks.
- `cp` / `mkdir` / `remove` / `ls` – file operations and listing.
- `open` / `openf` / `close` / `openTempFile` – file handle helpers.
- `subdir` / `subdirName` / `getDirectoryPath` / `getFilePath` – directory traversal helpers.
- `areFilesEqual` – byte-compare two files.

### `io/loader.hpp` (`mpl::loader`)

- `loader` – tabular data loader.
- `add` / `getColsToRead` – declare columns to read.
- `load` / `operator()` – load and access data.
- `min` / `max` / `minmax` / `tune` – column statistics / scaling.

## cli

### `cli/params.hpp` (`mpl`)

Typed parameter framework.

- `param_t<T>` / `param_base_t` – typed parameter base.
- `options_t` / `range_t<T>` / `list_t<T>` – constrained parameter kinds.
- `int_t` / `real_t` / `bool_t` / `string_t` / `image_t` – concrete parameter types.
- `params_t` – parameter set: `min`/`max`/`check` by key.

### `cli/algorithm.hpp` (`mpl`)

- `algorithm_t` – generic algorithm helper (works with `mpl/cli/params.hpp`).

## external

Thin wrappers over external tools / libraries.

### `external/gnuplot.hpp` (`mpl::gnuplot`) · **gnuplot binary**

- `gnuplot` – pipe to a gnuplot process.
- `sendLine` – send a raw gnuplot command.
- `setTitle` / `setGrid` / `setXLabel` / `setYLabel` – plot configuration.
- `setLogX` / `setLogY` / `unsetLogX` / `unsetLogY` – log-scale toggles.

### `external/fits.hpp` (`mpl::fits`) · **cfitsio + Astrometry**

- `fits::solveField()` – astrometric field solving of an image.
- `cvFits` – open a FITS file and expose it as a `cv::Mat` (`load`, header parsing).

## vision

### `vision/point4d.hpp` (`mpl::vision`)

- `point4d_t` – homogeneous 4D point (x,y,z,w).

### `vision/camera.hpp` (`mpl::vision`)

Class `camera_t` (pinhole camera model):

- focal length / optical center / omega – `get`/`set` intrinsics.
- sensor (size + pixel size) – `get`/`set` sensor parameters.
- distortion coefficients – `get`/`set` distortion.
- translation / rotation angle – `set`/`update`/`get` extrinsics.
- projection matrix – `get`/`set`; plus OpenGL projection & modelView matrices.

### `vision/cameraSystem.hpp` (`mpl::vision`)

- `cameraSystem_t<N>` – N-camera rig (`cameraSystem1_t`/`2_t`/`3_t`).
- `initFundamentalMatrix()` – compute pairwise fundamental matrices.
- `undistort()` – undistort with a camera's intrinsics.
- `reproject()` – project a 3D point into all cameras.
- `reconstruct()` – triangulate from 2D correspondences.

### `vision/convert.hpp` (`mpl::vision::convert`)

- `homogeneous()` – Cartesian → homogeneous (2D→3D, 3D→`point4d_t`).

### `vision/normalization.hpp` (`mpl::vision::normalization`)

- `internalParameters()` – normalize using intrinsics.
- `isotropic()` – isotropic normalization (Hartley & Zisserman p.107).
- `onBarycenter()` – center on the barycenter.

### `vision/undistort.hpp` (`mpl::vision`)

- `undistort()` – lens undistortion (many overloads for points/images).

### `vision/reprojection.hpp` (`mpl::vision`)

- `reproject()` – project 3D points or `point4d_t` to 2D with a projection matrix (overloaded).

### `vision/reconstruction.hpp` (`mpl::vision`)

- `reconstruct()` – triangulate a 3D/4D point from 2+ views (SVD-based).
- `reconstructionError()` – reconstruction/reprojection error (optional `maxError` early-out).

### `vision/utils.hpp` (`mpl::vision`)

Multi-view geometry toolbox.

- `initProjectionMatrix` / `loadProjectionMatrix` – build/load projection matrices.
- `initCameraMatrix` / `loadCameraMatrix` – build/load camera (intrinsic) matrices.
- `getCameraCenter()` – camera center from a projection matrix (H&Z p.156).
- `findPlanePassingBy()` – plane through given points.
- `essentialMatrixLinear()` / `RTFromEssential()` – essential matrix and pose recovery (SVD).
- `fundamentalMatrixFromEightPoints()` – 8-point fundamental matrix.
- `fundamentalFromProjections()` – fundamental matrix from a pair of projection matrices.
- `optimalTriangulation()` – optimal triangulation (H&Z).
- `epipolarLines()` – epipolar lines for a point.
- `anglesFromRTMatrix()` / `getRT()` – pose/angle extraction.
- `alignment()` – align point sets.
- `pseudoInverse()` – Moore–Penrose pseudo-inverse.
- `isOnConic()` – conic membership test.

## munkres

### `munkres/munkres.hpp` (`namespace munkres`, external library)

- `Munkres<T>` – Hungarian / Munkres optimal assignment; `solve()` on a cost matrix.

External vendored library (with matrix adapters under `munkres/include/`), kept
as-is during the reorganization.
