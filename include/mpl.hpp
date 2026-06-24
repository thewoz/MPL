/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017-2026
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
/*
 * mpl.hpp - umbrella header that pulls in the whole MPL library.
 *
 *   #include <mpl/mpl.hpp>
 *
 * Most of MPL depends on OpenCV. A few modules wrap heavy / optional
 * third-party backends and are therefore guarded by opt-out macros so this
 * umbrella stays usable even when those libraries are not installed:
 *
 *   MPL_WITH_CPLEX   - include math/cplex.hpp     (needs IBM CPLEX)
 *   MPL_WITH_FITS    - include core/fits.hpp      (needs cfitsio + Astrometry)
 *   MPL_WITH_SSH     - include core/ssh.hpp       (needs libssh)
 *   MPL_WITH_CURL    - include core/curl.hpp      (needs libcurl)
 *
 * Define the corresponding macro before including this header to skip a module.
 */

#ifndef _H_MPL_H_
#define _H_MPL_H_

// core
#include <mpl/core/stdio.hpp>
#include <mpl/core/loader.hpp>
#include <mpl/core/gnuplot.hpp>
#include <mpl/core/params.hpp>
#include <mpl/core/algorithm.hpp>
#include <mpl/core/stdlib.hpp>
#include <mpl/core/debug.hpp>
#include <mpl/core/log.hpp>
#include <mpl/core/opt.hpp>
#include <mpl/core/configure.hpp>
#include <mpl/core/profile.hpp>
#include <mpl/core/bimap.hpp>
#include <mpl/core/graph.hpp>
#include <mpl/core/range.hpp>

// math
#include <mpl/math/distance.hpp>
#include <mpl/math/math.hpp>
#include <mpl/math/matrix.hpp>
#include <mpl/math/stat.hpp>
#include <mpl/math/histogram.hpp>
#include <mpl/math/kalman.hpp>
#include <mpl/math/fit.hpp>
#include <mpl/math/angles.hpp>
#include <mpl/math/rk4.hpp>
#include <mpl/math/cubicspline.hpp>
#include <mpl/math/lagrange.hpp>
#include <mpl/math/multifit.hpp>
#include <mpl/math/clustering.hpp>
#include <mpl/core/neighbors.hpp>
#include <mpl/math/geometry.hpp>
#include <mpl/math/orientation.hpp>
#include <mpl/math/quaternion.hpp>
#include <mpl/math/kabsch.hpp>
#include <mpl/math/alphaShape.hpp>
#include <mpl/math/hull.hpp>
#include <mpl/math/mvee.hpp>

// vision
#include <mpl/vision/point4d.hpp>
#include <mpl/vision/camera.hpp>
#include <mpl/vision/convert.hpp>
#include <mpl/vision/normalization.hpp>
#include <mpl/vision/undistort.hpp>
#include <mpl/vision/reprojection.hpp>
#include <mpl/vision/reconstruction.hpp>
#include <mpl/vision/utils.hpp>
#include <mpl/vision/cameraSystem.hpp>

// --- optional / third-party backends

#ifdef MPL_WITH_CPLEX
#include <mpl/math/cplex.hpp>      // IBM CPLEX
#endif

#ifdef MPL_WITH_FITS
#include <mpl/core/fits.hpp>       // cfitsio + Astrometry
#endif

#ifdef MPL_WITH_SSH
#include <mpl/core/ssh.hpp>        // libssh
#endif

#ifdef MPL_WITH_CURL
#include <mpl/core/curl.hpp>       // libcurl
#endif

#endif // _H_MPL_H_
