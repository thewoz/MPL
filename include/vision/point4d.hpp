/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017
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

#ifndef _H_MPL_POINT4D_H_
#define _H_MPL_POINT4D_H_

#include <cstdlib>
#include <cstdio>

//*****************************************************************************/
// namespace vision
//*****************************************************************************/
namespace mpl {

  struct point4d_t {
    
    double x;
    double y;
    double z;
    double w;

  };

} /* namespace mpl */

#endif /* _H_MPL_POINT4D_H_ */
