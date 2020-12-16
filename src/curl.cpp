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
#include <mpl/web/curl.hpp>

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char* const argv []) {
  
  mpl::web::curl::get("https://www.space-track.org/ajaxauth/login", "identity=leonardo.parisi@gmail.com&password=XXXXXXXXXX&query=https://www.space-track.org/basicspacedata/query/class/tle_latest/ORDINAL/1/EPOCH/%3Enow-30/orderby/NORAD_CAT_ID/format/tle", "downloaded.dat");

  
 // mpl::web::curl::get("www.apple.com", "downloaded.dat");
  
  return 0;
  
}


