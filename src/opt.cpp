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

#include <mpl/opt.hpp>

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, const char * argv[]) {

  
  mpl::opt::addProgramName("Cicero");
  
  mpl::opt::addShortDescription("the program perform a 3D trajecotries recostruction from a set 2D ones");

  mpl::opt::addDescription("Lorem Ipsum is simply dummy text of the printing and typesetting industry");
  
  mpl::opt::addVersion("1.5");
  
  mpl::opt::addCredits("Leonardo Parisi - CoBBS Lab");
  
  mpl::opt::addLicense("Released under MIT License");

  mpl::opt::add("trajecotries", "trajecotries file", mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_MANDATORY);
  mpl::opt::add("p points",     "points file",       mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_MANDATORY);
  mpl::opt::add("m",            "compute median",    mpl::opt::NOT_HAVE_ARGUMENT, mpl::opt::IS_NOT_MANDATORY);
  mpl::opt::add("l",            "factor",            mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_NOT_MANDATORY, "1.5");
  mpl::opt::add("s",            "size",              mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_NOT_MANDATORY, "3");
  mpl::opt::add("k",            "kernel",            mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_NOT_MANDATORY, "5");

  mpl::opt::add("P pijk", "pijk file", "file formatting as: cam1x cam1y cam2x cam2y cam3x cam3y", mpl::opt::HAVE_ARGUMENT, mpl::opt::IS_MANDATORY);

  mpl::opt::init(argc, argv); exit(0);
  
  mpl::opt::usage();
    
  mpl::opt::getInfo("trajecotries");

  std::string trajecotries = mpl::opt::get("trajecotries");
  float factor             = mpl::opt::get<float>("l");
  bool activation          = mpl::opt::get<bool>("m");

  mpl::opt::set("trajecotries", "pippo");
  mpl::opt::set<float>("l", 1.5);
  mpl::opt::set<bool>("m", "ON");

  return 0;
  
}

void mpl::opt::finalize() {
  
    
}
