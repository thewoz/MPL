/*
 * MIT License
 *
 * Copyright © 2017
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "opt.hpp"

/*****************************************************************************/
// main //-g jjj - j leo -l pippo --casa mia
/*****************************************************************************/
int main(int argc, char * const argv []) {

  
  mpl::opt::program("Cicero", "3D trajecotries recostrucition from 2D ones" , "Lorem Ipsum is simply dummy text of the printing and typesetting industry. Lorem Ipsum has been the industry's standard dummy text ever since the 1500s, when an unknown printer took a galley of type and scrambled it to make a type specimen book. It has survived not only five centuries, but also the leap into electronic typesetting, remaining essentially unchanged. It was popularised in the 1960s with the release of Letraset sheets containing Lorem Ipsum passages, and more recently with desktop publishing software like Aldus PageMaker including versions of Lorem Ipsum.");
  
//  mpl::opt::add("trajecotries", "trajecotries file", mpl::opt::TYPE::FILE, mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_MANDATORY);
//  mpl::opt::add("p points",     "points file",       mpl::opt::TYPE::INT,  mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_MANDATORY);
//  mpl::opt::add("m",            "first frame",       mpl::opt::TYPE::REAL, mpl::opt::NOT_HAVE_ARGUMENT, mpl::opt::IS_NOT_MANDATORY, "star");
//  mpl::opt::add("l",            "factor",            mpl::opt::TYPE::STR,  mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_NOT_MANDATORY, "1.5");
//  mpl::opt::add("s",            "factor",            mpl::opt::TYPE::CHAR, mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_NOT_MANDATORY, "1.5");
//  mpl::opt::add("k",            "factor",            mpl::opt::TYPE::BOOL, mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_NOT_MANDATORY, "1.5");

  mpl::opt::add("trajecotries", "trajecotries file", mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_MANDATORY);
  mpl::opt::add("p points",     "points file",       mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_MANDATORY);
  mpl::opt::add("m",            "first frame",       mpl::opt::NOT_HAVE_ARGUMENT, mpl::opt::IS_NOT_MANDATORY, "star");
  mpl::opt::add("l",            "factor",            mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_NOT_MANDATORY, "1.5");
  mpl::opt::add("s",            "factor",            mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_NOT_MANDATORY, "1.5");
  mpl::opt::add("k",            "factor",            mpl::opt::HAVE_ARGUMENT,     mpl::opt::IS_NOT_MANDATORY, "1.5");

  
  mpl::opt::add("P pijk", "pijk file", "file formatting as: cam1x cam1y cam2x cam2y cam3x cam3y", mpl::opt::HAVE_ARGUMENT, mpl::opt::IS_MANDATORY);
  mpl::opt::add("t", "trajectories file", "file formatting as: cam1x cam1y cam2x cam2y cam3x cam3y", mpl::opt::HAVE_ARGUMENT, mpl::opt::IS_MANDATORY);

  
  mpl::opt::usage();
  
  //mpl::opt::init(argc, argv);
  
  //mpl::opt::info("t");

  //mpl::opt::get("t");
  
  
   
  return 0;
  
}
