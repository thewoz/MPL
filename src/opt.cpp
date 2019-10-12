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

#include <mpl/opt.hpp>

/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char * const argv []) {

  
  mpl::opt::addProgramName("Cicero");
  
  mpl::opt::addShortDescription("the program perform a 3D trajecotries recostruction from a set 2D ones");

  mpl::opt::addDescription("Lorem Ipsum is simply dummy text of the printing and typesetting industry. Lorem Ipsum has been the industry's standard dummy text ever since the 1500s, when an unknown printer took a galley of type and scrambled it to make a type specimen book. It has survived not only five centuries, but also the leap into electronic typesetting, remaining essentially unchanged. It was popularised in the 1960s with the release of Letraset sheets containing Lorem Ipsum passages, and more recently with desktop publishing software like Aldus PageMaker including versions of Lorem Ipsum.");
  
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

  mpl::opt::usage();
  
  //mpl::opt::init(argc, argv);
  
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
