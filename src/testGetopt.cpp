/*
 * MIT License
 *
 * Copyright © 2017 COBBS
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


#include <cstdlib>
#include <cstdio>

#include "getopt.hpp"



//-g jjj - j leo -l pippo --casa mia 
int main(int argc, const char * argv[]) {

  getopt::program("test");
  
  getopt::add("trajecotries", "trajecotries file", getopt::TYPE::FILE, getopt::HAVE_ARGUMENT,     getopt::IS_MANDATORY);
  getopt::add("p points",     "points file",       getopt::TYPE::INT,  getopt::HAVE_ARGUMENT,     getopt::IS_MANDATORY);
  getopt::add("m",            "first frame",       getopt::TYPE::REAL, getopt::NOT_HAVE_ARGUMENT, getopt::IS_NOT_MANDATORY, "star");
  getopt::add("l",            "factor",            getopt::TYPE::STR,  getopt::HAVE_ARGUMENT,     getopt::IS_NOT_MANDATORY, "1.5");
  getopt::add("s",            "factor",            getopt::TYPE::CHAR, getopt::HAVE_ARGUMENT,     getopt::IS_NOT_MANDATORY, "1.5");
  getopt::add("k",            "factor",            getopt::TYPE::BOOL, getopt::HAVE_ARGUMENT,     getopt::IS_NOT_MANDATORY, "1.5");

  
  getopt::add("P pijk", "pijk file", "file formatting as: cam1x cam1y cam2x cam2y cam3x cam3y", getopt::HAVE_ARGUMENT, getopt::IS_MANDATORY);

  
  getopt::init(argc, argv);
  
  //getopt::info("t");

  //getopt::get("t");
  
  return 0;
  
}
