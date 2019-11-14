/*
 * MIT License
 *
 * Copyright (c) 2017 Leonardo Parisi
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

#ifndef _H_MPL_OPENGL_PRINT_H_
#define _H_MPL_OPENGL_PRINT_H_

#include <cstdlib>
#include <cstdio>

#include <string>

#include <glm/glm.hpp>

#include "freeglut/fg_font_data.hpp"


/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

  //****************************************************************************//
  // string 3D
  //****************************************************************************//
  inline void glPrint(const std::string & string, GLfloat x, GLfloat y, GLfloat z, int font = BITMAP_HELVETICA_18, const glm::vec3 & color = glm::vec3(1.0f)){
    
    glColor3fv(glm::value_ptr(color));
    
    glRasterPos3f(x, y, z);
    
    glutBitmapString(font, string.c_str());
    
  }

  //****************************************************************************//
  // interger 3D
  //****************************************************************************//
  inline void glPrint(int value, GLfloat x, GLfloat y, GLfloat z, int font = BITMAP_HELVETICA_18, const glm::vec3 & color = glm::vec3(1.0f)){
    
    char string[PATH_MAX]; sprintf(string, "%d", value);
    
    glColor3fv(glm::value_ptr(color));

    glRasterPos3f(x, y, z);

    glutBitmapString(font, string);

  }

  //****************************************************************************//
  // float 3D
  //****************************************************************************//
  inline void glPrint(double value, GLfloat x, GLfloat y, GLfloat z, int font = BITMAP_HELVETICA_18, const glm::vec3 & color = glm::vec3(1.0f)){
    
    char string[PATH_MAX]; sprintf(string, "%f", value);
    
    glColor3fv(glm::value_ptr(color));

    glRasterPos3f(x, y, z);
    
    glutBitmapString(font, string);
    
  }

  //****************************************************************************//
  // string 2D
  //****************************************************************************//
  inline void glPrint(const std::string & string, GLfloat x, GLfloat y, int font = BITMAP_HELVETICA_18, const glm::vec3 & color = glm::vec3(1.0f)){
    
    glColor3fv(glm::value_ptr(color));

    glRasterPos2f(x, y);
    
    glutBitmapString(font, string.c_str());
    
  }

  //****************************************************************************//
  // interger 2D
  //****************************************************************************//
  inline void glPrint(int value, GLfloat x, GLfloat y, int font = BITMAP_HELVETICA_18, const glm::vec3 & color = glm::vec3(1.0f)){
    
    char string[PATH_MAX]; sprintf(string, "%d", value);
    
    glColor3fv(glm::value_ptr(color));

    glRasterPos2f(x, y);
    
    glutBitmapString(font, string);
    
  }

  //****************************************************************************//
  // float 2D
  //****************************************************************************//
  inline void glPrint(double value, double x, double y, int font = BITMAP_HELVETICA_18, const glm::vec3 & color = glm::vec3(1.0f)){
    
    char string[PATH_MAX]; sprintf(string, "%f", value);
    
    glColor3fv(glm::value_ptr(color));

    glRasterPos2f(x, y);
    
    glutBitmapString(font, string);
    
  }


} /* namespace mpl */

#endif /* _H_MPL_OPENGL_PRINT_H_ */
