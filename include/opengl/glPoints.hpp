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

#ifndef _H_MPL_OPENGL_POINTS_H_
#define _H_MPL_OPENGL_POINTS_H_

#include <cstdio>
#include <cstdlib>

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "glObject.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

  /*****************************************************************************/
  // Class glPoints
  /*****************************************************************************/
  class glPoints : public glObject {
    
  private:
        
    GLuint vao = -1;
    GLuint vbo = -1;
    GLuint ibo = -1;
    
    std::vector<cv::Point3f> points;
    
  public:
    
    /*****************************************************************************/
    // glPoints
    /*****************************************************************************/
    glPoints(const std::string & _name = "") : glObject(_name) { }
    glPoints(const std::vector<cv::Point3f> & _points, const glm::vec3 & _color = glm::vec3(0.0), const std::string & _name = "") : glObject(_name) { init(_points, _color); }
    
    /*****************************************************************************/
    // ~glPoints
    /*****************************************************************************/
    ~glPoints() {
      
      if(isInitedInGpu) {

        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ibo);
        glDeleteVertexArrays(1, &vao);
        
      }
      
    }
 
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(const std::vector<cv::Point3f> & _points, const glm::vec3 & _color = glm::vec3(0.0)) {
      
      DEBUG_LOG("glPoints::init(" + name + ")");

      glObject::initPlain();
    
      points = _points;

      color = _color;
              
      isInited = true;
      
    }
   
    /*****************************************************************************/
    // render
    /*****************************************************************************/
    void render(const glm::mat4 & projection, const glm::mat4 & view) {
      
      DEBUG_LOG("glPoints::render(" + name + ")");

//      glObject::renderBegin(projection, view);
//
//      glBindVertexArray(vao);
//
//      glDrawElements(GL_LINES, lenght, GL_UNSIGNED_INT, nullptr);
//
//      glBindVertexArray(0);
//
//      glObject::renderEnd();
      
      
      
      
      glEnable(GL_DEPTH_TEST);
      
      glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
      
      glEnable(GL_POINT_SPRITE);
      
      bindCoordBuffers();
      
      bindColorBuffers();
      
      glTexEnvi(GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE);
      
      glUseProgram(program);
      
      glUniform1f(glGetUniformLocation(program, "pointSize"),   pointSize);
      glUniform1f(glGetUniformLocation(program, "pixelSize"),   pixelSize);
      glUniform1f(glGetUniformLocation(program, "focalLenght"), focalLenght);
      
      glDrawArrays(GL_POINTS, frame * Trajectories_t::trajectoriesSize, Trajectories_t::trajectoriesSize);
      
      glUseProgram(0);
      
      glDisableClientState(GL_VERTEX_ARRAY);
      
      if(colorVboId != -1) glDisableClientState(GL_COLOR_ARRAY);
      
      glDisable(GL_POINT_SPRITE);
      
      glDisable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
      
      glDisable(GL_DEPTH_TEST);

    }
    
  private:
    
    /*****************************************************************************/
    // setInGpu
    /*****************************************************************************/
    void setInGpu() {
      
      DEBUG_LOG("glPoints::setInGpu(" + name + ")")

      glGenBuffers(1, &vbo[0]);
      glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
      glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), glm::value_ptr(vertices[0]), GL_STATIC_DRAW);
      
      glGenBuffers(1, &vbo[1]);
      glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
      glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), glm::value_ptr(vertices[0]), GL_STATIC_DRAW);
      
      
      
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
         
      glGenBuffers(1, &ibo);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(glm::uvec4), glm::value_ptr(indices[0]), GL_STATIC_DRAW);

      glBindVertexArray(0);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
          
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_OPENGL_POINTS_H_ */
