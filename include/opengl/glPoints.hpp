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
    GLuint vbo[2];
    
    std::vector<cv::Point3f> points;
    std::vector<glm::vec4> colors;

  public:
    
    /*****************************************************************************/
    // glPoints
    /*****************************************************************************/
    glPoints(const std::string & _name = "") : glObject(_name) { }
    glPoints(const std::vector<cv::Point3f> & _points, const glm::vec4 & color = glm::vec4(0.0), const std::string & _name = "") : glObject(_name) { init(_points, color); }
    
    /*****************************************************************************/
    // ~glPoints
    /*****************************************************************************/
    ~glPoints() {
      
      if(isInitedInGpu) {
        glDeleteBuffers(2, vbo);
        glDeleteVertexArrays(1, &vao);
      }
      
    }
 
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(const std::vector<cv::Point3f> & _points, const glm::vec4 & color = glm::vec4(0.0)) {
      
      DEBUG_LOG("glPoints::init(" + name + ")");
            
      shader.init("/usr/local/include/mpl/opengl/shader/sphere.vs", "/usr/local/include/mpl/opengl/shader/sphere.fs");
      
      points = _points;

      colors.resize(points.size(), color);
              
      isInited = true;

    }
   
    /*****************************************************************************/
    // render
    /*****************************************************************************/
    void render(const glm::mat4 & projection, const glm::mat4 & view) {
      
      DEBUG_LOG("glPoints::render(" + name + ")");

      glPoints::renderBegin(projection, view);
      
      glEnable(GL_DEPTH_TEST);

      glEnable(GL_PROGRAM_POINT_SIZE);

      glEnable(GL_BLEND);
      
      glEnableClientState(GL_VERTEX_ARRAY);
      glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
      glVertexPointer(3, GL_FLOAT, 0, 0);
      
      glEnableClientState(GL_COLOR_ARRAY);
      glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
      glColorPointer(4, GL_FLOAT, 0, 0);

      glDrawArrays(GL_POINTS, 0, (int)points.size());
      
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      
      glDisableClientState(GL_COLOR_ARRAY);
      glDisableClientState(GL_VERTEX_ARRAY);

      glDisable(GL_BLEND);

      glDisable(GL_PROGRAM_POINT_SIZE);

      glDisable(GL_DEPTH_TEST);

      glObject::renderEnd();
      
    }
    
  private:
    
    /*****************************************************************************/
    // setInGpu
    /*****************************************************************************/
    void setInGpu() {
      
      DEBUG_LOG("glPoints::setInGpu(" + name + ")");

      if(!isInitedInGpu) {
        
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        
        glGenBuffers(2, vbo);
        
        glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(cv::Point3f), points.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
        
        glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
        glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(glm::vec4), colors.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
      
      }
    
      isInitedInGpu = true;
          
    }
    
    /* ****************************************************************************/
    // renderBegin() -
    /* ****************************************************************************/
    void renderBegin(const glm::mat4 & projection, const glm::mat4 & view) {
      
      DEBUG_LOG("glPoints::renderBegin(" + name + ")");

      if(!isInited){
        fprintf(stderr, "line must be inited before render\n");
        abort();
      }
      
      if(isToInitInGpu()) initInGpu();
      
      shader.setName(name);

      shader.use();
      
      shader.setUniform("projection", projection);
      shader.setUniform("view", view);
      shader.setUniform("model", model);
            
      glEnable(GL_DEPTH_TEST);
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_OPENGL_POINTS_H_ */
