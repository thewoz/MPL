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

#ifndef _H_MPL_OPENGL_ELLIPSE_H_
#define _H_MPL_OPENGL_ELLIPSE_H_

#include <cstdio>
#include <cstdlib>

#include "glObject.hpp"


 void glDraw(unsigned int uiStacks = 5, unsigned int uiSlices = 5) const {
    
    
    glMatrixMode(GL_MODELVIEW);
    
    glPushMatrix();
    
    double matrix[16] = {1,0,0,0,0,1,0,0,0,0,1,0,1,0,0,1};
    
    //matrix[0] = rotation.at<double>(0,0); matrix[4] = rotation.at<double>(0,1); matrix[8]  = rotation.at<double>(0,2); matrix[12] = center.x;
    //matrix[1] = rotation.at<double>(1,0); matrix[5] = rotation.at<double>(1,1); matrix[9]  = rotation.at<double>(1,2); matrix[13] = center.y;
    //matrix[2] = rotation.at<double>(2,0); matrix[6] = rotation.at<double>(2,1); matrix[10] = rotation.at<double>(2,2); matrix[14] = center.z;
    
    matrix[0] = rotation.at<double>(0,0); matrix[4] = rotation.at<double>(1,0); matrix[8]  = rotation.at<double>(2,0); matrix[12] = center.x;
    matrix[1] = rotation.at<double>(0,1); matrix[5] = rotation.at<double>(1,1); matrix[9]  = rotation.at<double>(2,1); matrix[13] = center.y;
    matrix[2] = rotation.at<double>(0,2); matrix[6] = rotation.at<double>(1,2); matrix[10] = rotation.at<double>(2,2); matrix[14] = center.z;
    
    glMultMatrixd(matrix);
    
    glColor3f(0, 0, 0);
    
    glLineWidth(1.0);
    
    glEnable(GL_LINE_SMOOTH);
    
    float tStep = (M_PI) / (float)uiSlices;
    float sStep = (M_PI) / (float)uiStacks;
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for(float t = -M_PI/2; t <= (M_PI/2)+.0001; t += tStep)
    {
      glBegin(GL_TRIANGLE_STRIP);
      for(float s = -M_PI; s <= M_PI+.0001; s += sStep)
      {
        glVertex3f(a * cos(t) * cos(s), b * cos(t) * sin(s), c * sin(t));
        glVertex3f(a * cos(t+tStep) * cos(s), b * cos(t+tStep) * sin(s), c * sin(t+tStep));
      }
      glEnd();
    }
    
    glDisable(GL_LINE_SMOOTH);
    
    glPopMatrix();
    
  }
  
#endif



/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // Class glEllipse
  /*****************************************************************************/
  class glEllipse : public glObject {
    
  private:
    
    GLuint vao = -1;
    GLuint vbo = -1;
    
    
    GuLint uiStacks;
    GLuint uiSlices;
    
  public:
        
    /*****************************************************************************/
    // glEllipse
    /*****************************************************************************/
    glEllipse(const std::string & _name = "") : glObject(_name) { }

    /*****************************************************************************/
    // glEllipse
    /*****************************************************************************/
    glEllipse(GuLint _uiStacks, GLuint _uiSlices, int _style = glObject::STYLE::WIREFRAME, const glm::vec3 & _color = glm::vec3(0.0), const std::string & _name = "") : glObject(_name) { init(_uiStacks, _uiSlices, _style, _color); }
    
    /*****************************************************************************/
    // ~glEllipse
    /*****************************************************************************/
    ~glEllipse() {
      
      if(isInitedInGpu) {

        glDeleteBuffers(1, &vbo);
        glDeleteVertexArrays(1, &vao);
        
      }
      
    }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(GuLint _uiStacks, GLuint _uiSlices, int _style = glObject::STYLE::WIREFRAME, const glm::vec3 & _color = glm::vec3(0.0)) {

      DEBUG_LOG("glEllipse::init(" + name + ")");

      style = _style;
      
      color = _color;
      
      isInited = true;
      
    }
    
    /*****************************************************************************/
    // render
    /*****************************************************************************/
    void render(const glm::mat4 & projection, const glm::mat4 & view) {
            
      DEBUG_LOG("glEllipse::render(" + name + ")");

      glObject::renderBegin(projection, view);
      
      glBindVertexArray(vao);
      
      glEnableVertexAttribArray(0);
      
      if(style == glObject::STYLE::WIREFRAME) glDrawArrays(GL_LINE_LOOP, 0, 36);
      if(style == glObject::STYLE::SOLID)     glDrawArrays(GL_TRIANGLE_STRIP, 0, 36);
      
      glBindVertexArray(0);
      
      glObject::renderEnd();

    }
    
    private:

    /*****************************************************************************/
    // setInGpu
    /*****************************************************************************/
    void setInGpu() {
      
      DEBUG_LOG("glEllipse::setInGpu(" + name + ")");

      XXX
      
      glGenVertexArrays(1, &vao);
            
      glGenBuffers(1, &vbo);
      
      // fill buffer
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
      
      // link vertex attributes
      glBindVertexArray(vao);
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
      glEnableVertexAttribArray(2);
      glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
      
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glBindVertexArray(0);
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_OPENGL_ELLIPSE_H_ */

