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

#if(0)

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
    GLuint vbo[4];

    GLuint stacks;
    GLuint slices;
    
    GLfloat a;
    GLfloat b;
    GLfloat c;
    
  public:
        
    /*****************************************************************************/
    // glEllipse
    /*****************************************************************************/
    glEllipse(const std::string & _name = "") : glObject(_name) { }

    /*****************************************************************************/
    // glEllipse
    /*****************************************************************************/
    glEllipse(GLfloat _a, GLfloat _b, GLfloat _c, GLuint _uiStacks, GLuint _uiSlices, int _style = glObject::STYLE::WIREFRAME, const glm::vec3 & _color = glm::vec3(0.0), const std::string & _name = "") : glObject(_name) { init(_a, _b, _c, _uiStacks, _uiSlices, _style, _color); }
    
    /*****************************************************************************/
    // ~glEllipse
    /*****************************************************************************/
    ~glEllipse() {
      
      if(isInitedInGpu) {

        glDeleteBuffers(4, vbo);
        glDeleteVertexArrays(1, &vao);
        
      }
      
    }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(GLfloat _a, GLfloat _b, GLfloat _c, GLuint _uiStacks, GLuint _uiSlices, int _style = glObject::STYLE::WIREFRAME, const glm::vec3 & _color = glm::vec3(0.0)) {
      
      DEBUG_LOG("glEllipse::init(" + name + ")");
      
      glObject::initPlain();

      stacks = _uiStacks;
      slices = _uiSlices;
      
      a = _a;
      b = _b;
      c = _c;

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
      
      glDisable(GL_CULL_FACE);

      glBindVertexArray(vao);
      
      glEnableVertexAttribArray(0);
      
      if(style == glObject::STYLE::WIREFRAME) glDrawElements(GL_LINES,     (slices * stacks + slices) * 6, GL_UNSIGNED_INT, nullptr);
      if(style == glObject::STYLE::SOLID)     glDrawElements(GL_TRIANGLES, (slices * stacks + slices) * 6, GL_UNSIGNED_INT, nullptr);
      
      //if(style == glObject::STYLE::WIREFRAME) glDrawArrays(GL_LINE_LOOP, 0, 36);
      //if(style == glObject::STYLE::SOLID)     glDrawArrays(GL_TRIANGLE_STRIP, 0, 36);
      
      glBindVertexArray(0);
      
      glObject::renderEnd();

    }
    
    private:

    /*****************************************************************************/
    // setInGpu
    /*****************************************************************************/
    void setInGpu() {
      
      DEBUG_LOG("glEllipse::setInGpu(" + name + ")");
      
      const float _2pi = 2.0f * M_PI;
      
      std::vector<glm::vec3> positions;
      std::vector<glm::vec3> normals;
      std::vector<glm::vec2> textureCoords;
      
      for(int i = 0; i <= stacks; ++i) {
        
        // V texture coordinate
        float V = i / (float)stacks;
        float phi = V * M_PI;
        
        for( int j = 0; j <= slices; ++j) {
          
          // U texture coordinate
          float U = j / (float)slices;
          float theta = U * _2pi;
          
          float X = a * cos(theta) * cos(phi);
          float Y = b * cos(theta) * sin(phi);
          float Z = c * sin(theta);
          
          positions.push_back( glm::vec3( X, Y, Z) );
          normals.push_back( glm::vec3(X, Y, Z) );
          textureCoords.push_back( glm::vec2(U, V) );
          
        }
        
      }
      
      // Now generate the index buffer
      std::vector<GLuint> indicies;
      
      for(int i=0; i <slices*stacks+slices; ++i) {
        
        indicies.push_back(i);
        indicies.push_back(i + slices + 1);
        indicies.push_back(i + slices);
        
        indicies.push_back(i + slices + 1);
        indicies.push_back(i);
        indicies.push_back(i + 1);
        
      }
      
      glGenVertexArrays(1, &vao);
      glBindVertexArray(vao);
      
      glGenBuffers(4, vbo);
      
      glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
      glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec3), positions.data(), GL_STATIC_DRAW);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
      glEnableVertexAttribArray(0);
      
      glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
      glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW);
      glVertexAttribPointer(2, 3, GL_FLOAT, GL_TRUE, 0, nullptr);
      glEnableVertexAttribArray(2);
      
      glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
      glBufferData(GL_ARRAY_BUFFER, textureCoords.size() * sizeof(glm::vec2), textureCoords.data(), GL_STATIC_DRAW);
      glVertexAttribPointer(8, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
      glEnableVertexAttribArray(8);
      
      glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, vbo[3]);
      glBufferData( GL_ELEMENT_ARRAY_BUFFER, indicies.size() * sizeof(GLuint), indicies.data(), GL_STATIC_DRAW);
      
      glBindVertexArray(0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
      
      
#if(0)
      
      DEBUG_LOG("glEllipse::setInGpu(" + name + ")");

      GLfloat tStep = (M_PI) / (GLfloat)uiSlices;
      GLfloat sStep = (M_PI) / (GLfloat)uiStacks;

      std::vector<glm::vec3> vertices;

      for(GLfloat t = -M_PI/2; t <= (M_PI/2)+.0001; t += tStep) {
        
        for(GLfloat s = -M_PI; s <= M_PI+.0001; s += sStep) {
          vertices.push_back(glm::vec3(a * cos(t)       * cos(s), b * cos(t)       * sin(s), c * sin(t)));
          vertices.push_back(glm::vec3(a * cos(t+tStep) * cos(s), b * cos(t+tStep) * sin(s), c * sin(t+tStep)));
        }
        
      }
      
      glGenVertexArrays(1, &vao);
            
      glGenBuffers(1, &vbo);
      
      // fill buffer
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);
      
      // link vertex attributes
      glBindVertexArray(vao);
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (void*)0);
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));
      glEnableVertexAttribArray(2);
      glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (void*)(6 * sizeof(GLfloat)));
      
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      glBindVertexArray(0);
      
#endif
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_OPENGL_ELLIPSE_H_ */


