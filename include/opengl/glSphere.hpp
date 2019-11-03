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

#ifndef _H_MPL_OPENGL_SPHERE_H_
#define _H_MPL_OPENGL_SPHERE_H_

#include <cstdio>
#include <cstdlib>

#include "glObject.hpp"


/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

  /*****************************************************************************/
  // Class glSphere
  /*****************************************************************************/
  class glSphere : public glObject {
    
    private:
      
      GLuint vao;
      GLuint vbo[4];
      
      GLfloat radius;
      GLint   slices;
      GLint   stacks;
      
    public:
      
      /*****************************************************************************/
      // glSphere
      /*****************************************************************************/
      glSphere(const std::string & _name = "") : glObject(_name) { }
      
      /*****************************************************************************/
      // glSphere
      /*****************************************************************************/
      glSphere(GLfloat _radius, GLint _slices, GLint _stacks, int _style = glObject::STYLE::WIREFRAME, const glm::vec3 & _color = glm::vec3(1.0), const std::string & _name = "") : glObject(_name) {
        init(_radius, _slices, _stacks, _style, _color);
      }
      
      /*****************************************************************************/
      // ~glSphere
      /*****************************************************************************/
      ~glSphere() {
        
        if(isInitedInGpu) {
          glDeleteBuffers(4, vbo);
          glDeleteVertexArrays(1, &vao);
        }
        
      }
      
      /*****************************************************************************/
      // init
      /*****************************************************************************/
      void init(GLfloat _radius, GLint _slices, GLint _stacks, int _style = glObject::STYLE::WIREFRAME, const glm::vec3 & _color = glm::vec3(1.0)) {
        
        DEBUG_LOG("glSphere::init(" + name + ")");

        glObject::initPlain();
        
        radius = _radius;
        
        slices = _slices;
        
        stacks = _stacks;
        
        style = _style;
        
        color = _color;
        
        isInited = true;
        
      }
      
      /*****************************************************************************/
      // render
      /*****************************************************************************/
      void render(const glm::mat4 & projection, const glm::mat4 & view) {
        
        DEBUG_LOG("gSphere::setInGpu(" + name + ")");

        glObject::renderBegin(projection, view);
        
        glDisable(GL_CULL_FACE);
        
        glBindVertexArray(vao);
        
        glEnableVertexAttribArray(0);
        
        if(style == glObject::STYLE::WIREFRAME) glDrawElements(GL_LINES,     (slices * stacks + slices) * 6, GL_UNSIGNED_INT, nullptr);
        if(style == glObject::STYLE::SOLID)     glDrawElements(GL_TRIANGLES, (slices * stacks + slices) * 6, GL_UNSIGNED_INT, nullptr);
        
        glBindVertexArray(0);
        
        glObject::renderEnd();
        
      }
      
    private:
      
      /*****************************************************************************/
      // setInGpu
      /*****************************************************************************/
      void setInGpu() {
        
        DEBUG_LOG("glSphere::setInGpu(" + name + ")");
        
        const float _2pi = 2.0f * M_PI;
        
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> normals;
        std::vector<glm::vec2> textureCoords;
        
        for(int i = 0; i <= stacks; ++i) {
          
          // V texture coordinate.
          float V = i / (float)stacks;
          float phi = V * M_PI;
          
          for( int j = 0; j <= slices; ++j) {
            
            // U texture coordinate.
            float U = j / (float)slices;
            float theta = U * _2pi;
            
            float X = cos(theta) * sin(phi);
            float Y = cos(phi);
            float Z = sin(theta) * sin(phi);
            
            positions.push_back( glm::vec3( X, Y, Z) * radius );
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
        
      }
    
  };

} /* namespace mpl */

#endif /* _H_MPL_OPENGL_SPHERE_H_ */
