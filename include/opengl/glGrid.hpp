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

#ifndef _H_MPL_OPENGL_GRID_H_
#define _H_MPL_OPENGL_GRID_H_

#include <cstdio>
#include <cstdlib>

#include "glObject.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

  /*****************************************************************************/
  // Class glGrid
  /*****************************************************************************/
  class glGrid : public glObject {
    
  private:
        
    GLuint vao = -1;
    GLuint vbo = -1;
    GLuint ibo = -1;
    
    GLuint slices;
    GLuint lenght;
    
  public:
    
    /*****************************************************************************/
    // glGrid
    /*****************************************************************************/
    glGrid(const std::string & _name = "") : glObject(_name) { }
    glGrid(GLuint _slices, const glm::vec3 & _color = glm::vec3(0.0), const std::string & _name = "") : glObject(_name) { init(_slices, _color); }
    
    /*****************************************************************************/
    // ~glGrid
    /*****************************************************************************/
    ~glGrid() {
      
      if(isInitedInGpu) {

        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ibo);
        glDeleteVertexArrays(1, &vao);
        
      }
      
    }
 
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(GLuint _slices, const glm::vec3 & _color = glm::vec3(0.0)) {
      
      DEBUG_LOG("glGrid::init(" + name + ")");

      glObject::initPlain();
    
      slices = _slices;

      color = _color;
              
      isInited = true;
      
    }
   
    /*****************************************************************************/
    // render
    /*****************************************************************************/
    void render(const glm::mat4 & projection, const glm::mat4 & view) {
      
      DEBUG_LOG("glGrid::setInGpu(" + name + ")");

      glObject::renderBegin(projection, view);
          
      glBindVertexArray(vao);
          
      glDrawElements(GL_LINES, lenght, GL_UNSIGNED_INT, nullptr);

      glBindVertexArray(0);
            
      glObject::renderEnd();

    }
    
  private:
    
    /*****************************************************************************/
    // setInGpu
    /*****************************************************************************/
    void setInGpu() {
      
      DEBUG_LOG("glGrid::setInGpu(" + name + ")");

      std::vector<glm::vec3> vertices;
      std::vector<glm::uvec4> indices;
         
      for(int j=0; j<=slices; ++j) {
        for(int i=0; i<=slices; ++i) {
          float x = (float)i/(float)slices;
          float y = 0;
          float z = (float)j/(float)slices;
          vertices.push_back(glm::vec3(x, y, z));
        }
      }
           
      for(int j=0; j<slices; ++j) {
        for(int i=0; i<slices; ++i) {
          
          int row1 =  j    * (slices+1);
          int row2 = (j+1) * (slices+1);
          
          indices.push_back(glm::uvec4(row1+i, row1+i+1, row1+i+1, row2+i+1));
          indices.push_back(glm::uvec4(row2+i+1, row2+i, row2+i, row1+i));

        }
      }

      glGenVertexArrays(1, &vao);
      glBindVertexArray(vao);

      glGenBuffers(1, &vbo);
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), glm::value_ptr(vertices[0]), GL_STATIC_DRAW);
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
         
      glGenBuffers(1, &ibo);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(glm::uvec4), glm::value_ptr(indices[0]), GL_STATIC_DRAW);

      glBindVertexArray(0);
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
      glBindBuffer(GL_ARRAY_BUFFER, 0);
      
      lenght = (GLuint)indices.size()*4;
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_OPENGL_GRID_H_ */
