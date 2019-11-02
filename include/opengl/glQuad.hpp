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

#ifndef _H_MPL_GLQUAD_H_
#define _H_MPL_GLQUAD_H_

#include <cstdlib>
#include <cstdio>

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // glQuad
  /*****************************************************************************/
  class glQuad {
    
  private:
    
    bool isInited;

    GLuint vao = -1;
    GLuint vbo = -1;

    float vertices[20] = {
      -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
      -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
       1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
       1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
    };
    
  public:
    
    /*****************************************************************************/
    // glQuad - Constructor
    /*****************************************************************************/
    glQuad() : isInited(false) {  }
    
    ~glQuad() {
        
        if(vbo != -1) glDeleteBuffers(1, &vbo);
        if(vao != -1) glDeleteVertexArrays(1, &vao);
        
      }
      
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init() {
      
      glGenVertexArrays(1, &vao);
      
      
      glGenBuffers(1, &vbo);
      
      glBindVertexArray(vao);
      
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices, GL_STATIC_DRAW);
      
      glEnableVertexAttribArray(0);
      
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
      
      glEnableVertexAttribArray(1);
      
      glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
      
      isInited = true;
      
    }
    
    /*****************************************************************************/
    // render
    /*****************************************************************************/
    void render() const {
      
      if(!isInited){
        fprintf(stderr, "quad shader not inited\n");
        abort();
      }
      
      glBindVertexArray(vao);
      
      glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
      
      glBindVertexArray(0);
      
    }
    
  };
  
} /* namspace mpl */

#endif /* _H_MPL_GLQUAD_H_ */

