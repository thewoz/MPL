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

#ifndef _H_MPL_CUBE_H_
#define _H_MPL_CUBE_H_

#include <cstdio>
#include <cstdlib>

#include <glm/glm.hpp>

#include "glShader.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // Class glCube
  /*****************************************************************************/
  class glCube {
    
  private:
    
    GLuint vao;
    
    bool isInited;
    bool isInitedInGpu;

    glm::vec3 color;
    
    glm::vec3 center;
    
    glm::vec3 angle;
    
    glm::mat4 model;
    
    float scale;
    
    int style;
    
    glShader shader;
    
  public:
    
    enum CUBE_STYLE { SOLID_CUBE, WIREFRAME_CUBE };
    
    glCube() : isInited(false), isInitedInGpu(false) { }
    
    /*****************************************************************************/
    // glCube
    /*****************************************************************************/
    glCube(float _scale, int _style = WIREFRAME_CUBE, glm::vec3 _color = glm::vec3(0.0,0.0,0.0)) : isInitedInGpu(false) { init(_scale, _style, _color); }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(float _scale, int _style = WIREFRAME_CUBE, glm::vec3 _color = glm::vec3(0.0,0.0,0.0)) {

      shader.init("/usr/local/include/mpl/opengl/shader/plain.vs", "/usr/local/include/mpl/opengl/shader/plain.fs");
            
      center = glm::vec3(0.0,0.0,0.0);
      
      angle = glm::vec3(0.0,0.0,0.0);
      
      style = _style;
      
      color = _color;
      
      scale = _scale;
      
      updateModelMatrix();
      
      isInited = true;
      
    }
    
    /*****************************************************************************/
    // Position fuction
    /*****************************************************************************/
    void translate(const glm::vec3 & _center) { center = _center; updateModelMatrix(); }
    void rotate(const glm::vec3 & _angle) { angle = _angle; updateModelMatrix(); }
    void move(const glm::vec3 & _angle, const glm::vec3 & _center) { angle = _angle; center = _center; updateModelMatrix();}
    
    /*****************************************************************************/
    // setColor
    /*****************************************************************************/
    inline void setColor(const glm::vec3 & _color) { color = _color; }
    
    /*****************************************************************************/
    // setRenderStyle
    /*****************************************************************************/
    void setRenderStyle(int _style) { style = _style; }
    
    /*****************************************************************************/
    // render
    /*****************************************************************************/
    void render(const glm::mat4 & projection, const glm::mat4 & view, bool mode = WIREFRAME_CUBE, const glm::vec3 _color = glm::vec3(-1.0, -1.0, -1.0)) {
            
      if(!isInited){
        fprintf(stderr, "cube must be inited before render\n");
        abort();
      }
      
      if(!isInitedInGpu) initInGpu();
      
      //glEnableClientState(GL_VERTEX_ARRAY);
      
      //glEnable(GL_CULL_FACE);
      glDisable(GL_CULL_FACE);

      //glCullFace(GL_FRONT_AND_BACK);
      
      glEnable(GL_DEPTH_TEST);
      
      shader.use();
      
      glm::mat4 mvp = projection * view * model;
      
      shader.setUniform("mvp", mvp);
      
      if(_color.r != -1.0) shader.setUniform("color", _color);
      else                 shader.setUniform("color", color);
      
      glBindVertexArray(vao);
      
      glEnableVertexAttribArray(0);
      
      if(style == WIREFRAME_CUBE) glDrawArrays(GL_LINE_LOOP, 0, 36);
      if(style == SOLID_CUBE) glDrawArrays(GL_TRIANGLE_STRIP, 0, 36);
      
      glBindVertexArray(0);
      
      //glDisableClientState(GL_VERTEX_ARRAY);
      
      glDisable(GL_DEPTH_TEST);
      
    }
    
    /*****************************************************************************/
    // initInGpu
    /*****************************************************************************/
    void initInGpu() {
      
      if(!isInited){
        fprintf(stderr, "cube must be inited before set in GPU\n");
        abort();
      }
      
      shader.initInGpu();
      
      float vertices[] = {
        // back face
        -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
        1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
        1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right
        1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
        -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
        -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
        // front face
        -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
        1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
        1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
        1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
        -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
        -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
        // left face
        -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
        -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
        -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
        -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
        -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
        -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
        // right face
        1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
        1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
        1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right
        1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
        1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
        1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left
        // bottom face
        -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
        1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
        1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
        1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
        -1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
        -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
        // top face
        -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
        1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
        1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right
        1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
        -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
        -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f  // bottom-left
      };
      
      glGenVertexArrays(1, &vao);
      
      GLuint vbo;
      
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
      
      isInitedInGpu = true;
      
    }
    
  private:
    
    /*****************************************************************************/
    // updateModelMatrix
    /*****************************************************************************/
    void updateModelMatrix() {
      
      model = glm::mat4(1.0f);
      
      model = glm::translate(model, center); // Translate it down a bit so it's at the center of the scene
      
      model = glm::scale(model, glm::vec3(scale));
      
      model  = glm::rotate(model, angle.x, glm::vec3(1, 0, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      model  = glm::rotate(model, angle.y, glm::vec3(0, 1, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      model  = glm::rotate(model, angle.z, glm::vec3(0, 0, 1)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      
    }
    
  };
  
  
} /* namespace mpl */


#endif /* _H_MPL_CUBE_H_ */


