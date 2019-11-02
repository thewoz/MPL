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

#ifndef _H_MPL_LINE_H_
#define _H_MPL_LINE_H_

#include <cstdlib>
#include <cstdio>

#include <glm/glm.hpp>

#include <mpl/debug.hpp>

#include "glShader.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // Class glLine
  /*****************************************************************************/
  class glLine {
    
  private:
    
    GLFWwindow * contex;

    GLuint vao = -1;
    GLuint vbo = -1;
    
    bool isInited;
    bool isInitedInGpu;
    bool isToUpdateInGpu;
    
    glm::vec3 color;
    
    glm::vec3 center;
    glm::vec3 angle;
    glm::vec3 size;
    
    glm::mat4 model;
    
    //GLuint lenght;
    
    int style;
    
    glShader shader;
    
    std::vector<glm::vec3> vertices;
    
  public:
    
    /*****************************************************************************/
    // glLine
    /*****************************************************************************/
    glLine() : isInited(false), isInitedInGpu(false), isToUpdateInGpu(false) { }
    glLine(const std::vector<glm::vec3> & _vertices, const glm::vec3 & _color) : isInitedInGpu(false), isToUpdateInGpu(false) { init(_vertices, _color); }
    
    ~glLine() {
      
      if(vbo != -1) glDeleteBuffers(1, &vbo);
      if(vao != -1) glDeleteVertexArrays(1, &vao);
      
    }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
//    void init(glm::vec3 _color) {
//      std::vector<glm::vec3> _vertices;
//      init(_vertices, _color);
//    }
        
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(const glm::vec3 & _color) {
      
      DEBUG_LOG("glLine::init()");
      
      shader.init("/usr/local/include/mpl/opengl/shader/plain.vs", "/usr/local/include/mpl/opengl/shader/plain.fs");
                    
      center = glm::vec3(0.0,0.0,0.0);
       
      angle = glm::vec3(0.0,0.0,0.0);
       
      size = glm::vec3(1.0,1.0,1.0);
      
      color = _color;
      
      updateModelMatrix();
       
      isInited = true;
      
    }
         
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(const std::vector<glm::vec3> & _vertices, const glm::vec3 & _color) {
 
      init(_color);
      
      vertices = _vertices;
      
    }
    
    /*****************************************************************************/
    // Position fuction
    /*****************************************************************************/
    void translate(const glm::vec3 & _center) { center = _center; updateModelMatrix(); }
    void rotate(const glm::vec3 & _angle) { angle = _angle; updateModelMatrix(); }
    void scale(const glm::vec3 & _size) { size = _size; updateModelMatrix(); }
    void move(const glm::vec3 & _angle, const glm::vec3 & _center, const glm::vec3 & _size) { angle = _angle; center = _center; size = _size; updateModelMatrix();}
    
    /*****************************************************************************/
    // setColor
    /*****************************************************************************/
    inline void setColor(const glm::vec3 & _color) { color = _color; }
    
    /*****************************************************************************/
    // render
    /*****************************************************************************/
    void render(const glm::mat4 & projection, const glm::mat4 & view, const glm::vec3 _color = glm::vec3(-1.0, -1.0, -1.0)) {
      
      DEBUG_LOG("glLine::render()");

      if(!isInited){
        fprintf(stderr, "line must be inited before render\n");
        abort();
      }
      
      if(isToInitInGpu()) initInGpu();

      shader.use();
      
      glm::mat4 mvp = projection * view * model;
      
      shader.setUniform("mvp", mvp);
      
      if(_color.r != -1.0) shader.setUniform("color", _color);
      else                 shader.setUniform("color", color);
      
      glEnable(GL_DEPTH_TEST);

      glBindVertexArray(vao);
      
      glEnableVertexAttribArray(0);
      
      glDrawArrays(GL_LINE_STRIP, 0, (GLuint)vertices.size());
      
      glBindVertexArray(0);
            
      glDisable(GL_DEPTH_TEST);
      
    }
    
    /*****************************************************************************/
    // initInGpu
    /*****************************************************************************/
    void initInGpu() {
      
      if(!isInited){
        fprintf(stderr, "line must be inited before set in GPU\n");
        abort();
      }
      
      if(vertices.size() == 0){
        fprintf(stderr, "line must be at least a point\n");
        abort();
      }
      
      DEBUG_LOG("glLine::initInGpu()");
      
      shader.initInGpu();

      if(isToUpdateInGpu && vao != -1) {

        DEBUG_LOG("glLine::updateInGpu()");
        
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size()*sizeof(glm::vec3), glm::value_ptr(vertices[0]));
                
        glBindBuffer(GL_ARRAY_BUFFER,0);
                
        isToUpdateInGpu = false;
        
      } else {
        
        DEBUG_LOG("glLine::initInGpu()");
      
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        
        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), glm::value_ptr(vertices[0]), GL_DYNAMIC_DRAW);
        
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        
        glBindBuffer(GL_ARRAY_BUFFER,0);
        
        glBindVertexArray(0);
              
      }
        
      isInitedInGpu = true;
      
      contex = glfwGetCurrentContext();
      
    }
    
    /*****************************************************************************/
    // update() -
    /*****************************************************************************/
    void update(const std::vector<glm::vec3> & _vertices) {
      
      vertices = _vertices;
      
      isToUpdateInGpu = true;
      isInitedInGpu   = false;
      
    }
    
  private:
    
    /*****************************************************************************/
    // updateModelMatrix
    /*****************************************************************************/
    void updateModelMatrix() {
      
      model = glm::mat4(1.0f);
      
      model = glm::translate(model, center); // Translate it down a bit so it's at the center of the scene
      
      model = glm::scale(model, size);
      
      model  = glm::rotate(model, angle.x, glm::vec3(1, 0, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      model  = glm::rotate(model, angle.y, glm::vec3(0, 1, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      model  = glm::rotate(model, angle.z, glm::vec3(0, 0, 1)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      
    }
    
    /*****************************************************************************/
    // isToInitInGpu
    /*****************************************************************************/
    inline bool isToInitInGpu() {
      
      if(contex != glfwGetCurrentContext() || !isInitedInGpu) { vao = -1; return true; }

      return false;
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_LINE_H_ */
