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

#ifndef _H_MPL_AXES_H_
#define _H_MPL_AXES_H_

#include <cstdlib>
#include <cstdio>

#include <glm/glm.hpp>
//#include <glm/gtx/string_cast.hpp>

#include "glLine.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // Class glAxes
  /*****************************************************************************/
  class glAxes {
    
  private:
    
//    glLine xAxis;
//    glLine yAxis;
//    glLine zAxis;

    GLuint vaoX;
    GLuint vaoY;
    GLuint vaoZ;
    
    glm::vec3 center;
    glm::quat angle;
    glm::vec3 size;
    
    glm::mat4 model;

    glShader shader;
    
    bool isInited;
    bool isInitedInGpu;
    
  public:
    
    /*****************************************************************************/
    // glAxes
    /*****************************************************************************/
    glAxes(float _size = 1.0) : isInitedInGpu(false) { init(_size); }

    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(float _size = 1.0) {
      
      DEBUG_LOG("glAxes::init()");

      shader.init("/usr/local/include/mpl/opengl/shader/plain.vs", "/usr/local/include/mpl/opengl/shader/plain.fs");
            
      center = glm::vec3(0.0,0.0,0.0);
      
      angle = glm::vec3(0.0,0.0,0.0);
      
      size = glm::vec3(_size);
      
      updateModelMatrix();
      
      isInited = true;
      
    }
    
    /*****************************************************************************/
    // Position fuction
    /*****************************************************************************/
    void translate(const glm::vec3 & _center) { center = _center; updateModelMatrix(); }
    void rotate(const glm::quat & _angle) { angle = _angle; updateModelMatrix(); }
    void scale(const glm::vec3 & _size) { size = _size; updateModelMatrix(); }
    void move(const glm::vec3 & _angle, const glm::vec3 & _center, const glm::vec3 & _size) { angle = _angle; center = _center; size = _size; updateModelMatrix();}
    
    /*****************************************************************************/
    // draw
    /*****************************************************************************/
    void render(const glm::mat4 & projection, const glm::mat4 & view) {
      
      DEBUG_LOG("glAxes::render()");

      if(!isInited){
        fprintf(stderr, "axex must be inited before render\n");
        abort();
      }
      
      if(!isInitedInGpu) initInGpu();
                  
      shader.use();
      
      glm::mat4 mvp = projection * view * model;

      shader.setUniform("mvp", mvp);
      
      shader.setUniform("color", glm::vec3(1.0f,0.0f,0.0f));
      
      glEnable(GL_DEPTH_TEST);

      glBindVertexArray(vaoX);
      
      glEnableVertexAttribArray(0);
      
      glDrawArrays(GL_LINE_STRIP, 0, 2);
      
      shader.setUniform("color", glm::vec3(0.0f,1.0f,0.0f));
      
      glBindVertexArray(vaoY);
      
      glEnableVertexAttribArray(0);
      
      glDrawArrays(GL_LINE_STRIP, 0, 2);
      
      shader.setUniform("color", glm::vec3(0.0f,0.0f,1.0f));
      
      glBindVertexArray(vaoZ);
      
      glEnableVertexAttribArray(0);
      
      glDrawArrays(GL_LINE_STRIP, 0, 2);
      
      glBindVertexArray(0);
            
      glDisable(GL_DEPTH_TEST);

    }
    
    /*****************************************************************************/
    // initInGpu
    /*****************************************************************************/
    void initInGpu() {
      
      DEBUG_LOG("glAxes::initInGpu()");

      if(!isInited){
        fprintf(stderr, "axes must be inited before set in GPU\n");
        abort();
      }
      
      shader.initInGpu();

      glGenVertexArrays(1, &vaoX);
      glBindVertexArray(vaoX);
   
      std::vector<glm::vec3> verticesX(2, glm::vec3(0.0f)); verticesX[1] = glm::vec3(1.0f,0.0f,0.0f);
       
      GLuint vboX;
      glGenBuffers(1, &vboX);
      glBindBuffer(GL_ARRAY_BUFFER, vboX);
      glBufferData(GL_ARRAY_BUFFER, 2*sizeof(glm::vec3), &verticesX[0], GL_STATIC_DRAW);
       
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
      
      glGenVertexArrays(1, &vaoY);
      glBindVertexArray(vaoY);
       
      std::vector<glm::vec3> verticesY(2, glm::vec3(0.0f)); verticesY[1] = glm::vec3(0.0f,1.0f,0.0f);

      GLuint vboY;
      glGenBuffers(1, &vboY);
      glBindBuffer(GL_ARRAY_BUFFER, vboY);
      glBufferData(GL_ARRAY_BUFFER, 2*sizeof(glm::vec3), &verticesY[0], GL_STATIC_DRAW);
       
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
       
      glGenVertexArrays(1, &vaoZ);
      glBindVertexArray(vaoZ);
       
      std::vector<glm::vec3> verticesZ(2, glm::vec3(0.0f)); verticesZ[1] = glm::vec3(0.0f,0.0f,1.0f);

      GLuint vboZ;
      glGenBuffers(1, &vboZ);
      glBindBuffer(GL_ARRAY_BUFFER, vboZ);
      glBufferData(GL_ARRAY_BUFFER, 2*sizeof(glm::vec3), &verticesZ[0], GL_STATIC_DRAW);
       
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
       
      glBindBuffer(GL_ARRAY_BUFFER,0);
       
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

      model = glm::scale(model, size);

      model  = glm::rotate(model, angle.x, glm::vec3(1, 0, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      model  = glm::rotate(model, angle.y, glm::vec3(0, 1, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      model  = glm::rotate(model, angle.z, glm::vec3(0, 0, 1)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_LINE_H_ */
