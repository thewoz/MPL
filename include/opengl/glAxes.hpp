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

  public:
    
    /*****************************************************************************/
    // glAxes
    /*****************************************************************************/
    glAxes() : isInited(false) {  }

    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(float scale = 1.0) {
      
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
      
      shader.load("/usr/local/include/mpl/opengl/shader/plain.vs", "/usr/local/include/mpl/opengl/shader/plain.fs");
      
      center = glm::vec3(0.0,0.0,0.0);
      
      angle = glm::vec3(0.0,0.0,0.0);
      
      size = glm::vec3(scale);
      
      updateModelMatrix();
      
      isInited = true;
      
      
      
      
//      xAxis.init({glm::vec3(0.0f), glm::vec3(1.0f,0.0f,0.0f)}, glm::vec3(1.0,0.0,0.0));
//      yAxis.init({glm::vec3(0.0f), glm::vec3(0.0f,1.0f,0.0f)}, glm::vec3(0.0,1.0,0.0));
//      zAxis.init({glm::vec3(0.0f), glm::vec3(0.0f,0.0f,1.0f)}, glm::vec3(0.0,0.0,1.0));
//
//      xAxis.scale(glm::vec3(scale)); yAxis.scale(glm::vec3(scale)); zAxis.scale(glm::vec3(scale));
      
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
    void render(const glm::mat4 & projection, const glm::mat4 & view) const {

//      xAxis.render(projection, view);
//      yAxis.render(projection, view);
//      zAxis.render(projection, view);
      
      if(!isInited){
        fprintf(stderr, "line shader not inited\n");
        abort();
      }
      
      //glEnableClientState(GL_VERTEX_ARRAY);
      
      glEnable(GL_DEPTH_TEST);
      
      shader.use();
      
      glm::mat4 mvp = projection * view * model;
      
      shader.setUniform("mvp", mvp);
      
      shader.setUniform("color", glm::vec3(1.0f,0.0f,0.0f));
      
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

    }
    
  private:
    
    /*****************************************************************************/
    // updateModelMatrix
    /*****************************************************************************/
    void updateModelMatrix() {
      
//      model = glm::mat4(1.0f);
//
//      model = glm::translate(model, center); // Translate it down a bit so it's at the center of the scene
//
//      model = glm::scale(model, size);
//
//      model  = glm::rotate(model, angle.x, glm::vec3(1, 0, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
//      model  = glm::rotate(model, angle.y, glm::vec3(0, 1, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
//      model  = glm::rotate(model, angle.z, glm::vec3(0, 0, 1)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      
      glm::mat4 tm = glm::translate(glm::mat4(), center);
      
      glm::mat4 sm = glm::scale(glm::mat4(), size);
    
      glm::mat4 rm = glm::mat4_cast(angle);
      
      model = tm * rm * sm;
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_LINE_H_ */
