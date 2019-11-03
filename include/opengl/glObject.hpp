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

#ifndef _H_MPL_OPENGL_OBJECT_H_
#define _H_MPL_OPENGL_OBJECT_H_

#include <cstdio>
#include <cstdlib>

#include <string>

#include "glfw.hpp"

#include <glm/glm.hpp>

#include <mpl/debug.hpp>

#include "glShader.hpp"


/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

  /*****************************************************************************/
  // Class glObject
  /*****************************************************************************/
  class glObject {
    
    protected:
      
      std::string name;
    
      GLFWwindow * contex;
      
      glShader shader;
      
      bool isInited;
      bool isInitedInGpu;
      bool isToUpdateInGpu;
      
      glm::vec3 center;
      glm::vec3 angle;
      glm::vec3 size;
      
      glm::mat4 model;
      
      glm::vec3 color;
      
      int style;
      
    public:
      
      enum STYLE { SOLID, WIREFRAME };
      
      /*****************************************************************************/
      // glObject() -
      /*****************************************************************************/
      glObject(const std::string & _name = "") : isInited(false), isInitedInGpu(false), isToUpdateInGpu(false), name(_name) { }
      
      /*****************************************************************************/
      // init() -
      /*****************************************************************************/
      void init(const glm::vec3 & _center = glm::vec3(0.0), const glm::vec3 & _angle = glm::vec3(0.0), const glm::vec3 & _size = glm::vec3(1.0), const std::string & _name = "") {
        
        name = _name;
        
        DEBUG_LOG("glObject::init(" + name + ")");
        
        shader.init("/usr/local/include/mpl/opengl/shader/plain.vs", "/usr/local/include/mpl/opengl/shader/plain.fs");
        
        center = _center;
        
        angle = _angle;
        
        size = _size;
        
        updateModelMatrix();
        
        isInited = false;
        
        isInitedInGpu = false;
        
        isToUpdateInGpu = false;
        
      }
      
      /*****************************************************************************/
      // renderBegin() -
      /*****************************************************************************/
      void renderBegin(const glm::mat4 & projection, const glm::mat4 & view) {
        
        if(!isInited){
          fprintf(stderr, "line must be inited before render\n");
          abort();
        }
        
        if(isToInitInGpu()) initInGpu();
        
        shader.use();
        
        glm::mat4 mvp = projection * view * model;
        
        shader.setUniform("mvp", mvp);
        
        shader.setUniform("color", color);
        
        glEnable(GL_DEPTH_TEST);
        
      }
      
      /*****************************************************************************/
      // renderEnd() -
      /*****************************************************************************/
      void renderEnd() {
        
        glDisable(GL_DEPTH_TEST);
        
      }
      
      /*****************************************************************************/
      // initInGpu() -
      /*****************************************************************************/
      void initInGpu() {
        
        if(!isInited){
          fprintf(stderr, "axes must be inited before set in GPU\n");
          abort();
        }
        
        shader.initInGpu();
        
        setInGpu();
        
        isInitedInGpu = true;
        
        contex =  glfwGetCurrentContext();
        
      }
      
      /*****************************************************************************/
      // Position fuction
      /*****************************************************************************/
      inline void translate(const glm::vec3 & _center) { center = _center; updateModelMatrix(); }
      inline void rotate(const glm::vec3 & _angle) { angle = _angle; updateModelMatrix(); }
      inline void rotate(const glm::quat & _angle) { angle.x = _angle.x; angle.y = _angle.y; angle.z = _angle.z; updateModelMatrix(); }
      inline void scale(const glm::vec3 & _size) { size = _size; updateModelMatrix(); }
      inline void move(const glm::vec3 & _angle, const glm::vec3 & _center, const glm::vec3 & _size) { angle = _angle; center = _center; size = _size; updateModelMatrix();}
      
      /*****************************************************************************/
      // setColor() -
      /*****************************************************************************/
      inline void setColor(const glm::vec3 & _color) { color = _color; }
      
      /*****************************************************************************/
      // setStyle() -
      /*****************************************************************************/
      inline void setStyle(int _style) { style = _style; }
    
      /*****************************************************************************/
      // setName() -
      /*****************************************************************************/
    inline void setName(std::string _name) { name = _name; }
    
    private:
      
      /*****************************************************************************/
      // updateModelMatrix() -
      /*****************************************************************************/
      inline void updateModelMatrix() {
        
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
        
        if(contex != glfwGetCurrentContext() || !isInitedInGpu) { return true; }
        
        return false;
        
      }
      
    protected:
      
      /*****************************************************************************/
      // setInGpu
      /*****************************************************************************/
      virtual void setInGpu() = 0;
    
  }; /* class glObject */

} /* namespace mpl */

#endif /* _H_MPL_OPENGL_OBJECT_H_ */
