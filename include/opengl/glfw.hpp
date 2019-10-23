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

#ifndef _H_MPL_GLFW_H_
#define _H_MPL_GLFW_H_

#define GLFW_WITH_GLAD

#include <cstdlib>
#include <cstdio>

#ifdef GLFW_WITH_GLAD
  #include <glad/glad.h>
#endif

#ifdef GLFW_WITH_GLEW
  #include <GL/glew.h>
#endif

#include <GLFW/glfw3.h>

/*****************************************************************************/
// glfw
/*****************************************************************************/
namespace glfw {
  
  /*****************************************************************************/
  // glfwErrorCallback
  /*****************************************************************************/
  static void glfwErrorCallback(int error, const char * description) {
    
    fprintf(stderr, "GLFW error (%d): %s\n", error, description);
    
  }
  
  static bool inited = false;
  
  /*****************************************************************************/
  // init
  /*****************************************************************************/
  void init() {
    
    if(!inited) {
      
      // Set GLFW error callback
      glfwSetErrorCallback(glfwErrorCallback);
      
      // Init GLFW
      if(!glfwInit()) {
        fprintf(stderr, "GLFW init error\n");
        exit(EXIT_FAILURE);
      }
      
      // Set all the required options for GLFW
      
#ifdef __APPLE__
      
      // for OpenGL 2.0 context on macOS
      // glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
      // glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
      
      // for OpenGL 3.0+ context on macOS
      //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
      //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
      
      // for OpenGL 4.1 context on macOS
      glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
      glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
      
      glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);           // This telling the mac to deprecate everything before it basically
      glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // This telling the mac to use the core profile
      
#else
      glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
      glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
      glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
      glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
      
      inited = true;
      
    }
    
  }
  
/*****************************************************************************/
// close
/*****************************************************************************/
  void close() {
    
    inited = false;
      
    glfwTerminate();
    
  }
  
} /* namespace glfw */

#endif /* _H_MPL_GLFW_H_ */
