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

#ifndef _H_MPL_GLSHADOW_H_
#define _H_MPL_GLSHADOW_H_

#include <cstdlib>
#include <cstdio>

#include <glm/glm.hpp>

#include "glModel.hpp"
#include "glShader.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // glShadow
  /*****************************************************************************/
  class glShadow {
    
  public:
    
    glTextureDepthMap depthMap;
    
  private:
    
    glShader shader;
    
    GLuint FBO;
    
    GLfloat near = 1.0f;
    GLfloat far = 25.0f;
    
    GLuint width;
    GLuint height;
    
    glm::mat4 lightSpaceMatrix;
    
  public:
    
    /*****************************************************************************/
    // glShadow - Empty constructor
    /*****************************************************************************/
    glShadow() { }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(GLuint _width, GLuint _height, GLfloat _near, GLfloat _far) {
      
      width  = _width;
      height = _height;
      
      near = _near;
      far  = _far;
      
      // load and compile the shadow shader
      shader.load("shader/Shadow/shadowMap.vs", "shader/Shadow/shadowMap.fs");
      
      // configure depth map FBO
      glGenFramebuffers(1, &FBO);
      
      // create depth cubemap texture
      depthMap.init(width,height);
      
      // attach depth texture as FBO's depth buffer
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
      
      //glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthMap.getId(), 0);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap.getId(), 0);
      
      glDrawBuffer(GL_NONE);
      glReadBuffer(GL_NONE);
      
      // Always check that our framebuffer is ok
      if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        printf("DIOCANE ME\n");
      };
      
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      
      //fprintf(stderr, "DEBUG shadow shader: %d FBO: %d texture id: %d\n", shader.get(), FBO, depthMap.getId());
      
    }
    
    
    /*****************************************************************************/
    // compute
    /*****************************************************************************/
    void compute(const glm::vec3 & lightPosition, const glModel & model) {
      
      GLint viewport[4];
      
      glGetIntegerv(GL_VIEWPORT, viewport);
      
      //glm::mat4 projection = glm::perspective(glm::radians(45.0f), width/(GLfloat)height, near, far);
      glm::mat4 projection = glm::ortho(-1.5f, 1.5f, -1.5f, 1.5f, near, far);
      //glm::mat4 view     = glm::lookAt(lightPos, lightPos + glm::vec3(1.0f, 1.0f, 1.0f), camera.getUp());
      glm::mat4 view       = glm::lookAt(lightPosition, glm::vec3(0.0f), glm::vec3(0.0, 1.0, 0.0));
      
      
      //glm::vec3  a = camera.getUp();
      
      //printf("%f %f %f\n", a[0], a[1], a[2]);
      
      //mvpMatrix = projection * view * model.getMatrix();
      
      lightSpaceMatrix = projection * view;
      
      glEnable(GL_POLYGON_OFFSET_FILL);
      
      glPolygonOffset(1.0f, 1.0f);
      
      glViewport(0, 0, width, height);
      
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, FBO);
      
      glClearDepth(1.0);
      
      glClear(GL_DEPTH_BUFFER_BIT);
      
      glEnable(GL_DEPTH_TEST);
      glDepthFunc(GL_LESS);
      
      glEnable(GL_CULL_FACE);
      glCullFace(GL_FRONT);
      
      shader.use();
      
      shader.setUniform("lightSpaceMatrix", lightSpaceMatrix);
      shader.setUniform("model", model.getModelMatrix());
      
      model.render(shader, false);
      
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
      
      glDisable(GL_POLYGON_OFFSET_FILL);
      
      glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
      
      
    }
    
    /*****************************************************************************/
    // setInShader
    /*****************************************************************************/
    void setInShader(const glShader & program) const {
      
      
      depthMap.activate(0);
      
      //    glm::mat4 biasMatrix(
      //                         0.5, 0.0, 0.0, 0.0,
      //                         0.0, 0.5, 0.0, 0.0,
      //                         0.0, 0.0, 0.5, 0.0,
      //                         0.5, 0.5, 0.5, 1.0
      //                         );
      
      program.setUniform("shadowMap", 0);
      
      program.setUniform("lightSpaceMatrix", lightSpaceMatrix);
      
      //program.setUniform("depthBiasMVP", depthBiasMVP);
      
    }
    
    
  };
  
} /* namespace mpl */


#endif /* _H_MPL_GLSHADOW_H_ */
