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

#ifndef _H_MPL_GLLIGHT_H_
#define _H_MPL_GLLIGHT_H_

#include <cstdio>
#include <cstdlib>

#include <glm/glm.hpp>

#include "glShader.hpp"


/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // glLight
  /*****************************************************************************/
  class glLight {
    
  private:
    
    // contatore del numero di luci create
    static GLuint counter;
    
  public:
    
    // posizione della luce
    glm::vec3 position;
    
    // intensita della luce (colore)
    glm::vec3 intensities;
    
    // direzione della luce
    glm::vec3 direction;
    
    // attenuazione
    float attenuation;
    
    // ?
    float ambientCoefficient;
    
    // grandezza del cono
    float coneAngle;
    
    // direzione del cono
    glm::vec3 coneDirection;
    
    /*****************************************************************************/
    // glLight
    /*****************************************************************************/
    glLight() { counter++; }
    
    /*****************************************************************************/
    // setPosition
    /*****************************************************************************/
    void setPosition(const glm::vec3 & _position) { position = _position; }
    
    /*****************************************************************************/
    // setDirection
    /*****************************************************************************/
    void setDirection(const glm::vec3 & _direction) { direction = _direction; }
    
    /*****************************************************************************/
    // setInShader
    /*****************************************************************************/
    void setInShader(const mpl::glShader & shader, const glm::mat4 & view) const {
      
      glm::vec3 _position = glm::vec3(view * glm::vec4(position, 1));
      
      shader.setUniform("light.position", _position);
      shader.setUniform("light.direction", _position);
      shader.setUniform("light.ambient", glm::vec3(0.0f));
      shader.setUniform("light.diffuse", glm::vec3(1.0f));
      shader.setUniform("light.specular", glm::vec3(1.0f));
      
    }
    
  };
  
  GLuint glLight::counter = 0;
  
} /* namespace mpl */

#endif /* _H_MPL_GLLIGHT_H_ */
