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

#ifndef _H_MPL_GLCAMERA_H_
#define _H_MPL_GLCAMERA_H_

#include <cstdlib>
#include <cstdio>

#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
//#include <glm/gtx/string_cast.hpp>


/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // glCamera
  /*****************************************************************************/
  class glCamera {
    
  public:
    
    // Tipi di telecamera
    enum MODE { FREE, TARGET };
    
    // Tipi di movimento della camera
    enum MOVEMENT { FORWARD, BACKWARD, LEFT, RIGHT };

  private:
    
    // Tipologia della camera
    MODE mode;
    
    // Constanti
    GLfloat SPEED      =  2.00f;
    GLfloat SENSITIVTY =  0.25f;
    
    // Camera Attributes
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 right;
    glm::vec3 up;
    glm::vec3 worldUp;
    
    // Eular Angles
    GLfloat yaw;
    GLfloat pitch;
    
    glm::vec3 target;
    
    glm::mat4 projection;
    
    GLsizei width;
    GLsizei height;
    
  public:
    
    
    /*****************************************************************************/
    // glCamera() - Constructor of a easy camera
    /*****************************************************************************/
    glCamera() {}
    
    /*****************************************************************************/
    // glCamera() - Constructor of a easy camera
    /*****************************************************************************/
    glCamera(GLsizei _width, GLsizei _height, glm::vec3 _position = glm::vec3(0.0f)) {
      init(_width, _height, 45.0f, 0.01f, 100.0f, _position);
    }
    
    /*****************************************************************************/
    // glCamera() - Constructor of a full camera
    /*****************************************************************************/
    glCamera(GLsizei _width, GLsizei _height, float fov, float zNear, float zFar, glm::vec3 _position = glm::vec3(0.0f), MODE _mode = FREE, glm::vec3 _target = glm::vec3(0.0f)) {
      init(_width, _height, fov, zNear, zFar, _position, _mode, _target);
    }
    
    /*****************************************************************************/
    // init() - Inizializatore of a easy camera
    /*****************************************************************************/
    void init(GLsizei _width, GLsizei _height, glm::vec3 _position = glm::vec3(0.0f)) { init(_width, _height, 45.0f, 0.01f, 10.0f, _position); }
    
    /*****************************************************************************/
    // init() - Inizializatore di una camera
    /*****************************************************************************/
    void init(GLsizei _width, GLsizei _height, float fov, float zNear, float zFar, glm::vec3 _position = glm::vec3(0.0f), MODE _mode = FREE, glm::vec3 _target = glm::vec3(0.0f)) {
      
      //printf("%d %d %f %f %f\n", _width, _height, fov, zNear, zFar);
      
      position = _position;
      target   = _target;

      front   = glm::vec3(0.0f, 0.0f, -1.0f);
      worldUp = glm::vec3(0.0f, 1.0f,  0.0f);
      
      width  = _width;
      height = _height;
      
      yaw   = 0;
      pitch = 0;
      
      mode = _mode;
      
      projection = glm::perspective(glm::radians(fov), width/(float)height, zNear, zFar);
      
     // std::cout << "Camera projection matrix: " << glm::to_string(projection) << std::endl;
      
      updateCameraVectors();
      
    }
    
    /*****************************************************************************/
    // processKeyboard() - Processa i tasti premuti
    /*****************************************************************************/
    void processKeyboard(MOVEMENT direction, GLfloat deltaTime) {
      
      if(mode == FREE){

        GLfloat velocity = SPEED * deltaTime;
        
        if(direction == FORWARD)  position += front * velocity;
        if(direction == BACKWARD) position -= front * velocity;
        if(direction == LEFT)     position -= right * velocity;
        if(direction == RIGHT)    position += right * velocity;
        
      }
      
    }
    
    /*****************************************************************************/
    // processMouseMovement() - Processa il moviemento del mouse
    /*****************************************************************************/
    void processMouseMovement(GLfloat xOffset, GLfloat yOffset, GLboolean constrainPitch = true) {
      
      if(mode == FREE){

        xOffset *= SENSITIVTY;
        yOffset *= SENSITIVTY;
        
        yaw   += xOffset;
        pitch += yOffset;
        
        // Make sure that when pitch is out of bounds, screen doesn't get flipped
        if(constrainPitch) {
          if(pitch >  89.0f) pitch = 89.0f;
          if(pitch < -89.0f) pitch = -89.0f;
        }
        
        // Update Front, Right and Up Vectors using the updated Eular angles
        updateCameraVectors();
        
      }
      
    }
    
    /*****************************************************************************/
    // processMouseScroll() -
    /*****************************************************************************/
    void processMouseScroll(GLfloat yOffset) { }
    
    /*****************************************************************************/
    // setSensorSize() - Setto la grandezza in pixel del sensore
    /*****************************************************************************/
    void setSensorSize(GLsizei _width, GLsizei _height) { width = _width; height = _height; }
    
    /*****************************************************************************/
    // getSensorSize() - Ritorno la grandezza in pixel del sensore
    /*****************************************************************************/
    inline GLsizei getWidth()  const { return width;  }
    inline GLsizei getHeight() const { return height; }
    
    /*****************************************************************************/
    // setPosition() - Aggiorno la posizione della camera
    /*****************************************************************************/
    void setPosition(const glm::vec3 & _position) { if(mode!=FREE) position = _position; }
    void initPosition(const glm::vec3 & _position) { position = _position; }

    inline void setPitch(float _pitch) { pitch = _pitch; }
    inline void setYaw(float _yaw) { yaw = _yaw; }

    /*****************************************************************************/
    // setTarget() - Aggiorno la posizione del target
    /*****************************************************************************/
    void setTarget(const glm::vec3 & _target) { if(mode!=FREE) target = _target; }
    
    /*****************************************************************************/
    // Get projection and view matrixs
    /*****************************************************************************/
    inline glm::mat4 getProjection() const { return projection; }
    inline glm::mat4 getView()       const {
      if(mode == FREE)   return glm::lookAt(position, position + front, up);
      if(mode == TARGET) return glm::lookAt(position, target          , up);
      abort();
    }
    
    /*****************************************************************************/
    // getMode() - Camera mode
    /*****************************************************************************/
    inline MODE getMode() const { return mode; }
    
  private:
    
    /*****************************************************************************/
    //updateCameraVectors
    /*****************************************************************************/
    // Calculates the front vector from the Camera's (updated) Eular Angles
    void updateCameraVectors() {
      
      // Calculate the new Front vector
      glm::vec3 _front;
      
      _front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
      _front.y = sin(glm::radians(pitch));
      _front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
      
      front = glm::normalize(_front);
      
      // Also re-calculate the Right and Up vector
      right = glm::normalize(glm::cross(front, worldUp));
      
      // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
      up = glm::normalize(glm::cross(right, front));
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_GLWINDOW_H_ */

