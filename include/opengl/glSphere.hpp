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

#ifndef _H_MPL_SPHERE_H_
#define _H_MPL_SPHERE_H_

#include <cstdio>
#include <cstdlib>

#include <glm/glm.hpp>


#include "glShader.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

/*****************************************************************************/
// Class glSphere
/*****************************************************************************/
class glSphere {
  
private:
  
  GLuint vao;
  
  bool isInited;
  bool isInitedInGpu;

  glm::vec3 color;
  
  glm::vec3 center;
  glm::vec3 angle;
  glm::vec3 size;

  glm::mat4 model;
  
  float radius;
  int slices;
  int stacks;
  
  int style;
  
  glShader shader;
  
  GLuint vertexSize;
  
public:
  
  enum SPHERE_STYLE { SOLID_SPHERE, WIREFRAME_SPHERE };
  
  glSphere() : isInited(false), isInitedInGpu(false) { }
  
  /*****************************************************************************/
  // glSphere
  /*****************************************************************************/
  glSphere(float _radius, int _slices, int _stacks, int _style = WIREFRAME_SPHERE, glm::vec3 _color = glm::vec3(0.0,0.0,0.0)) : isInitedInGpu(false) {
    init(_radius, _slices, _stacks, _style, _color);
  }
  
  /*****************************************************************************/
  // init
  /*****************************************************************************/
  void init(float _radius, int _slices, int _stacks, int _style = WIREFRAME_SPHERE, glm::vec3 _color = glm::vec3(0.0,0.0,0.0)) {
    
    shader.init("/usr/local/include/mpl/opengl/shader/plain.vs", "/usr/local/include/mpl/opengl/shader/plain.fs");
    
    radius = _radius;
    
    slices = _slices;
    
    stacks = _stacks;
    
    style = _style;
    
    color = _color;
    
    center = glm::vec3(0.0,0.0,0.0);
    
    angle = glm::vec3(0.0,0.0,0.0);
    
    size = glm::vec3(1.0);
    
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
  void render(const glm::mat4 & projection, const glm::mat4 & view, bool mode = WIREFRAME_SPHERE, const glm::vec3 _color = glm::vec3(-1.0, -1.0, -1.0)) const {
    
    if(!isInited){
      fprintf(stderr, "sphere must be inited before render\n");
      abort();
    }
    
    if(!isInitedInGpu){
      fprintf(stderr, "sphere must be inited in gpu before render\n");
      abort();
    }
   
    shader.use();

    glm::mat4 mvp = projection * view * model;
    
    shader.setUniform("mvp", mvp);
    
    if(_color.r != -1.0) shader.setUniform("color", _color);
    else                 shader.setUniform("color", color);
        
    glDisable(GL_CULL_FACE);
    
    glEnable(GL_DEPTH_TEST);
    
    glBindVertexArray(vao);
    
    glEnableVertexAttribArray(0);
    
    if(style == WIREFRAME_SPHERE) glDrawElements(GL_LINES, vertexSize, GL_UNSIGNED_INT, nullptr);
    if(style == SOLID_SPHERE)     glDrawElements(GL_TRIANGLES, vertexSize, GL_UNSIGNED_INT, nullptr);
    
    glBindVertexArray(0);
        
    glDisable(GL_DEPTH_TEST);
    
  }
  
  /*****************************************************************************/
  // initInGpu
  /*****************************************************************************/
  void initInGpu() {
    
    if(!isInited){
      fprintf(stderr, "sphere must be inited before set in GPU\n");
      abort();
    }
    
    shader.initInGpu();

    const float _2pi = 2.0f * M_PI;
       
    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> textureCoords;
     
    for(int i = 0; i <= stacks; ++i) {
       
      // V texture coordinate.
      float V = i / (float)stacks;
      float phi = V * M_PI;
       
      for( int j = 0; j <= slices; ++j) {
         
        // U texture coordinate.
        float U = j / (float)slices;
        float theta = U * _2pi;
        
        float X = cos(theta) * sin(phi);
        float Y = cos(phi);
        float Z = sin(theta) * sin(phi);
        
        positions.push_back( glm::vec3( X, Y, Z) * radius );
        normals.push_back( glm::vec3(X, Y, Z) );
        textureCoords.push_back( glm::vec2(U, V) );
         
      }
       
    }
     
    // Now generate the index buffer
    std::vector<GLuint> indicies;
     
    for(int i = 0; i < slices * stacks + slices; ++i) {
      indicies.push_back( i );
      indicies.push_back( i + slices + 1  );
      indicies.push_back( i + slices );
       
      indicies.push_back( i + slices + 1  );
      indicies.push_back( i );
      indicies.push_back( i + 1 );
    }
         
    glGenVertexArrays( 1, &vao );
    glBindVertexArray( vao );
    
    GLuint vbos[4];
    glGenBuffers( 4, vbos );
    
    glBindBuffer( GL_ARRAY_BUFFER, vbos[0] );
    glBufferData( GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec3), positions.data(), GL_STATIC_DRAW );
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0 );
    
    glBindBuffer( GL_ARRAY_BUFFER, vbos[1] );
    glBufferData( GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW );
    glVertexAttribPointer( 2, 3, GL_FLOAT, GL_TRUE, 0, nullptr);
    glEnableVertexAttribArray( 2 );
    
    glBindBuffer( GL_ARRAY_BUFFER, vbos[2] );
    glBufferData( GL_ARRAY_BUFFER, textureCoords.size() * sizeof(glm::vec2), textureCoords.data(), GL_STATIC_DRAW );
    glVertexAttribPointer( 8, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray( 8 );
    
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, vbos[3] );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, indicies.size() * sizeof(GLuint), indicies.data(), GL_STATIC_DRAW );
    
    glBindVertexArray( 0 );
    glBindBuffer( GL_ARRAY_BUFFER, 0 );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
    
    vertexSize = (slices * stacks + slices) * 6;

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


#endif /* _H_MPL_SPHERE_H_ */
