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

#ifndef _H_MPL_GRID_H_
#define _H_MPL_GRID_H_

#include <cstdio>
#include <cstdlib>

#include <glm/glm.hpp>

#include "glShader.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

/*****************************************************************************/
// Class glGrid
/*****************************************************************************/
class glGrid {
  
private:
  
  GLuint vao;
  
  bool isInited;
  
  glm::vec3 color;
  
  glm::vec3 center;
  glm::vec3 angle;
  glm::vec3 size;
  
  glm::mat4 model;
  
  GLuint lenght;
  
  int style;
  
  glShader shader;
  
public:
  
  glGrid() : isInited(false) { }
  
  /*****************************************************************************/
  // glGrid
  /*****************************************************************************/
  glGrid(GLint N, glm::vec3 _color = glm::vec3(0.0,0.0,0.0)) { init(N, _color); }
  
  /*****************************************************************************/
  // init
  /*****************************************************************************/
  void init(GLint N, glm::vec3 _color = glm::vec3(0.0,0.0,0.0)) {
    
    std::vector<glm::vec3> vertices;
    std::vector<glm::uvec3> indices;
    
      for (int j=0; j<=N; ++j)
      {
        for (int i=0; i<=N; ++i)
        {
          float x = (float)i/(float)N;
          float y = 0;
          float z = (float)j/(float)N;
          vertices.push_back(glm::vec3(x, y, z));
        }
      }
      
      for (int j=0; j<N; ++j)
      {
        for (int i=0; i<N; ++i)
        {
          int row1 = j * (N+1);
          int row2 = (j+1) * (N+1);
          
          // triangle 1
          indices.push_back(glm::uvec3(row1+i, row1+i+1, row2+i+1));
          
          // triangle 2
          indices.push_back(glm::uvec3(row1+i, row2+i+1, row2+i));
        }
      }
    

    glGenVertexArrays( 1, &vao );
    glBindVertexArray( vao );
    
    GLuint vbo;
    glGenBuffers( 1, &vbo );
    glBindBuffer( GL_ARRAY_BUFFER, vbo );
    glBufferData( GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), glm::value_ptr(vertices[0]), GL_STATIC_DRAW );
    glEnableVertexAttribArray( 0 );
    glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, nullptr );
    
    GLuint ibo;
    glGenBuffers( 1, &ibo );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, ibo );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(glm::uvec3), glm::value_ptr(indices[0]), GL_STATIC_DRAW );
    
    glBindVertexArray( 0 );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
    glBindBuffer( GL_ARRAY_BUFFER, 0 );
    
    shader.load("/usr/local/include/mpl/opengl/shader/plain.vs", "/usr/local/include/mpl/opengl/shader/plain.fs");
    
    center = glm::vec3(0.0,0.0,0.0);
    
    angle = glm::vec3(0.0,0.0,0.0);
    
    size = glm::vec3(1.0,1.0,1.0);

    color = _color;
    
    lenght = (GLuint)vertices.size();
    
    updateModelMatrix();
    
    isInited = true;
    
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
  void render(const glm::mat4 & projection, const glm::mat4 & view, const glm::vec3 _color = glm::vec3(-1.0, -1.0, -1.0)) const {
    
    if(!isInited){
      fprintf(stderr, "grid shader not inited\n");
      abort();
    }
    
 //   glEnableClientState(GL_VERTEX_ARRAY);

    glEnable(GL_DEPTH_TEST);

    shader.use();

    glm::mat4 mvp = projection * view * model;
    
    shader.setUniform("mvp", mvp);
    
    if(_color.r != -1.0) shader.setUniform("color", _color);
    else                 shader.setUniform("color", color);
        
    glBindVertexArray(vao);
    
    glEnableVertexAttribArray(0);
    
    glDrawArrays(GL_LINES, 0, lenght);

    glBindVertexArray(0);
    
    //glDisableClientState(GL_VERTEX_ARRAY);
    
    glDisable(GL_DEPTH_TEST);

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


#endif /* _H_MPL_GRID_H_ */

//void generate_grid(int N, std::vector<glm::vec3> &vertices, std::vector<glm::uvec3> &indices)
//{
//  for (int j=0; j<=N; ++j)
//  {
//    for (int i=0; i<=N; ++i)
//    {
//      float x = (float)i/(float)N;
//      float y = 0;
//      float z = (float)j/(float)N;
//      vertices.push_back(glm::vec3(x, y, z));
//    }
//  }
//
//  for (int j=0; j<N; ++j)
//  {
//    for (int i=0; i<N; ++i)
//    {
//      int row1 = j * (N+1);
//      int row2 = (j+1) * (N+1);
//
//      // triangle 1
//      indices.push_back(glm::uvec3(row1+i, row1+i+1, row2+i+1));
//
//      // triangle 2
//      indices.push_back(glm::uvec3(row1+i, row2+i+1, row2+i));
//    }
//  }
//}

//GLuint generate_vao(const std::vector<glm::vec3> & vertices, const std::vector<glm::uvec3> & indices)
//{
//  GLuint  vao;
//  glGenVertexArrays( 1, &vao );
//  glBindVertexArray( vao );
//
//  GLuint vbo;
//  glGenBuffers( 1, &vbo );
//  glBindBuffer( GL_ARRAY_BUFFER, vbo );
//  glBufferData( GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), glm::value_ptr(vertices[0]), GL_STATIC_DRAW );
//  glEnableVertexAttribArray( 0 );
//  glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, nullptr );
//
//  GLuint ibo;
//  glGenBuffers( 1, &ibo );
//  glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, ibo );
//  glBufferData( GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(glm::uvec3), glm::value_ptr(indices[0]), GL_STATIC_DRAW );
//
//  glBindVertexArray( 0 );
//  glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
//  glBindBuffer( GL_ARRAY_BUFFER, 0 );
//
//  return vao;
//}


