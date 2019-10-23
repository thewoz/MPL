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
    bool isInitedInGpu;

    glm::vec3 color;
    
    glm::vec3 center;
    glm::vec3 angle;
    glm::vec3 size;
    
    glm::mat4 model;
    
    int lenght;
    
    int slices;
    
    int style;
    
    glShader shader;
    
  public:
    
    glGrid() : isInited(false), isInitedInGpu(false) { }
    
    /*****************************************************************************/
    // glGrid
    /*****************************************************************************/
    glGrid(int _slices, glm::vec3 _color = glm::vec3(0.0,0.0,0.0)) : isInitedInGpu(false) { init(_slices, _color); }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(int _slices, glm::vec3 _color = glm::vec3(0.0,0.0,0.0)) {
      
      shader.init("/usr/local/include/mpl/opengl/shader/plain.vs", "/usr/local/include/mpl/opengl/shader/plain.fs");
      
      slices = _slices;
      
      center = glm::vec3(0.0,0.0,0.0);
      
      angle = glm::vec3(0.0,0.0,0.0);
      
      size = glm::vec3(1.0,1.0,1.0);

      color = _color;
        
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
    void render(const glm::mat4 & projection, const glm::mat4 & view, const glm::vec3 _color = glm::vec3(-1.0, -1.0, -1.0)) {
      
      if(!isInited){
        fprintf(stderr, "grid must be inited before render\n");
        abort();
      }
      
      if(!isInitedInGpu) initInGpu();
      
      shader.use();

      glm::mat4 mvp = projection * view * model;
      
      shader.setUniform("mvp", mvp);
      
      if(_color.r != -1.0) shader.setUniform("color", _color);
      else                 shader.setUniform("color", color);
          
      glEnable(GL_DEPTH_TEST);

      glBindVertexArray(vao);
          
      glDrawElements(GL_LINES, lenght, GL_UNSIGNED_INT, NULL);

      glBindVertexArray(0);
            
      glDisable(GL_DEPTH_TEST);

    }
    
    /*****************************************************************************/
    // initInGpu
    /*****************************************************************************/
    void initInGpu() {
      
      if(!isInited){
        fprintf(stderr, "grid must be inited before set in GPU\n");
        abort();
      }
      
      shader.initInGpu();
      
      std::vector<glm::vec3> vertices;
      std::vector<glm::uvec4> indices;
         
      for(int j=0; j<=slices; ++j) {
        for(int i=0; i<=slices; ++i) {
          float x = (float)i/(float)slices;
          float y = 0;
          float z = (float)j/(float)slices;
          vertices.push_back(glm::vec3(x, y, z));
        }
      }
           
      for(int j=0; j<slices; ++j) {
        for(int i=0; i<slices; ++i) {
          
          int row1 =  j    * (slices+1);
          int row2 = (j+1) * (slices+1);
          
          indices.push_back(glm::uvec4(row1+i, row1+i+1, row1+i+1, row2+i+1));
          indices.push_back(glm::uvec4(row2+i+1, row2+i, row2+i, row1+i));

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
      glBufferData( GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(glm::uvec4), glm::value_ptr(indices[0]), GL_STATIC_DRAW);
      //glBufferData( GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(*indices.data()), indices.data(), GL_STATIC_DRAW);

      glBindVertexArray( 0 );
      glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
      glBindBuffer( GL_ARRAY_BUFFER, 0 );
      
      lenght = (GLuint)indices.size()*4;

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


