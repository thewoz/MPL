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

#ifndef _H_MPL_GLSHADER_H_
#define _H_MPL_GLSHADER_H_

#include <cstdlib>
#include <cstdio>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <mpl/stdio.hpp>

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // glShader
  /*****************************************************************************/
  class glShader {
    
  private:
    
    GLuint program;
    
  public:
    
    /*****************************************************************************/
    // glShader - Constructor generates the shader on the fly
    /*****************************************************************************/
    glShader() { }
    glShader(const GLchar * vertexPath, const GLchar * fragmentPath, const GLchar * geometryPath = NULL) { load(vertexPath,fragmentPath, geometryPath); }
    
    /*****************************************************************************/
    // load - Constructor generates the shader on the fly
    /*****************************************************************************/
    void load(const GLchar * vertexPath, const GLchar * fragmentPath, const GLchar * geometryPath = NULL) {
    
      printf("%s\n", vertexPath);
      printf("%s\n", fragmentPath);

      // 1. Retrieve the vertex/fragment source code from filePath
      std::string vertexCode;
      std::string fragmentCode;
      std::string geometryCode;
      
      std::ifstream vShaderFile;
      std::ifstream fShaderFile;
      std::ifstream gShaderFile;
      
      // ensures ifstream objects can throw exceptions:
      vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
      fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
      gShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

      try {
        
        // Open files
        vShaderFile.open(vertexPath);
        fShaderFile.open(fragmentPath);
        
        std::stringstream vShaderStream, fShaderStream;
        
        // Read file's buffer contents into streams
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();
        
        // close file handlers
        vShaderFile.close();
        fShaderFile.close();
        
        // Convert stream into string
        vertexCode   = vShaderStream.str();
        fragmentCode = fShaderStream.str();
        
        // if geometry shader path is present, also load a geometry shader
        if(geometryPath != NULL) {
          
          gShaderFile.open(geometryPath);
          std::stringstream gShaderStream;
          gShaderStream << gShaderFile.rdbuf();
          gShaderFile.close();
          geometryCode = gShaderStream.str();
          
        }
        
      } catch (std::system_error & e) {
        std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << strerror(errno) << std::endl;
        abort();
      }
      
      const GLchar *vShaderCode = vertexCode.c_str();
      const GLchar *fShaderCode = fragmentCode.c_str();
      
      // 2. Compile shaders
      GLuint vertex, fragment;
      GLint success;
      GLchar infoLog[512];
      
      // Vertex Shader
      vertex = glCreateShader(GL_VERTEX_SHADER);
      glShaderSource(vertex, 1, &vShaderCode, NULL);
      glCompileShader(vertex);
      
      // Print compile errors if any
      glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
      
      if(!success) {
        glGetShaderInfoLog( vertex, 512, NULL, infoLog );
        std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
        abort();
      }
      
      // Fragment Shader
      fragment = glCreateShader(GL_FRAGMENT_SHADER);
      glShaderSource(fragment, 1, &fShaderCode, NULL);
      glCompileShader(fragment);
      
      // Print compile errors if any
      glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
      
      if(!success){
        glGetShaderInfoLog(fragment, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
        abort();
      }
      
      // if geometry shader is given, compile geometry shader
      GLuint geometry = 0;
      if(geometryPath != NULL) {
        const GLchar * gShaderCode = geometryCode.c_str();
        geometry = glCreateShader(GL_GEOMETRY_SHADER);
        glShaderSource(geometry, 1, &gShaderCode, NULL);
        glCompileShader(geometry);
        glGetShaderiv(geometry, GL_COMPILE_STATUS, &success);
        
        if(!success) {
          glGetShaderInfoLog(vertex, 512, NULL, infoLog);
          std::cout << "ERROR::SHADER::GEOMETRY::COMPILATION_FAILED\n" << infoLog << std::endl;
          abort();
        }
      }
      
      // Shader Program
      program = glCreateProgram();
      
      glAttachShader(program, vertex);
      glAttachShader(program, fragment);
      
      if(geometryPath != NULL) glAttachShader(program, geometry);
      
      glLinkProgram(program);
      
      // Print linking errors if any
      glGetProgramiv(program, GL_LINK_STATUS, &success);
      
      if(!success) {
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        abort();
      }
      
      // Delete the shaders as they're linked into our program now and no longer necessery
      glDeleteShader(vertex);
      glDeleteShader(fragment);
      
     // fprintf(stderr, "DEBUG SHADER compile program: %d - %s %s %s \n", program, vertexPath, fragmentPath, (geometryPath != NULL) ? geometryPath : " ");
      
      if(geometryPath != NULL) glDeleteShader(geometry);
      
    }
    
    /*****************************************************************************/
    // use - use the current shader
    /*****************************************************************************/
    inline void use() const { glUseProgram(program); }
    
    /*****************************************************************************/
    // get - get the current shader program
    /*****************************************************************************/
    inline GLuint get() const { return program; }
    
    /*****************************************************************************/
    // setUniform
    /*****************************************************************************/
    template <typename T>
    inline void setUniform(const std::string & name, const T & value) const {
      
      GLint location = glGetUniformLocation(program, name.c_str());
      
      if(location == -1 &&  glGetError() != 0){
        fprintf(stderr, "error in get uniform location \"%s\" in program %d: %s\n", name.c_str(), program, "");//glewGetErrorString(glGetError()));
        abort();
      }
      
      //fprintf(stderr, "DEBUG DRAW SHADER set on %d '%s' to %d\n", location, name.c_str(), value);
      
      setUniform(location, value);
      
    }
    
  private:
    
    inline void setUniform(GLint location, const unsigned int   & value) const { glUniform1i(location, value); }
    inline void setUniform(GLint location, const int   & value) const { glUniform1ui(location, value); }
    inline void setUniform(GLint location, const float & value) const { glUniform1f(location, value); }
    inline void setUniform(GLint location, const glm::vec3 & value) const { glUniform3fv(location, 1, &value[0]); }
    inline void setUniform(GLint location, const glm::mat4 & value) const { glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(value)); }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_GLSHADER_H_ */
