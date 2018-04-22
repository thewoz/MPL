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

#ifndef _H_MPL_GLTEXTURE_H_
#define _H_MPL_GLTEXTURE_H_

#include <cstdlib>
#include <cstdio>

#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <SOIL2/SOIL2.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // glTexture
  /*****************************************************************************/
  class glTexture {
    
  protected:
    
    /* texture id */
    GLuint id;
    
    /*****************************************************************************/
    // glTexture - Empty constructor
    /*****************************************************************************/
    glTexture(){}
    
  public:
    
    inline GLuint getId() const { return id; }
    
  protected:
    
    /*****************************************************************************/
    // activate
    /*****************************************************************************/
    inline void _activate(GLenum target, GLenum unit) const {
      
      // Active proper texture unit before binding
      glActiveTexture(GL_TEXTURE0 + unit);
      
      //And finally bind the texture
      glBindTexture(target, id);
      
    }
    
  };
  
  /*****************************************************************************/
  // glTexture2D
  /*****************************************************************************/
  class glTexture2D : public glTexture {
    
  private:
    
    bool isInitialized;
    
    /* texture type */
    std::string type;
    
    /* texture path */
    std::string path;
    
    /* texture name */
    std::string name;
    
  public:
    
    /*****************************************************************************/
    // glTexture2D
    /*****************************************************************************/
    glTexture2D() : isInitialized(false) { }
    
    /*****************************************************************************/
    // glTexture2D
    /*****************************************************************************/
    glTexture2D(const std::string type, const std::string & filename, const std::string & directory) {
      init(type, filename, directory);
    }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(const std::string type, const std::string & filename, const std::string & directory){
      
      //Generate texture ID and load texture data
      
      name  = filename;
      
      path = directory + '/' + filename;
      
      int width, height;
      
      unsigned char * image = SOIL_load_image(path.c_str(), &width, &height, 0, SOIL_LOAD_RGB);
      
      glGenTextures(1, &id);
      
      // Assign texture to ID
      glBindTexture(GL_TEXTURE_2D, id);
      
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
      
      glGenerateMipmap(GL_TEXTURE_2D);
      
      // Parameters
      //    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      //    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      //    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
      //    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_REPEAT);
      
      glBindTexture(GL_TEXTURE_2D, 0);
      
      SOIL_free_image_data(image);
      
      this->type = "material." + type;
      
      //fprintf(stderr, "DEBUG TEXTURE create %s texture '%s' id %d\n", type.c_str(), name.c_str(), id);
      
      isInitialized = true;
      
    }
    
    /*****************************************************************************/
    // getType
    /*****************************************************************************/
    inline std::string getType() const { return type; }
    
    /*****************************************************************************/
    // activate
    /*****************************************************************************/
    inline void activate(GLenum unit) const {
      
      if(!isInitialized) {
        fprintf(stderr, "error: the glTexture2D must initialize before use it\n");
        abort();
      }
      
      _activate(GL_TEXTURE_2D, unit);
      
      //fprintf(stderr, "DEBUG DRAW TEXTURE activate %s texture '%s' id %d on unit %d\n", type.c_str(), name.c_str(), id, unit);
      
    }
    
  };
  
  /*****************************************************************************/
  // glTextureDepthMap
  /*****************************************************************************/
  class glTextureDepthMap : public glTexture {
    
  private:
    
    GLuint width;
    GLuint height;
    
    bool isInitialized;
    
  public:
    
    /*****************************************************************************/
    // glTextureDepthMap
    /*****************************************************************************/
    glTextureDepthMap() : isInitialized(false) { }
    
    /*****************************************************************************/
    // TextureDepthMap
    /*****************************************************************************/
    glTextureDepthMap(GLuint _width, GLuint _height) { init(_width, _height); }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(GLuint _width, GLuint _height) {
      
      width  = _width;
      height = _height;
      
      glGenTextures(1, &id);
      
      glBindTexture(GL_TEXTURE_2D, id);
      
      glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT,         0);
      //glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,   width, height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, 0);
      
//      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //GL_NEAREST
//      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //GL_NEAREST
      
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); //GL_NEAREST
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); //GL_NEAREST
      
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      
      //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
      //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
      
      //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
      //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
      
//      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
//      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
      
      //#define GL_LESS 0x0201
      //#define GL_EQUAL 0x0202
      //#define GL_LEQUAL 0x0203
      //#define GL_GREATER 0x0204
      //#define GL_NOTEQUAL 0x0205
      //#define GL_GEQUAL 0x0206
      //#define GL_ALWAYS 0x0207
      
//      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
//      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
      
//      float borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
//
//      glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
      
      isInitialized = true;
      
    }
    
    /*****************************************************************************/
    // activate
    /*****************************************************************************/
    inline void activate(GLenum unit) const {
      
      if(!isInitialized) {
        fprintf(stderr, "error: the TextureDepthMap must initialize before use it\n");
        abort();
      }
      
      _activate(GL_TEXTURE_2D, unit);
      
      //fprintf(stderr, "DEBUG TEXTURE activate GL_TEXTURE_2D id %d on unit %d\n", id, unit);
      
    }
    
    
  };
  
  /*****************************************************************************/
  // glTextureCubeMap
  /*****************************************************************************/
  class glTextureCubeMap : public glTexture {
    
  private:
    
    GLuint width;
    GLuint height;
    
    bool isInitialized;
    
  public:
    
    /*****************************************************************************/
    // glTextureCubeMap
    /*****************************************************************************/
    glTextureCubeMap() : isInitialized(false) { }
    
    /*****************************************************************************/
    // glTextureCubeMap
    /*****************************************************************************/
    glTextureCubeMap(GLuint _width, GLuint _height) { init(_width, _height); }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(GLuint _width, GLuint _height) {
      
      
      width  = _width;
      height = _height;
      
      glGenTextures(1, &id);
      
      glBindTexture(GL_TEXTURE_CUBE_MAP, id);
      
      for(GLuint i=0; i<6; ++i)
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X+i, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
      
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
      
      //fprintf(stderr, "DEBUG TEXTURE create GL_TEXTURE_CUBE_MAP texture id %d\n", id);
      
      isInitialized = true;
      
    }
    
    /*****************************************************************************/
    // activate
    /*****************************************************************************/
    inline void activate(GLenum unit) const {
      
      if(!isInitialized) {
        fprintf(stderr, "error: the TextureCubeMap must initialize before use it\n");
        abort();
      }
      
      _activate(GL_TEXTURE_CUBE_MAP, unit);
      
      //fprintf(stderr, "DEBUG TEXTURE activate GL_TEXTURE_CUBE_MAP id %d on unit %d\n", id, unit);
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_GLTEXTURE_H_ */
