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

#ifndef _H_MPL_OPENGL_GLMATERIAL_H_
#define _H_MPL_OPENGL_GLMATERIAL_H_

#include <cstdlib>
#include <cstdio>

#include <vector>
#include <string>

#include <glm/glm.hpp>

#include <assimp/scene.h>

#include "glTexture.hpp"
#include "glShader.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // glMaterial
  /*****************************************************************************/
  class glMaterial {
    
  private:
    
    // material name
    std::string name;
    
    glm::vec3 ke = {0, 0, 0}; // emission color
    glm::vec3 ka = {0, 0, 0}; // ambient color
    glm::vec3 kd = {0, 0, 0}; // diffuse color
    glm::vec3 ks = {0, 0, 0}; // specular color
    glm::vec3 kr = {0, 0, 0}; // reflection color
    glm::vec3 kt = {0, 0, 0}; // transmission color
    
    // illum mode
    int illum = 2;
    
    float d  = 1.0f; // opacity
    float ns = 1.0f; // phong exponent for ks
    float ni = 0.98f; // index of refraction
    
    // material textures
    std::vector<mpl::glTexture2D> textures;
    
    bool haveDiffuseTexture = false;
    bool haveSpecularTexture = false;
    bool haveAmbientTexture = false;
    bool haveEmissimeTexture = false;
    bool haveHeightTexture = false;
    bool haveNormalsTexture = false;
    bool haveShininessTexture = false;
    bool haveOpacityTexture = false;
    bool haveDisplactemenTexture = false;
    bool haveLightTexture = false;
    bool haveReflectionTexture = false;
    
    bool isInited;
    bool isInitializedInGpu;
    bool isBinded;
    
  public:
    
    /*****************************************************************************/
    // glMaterial - Empty constructor
    /*****************************************************************************/
    glMaterial() : isInited(false), isInitializedInGpu(false), isBinded(false) { }
    
    /*****************************************************************************/
    // glMaterial - Assimp material constructor
    /*****************************************************************************/
    glMaterial(const aiMaterial * material, const std::string & path) : isInitializedInGpu(false), isBinded(false) {
      
      aiString tmpName;
      
      material->Get(AI_MATKEY_NAME, tmpName);
      
      name = tmpName.C_Str();
      
      material->Get(AI_MATKEY_COLOR_EMISSIVE,    ke[0]);
      material->Get(AI_MATKEY_COLOR_AMBIENT,     ka[0]);
      material->Get(AI_MATKEY_COLOR_DIFFUSE,     kd[0]);
      material->Get(AI_MATKEY_COLOR_SPECULAR,    ks[0]);
      material->Get(AI_MATKEY_COLOR_REFLECTIVE,  kr[0]);
      material->Get(AI_MATKEY_COLOR_TRANSPARENT, kt[0]);
      
      material->Get(AI_MATKEY_SHININESS, ns);
      
      material->Get(AI_MATKEY_REFRACTI, ni);
      
      material->Get(AI_MATKEY_SHADING_MODEL, illum);
      
      material->Get(AI_MATKEY_OPACITY, d);
      
      // 1. Diffuse maps
      loadTextures(material, aiTextureType_DIFFUSE, "diffuseTexture", path);
      
      // 2. Specular maps
      loadTextures(material, aiTextureType_SPECULAR, "specularTexture", path);
      
      // 3. Ambient maps
      loadTextures(material, aiTextureType_AMBIENT, "ambientTexture", path);
      
      // 4. Emissive maps
      loadTextures(material, aiTextureType_EMISSIVE, "emissimeTexture", path);
      
      // 5. Height maps
      loadTextures(material, aiTextureType_HEIGHT, "heightTexture", path);
      
      // 6. Normals maps
      loadTextures(material, aiTextureType_NORMALS, "normalsTexture", path);
      
      // 7. Shininess maps
      loadTextures(material, aiTextureType_SHININESS, "shininessTexture", path);
      
      // 8. Opacity maps
      loadTextures(material, aiTextureType_OPACITY, "opacityTexture", path);
      
      // 9. Displacement maps
      loadTextures(material, aiTextureType_DISPLACEMENT, "displactemenTexture", path);
      
      // 10. Light maps
      loadTextures(material, aiTextureType_LIGHTMAP, "lightTexture", path);
      
      // 11. Reflection maps
      loadTextures(material, aiTextureType_REFLECTION, "reflectionTexture", path);
      
      //fprintf(stderr, "DEBUG CREATE MATERIAL '%s'\n", name.c_str());
      
      isInited = true;
      
    }
    
    /*****************************************************************************/
    // setInShader
    /*****************************************************************************/
    void setInShader(const mpl::glShader & shader) const {
      
      // set available textures
      shader.setUniform("material.haveDiffuseTexture",      haveDiffuseTexture);
      shader.setUniform("material.haveSpecularTexture",     haveSpecularTexture);
      shader.setUniform("material.haveAmbientTexture",      haveAmbientTexture);
      shader.setUniform("material.haveEmissimeTexture",     haveEmissimeTexture);
      shader.setUniform("material.haveHeightTexture",       haveHeightTexture);
      shader.setUniform("material.haveNormalsTexture",      haveNormalsTexture);
      shader.setUniform("material.haveShininessTexture",    haveShininessTexture);
      shader.setUniform("material.haveOpacityTexture",      haveOpacityTexture);
      shader.setUniform("material.haveDisplactemenTexture", haveDisplactemenTexture);
      shader.setUniform("material.haveLightTexture",        haveLightTexture);
      shader.setUniform("material.haveReflectionTexture",   haveReflectionTexture);
      
      // set colors
      shader.setUniform("material.emissiveColor",   ke);
      shader.setUniform("material.ambientColor",    ka);
      shader.setUniform("material.diffuseColor",    kd);
      shader.setUniform("material.specularColor",   ks);
      shader.setUniform("material.reflectiveColor", kr);
      shader.setUniform("material.trasparentColor", kt);
      
      // set
      shader.setUniform("material.shininess",  ns);
      shader.setUniform("material.refraction", ni);
      shader.setUniform("material.opacity",     d);
      
    }
    
    /*****************************************************************************/
    // bindTextures
    /*****************************************************************************/
    void bindTextures(const mpl::glShader & shader) {
      
      if(!isInitializedInGpu) {
        fprintf(stderr, "Error glMaterial: the textures must be initialized in the GPU before bind it\n");
        abort();
      }
      
      if(isBinded) {
        fprintf(stderr, "Error glMaterial: the textures are allready binded\n");
        abort();
      }
      
      for(GLuint i=0; i<textures.size(); i++) {
        textures[i].activate(i + 1);
        shader.setUniform(textures[i].getType(), i + 1);
      }
      
      isBinded = true;
      
    }
    
    /*****************************************************************************/
    // unbindTexture() -
    /*****************************************************************************/
    void unbindTexture(GLuint shader) {
      
      if(!isBinded) {
        fprintf(stderr, "Error glMaterial: the textures are not binded\n");
        abort();
      }
      
      // Always good practice to set everything back to defaults once configured.
      for(GLuint i=0; i<textures.size(); i++) {
        glActiveTexture(GL_TEXTURE0 + i);
        glBindTexture(GL_TEXTURE_2D, 0);
      }
       
      isBinded = false;

    }
    
    /*****************************************************************************/
    // initInGpu() -
    /*****************************************************************************/
    void initInGpu() {
      
      if(!isInited) {
        fprintf(stderr, "Error glMaterial: material must be inizializadet before set in Gpu\n");
        abort();
      }
      
      for(size_t i=0; i<textures.size(); ++i) {
        textures[i].initInGpu();
      }
      
      isInitializedInGpu = true;
      
    }
    
    /*****************************************************************************/
    // print() -
    /*****************************************************************************/
    void print(FILE * output = stdout) const {
      
      fprintf(output, "%s\n", name.c_str());
      
      fprintf(output, "Ke %f %f %f\n", ke[0], ke[1], ke[2]);
      fprintf(output, "Ka %f %f %f\n", ka[0], ka[1], ka[2]);
      fprintf(output, "Kd %f %f %f\n", kd[0], kd[1], kd[2]);
      fprintf(output, "Ks %f %f %f\n", ks[0], ks[1], ks[2]);
      fprintf(output, "Kr %f %f %f\n", kr[0], kr[1], kr[2]);
      fprintf(output, "Kt %f %f %f\n", kt[0], kt[1], kt[2]);
      
      fprintf(output, "Ns %f\n", ns);
      fprintf(output, "Ni %f\n", ni);
      
      fprintf(output, "d %f\n", d);
      
      fprintf(output, "illum %d\n", illum);
      
      fprintf(output, "\n");
      
    }
    
  private:
    
    /***************************************************************************************/
    // loadTextures() - 
    /***************************************************************************************/
    // Checks all material textures of a given type and loads the textures if they're not loaded yet.
    // The required info is returned as a Texture struct.
    void loadTextures(const aiMaterial * material, aiTextureType type, const std::string & typeName, const std::string & path) {
      
      if(material->GetTextureCount(type) > 0) {
        
        aiString filename;
        
        //NOTE: prendo solo la prima
        material->GetTexture(type, 0, &filename);
        
        textures.push_back(mpl::glTexture2D(typeName, filename.C_Str(), path));
        
        if(typeName == "diffuseTexture")      haveDiffuseTexture      = true;
        if(typeName == "specularTexture")     haveSpecularTexture     = true;
        if(typeName == "ambientTexture")      haveAmbientTexture      = true;
        if(typeName == "emissimeTexture")     haveEmissimeTexture     = true;
        if(typeName == "heightTexture")       haveHeightTexture       = true;
        if(typeName == "normalsTexture")      haveNormalsTexture      = true;
        if(typeName == "shininessTexture")    haveShininessTexture    = true;
        if(typeName == "opacityTexture")      haveOpacityTexture      = true;
        if(typeName == "displactemenTexture") haveDisplactemenTexture = true;
        if(typeName == "lightTexture")        haveLightTexture        = true;
        if(typeName == "reflectionTexture")   haveReflectionTexture   = true;
        
      }
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_OPENGL_GLMATERIAL_H_ */
