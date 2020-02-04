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

#ifndef _H_MPL_GLMODEL_H_
#define _H_MPL_GLMODEL_H_

#include <cstdio>
#include <cstdlib>

#include <vector>
#include <string>

#include <glm/glm.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>

#include <mpl/debug.hpp>
#include <mpl/stdio.hpp>

#include "glMesh.hpp"
#include "glShader.hpp"
#include "glLight.hpp"

#include "glObject.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /***************************************************************************************/
  // glModel
  /***************************************************************************************/
  class glModel : public glObject {
    
  private:
    
    std::vector<glMesh> meshes;
  
    mpl::glLight light;

  public:
    
    /***************************************************************************************/
    // glModel() - Empty constructor
    /***************************************************************************************/
    glModel() : glObject() { }
    
    /***************************************************************************************/
    // glModel() - Constructor, expects a filepath to a 3D model
    /***************************************************************************************/
    glModel(const std::string & path, GLfloat sizeFactor = 1.0) : glObject() { init(path, sizeFactor); }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(std::string path, GLfloat sizeFactor = 1.0f) {
                
      name = mpl::io::name(path);
      
      DEBUG_LOG("glModel::init(" + name + ")");

      glObject::initModel();

      mpl::io::expandPath(path);

      // ASSIMP reader file
      Assimp::Importer importer;
      
      // Load the model via ASSIMP
      const aiScene *scene = importer.ReadFile(path, aiProcess_CalcTangentSpace
                                               | aiProcess_Triangulate
                                               | aiProcess_JoinIdenticalVertices
                                               | aiProcess_FlipUVs
                                               );
      
      // Check for errors - if is Not Zero
      if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        fprintf(stderr, "ERROR::ASSIMP:: %s\n", importer.GetErrorString());
        abort();
      }
      
      // Retrieve the directory path of the filepath
      std::string directory = path.substr(0, path.find_last_of('/'));
      
      // Process ASSIMP's root node recursively
      processNode(scene->mRootNode, scene, directory);
      
      normalize(sizeFactor, false);
      
      glm::vec3 modelCenter, modelSizeVec;
      
      float modelRadius;
      
      //TODO: ci sta un problema su come mi calcolo il centro
      getBounds(modelCenter, modelSizeVec, modelRadius);
            
      isInited = true;
      
    }
    
    /***************************************************************************************/
    // setLight() - Set the light
    /***************************************************************************************/
    void setLight(const glm::vec3 & _position, const glm::vec3 & _direction) {
      
      light.setPosition(_position);
      light.setDirection(_direction);

    }
    
    /***************************************************************************************/
    // render() - Render the model, and thus all its meshes
    /***************************************************************************************/
    void render(const glm::mat4 & projection, const glm::mat4 & view, bool withMaterials = true) {
            
      DEBUG_LOG("glModel::render(" + name + ")");

      renderBegin(projection, view);
      
      for(std::size_t i=0; i<meshes.size(); ++i) {
        meshes[i].render(shader, withMaterials);
      }
      
      renderEnd();
      
    }
    
    /***************************************************************************************/
    // render() - Render the model, and thus all its meshes
    /***************************************************************************************/
    void renderBegin(const glm::mat4 & projection, const glm::mat4 & view) {
      
      glObject::renderBegin(projection, view);
     
      light.setInShader(shader, view);
      
      shader.setUniform("withShadow", false);
      
      glEnable(GL_CULL_FACE);
           
      glCullFace(GL_BACK);
      
    }
      
    /***************************************************************************************/
    // render() - Render the model, and thus all its meshes
    /***************************************************************************************/
    void renderEnd() {
         
      glDisable(GL_CULL_FACE);

      glObject::renderEnd();
         
    }
    
    /***************************************************************************************/
    // getBounds() - Compute the bounds of the model (center, size, radius)
    /***************************************************************************************/
    void getBounds(glm::vec3 & _center, glm::vec3 & size, float & radius) const {
      
      _center = glm::vec3(0.0f);
      size    = glm::vec3(0.0f);
      radius  = 0;
      
      int counter = 0;
      
      for(std::size_t i=0; i<meshes.size(); ++i){
        
        glm::vec3 tmp_center; glm::vec3 tmp_size; float tmp_radius;
        
        meshes[i].bounds(tmp_center, tmp_size, tmp_radius);
        
        _center += tmp_center;
        
        if(tmp_radius > radius) radius = tmp_radius;
        
        if(tmp_size.x > size.x) size.x = tmp_size.x;
        if(tmp_size.y > size.y) size.y = tmp_size.y;
        if(tmp_size.z > size.z) size.z = tmp_size.z;
        
        ++counter;
        
      }
      
      _center /= (double) counter;
      
    }
    
    /***************************************************************************************/
    // getRadius() - Compute the radius of the model
    /***************************************************************************************/
    float getRadius() const {
         
      glm::vec3 center; glm::vec3 size; float radius;
      
      getBounds(center, size, radius);
      
      return radius;
      
    }
    
    /***************************************************************************************/
    // setInGpu() - Copy the model into the GPU
    /***************************************************************************************/
    void setInGpu() {
      
      DEBUG_LOG("glModel::setInGpu(" + name + ")");
      
      _setInGpu();
      
      for(int i=0; i<meshes.size(); ++i) meshes[i].initInGpu();
      
    }
    
    /***************************************************************************************/
    // size() -
    /***************************************************************************************/
    std::size_t size() const { return meshes.size(); }
    
    /***************************************************************************************/
    // operator [] -
    /***************************************************************************************/
    glMesh & operator [] (std::size_t index) { return meshes[index]; }
    
    
  private:
        
    /***************************************************************************************/
    // normalize() - Normalize and set the center of the model
    /***************************************************************************************/
    void normalize(double scaleTo, bool m_center = false) {
      
      //TODO: controlla bounds
      
      glm::vec3 _center; glm::vec3 size; float radius = 0.0f;
      
      getBounds(_center, size, radius);
            
      double scalingFactor = scaleTo / radius;
      
      glm::vec3 offset = glm::vec3(0.0f);
      
      if(m_center) offset = -_center;
      
      for(std::size_t i=0; i<meshes.size(); ++i)
        meshes[i].scale(scalingFactor, offset);
      
//    getBounds(center, size, radius);
      
    }
    
    /***************************************************************************************/
    // processNode() - Processes a node in a recursive fashion. Processes each individual mesh
    //                 located at the node and repeats this process on its children nodes (if any)
    /***************************************************************************************/
    void processNode(const aiNode * node, const aiScene * scene, const std::string & path) {
      
      //printf("%s\n", node->mName.C_Str());
      
      // Process each mesh located at the current node
      for(GLuint i=0; i<node->mNumMeshes; ++i) {
        
        // The node object only contains indices to index the actual objects in the scene.
        // The scene contains all the data, node is just to keep stuff organized (like relations between nodes).
        aiMesh * mesh = scene->mMeshes[node->mMeshes[i]];
        
        meshes.push_back(glMesh(mesh, scene, path));
        
      }
      
      // After we've processed all of the meshes (if any) we then recursively process each of the children nodes
      for(GLuint i=0; i<node->mNumChildren; ++i) {
        processNode(node->mChildren[i], scene, path);
      }
      
    }
    
  };
  
} /* namespace mpl */

#endif /* _H_MPL_GLMODEL_H_ */
