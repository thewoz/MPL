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

#include "glMesh.hpp"
#include "glShader.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /***************************************************************************************/
  // glModel
  /***************************************************************************************/
  class glModel {
    
  private:
    
    // Model Meshes
    std::vector<glMesh> meshes;
    
    // Model center
    glm::vec3 center;
    
    // Model rotation angles
    glm::quat angles;
    
  public:
    
    /***************************************************************************************/
    // glModel() - Constructor, expects a filepath to a 3D model
    /***************************************************************************************/
    glModel(const std::string & path) { load(path); }
    
    /***************************************************************************************/
    // glModel() - Empty constructor
    /***************************************************************************************/
    glModel() { }
    
    /***************************************************************************************/
    // render() - Render the model, and thus all its meshes
    /***************************************************************************************/
    void render(const glShader & shader, bool withMaterials = true) const {
      for(std::size_t i=0; i<meshes.size(); ++i) meshes[i].render(shader, withMaterials);
    }
    
    /*****************************************************************************/
    // getModelMatrix() -
    /*****************************************************************************/
    glm::mat4 getModelMatrix() const {
      
      glm::mat4 tm = glm::translate(glm::mat4(), center);
      
      glm::mat4 sm = glm::mat4();
      
// NOTE: i quat in glm applica le rotazioni in ZYX
//      glm::vec3 tmp = glm::eulerAngles(angles);
//      glm::mat4 rm = glm::mat4();
//      rm = glm::rotate(rm, tmp.z, glm::vec3(0.0, 0.0, 1.0));
//      rm = glm::rotate(rm, tmp.y, glm::vec3(0.0, 1.0, 0.0));
//      rm = glm::rotate(rm, tmp.x, glm::vec3(1.0, 0.0, 0.0));

// NOTE: mat4_cast in glm applica le rotazioni in ZYX
      glm::mat4 rm = glm::mat4_cast(angles);

      glm::mat4 model = tm * rm * sm;
      
      return model;
      
    }
    
    /*****************************************************************************/
    //  Set translation and rotation of the model
    /*****************************************************************************/
    void setTranslation(const glm::vec3 _center) { center = _center; }
    void setRotation(const glm::quat _angles) { angles = _angles; }
    
    /*****************************************************************************/
    //  getCenter() -  Get model center
    /*****************************************************************************/
    glm::vec3 getCenter() const { return center; }
    
    /***************************************************************************************/
    // scale() - Scale and center the model
    /***************************************************************************************/
    void scale(float scaleFactor, const glm::vec3 & offset) {
      for(std::size_t i=0; i<meshes.size(); ++i) meshes[i].scale(scaleFactor, offset);
    }
    
    /***************************************************************************************/
    // getBounds() - Compute the bounds of the model (center, size, radius)
    /***************************************************************************************/
    void getBounds(glm::vec3 & center, glm::vec3 & size, double & radius) const {
      
      center = glm::vec3(0.0f);
      size   = glm::vec3(0.0f);
      radius = 0;
      
      for(std::size_t i=0; i<meshes.size(); ++i){
        
        glm::vec3 tmp_center; glm::vec3 tmp_size; double tmp_radius;
        
        meshes[i].bounds(tmp_center, tmp_size, tmp_radius);
        
        center += tmp_center;
        
        if(tmp_radius > radius) radius = tmp_radius;
        
        if(tmp_size.x > size.x) size.x = tmp_size.x;
        if(tmp_size.y > size.y) size.y = tmp_size.y;
        if(tmp_size.z > size.z) size.z = tmp_size.z;
        
      }
      
      center /= (double) meshes.size();
      
    }
    
    /***************************************************************************************/
    // normalize() - Normalize and set the center of the model
    /***************************************************************************************/
    void normalize(double scaleTo, bool m_center = true) {
      
      //TODO: controlla bounds
      
      glm::vec3 center; glm::vec3 size; double radius = 0.0f;
      
      getBounds(center, size, radius);
      
      //printf("center (%f,%f,%f) size (%f,%f,%f) radius %f\n", center.x, center.y, center.z, size.x, size.y, size.z, radius);
      
      double scalingFactor = scaleTo / radius;
      
      glm::vec3 offset = glm::vec3(0.0f);
      
      if(m_center) offset = -center;
      
      scale(scalingFactor, offset);
      
      getBounds(center, size, radius);
      
      ///printf("center (%f,%f,%f) size (%f,%f,%f) radius %f\n", center.x, center.y, center.z, size.x, size.y, size.z, radius);
      
    }
    
    /***************************************************************************************/
    // load() - Load the 3D Model
    /***************************************************************************************/
    void load(std::string path) {
      
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
            
    }
    
    /***************************************************************************************/
    // setInGpu() - Copy the model into the GPU
    /***************************************************************************************/
    void setInGpu() { for(int i=0; i<meshes.size(); ++i) meshes[i].setInGpu(); }
    
  private:
    
    /***************************************************************************************/
    // processNode() - Processes a node in a recursive fashion. Processes each individual mesh
    //                 located at the node and repeats this process on its children nodes (if any)
    /***************************************************************************************/
    void processNode(const aiNode * node, const aiScene * scene, const std::string & path) {
      
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
