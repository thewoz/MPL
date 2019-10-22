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

#include <mpl/stdio.hpp>

#include "glMesh.hpp"
#include "glShader.hpp"
#include "glLight.hpp"


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
        
    // Shader user for rendering
    glShader shader;
    
    mpl::glLight light;
      
    glm::mat4 model;

    GLfloat scale;

    bool isInited;
    bool isInitedInGpu;

  public:
    
    /***************************************************************************************/
    // glModel() - Constructor, expects a filepath to a 3D model
    /***************************************************************************************/
    glModel(std::string path, GLfloat sizeFactor = 1.0f) : isInitedInGpu(false)  { init(path, sizeFactor); }
    
    /***************************************************************************************/
    // glModel() - Empty constructor
    /***************************************************************************************/
    glModel() : isInited(false), isInitedInGpu(false) { }
    
    /*****************************************************************************/
    // init
    /*****************************************************************************/
    void init(std::string path, GLfloat sizeFactor = 1.0f) {
                
      shader.init("~/Research/MPL/include/opengl/shader/model.vs", "~/Research/MPL/include/opengl/shader/model.frag");

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
      
      //initInGpu();
            
      center = glm::vec3(0.0);
      
      angles = glm::vec3(0.0);
      
      scale = 1.0;
      
      updateModelMatrix();
      
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
      
      if(!isInited){
        fprintf(stderr, "model must be inited before render\n");
        abort();
      }
      
      if(!isInitedInGpu){
        fprintf(stderr, "model must be inited in gpu before render\n");
        abort();
      }
      
      glEnable(GL_DEPTH_TEST);
       
      glEnable(GL_CULL_FACE);
       
      glCullFace(GL_BACK);
      
      shader.use();
           
      light.setInShader(shader, view);
      
      shader.setUniform("projection", projection);
      shader.setUniform("view", view);
      shader.setUniform("model", model);
      shader.setUniform("withShadow", false);
      
    }
      
    /***************************************************************************************/
    // render() - Render the model, and thus all its meshes
    /***************************************************************************************/
    void renderEnd() {
         
      glDisable(GL_DEPTH_TEST);
            
      glDisable(GL_CULL_FACE);
         
    }
    
    /*****************************************************************************/
    // getShader() -
    /*****************************************************************************/
    const glShader & getShader() const { return shader; }
    
    /*****************************************************************************/
    // getModelMatrix() -
    /*****************************************************************************/
    glm::mat4 getModelMatrix() const { return model; }
    
    /*****************************************************************************/
    // Position fuction
    /*****************************************************************************/
    void translate(const glm::vec3 & _center) { center = _center; updateModelMatrix(); }
    void rotate(const glm::quat & _angles) { angles = _angles; updateModelMatrix(); }
    void move(const glm::vec3 & _angles, const glm::vec3 & _center) { angles = _angles; center = _center; updateModelMatrix(); }
    
    /*****************************************************************************/
    //  
    /*****************************************************************************/
    glm::vec3 getTranslation() const { return center; }
    glm::quat getRotation()    const { return angles; }
    
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
    // initInGpu() - Copy the model into the GPU
    /***************************************************************************************/
    void initInGpu() {
      
      if(!isInited){
         fprintf(stderr, "model must be inited before set in GPU\n");
         abort();
       }
      
      shader.initInGpu();
      
      for(int i=0; i<meshes.size(); ++i) meshes[i].initInGpu();
      
      isInitedInGpu = true;
      
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
        
    /*****************************************************************************/
    // updateModelMatrix
    /*****************************************************************************/
    void updateModelMatrix() {
      
      model = glm::mat4(1.0f);
      
      model = glm::translate(model, center); // Translate it down a bit so it's at the center of the scene
      
      model = glm::scale(model, glm::vec3(scale));

      model  = glm::rotate(model, angles.x, glm::vec3(1, 0, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      model  = glm::rotate(model, angles.y, glm::vec3(0, 1, 0)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      model  = glm::rotate(model, angles.z, glm::vec3(0, 0, 1)); // where x, y, z is axis of rotation (e.g. 0 1 0)
      
    }
    
    /***************************************************************************************/
    // normalize() - Normalize and set the center of the model
    /***************************************************************************************/
    void normalize(double scaleTo, bool m_center = false) {
      
      //TODO: controlla bounds
      
      glm::vec3 _center; glm::vec3 size; float radius = 0.0f;
      
      getBounds(_center, size, radius);
      
//    printf("center (%f,%f,%f) size (%f,%f,%f) radius %f\n", center.x, center.y, center.z, size.x, size.y, size.z, radius);
      
      double scalingFactor = scaleTo / radius;
      
      glm::vec3 offset = glm::vec3(0.0f);
      
      if(m_center) offset = -_center;
      
      //scale(scalingFactor, offset);
      for(std::size_t i=0; i<meshes.size(); ++i)
        meshes[i].scale(scalingFactor, offset);
      
//    getBounds(center, size, radius);

//    printf("center (%f,%f,%f) size (%f,%f,%f) radius %f\n", center.x, center.y, center.z, size.x, size.y, size.z, radius);
      
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
