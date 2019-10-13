/*
 * MIT License
 *
 * Copyright © 2019
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

#include <cstdio>
#include <cstdlib>

#include <mpl/opengl/opengl.hpp>


/*****************************************************************************/
// main
/*****************************************************************************/
int main(int argc, char * const argv []) {

#if(0)
  
  mpl::glWindow window;
    
  window.create(800, 600, "ModelView");

  window.makeContextCurrent();
    
  while(!window.shouldClose()) {
      
    window.renderBegin();
      
    float ratio = window.width / (float) window.height;
    glViewport(0, 0, window.width, window.height);
      
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef((float) glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
    glBegin(GL_TRIANGLES);
    glColor3f(1.f, 0.f, 0.f);
    glVertex3f(-0.6f, -0.4f, 0.f);
    glColor3f(0.f, 1.f, 0.f);
    glVertex3f(0.6f, -0.4f, 0.f);
    glColor3f(0.f, 0.f, 1.f);
    glVertex3f(0.f, 0.6f, 0.f);
    glEnd();
            
    window.renderEnd();
      
  }

#endif
  
  
#if(0)
  
  mpl::glWindow window;

  window.create(800, 600, "ModelView");

  window.makeContextCurrent();
  
  mpl::glAxes axes; axes.init();
  mpl::glSphere sphere; sphere.init(1.0, 10, 10, mpl::glSphere::WIREFRAME_SPHERE, glm::vec3(0.0,0.0,1.0));
  mpl::glGrid grid; grid.init(10, glm::vec3(0.0,1.0,1.0));
  mpl::glLine line; line.init({glm::vec3(0.0,1.0,1.0),glm::vec3(1.0,1.0,1.0)}, glm::vec3(1.0,0.0,1.0));
  mpl::glCube cube; cube.init(0.01, mpl::glCube::SOLID_CUBE, glm::vec3(1.0,0.0,0.0));

  while(!window.shouldClose()) {
    
    window.renderBegin();
  
    axes.render(window.getProjection(), window.getView());

    sphere.render(window.getProjection(), window.getView());

    grid.render(window.getProjection(), window.getView());

    line.render(window.getProjection(), window.getView());

    cube.render(window.getProjection(), window.getView());

    window.renderEnd();
      
  }

#endif
  
#if(1)
  
  mpl::glWindow window;

  window.create(800, 600, "ModelView");

  window.makeContextCurrent();
  
  mpl::glModel model("/Users/thewoz/Research/MPL/include/opengl/model/Trex/Trex.fbx");

  model.setLight(glm::vec3(1.0), glm::vec3(-1.0));
    
  window.addCamera(45.0, 0.1, 10.0, glm::vec3(1.1, 1.3, 1.4), mpl::glCamera::MODE::TARGET, glm::vec3(0.01, 0.01, 0.01));

  window.changeCamera();
    
  mpl::glAxes axes; axes.init();

  uint32_t frame = 0;

  while(!window.shouldClose()) {
        
    window.renderBegin();
    
    axes.render(window.getProjection(), window.getView());
    
    model.rotate(glm::quat(glm::vec3(glm::sin(glm::radians((float)frame)),
                                     glm::sin(glm::radians((float)frame)),
                                     glm::cos(glm::radians((float)frame)))));

    model.render(window.getProjection(), window.getView());
    
    window.renderEnd();
    
    ++frame;
      
  }
      
#endif

  return 0;
  
}
