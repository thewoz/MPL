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

#ifndef _H_MPL_GLWINDOW_H_
#define _H_MPL_GLWINDOW_H_

#include <cstdlib>
#include <cstdio>

#include <vector>

#include "glfw.h"
#include "tiff.h"

#include "glCamera.hpp"

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // glWindow
  /*****************************************************************************/
  class glWindow {
    
  private:

    // Id univoco della finestra
    unsigned int id;
    
    // Indice della camera corrente
    unsigned int currentCameraIndex;
    
    // Puntatore della finestra GLFW
    GLFWwindow * window;
    
    // Ultima posizione del mouse nella finestra
    GLfloat lastX; GLfloat lastY;
    
    // Variabile vera ogni volta che il mouse entra per la prima volta nella finestra
    bool firstMouse;
    
    // Variabile vera se il mouse e' sopra la finestra
    bool onFocus;
    
    // Vettore delle telecamere attive nella finestra
    std::vector<mpl::glCamera> cameras;
        
    // Tempo dell'ultimo rendering
    GLfloat lastTime;
    
    // Memoria di appoggio per salvare il contenuto della finestra
    GLubyte * image;

    // Background color
    glm::vec3 background;
    
    bool inputDisable;
    
  public:
    
    GLint width;
    GLint height;
    
  protected:
    
    // Contatore del numero di finestre create
    static unsigned int windowsCounter;
    
    // Camera corrente
    mpl::glCamera * currentCamera;
    
    bool keys[1024] = {false, };

    // Tempo passato dall'ultima volta che e' stato effettuato il rendering
    GLfloat deltaTime;
    
    //TODO:
    // mettere un controllo su offscreen
    
  public:
    
    /*****************************************************************************/
    // glWindow() - Costruttore vuoto
    /*****************************************************************************/
    glWindow() { window = NULL; image = NULL; }
    
    /*****************************************************************************/
    // ~glWindow() - Distruttore
    /*****************************************************************************/
    ~glWindow() {
      if(window != NULL) { glfwDestroyWindow(window); window = NULL; --windowsCounter; }
      if(image  != NULL) { free(image); image = NULL; }
      if(windowsCounter == 0) { glfw::close(); }
    }
    
    /*****************************************************************************/
    // create() - Crea una nuova finestra
    /*****************************************************************************/
    void create(GLint _width, GLint _height, const char * title = "OpenGL window") {
      
      glfw::init();
            
      // Create a GLFWwindow object that we can use for GLFW's functions
      window = glfwCreateWindow(_width, _width, title, NULL, NULL);
      
      if(window == NULL) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        abort();
      }
      
      glfwMakeContextCurrent(window);
      
      glfwGetFramebufferSize(window, &width, &height);
      
#ifdef GLFW_WITH_GLAD
//      if(!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
//        //std::cout << "Failed to initialize OpenGL context" << std::endl;
//        abort();
//      }
#endif
      
#ifdef GLFW_WITH_GLEW
      // Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
      glewExperimental = GL_TRUE;
      
      // Initialize GLEW to setup the OpenGL Function pointers
      if(GLEW_OK != glewInit( )) {
        //std::cout << "Failed to initialize GLEW" << std::endl;
        abort();
      }
#endif
      
      glfwSetWindowUserPointer(window, this);

      glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
      glfwSetWindowCloseCallback(window, windowCloseCallback);
      glfwSetCursorPosCallback(window, cursorPosCallback);
      glfwSetMouseButtonCallback(window, mouseButtonCallback);
      glfwSetScrollCallback(window, scrollCallback);
      glfwSetKeyCallback(window, keyCallback);
      glfwSetCursorEnterCallback(window, cursorEnterCallback);

      glfwSwapInterval(1);
      
      firstMouse = true;

      onFocus = false;

      id = windowsCounter++;

      background = glm::vec3(0.0f, 0.1f, 0.2f);

      cameras.push_back(glCamera(width, height));

      currentCameraIndex = 0;

      currentCamera = &cameras[currentCameraIndex];

      inputDisable = false;
      
    }
    
    /*****************************************************************************/
    // createOffscreen() - Crea una nuova finestra
    /*****************************************************************************/
    void createOffscreen(GLint _width, GLint _height) {
      
      glfw::init();
      
      glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
      
      // Create a GLFWwindow object that we can use for GLFW's functions
      window = glfwCreateWindow(_width, _height, "notitle", NULL, NULL);
      
      if(window == NULL) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        abort();
      }
      
      glfwMakeContextCurrent(window);
      
      glfwGetFramebufferSize(window, &width, &height);
      
#ifdef GLFW_WITH_GLAD
      if(!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        //std::cout << "Failed to initialize OpenGL context" << std::endl;
        abort();
      }
#endif
      
#ifdef GLFW_WITH_GLEW
      // Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
      glewExperimental = GL_TRUE;
      
      // Initialize GLEW to setup the OpenGL Function pointers
      if(GLEW_OK != glewInit( )) {
        //std::cout << "Failed to initialize GLEW" << std::endl;
        abort();
      }
#endif
     
//      glfwSetWindowUserPointer(window, this);
      
//      glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
//      glfwSetWindowCloseCallback(window, windowCloseCallback);
//      glfwSetCursorPosCallback(window, cursorPosCallback);
//      glfwSetMouseButtonCallback(window, mouseButtonCallback);
//      glfwSetScrollCallback(window, scrollCallback);
//      glfwSetKeyCallback(window, keyCallback);
//      glfwSetCursorEnterCallback(window, cursorEnterCallback);

      glfwSwapInterval(1);
      
      firstMouse = true;
      
      onFocus = false;
      
      id = windowsCounter++;
      
      background = glm::vec3(0.0f, 0.1f, 0.2f);
      
      cameras.push_back(glCamera(width, height));
      
      currentCameraIndex = 0;
      
      currentCamera = &cameras[currentCameraIndex];
      
      inputDisable = false;
      
    }
    
    
    /*****************************************************************************/
    // destroy() -
    /*****************************************************************************/
    void destroy() {
      if(window != NULL) { glfwDestroyWindow(window); window = NULL; --windowsCounter; }
    }
    
    //****************************************************************************//
    // updateCurrentCamera() -
    //****************************************************************************//
    void updateCurrentCamera(float fov, float zNear, float zFar, glm::vec3 position = glm::vec3(0.0f), glCamera::MODE mode = glCamera::MODE::FREE, glm::vec3 target = glm::vec3(0.0f)) {
      
      currentCamera->init(currentCamera->getWidth(), currentCamera->getHeight(), fov, zNear, zFar, position, mode, target);
      
    }
    
  private:
    
    //****************************************************************************//
    // CallBack GLFW Wrapper
    //****************************************************************************//
    static inline void windowCloseCallback(GLFWwindow* window) {
      ((glWindow*)glfwGetWindowUserPointer(window))->windowCloseCallback();
    }
    
    static inline void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
      ((glWindow*)glfwGetWindowUserPointer(window))->framebufferSizeCallback(width, height);
    }
    
    static inline void cursorPosCallback(GLFWwindow* window, double xPos, double yPos) {
      ((glWindow*)glfwGetWindowUserPointer(window))->cursorPosCallback(xPos, yPos);
    }
    
    static inline void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
      ((glWindow*)glfwGetWindowUserPointer(window))->mouseButtonCallback(button, action, mods);
    }
    
    static inline void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
      ((glWindow*)glfwGetWindowUserPointer(window))->scrollCallback(xoffset, yoffset);
    }
    
    static inline void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
      ((glWindow*)glfwGetWindowUserPointer(window))->keyCallback(key, scancode, action, mods);
    }
    
    static inline void  cursorEnterCallback(GLFWwindow* window, int entered) {
      ((glWindow*)glfwGetWindowUserPointer(window))->cursorEnter(entered);
    }
    
    //****************************************************************************//
    // External callback interfaces
    //****************************************************************************//
    inline void virtual scroll(double xoffset, double yoffset) { };
    inline void virtual keyboard(int key, int scancode, int action, int mods) { };
    inline void virtual cursorPos(double xPos, double yPos, double xoffset, double yoffset) { };
    inline void virtual mouseButton(int button, int action, int mods) { };
    inline void virtual cursorEnter(int entered) {
      if(entered) { onFocus = true; /*firstMouse = true;*/ } else { onFocus = false; }
    }
    
    
    //****************************************************************************//
    // Callback
    //****************************************************************************//
    inline void windowCloseCallback() {
      glfwSetWindowShouldClose(window, GL_TRUE);
    }
    
    inline void framebufferSizeCallback(int width, int height) {
      for(std::size_t i=0; i<cameras.size(); ++i)
        cameras[i].setSensorSize(width, height);
    }
    
    inline void scrollCallback(double xoffset, double yoffset) { scroll(xoffset, yoffset); }
    inline void keyCallback(int key, int scancode, int action, int mods) {
      
      if(GLFW_KEY_ESCAPE == key && GLFW_PRESS == action) {
        setShouldClose(GL_TRUE);
      }
      
      if(key >= 0 && key < 1024) {
        
        if(action == GLFW_PRESS) {
          keys[key] = true;
        } else if (action == GLFW_RELEASE) {
          keys[key] = false;
        }
        
        if(currentCamera->getMode() != glCamera::MODE::TARGET) {
        
          //Moves/alters the camera positions based on user input
          if(keys[GLFW_KEY_W] || keys[GLFW_KEY_UP])    currentCamera->processKeyboard(glCamera::MOVEMENT::FORWARD, deltaTime);
          if(keys[GLFW_KEY_S] || keys[GLFW_KEY_DOWN])  currentCamera->processKeyboard(glCamera::MOVEMENT::BACKWARD, deltaTime);
          if(keys[GLFW_KEY_A] || keys[GLFW_KEY_LEFT])  currentCamera->processKeyboard(glCamera::MOVEMENT::LEFT, deltaTime);
          if(keys[GLFW_KEY_D] || keys[GLFW_KEY_RIGHT]) currentCamera->processKeyboard(glCamera::MOVEMENT::RIGHT, deltaTime);
          
        }
        
        if(cameras.size() > 1) if(keys[GLFW_KEY_C]) { changeCamera(); }
        
        keyboard(key, scancode, action, mods);

      }
      
    }
    
    inline void cursorPosCallback(double xPos, double yPos){
      //    if(cursorPos(xpos, ypos)){
      //      //cameraMouseCallback(buttonMouse, xpos - oldXpos, ypos - oldYpos);
      //      oldXpos = xpos; oldYpos = ypos;
      //    }
      
      if(onFocus) {
      
        if(firstMouse) {
          lastX = xPos;
          lastY = yPos;
          firstMouse = false;
        }
        
        GLfloat xOffset;
        GLfloat yOffset;
        
        xOffset = xPos - lastX;
        yOffset = lastY - yPos;  // Reversed since y-coordinates go from bottom to left
        
        lastX = xPos;
        lastY = yPos;
        
        currentCamera->processMouseMovement(xOffset, yOffset);
        
        cursorPos(lastX, lastY, xOffset, yOffset);
      
      }
        
    }
    
    inline void mouseButtonCallback(int button, int action, int mods) {
      mouseButton(button, action, mods);
      //    if(action == GLFW_PRESS){
      //      buttonMouse = button;
      //      if(buttonMouse==GLFW_MOUSE_BUTTON_1 && mods==GLFW_MOD_CONTROL) buttonMouse = GLFW_MOUSE_BUTTON_2;
      //      else if(button==GLFW_MOUSE_BUTTON_1 && mods==GLFW_MOD_SHIFT)   buttonMouse = GLFW_MOUSE_BUTTON_3;
      //    } else if(action == GLFW_RELEASE){ buttonMouse = -1; }
    }
    
    
    
  public:
    
    //****************************************************************************//
    // Window Interface funtions
    //****************************************************************************//
    inline void setPosition(int xPos, int yPos){
      glfwSetWindowPos(window, xPos, yPos);
    }
    
    inline void setTitle(const char * title){
      glfwSetWindowTitle(window, title);
    }
    
    inline bool shouldClose(){
      return glfwWindowShouldClose(window);
    }
    
    inline void setShouldClose(bool mode){
      glfwSetWindowShouldClose(window, mode);
    }
    
    inline void setCursorInputMode(int mode) {
      glfwSetInputMode(window, GLFW_CURSOR, mode);
    }
    
    inline void getCursorPos(double & x, double & y){
      glfwGetCursorPos(window, &x, &y);
    }
    
    inline void setSize(int width, int height){
      glfwSetWindowSize(window, width, height);
      for(std::size_t i=0; i<cameras.size(); ++i)
        cameras[i].setSensorSize(width, height);
    }
    
    inline void setBackgournd(const glm::vec3 & _background){
      background = _background;
    }
    
    inline void disableInput() { inputDisable = true; }
    inline void enableInput() { inputDisable = false; }

    inline void hide() { glfwHideWindow(window); }
    inline void show() { glfwShowWindow(window); }
    inline void iconify() { glfwIconifyWindow(window); }

    //****************************************************************************//
    // Window Camera Interface funtions
    //****************************************************************************//
    inline void addCamera(float fov, float zNear, float zFar, glm::vec3 position = glm::vec3(0.0f), glCamera::MODE mode = glCamera::FREE, glm::vec3 target = glm::vec3(0.0f)) {
      cameras.push_back(glCamera(currentCamera->getWidth(), currentCamera->getHeight(), fov, zNear, zFar, position, mode, target));
      currentCamera = &cameras[currentCameraIndex];
    }
    
    /*****************************************************************************/
    // get projection and view matrix
    /*****************************************************************************/
    inline glm::mat4 getProjection() const { return currentCamera->getProjection(); }
    inline glm::mat4 getView()       const { return currentCamera->getView();       }

    inline void  makeContextCurrent() {
      
       glfwMakeContextCurrent(window);
      
    }
    
    /*****************************************************************************/
    // renderBegin
    /*****************************************************************************/
    void renderBegin() {
      
      if(window == NULL) {
        fprintf(stderr, "glWindow error: the window is not initialized\n");
        abort();
      }
      
      glfwMakeContextCurrent(window);
      
      GLfloat currentTime = glfwGetTime();
      
      deltaTime = currentTime - lastTime;
      lastTime  = currentTime;
      
      //if(id==0 && !inputDisable) glfwPollEvents();
      
      glClearColor(background.r, background.g, background.b, 1.0f);

      glViewport(0, 0, currentCamera->getWidth(), currentCamera->getHeight());
      
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      
    }
    
    /*****************************************************************************/
    // renderEnd
    /*****************************************************************************/
    inline void renderEnd() {

      glfwSwapBuffers(window);
      
      if(id==0 && !inputDisable) glfwPollEvents();
      
    }

    /*****************************************************************************/
    // snapshot
    /*****************************************************************************/
    void snapshot(const char * filepath) {
      
      static int snapshotNum = 0;
      
      char filename[1024];
      
      sprintf(filename, "%s/snapshot%04d.tiff", filepath, ++snapshotNum);
      
      glReadBuffer(GL_BACK);
      
      tiff::snapshot(currentCamera->getWidth(), currentCamera->getHeight(), filename);
      
    }
    
    /*****************************************************************************/
    // pixelsValue
    /*****************************************************************************/
    void pixelsValue(glm::uvec3 & rbg) {
      
      glReadBuffer(GL_BACK);

      double pixelsNum = currentCamera->getWidth() * currentCamera->getHeight();

      // creo un immagine grande quanto la finestra
      image = (GLubyte *) realloc(image, pixelsNum * sizeof(GLubyte) * 3);
      
      glPixelStorei(GL_PACK_ALIGNMENT, 1);
      
      glReadPixels(0, 0, currentCamera->getWidth(), currentCamera->getHeight(), GL_RGB, GL_UNSIGNED_BYTE, image);
      
      rbg = glm::uvec3(0);
      
      int counter = 0;
      
      for(int p=0; p<pixelsNum; ++p){
        
       // salto i pixels neri
       if(image[(p*3)] == 0 && image[(p*3)+1] == 0 && image[(p*3)+2] == 0) continue;

        rbg.r += image[(p*3)];
        rbg.g += image[(p*3)+1];
        rbg.b += image[(p*3)+2];
        
        counter++;
        
      }
      
      if(counter != 0) {
      
        rbg.r /= counter;
        rbg.g /= counter;
        rbg.b /= counter;

      }
      
      //return sum / (float)num;
      
    }
    
  protected:
    
    /*****************************************************************************/
    // changeCamera
    /*****************************************************************************/
    void changeCamera() {
      
      if(currentCameraIndex + 1 < cameras.size()) { ++currentCameraIndex; } else { currentCameraIndex = 0; }
      
      currentCamera = &cameras[currentCameraIndex];

    }
    
  };
  
  unsigned int glWindow::windowsCounter = 0;
  
} /* namespace mpl */

#endif /* _H_MPL_GLWINDOW_H_ */


//    /*****************************************************************************/
//    // setupProjection
//    /*****************************************************************************/
//    void setProjection(float fov, float zNear, float zFar, std::size_t camera_index) {
//
//      //zNear = _zNear;
//      //zFar  = _zFar;
//
//      //        float focalLenght = FROM_MILLIMETER_TO_ER(1024);
//      //        float pixelSize   = FROM_METER_TO_ER(1.3e-5);
//      //
//      //        projection = glm::frustum( (float)(-width  * 0.5 * pixelSize),
//      //                                   (float)(width   * 0.5 * pixelSize),
//      //                                   (float)(-height * 0.5 * pixelSize),
//      //                                   (float)(height  * 0.5 * pixelSize), focalLenght, zFar);
//
//
//      projection = glm::perspective(glm::radians(fov), width/(float)height, zNear, zFar);
//
//      //printf("%f %f %f\n", fov, zNear, zFar);
//
//    }
