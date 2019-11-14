/*
 * MIT License
 *
 * Copyright © 2017 COBBS
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

#ifndef _H_MPL_GNUPLOT_H_
#define _H_MPL_GNUPLOT_H_

#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>

/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {

  /*****************************************************************************/
  // class gnuplot
  /*****************************************************************************/
  class gnuplot {
    
  public:
    
    /*****************************************************************************/
    // gnuplot
    /*****************************************************************************/
    inline gnuplot(bool persist = true) {
      
      std::cout << "Opening gnuplot... ";
      
      pipe = popen(persist ? "gnuplot -persist" : "gnuplot", "w");
      
      if(!pipe)
        std::cout << "failed!" << std::endl;
      else
        std::cout << "succeded." << std::endl;
    }
    
    /*****************************************************************************/
    // ~gnuplot
    /*****************************************************************************/
    inline virtual ~gnuplot(){
      if(pipe) pclose(pipe);
    }
    
    /*****************************************************************************/
    // sendLine() -
    /*****************************************************************************/
    void sendLine(const std::string& text, bool useBuffer = false){
      if (!pipe) return;
      if (useBuffer)
        buffer.push_back(text + "\n");
      else
        fputs((text + "\n").c_str(), pipe);
    }
    
    /*****************************************************************************/
    // sendEndOfData() -
    /*****************************************************************************/
    void sendEndOfData(unsigned repeatBuffer = 1){
      if (!pipe) return;
      for (unsigned i = 0; i < repeatBuffer; i++) {
        for (auto& line : buffer) fputs(line.c_str(), pipe);
        fputs("e\n", pipe);
      }
      fflush(pipe);
      buffer.clear();
    }
    
    /*****************************************************************************/
    // sendNewDataBlock() -
    /*****************************************************************************/
    void sendNewDataBlock(){
      sendLine("\n", !buffer.empty());
    }
    
    /*****************************************************************************/
    // writeBufferToFile() -
    /*****************************************************************************/
    void writeBufferToFile(const std::string& fileName){
      std::ofstream fileOut(fileName);
      for (auto& line : buffer) fileOut << line;
      fileOut.close();
    }
    
  private:
    
    // Uccido l'operatore copia
    void operator = (gnuplot const & ) = delete;
    
    // pipe per comunicare con gnuplot
    FILE * pipe;
    
    // buffer dei comandi
    std::vector<std::string> buffer;
    
  };
  
} /* namespace mpl::math */


#endif /* _H_MPL_GNUPLOT_H_ */
