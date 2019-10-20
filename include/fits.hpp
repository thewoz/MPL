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

#ifndef _H_MPL_CVFIT_H_
#define _H_MPL_CVFIT_H_

#include <cstdlib>
#include <cstdio>

#include <string>
#include <array>
#include <map>

#include <opencv2/opencv.hpp>

#include <fitsio.h>


/*****************************************************************************/
// namespace mpl
/*****************************************************************************/
namespace mpl {
  
  /*****************************************************************************/
  // namespace fits
  /*****************************************************************************/
  namespace fits {
    
    /*****************************************************************************/
    // solveField
    /*****************************************************************************/
    bool solveField(const std::string & filename, const std::string & _output = "") {
      
      bool solved = false;
      
      //std::string filepathTmp       = std::regex_replace(filepath, std::regex("\\s+"), "\\\\ ");
      //std::string filepathSolvedTmp = std::regex_replace(filepathSolved, std::regex("\\s+"), "\\\\ ");
      
      std::string output;
      
      // Mi creo il path per l'immagine risolta da Astrometry
      if(_output.empty()) {
        output = mpl::io::dirname(filename) + "/" + mpl::io::name(filename) + "_solved.fit";
      } else { output = _output; }
      
      std::string command = "solve-field --new-fits " + output + " --no-plots --match none --rdls none --corr none --index-xyls none --overwrite --downsample 2 --tweak-order 2  --scale-units degwidth --scale-low 0.1 " + filename + " 2>/dev/null";
            
      // apro la pipe
      FILE * fp = popen(command.c_str(), "r");
      
      // controllo se mi da errore
      if(fp == NULL) {
        fprintf(stderr, "error in open pipe\n");
        abort();
      }
      
      char line[PATH_MAX];
      
      // leggo cosa passa sulla pipe
      while(fgets(line, PATH_MAX, fp) != NULL){
        //printf("%s", line);
        if(strncmp(line, "Field 1: solved", strlen("Field 1: solved")) == 0) {
          solved = true;
          //break;
        }
        
      }
      
      // Chiudo la pipe
      if(pclose(fp) == -1) {
        fprintf(stderr, "error in close pipe\n");
        abort();
      }
      
      return solved;
      
    }
  
  } /* namespace fits */

  
  
  /*****************************************************************************/
  // class cvFits
  /*****************************************************************************/
  class cvFits : public cv::Mat {
    
  private:
    
    // FITS's image size
    int height; int width;
    
    // Path of the FITS file
    std::string filepath;
    
    std::string filepathSolved;
    
    // Mappa dove mi segno le info nell'header del file FITS
    std::map<std::string,std::string> fitInfo;
    
    
    /*****************************************************************************/
    // splitFitsCard
    /*****************************************************************************/
    std::array<std::string,3> splitFitsCard(const std::string & str) {
      
      int mode = 0;
      
      std::array<std::string,3> splited;
      
      for(int i=0; i<str.length(); ++i) {
        
        if(str[i] != '=' && (str[i] != '\'' && mode <2) && str[i] != '/') {
          splited[mode] += str[i];
        } else if((str[i] == '=' && mode <2) || str[i] == '/') { ++mode; }
        
      }
      
      return splited;
      
    }
    
    
    /*****************************************************************************/
    // Read the header of a FITS file
    /*****************************************************************************/
    void readFitsHeader(fitsfile * fptr) {
      
      // Standard string lengths defined in fitsioc.h
      char card[FLEN_CARD];
      
      // Status of the fitio functions
      int status = 0;
      
      // Not used
      int hdutype;
      
      // Attempt to move to next HDU
      for(int i = 1; !(fits_movabs_hdu(fptr, i, &hdutype, &status)); ++i) {
        
        // Number of keywords
        int nkeys;
        // Not used
        int keypos;
        
        // Get number of keywords
        if(fits_get_hdrpos(fptr, &nkeys, &keypos, &status)) {
          fits_report_error(stderr, status);
          abort();
        }
        
        // Print the found keywork
        for(int j = 1; j <= nkeys; ++j) {
          
          if(fits_read_record(fptr, j, card, &status)) {
            fits_report_error(stderr, status);
            abort();
          }
          
          // splitto la card dell'header
          std::array<std::string,3> info = splitFitsCard(card);
          
          // e me la salvo
          fitInfo[info[0]] = info[1];
          
          //printf("%s\n",card);
          
        }
        
      }
      
      // If we got not the expected EOF error
      if(status != END_OF_FILE) {
        fits_report_error(stderr, status);
        abort();
      } else { status = 0; }
      
      long naxes[2];
      int nfound;
      
      /* read the NAXIS1 and NAXIS2 keyword to get image size */
      if(fits_read_keys_lng(fptr, "NAXIS", 1, 2, naxes, &nfound, &status)) {
        fits_report_error(stderr, status);
        abort();
      }
      
      width  = (int)naxes[0];
      height = (int)naxes[1];
      
    }
    
    
    /*****************************************************************************/
    // Read a FITS image
    /*****************************************************************************/
    void readFitsImage() {
      
      // Pointer to the FITS file
      fitsfile * fptr;
      
      // Status of the fits functions
      int status = 0;
      
      // Open the fit file
      if(fits_open_file(&fptr, filepath.c_str(), READONLY, &status)) {
        fits_report_error(stderr, status);
        abort();
      }
      
      // Read the fit header
      readFitsHeader(fptr);
      
      // Alloco lo spazio in opencv
      cv::Mat tmp(height, width, CV_32FC1);
      
      // Data max/min values
      float datamin =  FLT_MAX;
      float datamax = -FLT_MAX;
      
      // Number of the read row of the image
      int row = 0;
      
      // Next pixels to be read in image
      long fpixel = 1;
      
      // Ciclo sulle righe
      while(row < height) {
        
        // Image row pointer
        float * rowPtr = tmp.ptr<float>(height-row-1);
        
        /* Note that even though the FITS images contains unsigned integer */
        /* pixel values (or more accurately, signed integer pixels with    */
        /* a bias of 32768),  this routine is reading the values into a    */
        /* float array.   Cfitsio automatically performs the datatype      */
        /* conversion in cases like this.                                  */
        if(fits_read_img(fptr, TFLOAT, fpixel, width, NULL, rowPtr, NULL, &status) ){
          fits_report_error(stderr, status);
          abort();
        }
        
        // Cerco il massimo e il minimo
        for(int i=0; i<width; ++i) {
          //if(rowPtr[i] < datamin) datamin = rowPtr[i];
          //if(rowPtr[i] > datamax) datamax = rowPtr[i];
          if(rowPtr[i] < datamin && rowPtr[i] !=     0) datamin = rowPtr[i];
          if(rowPtr[i] > datamax && rowPtr[i] != 65535) datamax = rowPtr[i];
        }
        
        // Update the next pixel to be read in image
        fpixel += width;
        
        // Update the row read of the image
        ++row;
        
      }
      
      // Close the fits file
      if(fits_close_file(fptr, &status)) {
        fits_report_error(stderr, status);
        abort();
      } else { fptr = NULL; }
      
      tmp.convertTo(*this, CV_8UC1, 255.0/(datamax-datamin), -255.0*datamin/(datamax-datamin));
      
    }
    
    
  public:
    
    /*****************************************************************************/
    // cvFits
    /*****************************************************************************/
    cvFits() { }
    cvFits(const std::string & _filepath) { load(_filepath); }
    
    /*****************************************************************************/
    // load
    /*****************************************************************************/
    cv::Mat load(const std::string & _filepath) {
      
      // Mi segno il path dell'immagine per solveField()
      filepath = _filepath;
      
      // Carico l'immagine e la converto in cv::Mat
      readFitsImage();
      
      return *this;
      
    }
    
    
    /*****************************************************************************/
    // solveField
    /*****************************************************************************/
    bool solveField() { return mpl::fits::solveField(filepath); }
    
  };

} /* namespace mpl */

#endif /* _H_MPL_CVFIT_H_ */

