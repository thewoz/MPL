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

#ifndef _H_MPL_CURL_H
#define _H_MPL_CURL_H

#include <cstdio>
#include <cstdlib>

#include <cstring>
#include <cerrno>

#include <curl/curl.h>

//*****************************************************************************/
// namespace mpl::web::curl
//*****************************************************************************/
namespace mpl::web::curl {
  
  //*****************************************************************************/
  // get
  //*****************************************************************************/
  int get(const char * url, const char * outputFile, bool pedantic = false, bool verbose = false) {
    
    // init curl handler
    CURL * curl = curl_easy_init();
    
    if(curl) {
      
      // apro il file di output
      FILE * output = fopen(outputFile, "wb");
      
      // controllo se ci sono errori
      if(output == NULL) {
        if(verbose) fprintf(stderr, "%s curl fails to open the output file: '%s': %s\n", (pedantic) ? "error" : "warning", outputFile, strerror(errno));
        curl_easy_cleanup(curl);
        if(pedantic) abort(); else return 1;
      }
      
      // definico le operazioni che deve fare
      curl_easy_setopt(curl, CURLOPT_URL, url);
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, NULL);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, output);
      
      // eseguo i comandi
      CURLcode res = curl_easy_perform(curl);
      
      // pulisco curl
      curl_easy_cleanup(curl);
      
      // chiudo il file
      fclose(output);
      
      // controlo gli errori
      if(res != CURLE_OK) {
        if(verbose) fprintf(stderr, "%s curl was no able get the files '%s': %s\n", (pedantic) ? "error" : "warning", url, curl_easy_strerror(res));
        if(pedantic) abort(); else return 1;
      }
      
      return 0;
      
    } else {
      if(verbose) fprintf(stderr, "%s curl fail to initialize\n", (pedantic) ? "error" : "warning");
      if(pedantic) abort(); else return 1;
    }
    
  }
  
  //*****************************************************************************/
  // get
  //****************************************************************************/
  int get(const char * url, const char * post, const char * outputFile, bool pedantic = false, bool verbose = false) {
    
    // init curl handler
    CURL * curl = curl_easy_init();
    
    if(curl) {
      
      // apro il file di output
      FILE * output = fopen(outputFile, "wb");
      
      // controllo se ci sono errori
      if(output == NULL) {
        if(verbose) fprintf(stderr, "%s curl fails to open the output file: '%s': %s\n", (pedantic) ? "error" : "warning", outputFile, strerror(errno));
        curl_easy_cleanup(curl);
        if(pedantic) abort(); else return 1;
      }
      
      // definico le operazioni che deve fare
      curl_easy_setopt(curl, CURLOPT_URL, url);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post);
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, NULL);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, output);
      
      // eseguo i comandi
      CURLcode res = curl_easy_perform(curl);
      
      // pulisco curl
      curl_easy_cleanup(curl);
      
      // chiudo il file
      fclose(output);
      
      // controlo gli errori
      if(res != CURLE_OK) {
        if(verbose) fprintf(stderr, "%s curl was no able get the files '%s': %s\n", (pedantic) ? "error" : "warning", url, curl_easy_strerror(res));
        if(pedantic) abort(); else return 1;
      }
      
      return 0;
      
    } else {
      if(verbose) fprintf(stderr, "%s curl fail to initialize\n", (pedantic) ? "error" : "warning");
      if(pedantic) abort(); else return 1;
    }
    
  }
  
} /* namespace curl */

#endif /* _H_MPL_CURL_H */
