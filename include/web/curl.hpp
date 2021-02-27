/*
 * GNU GENERAL PUBLIC LICENSE
 *
 * Copyright (C) 2017
 * Created by Leonardo Parisi (leonardo.parisi[at]gmail.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
      FILE * input = std::tmpfile(); //fopen(outputFile, "wb");
      
      // controllo se ci sono errori
      if(input == NULL) {
        if(verbose) fprintf(stderr, "%s curl fails to open the output tmp file: %s\n", (pedantic) ? "error" : "warning", strerror(errno));
        curl_easy_cleanup(curl);
        if(pedantic) abort(); else return 1;
      }
      
      // definico le operazioni che deve fare
      curl_easy_setopt(curl, CURLOPT_URL, url);
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, NULL);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, input);
      
      // eseguo i comandi
      CURLcode res = curl_easy_perform(curl);
      
      // pulisco curl
      curl_easy_cleanup(curl);
      
      // controlo gli errori
      if(res != CURLE_OK) {
        
        if(verbose) fprintf(stderr, "%s curl was no able get the files '%s': %s\n", (pedantic) ? "error" : "warning", url, curl_easy_strerror(res));
        if(pedantic) abort(); else return 1;
        
      } else {
        
        std::rewind(input);
        
        FILE * output = fopen(outputFile, "wb");
        
        // controllo se ci sono errori
        if(output == NULL) {
          fprintf(stderr, "error curl fail to copy the file\n");
          abort();
        }
        
        char buf[BUFSIZ];
        
        size_t size;
        
        while((size = fread(buf, 1, BUFSIZ, input))) {
          fwrite(buf, 1, size, output);
        }
        
        fclose(output);
        
      }
      
      fclose(input);
      
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
      FILE * input = fopen(outputFile, "wb");
      
      // controllo se ci sono errori
      if(input == NULL) {
        if(verbose) fprintf(stderr, "%s curl fails to open the output tmp file: %s\n", (pedantic) ? "error" : "warning", strerror(errno));
        curl_easy_cleanup(curl);
        if(pedantic) abort(); else return 1;
      }
      
      // definico le operazioni che deve fare
      curl_easy_setopt(curl, CURLOPT_URL, url);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, post);
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, NULL);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, input);
      
      // eseguo i comandi
      CURLcode res = curl_easy_perform(curl);
      
      // pulisco curl
      curl_easy_cleanup(curl);
      
      // controlo gli errori
      if(res != CURLE_OK) {
        if(verbose) fprintf(stderr, "%s curl was no able get the files '%s': %s\n", (pedantic) ? "error" : "warning", url, curl_easy_strerror(res));
        if(pedantic) abort(); else return 1;
      }
      
      return 0;
      
    } else {
      
      if(verbose) fprintf(stderr, "%s curl fail to initialize\n", (pedantic) ? "error" : "warning");
      if(pedantic) abort(); else return 1;
      
    }  else {
      
      std::rewind(input);
      
      FILE * output = fopen(outputFile, "wb");
      
      // controllo se ci sono errori
      if(output == NULL) {
        fprintf(stderr, "error curl fail to copy the file\n");
        abort();
      }
      
      char buf[BUFSIZ];
      
      size_t size;
      
      while((size = fread(buf, 1, BUFSIZ, input))) {
        fwrite(buf, 1, size, output);
      }
      
      fclose(output);
      
    }
    
    fclose(input);
    
  }
  
} /* namespace curl */

#endif /* _H_MPL_CURL_H */
