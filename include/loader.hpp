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


#ifndef _H_MPL_LOADER_H_
#define _H_MPL_LOADER_H_

#include <cstdlib>
#include <cstdio>

#include <cstring>

#include <vector>
#include <string>
#include <sstream>

#include <algorithm>

#include <functional>

#define WITH_MPL_STDIO

#ifdef WITH_MPL_STDIO
  #include "stdio.hpp"
#else
  #include <cerrno>
#endif

//****************************************************************************
// namespace mpl
//****************************************************************************
namespace mpl {
  
  //****************************************************************************
  // class loader
  //****************************************************************************
  class loader {
    
  public:
    
    //****************************************************************************
    // data
    //****************************************************************************
    class data {
      
      private:
      
        // vettore dove vengono salvati i valori delle colonne lette
        std::vector<std::string> arguments;
      
      public:
      
        //****************************************************************************
        // add
        //****************************************************************************
        void add(std::string argument) { arguments.push_back(argument); }
      
        //****************************************************************************
        // get
        //****************************************************************************
        template <typename T>
        T get(uint32_t index) const {
          
          if(index > arguments.size()-1){
            fprintf(stderr, "The requested argument (%u) does not exist\n", index);
            abort();
          }
        
          std::stringstream iss(arguments[index]);
        
          T value;
        
          iss >> value;
        
          return value;
        
        }
      
      }; /* class data */
    
  private:
    
    //****************************************************************************
    // skipWhite
    //****************************************************************************
    inline const char * skipWhite(const std::string & line){
      
      for(uint32_t i=0; i<line.length(); ++i)
        if(!isspace(line[i])) return &line[i];
      
      return &line[line.length()-1];
      
    }
    
    //****************************************************************************
    // isToSkip
    //****************************************************************************
    inline bool isToSkip(const std::string & line){
      
      const std::string & ptr = skipWhite(line);
      
      if(ptr.length() == 1 && std::isspace(line[0])) return true;
      
      if(ptr.length() < 1) return true;
      
      if(ptr[0] == '#') return true;
      
      return false;
      
    }
    
    //****************************************************************************
    // getColsToRead
    //****************************************************************************
    void getColsToRead(const std::string & colsFormat, std::vector<uint32_t> & columns){
      
      std::stringstream iss(colsFormat);
      
      int value;
      
      while(iss.good()){
        
        iss >> value;
                
        columns.push_back(value-1);
        
      }
      
    }
 
  public:
    
    //****************************************************************************
    // loader
    //****************************************************************************
    loader(){};
    
    //****************************************************************************
    // loader
    //****************************************************************************
    loader(const std::string & inputFile, const std::string & columnsToRead, std::function<void(const loader::data & arguments)> fillerFun = NULL) {
      
      (*this)(inputFile, columnsToRead, fillerFun);
      
    }
    
    //****************************************************************************
    // load
    //****************************************************************************
    void load(const std::string & inputFile, const std::string & columnsToRead, std::function<void(const loader::data & arguments)> fillerFun = NULL) {
      
      (*this)(inputFile, columnsToRead, fillerFun);
      
    }
    
    //****************************************************************************
    // operator ()
    //****************************************************************************
    void operator () (const std::string & inputFile, const std::string & columnsToRead, std::function<void(const loader::data & arguments)> fillerFun = NULL) {
 
      if(fillerFun == NULL) fillerFun = std::bind(&loader::filler, this, std::placeholders::_1);
      
      // apro il file in lettura
#ifdef WITH_MPL_STDIO
      FILE * input = io::open(inputFile, "r");
#else
      FILE * input = fopen(inputFile.c_str(), "r");
      
      if(input==NULL){
        fprintf(stderr, "error in opening file '%s': %s\n", inputFile.c_str(), strerror(errno));
        abort();
      }
#endif
      
      // vettore che contiene i numeri delle colonne da leggere
      std::vector<uint32_t> columns;
            
      // riempio il vettore delle colonne da leggere
      getColsToRead(columnsToRead, columns);
      
      // variabile di appoggio di lettura di una linea del file
      char line[PATH_MAX];
      
      // variabile di appoggio per un singolo argomento letto
      //std::string arg;
      char arg[1024];
      
      // ciclo su tutte le linee del file
      while(fgets(line, PATH_MAX, input)){
                
        // se la linea e vuota o va saltata la salto
        if(isToSkip(line)) continue;
        
        // variabile di che segna il numero di argomenti letti sulla singola linea
        uint32_t read = 0;
        
        // variabile per scorrere le colonne da salvare
        uint32_t colIndex = 0;
        
        // variabile dove mi vegno le varie colonne lette
        loader::data arguments;
        
        //std::stringstream iss(line);
        
        int status = 0;
        
        // ciclo su numero di argomenti da leggere
        do {
          
          status = sscanf(line, "%s", arg);
          //iss >> arg;

          // se la colonna non va salvata
          if(read != columns[colIndex]) { ++read; continue; }
                    
          // salvo l'argomento letto
          arguments.add(arg);
          
          // se ho letto tutte le colonne che mi interessavano
          if(read == columns.back()) { break; }
          
          // incremento il contantatore delle colonne da salvare lette
          ++colIndex;
          
          // incremento il contantatore delle colonne lette
          ++read;
          
        //} while(!iss.eof());// ciclo su tutte le colonne del file
        } while(status != EOF);// ciclo su tutte le colonne del file

        // se non ho letto tutte le colonne che mi aspettavo
        if(read != columns.back()) { fprintf(stderr, "error in read \"%s\" bad number of columns\n", inputFile.c_str()); abort(); }
                
        // fillo i dati
        fillerFun(arguments);
        
      } // ciclo su tutte le righe del file
      
#ifdef WITH_MPL_STDIO
      io::close(input);
#else
      fclose(input);
#endif
      
    }
    
    //****************************************************************************
    // operator ()
    //****************************************************************************
    template <typename T>
    inline void operator () (const std::string & inputFile, const std::string & columnsToRead, T & container, void(T::*fillerFun)(const loader::data & arguments)) {
      (*this)(inputFile, columnsToRead, std::bind(fillerFun, &container, std::placeholders::_1));
    }
    
    //****************************************************************************
    // operator ()
    //****************************************************************************
    template <typename T>
    inline void operator () (const std::string & inputFile, const std::string & columnsToRead, T * container, void(T::*fillerFun)(const loader::data & arguments)){
      (*this)(inputFile, columnsToRead, std::bind(fillerFun, container, std::placeholders::_1));
    }
    
    //****************************************************************************
    // minmax
    //****************************************************************************
    template <typename T>
    void minmax(const std::string & inputFile, uint32_t column, T & min, T & max){
      
      // apro il file in lettura
#ifdef WITH_MPL_STDIO
      FILE * input = io::open(inputFile, "r");
#else
      FILE * input = fopen(inputFile, "r");
      
      if(input==NULL){
        fprintf(stderr, "error in opening file '%s': %s\n", inputFile, strerror(errno));
        abort();
      }
#endif
      
      // variabile di appoggio di lettura di una linea del file
      char line[PATH_MAX];
      
      // variabile di appoggio per un singolo argomento letto
      T value;
      
      min = std::numeric_limits<T>::max();
      max = std::numeric_limits<T>::lowest();
      
      column -= 1;
      
      //std::string arg;
      char arg[1024];
            
      // ciclo su tutte le linee del file
      while(fgets(line, PATH_MAX, input)){
                
        // se la linea e vuota o va saltata la salto
        if(isToSkip(line)) continue;
        
        // variabile di che segna il numero di argomenti letti sulla singola linea
        uint32_t read = 0;
        
        //std::stringstream iss(line);

        // ciclo su numero di argomenti da leggere
        int status = 0;
          
        // ciclo su numero di argomenti da leggere
        do {
        //while(iss >> arg && !iss.eof()) {

          status = sscanf(line, "%s", arg);
          
          // se la colonna non va salvata
          if(read++ != column) continue;
          
          //iss.str(arg);
          
          //iss >> value;
          
          value = atof(arg);
          
          if(min > value) min = value;
          if(max < value) max = value;
          
          break;
          
        //} // ciclo su tutte le colonne del file
        } while(status != EOF);// ciclo su tutte le colonne del file
        
        // se non ho letto tutte le colonne che mi aspetavo
        if((read-1) != column) { fprintf(stderr, "error\n"); abort(); }
        
      } // ciclo su tutte le righe del file
      
#ifdef WITH_MPL_STDIO
      io::close(input);
#else
      fclose(input);
#endif
      
    }
    
    //****************************************************************************
    // max
    //****************************************************************************
    template <typename T>
    void max(const std::string & inputFile, uint32_t column, T & max){ T min; minmax(inputFile, column, min, max); }
    
    //****************************************************************************
    // min
    //****************************************************************************
    template <typename T>
    void min(const std::string & inputFile, uint32_t column, T & min){ T max; minmax(inputFile, column, min, max); }
    
  protected:
    
    //****************************************************************************
    // loader
    //****************************************************************************
    virtual void filler(const loader::data & arguments) { };
    
  }; /* class load */
  
  //****************************************************************************
  // get
  //****************************************************************************
  template <>
  inline const std::string & loader::data::get(uint32_t index) const {
    
    if(index > arguments.size()-1){
      fprintf(stderr, "The requested argument (%u) does not exist\n", index);
      abort();
    }
    
    return arguments[index];
    
  }
  
} /* namespace mpl */

#endif /* _H_MPL_LOADER_H_ */
