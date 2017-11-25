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


#ifndef _COBBS_LOADER_H_
#define _COBBS_LOADER_H_

#include <cstdlib>
#include <cstdio>

#include <cstring>

#include <vector>
#include <string>
#include <sstream>

#include <algorithm>

#include <functional>

#include <cobbs/stdio.hpp>

/*****************************************************************************/
// arguments_t
/*****************************************************************************/
class arguments_t {
  
private:
  
  // vettore dove vengono salvati i valori delle colonne lette
  std::vector<std::string> arguments;

public:
  
  /*****************************************************************************/
  // add
  /*****************************************************************************/
  void add(std::string argument){ arguments.push_back(argument); }

  /*****************************************************************************/
  // get
  /*****************************************************************************/
  template <typename T>
  T get(uint32_t index) const {
   
    if(index > arguments.size()-1){
      fprintf(stderr, "The requested argument does not exist\n");
      abort();
    }
    
    std::stringstream iss(arguments[index]);
    
    T value;
    
    iss >> value;
    
    return value;
    
  }
  
};



/*****************************************************************************/
// get
/*****************************************************************************/
template <>
const char * arguments_t::get(uint32_t index) const {
  
  if(index > arguments.size()-1){
    fprintf(stderr, "The requested argument does not exist\n");
    abort();
  }
  
  return arguments[index].c_str();
  
}


/*****************************************************************************/
// loader_t
/*****************************************************************************/
class loader_t {
  
private:
  
  /*****************************************************************************/
  // skipWhite
  /*****************************************************************************/
  inline const char * skipWhite(const char * line){
    
    for(uint32_t i=0; i<strlen(line); ++i)
      if(!isspace(line[i])) return &line[i];
    
    return &line[strlen(line)-1];
    
  }
  
  /*****************************************************************************/
  // isToSkip
  /*****************************************************************************/
  inline bool isToSkip(const char * line){
    
    const char * ptr = skipWhite(line);
    
    if(strlen(ptr) == 1 && isspace(line[0])) return true;
    
    if(strlen(ptr) < 1) return true;
    
    if(ptr[0] == '#') return true;
    
    return false;
    
  }
  
  /*****************************************************************************/
  // getColsToRead
  /*****************************************************************************/
  void getColsToRead(const char * colsFormat, std::vector<uint32_t> & columns){
    
    std::stringstream iss(colsFormat);
    
    int value;
    
    while(iss.good()){
    
      iss >> value;
    
      columns.push_back(value-1);

    }
  
  }
  
  /*
  void somefunction(void (*fptr)(void*, int, int), void * context = 0) {
    fptr(context, 17, 42);
  }
*/
public:
  
  loader_t(){};

  
  /*****************************************************************************/
  // load
  /*****************************************************************************/
  //template<typename T>
  //void load(const char * inputFile, const char * columnsToRead, void T::fillerFun(const arguments_t & arguments) = NULL){
  void load(const char * inputFile, const char * columnsToRead, std::function<void(const arguments_t & arguments)> fillerFun = NULL){
  //void load(const char * inputFile, const char * columnsToRead, std::function<void(const T, const arguments_t & arguments)> fillerFun = NULL){
  //void load(const char * inputFile, const char * columnsToRead, std::function<void(const T, const arguments_t & arguments)> fillerFun = NULL){
  //void load(const char * inputFile, const char * columnsToRead, void(*fillerFun)(const arguments_t & arguments) = NULL){
  //void load(const char * inputFile, const char * columnsToRead, void(T::*fillerFun)(const arguments_t & arguments) = NULL){
    
    //std::function<void(const Foo&, int)>
    
    if(fillerFun == NULL) fillerFun = std::bind(&loader_t::filler, this, std::placeholders::_1);
    
    //printf("inputFile %s\n", inputFile);
    
    // apro il file in lettura
    FILE * input = io::openf(inputFile, "r");
    
    // vettore che contiene i numeri delle colonne da leggere
    std::vector<uint32_t> columns;

     // riempio il vettore delle colonne da leggere
    getColsToRead(columnsToRead, columns);
    
    // variabile di appoggio di lettura di una linea del file
    char line[PATH_MAX];
    
    // variabile di appoggio per un singolo argomento letto
    std::string arg;
        
    // ciclo su tutte le linee del file
    while(fgets(line, PATH_MAX, input)){
      
      // se la linea e vuota o va saltata la salto
      if(isToSkip(line)) continue;
      
      // variabile di che segna il numero di argomenti letti sulla singola linea
      uint32_t read = 0;
      
      // variabile per scorrere le colonne da salvare
      uint32_t colIndex = 0;
      
      // variabile dove mi vegno le varie colonne lette
      arguments_t arguments;
      
      std::stringstream iss(line);

      // ciclo su numero di argomenti da leggere
      while(iss.good()){
   
        iss >> arg;
        
        // se la colonna non va salvata
        if(read++ != columns[colIndex]) continue;
        
        // salvo l'argomento letto
        arguments.add(arg);
        
        // incremento il contantatore delle colonne da salvare lette
        ++colIndex;
        
        // se ho letto tutte le colonne che mi interessavano
        if(colIndex >= columns.size()) break;
        
      } // ciclo su tutte le colonne del file
      
      // se non ho letto tutte le colonne che mi aspetavo
      if(colIndex != columns.size()) { fprintf(stderr, "errro\n"); abort(); }
      
      // fillo i dati
      fillerFun(arguments);
      
    } // ciclo su tutte le righe del file
   
    io::closef(input);
    
  }
  
  /*****************************************************************************/
  // minmax
  /*****************************************************************************/
  template <typename T>
  void minmax(const char * inputFile, uint32_t column, T & min, T & max){
    
    // apro il file in lettura
    FILE * input = io::openf(inputFile, "r");
    
    // variabile di appoggio di lettura di una linea del file
    char line[PATH_MAX];
    
    // variabile di appoggio per un singolo argomento letto
    T value;

    min = std::numeric_limits<T>::max();
    max = std::numeric_limits<T>::lowest();

    column -= 1;
    
    std::string arg;

    // ciclo su tutte le linee del file
    while(fgets(line, PATH_MAX, input)){
      
      // se la linea e vuota o va saltata la salto
      if(isToSkip(line)) continue;
      
      // variabile di che segna il numero di argomenti letti sulla singola linea
      uint32_t read = 0;
      
      std::stringstream iss(line);
      
      // ciclo su numero di argomenti da leggere
      while(iss.good()){
        
        iss >> arg;
      
        // se la colonna non va salvata
        if(read++ != column) continue;
        
        iss.str(arg);
        
        iss >> value;
        
        if(min > value) min = value;
        if(max < value) max = value;

        break;
        
      } // ciclo su tutte le colonne del file
      
      // se non ho letto tutte le colonne che mi aspetavo
      if((read-1) != column) { fprintf(stderr, "errro\n"); abort(); }
      
    } // ciclo su tutte le righe del file
    
    io::closef(input);
    
  }
  
  /*****************************************************************************/
  // max
  /*****************************************************************************/
  template <typename T>
  void max(const char * inputFile, uint32_t column, T & max){ T min; minmax(inputFile, column, min, max); }
    
  /*****************************************************************************/
  // min
  /*****************************************************************************/
  template <typename T>
  void min(const char * inputFile, uint32_t column, T & min){ T max; minmax(inputFile, column, min, max); }
    
  template <typename T>
  inline void load(const char * inputFile, const char * columnsToRead, T & container, void(T::*fillerFun)(const arguments_t & arguments)){
    load(inputFile, columnsToRead, std::bind(fillerFun, &container, std::placeholders::_1));
  }
  
  template <typename T>
  inline void load(const char * inputFile, const char * columnsToRead, T * container, void(T::*fillerFun)(const arguments_t & arguments)){
    load(inputFile, columnsToRead, std::bind(fillerFun, container, std::placeholders::_1));
  }
  
protected:
    
  /*****************************************************************************/
  // loader
  /*****************************************************************************/
  virtual void filler(const arguments_t & arguments) { };
  
}; /* class loader_t */


#endif /* _COBBS_LOADER_H_ */
