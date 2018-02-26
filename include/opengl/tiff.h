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

#ifndef _H_MPL_TIFF_H_
#define _H_MPL_TIFF_H_

#include <cstdlib>
#include <cstdio>
#include <cerrno>

#ifdef __APPLE__
  #include <OpenGL/gl.h>
#else
  #include <GL/gl.h>
#endif

//****************************************************************************//
// namespace tiff
//****************************************************************************//
namespace tiff {
  
#include <tiffio.h>
  
  //****************************************************************************//
  // snapshot
  //****************************************************************************//
  void snapshot(int width, int height, const char * outputfile) {
    
    TIFF * file = TIFFOpen(outputfile, "w");
    
    if(file==NULL){
      fprintf(stderr, "file '%s' line %d function '%s': error in opening file '%s'\n", __FILE__, __LINE__, __func__, outputfile);
      exit(EXIT_FAILURE);
    }
    
    GLubyte * image = (GLubyte *) malloc(width * height * sizeof (GLubyte) * 3);
    
    if(image==NULL){
      fprintf(stderr, "file '%s' line %d function '%s': error in malloc %s\n", __FILE__, __LINE__, __func__, strerror(errno));
      abort();
    }
    
    /* OpenGL's default 4 byte pack alignment would leave extra bytes at the
     end of each image row so that each full row contained a number of bytes
     divisible by 4.  Ie, an RGB row with 3 pixels and 8-bit componets would
     be laid out like "RGBRGBRGBxxx" where the last three "xxx" bytes exist
     just to pad the row out to 12 bytes (12 is divisible by 4). To make sure
     the rows are packed as tight as possible (no row padding), set the pack
     alignment to 1. */
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
    
    // TODO: controllare il valore di ritorno di glReadPixels
    
    TIFFSetField(file, TIFFTAG_IMAGEWIDTH, (uint32) width);
    TIFFSetField(file, TIFFTAG_IMAGELENGTH, (uint32) height);
    TIFFSetField(file, TIFFTAG_BITSPERSAMPLE, 8);
    TIFFSetField(file, TIFFTAG_COMPRESSION, COMPRESSION_PACKBITS);
    TIFFSetField(file, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
    TIFFSetField(file, TIFFTAG_SAMPLESPERPIXEL, 3);
    TIFFSetField(file, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(file, TIFFTAG_ROWSPERSTRIP, 1);
    TIFFSetField(file, TIFFTAG_IMAGEDESCRIPTION, "");
    
    // mi serve per non scordarmi il valore del puntatore originale
    GLubyte * p = image;
    
    for(int i=height - 1; i>=0; --i) {
      
      if(TIFFWriteScanline(file, p, i, 0) < 0) {
        fprintf(stderr, "file '%s' line %d function '%s': error in TIFFWriteScanline\n", __FILE__, __LINE__, __func__);
        abort();
      }
      
      p += width * sizeof (GLubyte) * 3;
      
    }
    
    TIFFClose(file);
    
    free(image);
    
  }
  
} /* namespace tiff */

#endif /* _H_MPL_TIFF_H_ */
