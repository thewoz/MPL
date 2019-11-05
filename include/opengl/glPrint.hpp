/*
 * MIT License
 *
 * Copyright (c) 2017 Leonardo Parisi
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

#ifndef _H_GLTF_H_
#define _H_GLTF_H_

#include <cstdlib>
#include <cstdio>

#include <cstdarg>

#ifdef __APPLE__
  #include <OpenGL/gl.h>
#else
  #include <GL/gl.h>
#endif

/*
 * -- Font structure taken from the file fg_internal.h of the freeglut project.
 * -- See COPYING file in freeglut folder for licensing policy.
 */

/* The bitmap font structure */
struct SFG_Font
{
  const char*     Name;         /* The source font name             */
  int             Quantity;     /* Number of chars in font          */
  int             Height;       /* Height of the characters         */
  const GLubyte** Characters;   /* The characters mapping           */
  float           xorig, yorig; /* Relative origin of the character */
};

/* including file of freeglut font data */
#include "freeglut/fg_font_data.c"

/* enum of the font defined in fg_font_data.c */
enum { BITMAP_8_BY_13, BITMAP_9_BY_15, BITMAP_HELVETICA_10, BITMAP_HELVETICA_12, BITMAP_HELVETICA_18,BITMAP_TIMES_ROMAN_10, BITMAP_TIMES_ROMAN_24 };

/*
 * -- Private font function taken from the file fg_font.c of the freeglut project.
 * -- See COPYING file in freeglut folder for licensing policy.
 */
const inline SFG_Font * fontByID(int font) {
  
  if( font == BITMAP_8_BY_13        )
    return &fgFontFixed8x13;
  if( font == BITMAP_9_BY_15        )
    return &fgFontFixed9x15;
  if( font == BITMAP_HELVETICA_10   )
    return &fgFontHelvetica10;
  if( font == BITMAP_HELVETICA_12   )
    return &fgFontHelvetica12;
  if( font == BITMAP_HELVETICA_18   )
    return &fgFontHelvetica18;
  if( font == BITMAP_TIMES_ROMAN_10 )
    return &fgFontTimesRoman10;
  if( font == BITMAP_TIMES_ROMAN_24 )
    return &fgFontTimesRoman24;
  
  return 0;
  
}

/*
 * -- Raster text function taken from the file fg_font.c of the freeglut project.
 * -- See COPYING file in freeglut folder for licensing policy.
 */
void glutBitmapString(int fontID, const char * string){
  
  unsigned char c;
  
  float x = 0.0f;
  
  const SFG_Font * font = fontByID(fontID);
  
  if(!font){
    fprintf(stderr, "glutBitmapString: bitmap font 0x%08x not found. Make sure you're not passing a stroke font.\n", fontID);
    abort();
  }
  
  if(!string || ! *string) return;
  
  glPushClientAttrib(GL_CLIENT_PIXEL_STORE_BIT);
  
  glPixelStorei( GL_UNPACK_SWAP_BYTES,  GL_FALSE );
  glPixelStorei( GL_UNPACK_LSB_FIRST,   GL_FALSE );
  glPixelStorei( GL_UNPACK_ROW_LENGTH,  0        );
  glPixelStorei( GL_UNPACK_SKIP_ROWS,   0        );
  glPixelStorei( GL_UNPACK_SKIP_PIXELS, 0        );
  glPixelStorei( GL_UNPACK_ALIGNMENT,   1        );
  
  /*
   * Step through the string, drawing each character.
   * A newline will simply translate the next character's insertion
   * point back to the start of the line and down one line.
   */
  while(( c = *string++))
    if(c == '\n')
    {
      glBitmap(0, 0, 0, 0, -x, (float) -font->Height, NULL);
      x = 0.0f;
    }
    else  /* Not an EOL, draw the bitmap character */
    {
      const GLubyte* face = font->Characters[c];
      
      glBitmap(
               face[0], font->Height,    /* Bitmap's width and height    */
               font->xorig, font->yorig, /* The origin in the font glyph */
               (float)(face[0]), 0.0,    /* The raster advance; inc. x,y */
               (face+1)                  /* The packed bitmap data...    */
               );
      
      x += (float)(face[0]);
    }
  
  glPopClientAttrib();
  
}



//****************************************************************************//
// string 3D
//****************************************************************************//
inline void glPrint(cost char * string, int font, GLfloat x, GLfloat y, GLfloat z){
  
  glRasterPos3f(x, y, z);
  
  glutBitmapString(font, string);
  
}

//****************************************************************************//
// interger 3D
//****************************************************************************//
inline void glPrint(int value, int font, GLfloat x, GLfloat y, GLfloat z){
  
  char string[PATH_MAX]; sprintf(string, "%d", value);
  
  glRasterPos3f(x, y, z);

  glutBitmapString(font, string);

}

//****************************************************************************//
// float 3D
//****************************************************************************//
inline void glPrint(double value, int font, GLfloat x, GLfloat y, GLfloat z){
  
  char string[PATH_MAX]; sprintf(string, "%f", value);
  
  glRasterPos3f(x, y, z);
  
  glutBitmapString(font, string);
  
}

//****************************************************************************//
// string 2D
//****************************************************************************//
inline void glPrint(cost char * string, int font, GLfloat x, GLfloat y, GLfloat z){
  
  glRasterPos2f(x, y);
  
  glutBitmapString(font, string);
  
}

//****************************************************************************//
// interger 2D
//****************************************************************************//
inline void glPrint(int value, int font, GLfloat x, GLfloat y, GLfloat z){
  
  char string[PATH_MAX]; sprintf(string, "%d", value);
  
  glRasterPos2f(x, y);
  
  glutBitmapString(font, string);
  
}

//****************************************************************************//
// float 2D
//****************************************************************************//
inline void glPrint(double value, int font, double x, double y, double z){
  
  char string[PATH_MAX]; sprintf(string, "%f", value);
  
  glRasterPos2f(x, y);
  
  glutBitmapString(font, string);
  
}

//***************************************************************************************************//
// glPrintf 3D
//***************************************************************************************************//
inline void glPrint(int font, double x, double y, double z, const char * format, ...) {
  
  char string[PATH_MAX];
  
  va_list ap;
  
  va_start(ap, format);
  
  vfprintf(string, format, ap);
  
  flush();
  
  va_end(ap);
  
  glRasterPos3f(x, y, z);
  
  glutBitmapString(font, string);
  
}

//***************************************************************************************************//
// glPrintf 3D
//***************************************************************************************************//
inline void glPrint(int font, double x, double y, const char * format, ...) {
  
  char string[PATH_MAX];
  
  va_list ap;
  
  va_start(ap, format);
  
  vfprintf(string, format, ap);
  
  flush();
  
  va_end(ap);
  
  glRasterPos3f(x, y);
  
  glutBitmapString(font, string);
  
}

#endif /* _H_GLTF_H_ */
