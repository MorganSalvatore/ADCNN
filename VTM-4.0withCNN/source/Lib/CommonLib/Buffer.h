/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     Buffer.h
 *  \brief    Low-overhead class describing 2D memory layout
 */

#ifndef __BUFFER__
#define __BUFFER__

#include "Common.h"
#include "CommonDef.h"
#include "ChromaFormat.h"
#include "MotionInfo.h"

#include <string.h>
#include <type_traits>
#include <typeinfo>

// ---------------------------------------------------------------------------
// AreaBuf struct
// ---------------------------------------------------------------------------

#if ENABLE_SIMD_OPT_BUFFER
#ifdef TARGET_SIMD_X86

struct PelBufferOps
{
  PelBufferOps();

  void initPelBufOpsX86();
  template<X86_VEXT vext>
  void _initPelBufOpsX86();

  void ( *addAvg4 )       ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height,            int shift, int offset, const ClpRng& clpRng );
  void ( *addAvg8 )       ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height,            int shift, int offset, const ClpRng& clpRng );
  void ( *reco4 )         ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height,                                   const ClpRng& clpRng );
  void ( *reco8 )         ( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, int width, int height,                                   const ClpRng& clpRng );
  void ( *linTf4 )        ( const Pel* src0, int src0Stride,                                  Pel *dst, int dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng, bool bClip );
  void ( *linTf8 )        ( const Pel* src0, int src0Stride,                                  Pel *dst, int dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng, bool bClip );
  void(*addBIOAvg4)    (const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng);
#if JVET_M0063_BDOF_FIX
  void(*bioGradFilter) (Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth);
  void(*calcBIOPar)    (const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG, const int bitDepth);
#else
  void(*bioGradFilter) (Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY);
  void(*calcBIOPar)    (const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG);
#endif
  void(*calcBlkGradient)(int sx, int sy, int    *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize);
#if JVET_M0147_DMVR
  void(*copyBuffer)(Pel *src, int srcStride, Pel *dst, int dstStride, int width, int height);
  void(*padding)(Pel *dst, int stride, int width, int height, int padSize);
#endif
#if ENABLE_SIMD_OPT_GBI
  void ( *removeWeightHighFreq8)  ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height, int shift, int gbiWeight);
  void ( *removeWeightHighFreq4)  ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height, int shift, int gbiWeight);
  void ( *removeHighFreq8)        ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height);
  void ( *removeHighFreq4)        ( Pel* src0, int src0Stride, const Pel* src1, int src1Stride, int width, int height);
#endif
};

extern PelBufferOps g_pelBufOP;

#endif
#endif


#if JVET_M0147_DMVR
void paddingCore(Pel *ptr, int stride, int width, int height, int padSize);
void copyBufferCore(Pel *src, int srcStride, Pel *Dst, int dstStride, int width, int height);
#endif

template<typename T>
struct AreaBuf : public Size
{
  T*        buf;
  int       stride;
  // the proper type causes awful lot of errors
  //ptrdiff_t stride;

  AreaBuf()                                                                               : Size(),                  buf( NULL ), stride( 0 )          { }
  AreaBuf( T *_buf, const Size &size )                                                    : Size( size ),            buf( _buf ), stride( size.width ) { }
  AreaBuf( T *_buf, const int &_stride, const Size &size )                                : Size( size ),            buf( _buf ), stride( _stride )    { }
  AreaBuf( T *_buf, const SizeType &_width, const SizeType &_height )                     : Size( _width, _height ), buf( _buf ), stride( _width )     { }
  AreaBuf( T *_buf, const int &_stride, const SizeType &_width, const SizeType &_height ) : Size( _width, _height ), buf( _buf ), stride( _stride )    { }

  operator AreaBuf<const T>() const { return AreaBuf<const T>( buf, stride, width, height ); }

  void fill                 ( const T &val );
  void memset               ( const int val );

  void copyFrom             ( const AreaBuf<const T> &other );

  void reconstruct          ( const AreaBuf<const T> &pred, const AreaBuf<const T> &resi, const ClpRng& clpRng);
  void copyClip             ( const AreaBuf<const T> &src, const ClpRng& clpRng);

  void subtract             ( const AreaBuf<const T> &other );
  void extendSingleBorderPel();
  void extendBorderPel      (  unsigned margin );
  void addWeightedAvg       ( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng, const int8_t gbiIdx);
  void removeWeightHighFreq ( const AreaBuf<T>& other, const bool bClip, const ClpRng& clpRng, const int8_t iGbiWeight);
  void addAvg               ( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng );
  void removeHighFreq       ( const AreaBuf<T>& other, const bool bClip, const ClpRng& clpRng);
  void updateHistogram      ( std::vector<int32_t>& hist ) const;

  T    meanDiff             ( const AreaBuf<const T> &other ) const;
  void subtract             ( const T val );

  void linearTransform      ( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng );

  void transposedFrom       ( const AreaBuf<const T> &other );

  void toLast               ( const ClpRng& clpRng );

#if JVET_M0427_INLOOP_RESHAPER
  void rspSignal            ( std::vector<Pel>& pLUT );
  void scaleSignal          ( const int scale, const bool dir , const ClpRng& clpRng);
  T    computeAvg           ( ) const;
#endif
        T& at( const int &x, const int &y )          { return buf[y * stride + x]; }
  const T& at( const int &x, const int &y ) const    { return buf[y * stride + x]; }

        T& at( const Position &pos )                 { return buf[pos.y * stride + pos.x]; }
  const T& at( const Position &pos ) const           { return buf[pos.y * stride + pos.x]; }


        T* bufAt( const int &x, const int &y )       { return &at( x, y ); }
  const T* bufAt( const int &x, const int &y ) const { return &at( x, y ); }

        T* bufAt( const Position& pos )              { return &at( pos ); }
  const T* bufAt( const Position& pos ) const        { return &at( pos ); }

  AreaBuf<      T> subBuf( const Position &pos, const Size &size )                                    { return AreaBuf<      T>( bufAt( pos  ), stride, size   ); }
  AreaBuf<const T> subBuf( const Position &pos, const Size &size )                              const { return AreaBuf<const T>( bufAt( pos  ), stride, size   ); }
  AreaBuf<      T> subBuf( const int &x, const int &y, const unsigned &_w, const unsigned &_h )       { return AreaBuf<      T>( bufAt( x, y ), stride, _w, _h ); }
  AreaBuf<const T> subBuf( const int &x, const int &y, const unsigned &_w, const unsigned &_h ) const { return AreaBuf<const T>( bufAt( x, y ), stride, _w, _h ); }
};

typedef AreaBuf<      Pel>  PelBuf;
typedef AreaBuf<const Pel> CPelBuf;

typedef AreaBuf<      TCoeff>  CoeffBuf;
typedef AreaBuf<const TCoeff> CCoeffBuf;

typedef AreaBuf<      MotionInfo>  MotionBuf;
typedef AreaBuf<const MotionInfo> CMotionBuf;


#define SIZE_AWARE_PER_EL_OP( OP, INC )                     \
if( ( width & 7 ) == 0 )                                    \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 8 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
      OP( x + 2 );                                          \
      OP( x + 3 );                                          \
      OP( x + 4 );                                          \
      OP( x + 5 );                                          \
      OP( x + 6 );                                          \
      OP( x + 7 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else if( ( width & 3 ) == 0 )                               \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 4 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
      OP( x + 2 );                                          \
      OP( x + 3 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else if( ( width & 1 ) == 0 )                               \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x += 2 )                     \
    {                                                       \
      OP( x + 0 );                                          \
      OP( x + 1 );                                          \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}                                                           \
else                                                        \
{                                                           \
  for( int y = 0; y < height; y++ )                         \
  {                                                         \
    for( int x = 0; x < width; x++ )                        \
    {                                                       \
      OP( x );                                              \
    }                                                       \
                                                            \
    INC;                                                    \
  }                                                         \
}

template<typename T>
void AreaBuf<T>::fill(const T &val)
{
  if( sizeof( T ) == 1 )
  {
    if( width == stride )
    {
      ::memset( buf, reinterpret_cast< const signed char& >( val ), width * height * sizeof( T ) );
    }
    else
    {
      T* dest = buf;
      size_t line = width * sizeof( T );

      for( unsigned y = 0; y < height; y++ )
      {
        ::memset( dest, reinterpret_cast< const signed char& >( val ), line );

        dest += stride;
      }
    }
  }
  else if( T( 0 ) == val )
  {
    if( width == stride )
    {
      ::memset( buf, 0, width * height * sizeof( T ) );
    }
    else
    {
      T* dest = buf;
      size_t line = width * sizeof( T );

      for( unsigned y = 0; y < height; y++ )
      {
        ::memset( dest, 0, line );

        dest += stride;
      }
    }
  }
  else
  {
    T* dest = buf;

#define FILL_INC        dest      += stride
#define FILL_OP( ADDR ) dest[ADDR] = val

    SIZE_AWARE_PER_EL_OP( FILL_OP, FILL_INC );

#undef FILL_INC
#undef FILL_OP
  }
}

template<typename T>
void AreaBuf<T>::memset( const int val )
{
  if( width == stride )
  {
    ::memset( buf, val, width * height * sizeof( T ) );
  }
  else
  {
    T* dest = buf;
    size_t line = width * sizeof( T );

    for( int y = 0; y < height; y++ )
    {
      ::memset( dest, val, line );

      dest += stride;
    }
  }
}

template<typename T>
void AreaBuf<T>::copyFrom( const AreaBuf<const T> &other )
{
#if !defined(__GNUC__) || __GNUC__ > 5
  static_assert( std::is_trivially_copyable<T>::value, "Type T is not trivially_copyable" );
#endif

  VTMCHECK( width  != other.width,  "Incompatible size" );
  VTMCHECK( height != other.height, "Incompatible size" );

  if( buf == other.buf )
  {
    return;
  }

  if( width == stride && stride == other.stride )
  {
    memcpy( buf, other.buf, width * height * sizeof( T ) );
  }
  else
  {
          T* dst         = buf;
    const T* src         = other.buf;
    const unsigned srcStride = other.stride;

    for( unsigned y = 0; y < height; y++ )
    {
      memcpy( dst, src, width * sizeof( T ) );

      dst += stride;
      src += srcStride;
    }
  }
}


template<typename T>
void AreaBuf<T>::subtract( const AreaBuf<const T> &other )
{
  VTMCHECK( width  != other.width,  "Incompatible size" );
  VTMCHECK( height != other.height, "Incompatible size" );

        T* dest =       buf;
  const T* subs = other.buf;

#define SUBS_INC        \
  dest +=       stride; \
  subs += other.stride; \

#define SUBS_OP( ADDR ) dest[ADDR] -= subs[ADDR]

  SIZE_AWARE_PER_EL_OP( SUBS_OP, SUBS_INC );

#undef SUBS_OP
#undef SUBS_INC
}

template<typename T>
void AreaBuf<T>::copyClip( const AreaBuf<const T> &src, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::copyClip( const AreaBuf<const Pel> &src, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::reconstruct( const AreaBuf<const T> &pred, const AreaBuf<const T> &resi, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::reconstruct( const AreaBuf<const Pel> &pred, const AreaBuf<const Pel> &resi, const ClpRng& clpRng );


template<typename T>
void AreaBuf<T>::addAvg( const AreaBuf<const T> &other1, const AreaBuf<const T> &other2, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::addAvg( const AreaBuf<const Pel> &other1, const AreaBuf<const Pel> &other2, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::linearTransform( const int scale, const int shift, const int offset, bool bClip, const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::toLast( const ClpRng& clpRng )
{
  THROW( "Type not supported" );
}

template<>
void AreaBuf<Pel>::toLast( const ClpRng& clpRng );

template<typename T>
void AreaBuf<T>::removeWeightHighFreq(const AreaBuf<T>& other, const bool bClip, const ClpRng& clpRng, const int8_t gbiWeight)
{
  const int8_t gbiWeightOther = g_GbiWeightBase - gbiWeight;
  const int8_t log2WeightBase = g_GbiLog2WeightBase;

  const Pel* src = other.buf;
  const int  srcStride = other.stride;

  Pel* dst = buf;
  const int  dstStride = stride;

#if ENABLE_SIMD_OPT_GBI
  if(!bClip)
  {
    if(!(width & 7))
      g_pelBufOP.removeWeightHighFreq8(dst, dstStride, src, srcStride, width, height, 16, gbiWeight);
    else if(!(width & 3))
      g_pelBufOP.removeWeightHighFreq4(dst, dstStride, src, srcStride, width, height, 16, gbiWeight);
    else
      VTMCHECK(true, "Not supported");
  }
  else
  {
#endif
    int normalizer = ((1 << 16) + (gbiWeight > 0 ? (gbiWeight >> 1) : -(gbiWeight >> 1))) / gbiWeight;
    int weight0 = normalizer << log2WeightBase;
    int weight1 = gbiWeightOther * normalizer;
#define REM_HF_INC  \
  src += srcStride; \
  dst += dstStride; \

#define REM_HF_OP_CLIP( ADDR ) dst[ADDR] = ClipPel<T>( (dst[ADDR]*weight0 - src[ADDR]*weight1 + (1<<15))>>16, clpRng )
#define REM_HF_OP( ADDR )      dst[ADDR] =             (dst[ADDR]*weight0 - src[ADDR]*weight1 + (1<<15))>>16

    if(bClip)
    {
      SIZE_AWARE_PER_EL_OP(REM_HF_OP_CLIP, REM_HF_INC);
    }
    else
    {
      SIZE_AWARE_PER_EL_OP(REM_HF_OP, REM_HF_INC);
    }

#undef REM_HF_INC
#undef REM_HF_OP
#undef REM_HF_OP_CLIP
#if ENABLE_SIMD_OPT_GBI
  }
#endif
}

template<typename T>
void AreaBuf<T>::removeHighFreq( const AreaBuf<T>& other, const bool bClip, const ClpRng& clpRng )
{
  const T*  src       = other.buf;
  const int srcStride = other.stride;

        T*  dst       = buf;
  const int dstStride = stride;

#if ENABLE_SIMD_OPT_GBI
  if (!bClip)
  {
    if(!(width & 7))
      g_pelBufOP.removeHighFreq8(dst, dstStride, src, srcStride, width, height);
    else if (!(width & 3))
      g_pelBufOP.removeHighFreq4(dst, dstStride, src, srcStride, width, height);
    else
      VTMCHECK(true, "Not supported");
  }
  else
  {
#endif

#define REM_HF_INC  \
  src += srcStride; \
  dst += dstStride; \

#define REM_HF_OP_CLIP( ADDR ) dst[ADDR] = ClipPel<T>( 2 * dst[ADDR] - src[ADDR], clpRng )
#define REM_HF_OP( ADDR )      dst[ADDR] =             2 * dst[ADDR] - src[ADDR]

  if( bClip )
  {
    SIZE_AWARE_PER_EL_OP( REM_HF_OP_CLIP, REM_HF_INC );
  }
  else
  {
    SIZE_AWARE_PER_EL_OP( REM_HF_OP,      REM_HF_INC );
  }

#undef REM_HF_INC
#undef REM_HF_OP
#undef REM_HF_OP_CLIP

#if ENABLE_SIMD_OPT_GBI
  }
#endif
}


template<typename T>
void AreaBuf<T>::updateHistogram( std::vector<int32_t>& hist ) const
{
  const T* data = buf;
  for( std::size_t y = 0; y < height; y++, data += stride )
  {
    for( std::size_t x = 0; x < width; x++ )
    {
      hist[ data[x] ]++;
    }
  }
}

template<typename T>
void AreaBuf<T>::extendBorderPel( unsigned margin )
{
  T*  p = buf;
  int h = height;
  int w = width;
  int s = stride;

  VTMCHECK( ( w + 2 * margin ) > s, "Size of buffer too small to extend" );
  // do left and right margins
  for( int y = 0; y < h; y++ )
  {
    for( int x = 0; x < margin; x++ )
    {
      *( p - margin + x ) = p[0];
      p[w + x] = p[w - 1];
    }
    p += s;
  }

  // p is now the (0,height) (bottom left of image within bigger picture
  p -= ( s + margin );
  // p is now the (-margin, height-1)
  for( int y = 0; y < margin; y++ )
  {
    ::memcpy( p + ( y + 1 ) * s, p, sizeof( T ) * ( w + ( margin << 1 ) ) );
  }

  // pi is still (-marginX, height-1)
  p -= ( ( h - 1 ) * s );
  // pi is now (-marginX, 0)
  for( int y = 0; y < margin; y++ )
  {
    ::memcpy( p - ( y + 1 ) * s, p, sizeof( T ) * ( w + ( margin << 1 ) ) );
  }
}
template<typename T>
T AreaBuf<T>::meanDiff( const AreaBuf<const T> &other ) const
{
  int64_t acc = 0;

  VTMCHECK( width  != other.width,  "Incompatible size" );
  VTMCHECK( height != other.height, "Incompatible size" );

  const T* src1 =       buf;
  const T* src2 = other.buf;

#define MEAN_DIFF_INC   \
  src1 +=       stride; \
  src2 += other.stride; \

#define MEAN_DIFF_OP(ADDR) acc += src1[ADDR] - src2[ADDR]

  SIZE_AWARE_PER_EL_OP( MEAN_DIFF_OP, MEAN_DIFF_INC );

#undef MEAN_DIFF_INC
#undef MEAN_DIFF_OP

  return T( acc / area() );
}

#if ENABLE_SIMD_OPT_BUFFER && defined(TARGET_SIMD_X86)
template<> void AreaBuf<Pel>::subtract( const Pel val );
#endif

template<typename T>
void AreaBuf<T>::subtract( const T val )
{
  T* dst = buf;

#define OFFSET_INC       dst       += stride
#define OFFSET_OP(ADDR)  dst[ADDR] -= val

  SIZE_AWARE_PER_EL_OP( OFFSET_OP, OFFSET_INC );

#undef OFFSET_INC
#undef OFFSET_OP
}

template<typename T>
void AreaBuf<T>::transposedFrom( const AreaBuf<const T> &other )
{
  VTMCHECK( width * height != other.width * other.height, "Incompatible size" );

        T* dst  =       buf;
  const T* src  = other.buf;
  width         = other.height;
  height        = other.width;
  stride        = stride < width ? width : stride;

  for( unsigned y = 0; y < other.height; y++ )
  {
    for( unsigned x = 0; x < other.width; x++ )
    {
      dst[y + x*stride] = src[x + y*other.stride];
    }
  }
}


#ifndef DONT_UNDEF_SIZE_AWARE_PER_EL_OP
#undef SIZE_AWARE_PER_EL_OP
#endif // !DONT_UNDEF_SIZE_AWARE_PER_EL_OP

// ---------------------------------------------------------------------------
// UnitBuf struct
// ---------------------------------------------------------------------------

struct UnitArea;

template<typename T>
struct UnitBuf
{
  typedef static_vector<AreaBuf<T>,       MAX_NUM_COMPONENT> UnitBufBuffers;
  typedef static_vector<AreaBuf<const T>, MAX_NUM_COMPONENT> ConstUnitBufBuffers;

  ChromaFormat chromaFormat;
  UnitBufBuffers bufs;

  UnitBuf() : chromaFormat( NUM_CHROMA_FORMAT ) { }
  UnitBuf( const ChromaFormat &_chromaFormat, const UnitBufBuffers&  _bufs ) : chromaFormat( _chromaFormat ), bufs( _bufs ) { }
  UnitBuf( const ChromaFormat &_chromaFormat,       UnitBufBuffers&& _bufs ) : chromaFormat( _chromaFormat ), bufs( std::forward<UnitBufBuffers>( _bufs ) ) { }
  UnitBuf( const ChromaFormat &_chromaFormat, const AreaBuf<T>  &blkY ) : chromaFormat( _chromaFormat ), bufs{ blkY } { }
  UnitBuf( const ChromaFormat &_chromaFormat,       AreaBuf<T> &&blkY ) : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY) } { }
  UnitBuf( const ChromaFormat &_chromaFormat, const AreaBuf<T>  &blkY, const AreaBuf<T>  &blkCb, const AreaBuf<T>  &blkCr ) : chromaFormat( _chromaFormat ), bufs{ blkY, blkCb, blkCr } { }
  UnitBuf( const ChromaFormat &_chromaFormat,       AreaBuf<T> &&blkY,       AreaBuf<T> &&blkCb,       AreaBuf<T> &&blkCr ) : chromaFormat( _chromaFormat ), bufs{ std::forward<AreaBuf<T> >(blkY), std::forward<AreaBuf<T> >(blkCb), std::forward<AreaBuf<T> >(blkCr) } { }

  operator UnitBuf<const T>() const
  {
    return UnitBuf<const T>( chromaFormat, ConstUnitBufBuffers( bufs.begin(), bufs.end() ) );
  }

        AreaBuf<T>& get( const ComponentID comp )        { return bufs[comp]; }
  const AreaBuf<T>& get( const ComponentID comp )  const { return bufs[comp]; }

        AreaBuf<T>& Y()        { return bufs[0]; }
  const AreaBuf<T>& Y()  const { return bufs[0]; }
        AreaBuf<T>& Cb()       { return bufs[1]; }
  const AreaBuf<T>& Cb() const { return bufs[1]; }
        AreaBuf<T>& Cr()       { return bufs[2]; }
  const AreaBuf<T>& Cr() const { return bufs[2]; }

  void fill                 ( const T &val );
  void copyFrom             ( const UnitBuf<const T> &other );
  void reconstruct          ( const UnitBuf<const T> &pred, const UnitBuf<const T> &resi, const ClpRngs& clpRngs );
  void copyClip             ( const UnitBuf<const T> &src, const ClpRngs& clpRngs );
  void subtract             ( const UnitBuf<const T> &other );
  void addWeightedAvg       ( const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const uint8_t gbiIdx = GBI_DEFAULT, const bool chromaOnly = false, const bool lumaOnly = false);
  void addAvg               ( const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const bool chromaOnly = false, const bool lumaOnly = false);
  void extendSingleBorderPel();
  void extendBorderPel      ( unsigned margin );
  void removeHighFreq       ( const UnitBuf<T>& other, const bool bClip, const ClpRngs& clpRngs
                            , const int8_t gbiWeight = g_GbiWeights[GBI_DEFAULT]
                            );

        UnitBuf<      T> subBuf (const UnitArea& subArea);
  const UnitBuf<const T> subBuf (const UnitArea& subArea) const;
};

typedef UnitBuf<      Pel>  PelUnitBuf;
typedef UnitBuf<const Pel> CPelUnitBuf;

typedef UnitBuf<      TCoeff>  CoeffUnitBuf;
typedef UnitBuf<const TCoeff> CCoeffUnitBuf;

template<typename T>
void UnitBuf<T>::fill( const T &val )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].fill( val );
  }
}

template<typename T>
void UnitBuf<T>::copyFrom( const UnitBuf<const T> &other )
{
  VTMCHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].copyFrom( other.bufs[i] );
  }
}



template<typename T>
void UnitBuf<T>::subtract( const UnitBuf<const T> &other )
{
  VTMCHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].subtract( other.bufs[i] );
  }
}

template<typename T>
void UnitBuf<T>::copyClip(const UnitBuf<const T> &src, const ClpRngs& clpRngs)
{
  VTMCHECK( chromaFormat != src.chromaFormat, "Incompatible formats" );

  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].copyClip( src.bufs[i], clpRngs.comp[i] );
  }
}


template<typename T>
void UnitBuf<T>::reconstruct(const UnitBuf<const T> &pred, const UnitBuf<const T> &resi, const ClpRngs& clpRngs)
{
  VTMCHECK( chromaFormat != pred.chromaFormat, "Incompatible formats" );
  VTMCHECK( chromaFormat != resi.chromaFormat, "Incompatible formats" );

  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].reconstruct( pred.bufs[i], resi.bufs[i], clpRngs.comp[i] );
  }
}

template<typename T>
void UnitBuf<T>::addWeightedAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const uint8_t gbiIdx /* = GBI_DEFAULT */, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */)
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend = lumaOnly ? 1 : bufs.size();

  VTMCHECK(lumaOnly && chromaOnly, "should not happen");

  for(size_t i = istart; i < iend; i++)
  {
    bufs[i].addWeightedAvg(other1.bufs[i], other2.bufs[i], clpRngs.comp[i], gbiIdx);
  }
}

template<typename T>
void UnitBuf<T>::addAvg(const UnitBuf<const T> &other1, const UnitBuf<const T> &other2, const ClpRngs& clpRngs, const bool chromaOnly /* = false */, const bool lumaOnly /* = false */)
{
  const size_t istart = chromaOnly ? 1 : 0;
  const size_t iend   = lumaOnly   ? 1 : bufs.size();

  VTMCHECK( lumaOnly && chromaOnly, "should not happen" );

  for( size_t i = istart; i < iend; i++)
  {
    bufs[i].addAvg( other1.bufs[i], other2.bufs[i], clpRngs.comp[i]);
  }
}

template<typename T>
void UnitBuf<T>::extendSingleBorderPel()
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].extendSingleBorderPel();
  }
}

template<typename T>
void UnitBuf<T>::extendBorderPel( unsigned margin )
{
  for( unsigned i = 0; i < bufs.size(); i++ )
  {
    bufs[i].extendBorderPel( margin );
  }
}

template<typename T>
void UnitBuf<T>::removeHighFreq( const UnitBuf<T>& other, const bool bClip, const ClpRngs& clpRngs
                               , const int8_t gbiWeight
                               )
{
  if(gbiWeight != g_GbiWeights[GBI_DEFAULT])
  {
    bufs[0].removeWeightHighFreq(other.bufs[0], bClip, clpRngs.comp[0], gbiWeight);
    return;
  }
  bufs[0].removeHighFreq(other.bufs[0], bClip, clpRngs.comp[0]);

}

template<typename T>
UnitBuf<T> UnitBuf<T>::subBuf( const UnitArea& subArea )
{
  UnitBuf<T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( auto &subAreaBuf : bufs )
  {
    subBuf.bufs.push_back( subAreaBuf.subBuf( subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size() ) );
    blockIdx++;
  }

  return subBuf;
}


template<typename T>
const UnitBuf<const T> UnitBuf<T>::subBuf( const UnitArea& subArea ) const
{
  UnitBuf<const T> subBuf;
  subBuf.chromaFormat = chromaFormat;
  unsigned blockIdx = 0;

  for( const auto &subAreaBuf : bufs )
  {
    subBuf.bufs.push_back( subAreaBuf.subBuf( subArea.blocks[blockIdx].pos(), subArea.blocks[blockIdx].size() ) );
    blockIdx++;
  }

  return subBuf;
}

// ---------------------------------------------------------------------------
// PelStorage struct (PelUnitBuf which allocates its own memory)
// ---------------------------------------------------------------------------

struct UnitArea;
struct CompArea;

struct PelStorage : public PelUnitBuf
{
  PelStorage();
  ~PelStorage();

  void swap( PelStorage& other );
  void createFromBuf( PelUnitBuf buf );
  void create( const UnitArea &_unit );
  void create( const ChromaFormat &_chromaFormat, const Area& _area, const unsigned _maxCUSize = 0, const unsigned _margin = 0, const unsigned _alignment = 0, const bool _scaleChromaMargin = true );
  void destroy();

         PelBuf getBuf( const CompArea &blk );
  const CPelBuf getBuf( const CompArea &blk ) const;

         PelBuf getBuf( const ComponentID CompID );
  const CPelBuf getBuf( const ComponentID CompID ) const;

         PelUnitBuf getBuf( const UnitArea &unit );
  const CPelUnitBuf getBuf( const UnitArea &unit ) const;
  Pel *getOrigin( const int id ) const { return m_origin[id]; }

private:

  Pel *m_origin[MAX_NUM_COMPONENT];
};


#endif
