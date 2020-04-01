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

/** \file     YuvX86.cpp
    \brief    SIMD averaging.
*/

//! \ingroup CommonLib
//! \{


#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Buffer.h"


#if ENABLE_SIMD_OPT_BUFFER
#ifdef TARGET_SIMD_X86

template< X86_VEXT vext, int W >
void addAvg_SSE( const int16_t* src0, int src0Stride, const int16_t* src1, int src1Stride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng )
{
  if( W == 8 )
  {
    // TODO: AVX2 impl
    {
      __m128i vzero    = _mm_setzero_si128();
      __m128i voffset  = _mm_set1_epi32( offset );
      __m128i vibdimin = _mm_set1_epi16( clpRng.min );
      __m128i vibdimax = _mm_set1_epi16( clpRng.max );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 8 )
        {
          __m128i vsrc0 = _mm_loadu_si128( ( const __m128i * )&src0[col] );
          __m128i vsrc1 = _mm_loadu_si128( ( const __m128i * )&src1[col] );

          __m128i vtmp, vsum, vdst;
          vsum = _mm_cvtepi16_epi32   ( vsrc0 );
          vdst = _mm_cvtepi16_epi32   ( vsrc1 );
          vsum = _mm_add_epi32        ( vsum, vdst );
          vsum = _mm_add_epi32        ( vsum, voffset );
          vtmp = _mm_srai_epi32       ( vsum, shift );

          vsrc0 = _mm_unpackhi_epi64  ( vsrc0, vzero );
          vsrc1 = _mm_unpackhi_epi64  ( vsrc1, vzero );
          vsum = _mm_cvtepi16_epi32   ( vsrc0 );
          vdst = _mm_cvtepi16_epi32   ( vsrc1 );
          vsum = _mm_add_epi32        ( vsum, vdst );
          vsum = _mm_add_epi32        ( vsum, voffset );
          vsum = _mm_srai_epi32       ( vsum, shift );
          vsum = _mm_packs_epi32      ( vtmp, vsum );

          vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
          _mm_storeu_si128( ( __m128i * )&dst[col], vsum );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  +=  dstStride;
      }
    }
  }
  else if( W == 4 )
  {
    __m128i vzero     = _mm_setzero_si128();
    __m128i voffset   = _mm_set1_epi32( offset );
    __m128i vibdimin  = _mm_set1_epi16( clpRng.min );
    __m128i vibdimax  = _mm_set1_epi16( clpRng.max );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vsum = _mm_loadl_epi64  ( ( const __m128i * )&src0[col] );
        __m128i vdst = _mm_loadl_epi64  ( ( const __m128i * )&src1[col] );
        vsum = _mm_cvtepi16_epi32       ( vsum );
        vdst = _mm_cvtepi16_epi32       ( vdst );
        vsum = _mm_add_epi32            ( vsum, vdst );
        vsum = _mm_add_epi32            ( vsum, voffset );
        vsum = _mm_srai_epi32           ( vsum, shift );
        vsum = _mm_packs_epi32          ( vsum, vzero );

        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
        _mm_storel_epi64( ( __m128i * )&dst[col], vsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
}

#if JVET_M0147_DMVR
template<X86_VEXT vext>
void copyBufferSimd(Pel *src, int srcStride, Pel *dst, int dstStride, int width, int height)
{
  __m128i x;
#ifdef USE_AVX2
  __m256i x16;
#endif
  int j, temp;
  for (int i = 0; i < height; i++)
  {
    j = 0;
    temp = width;
#ifdef USE_AVX2
    while ((temp >> 4) > 0)
    {
      x16 = _mm256_loadu_si256((const __m256i*)(&src[i * srcStride + j]));
      _mm256_storeu_si256((__m256i*)(&dst[i * dstStride + j]), x16);
      j += 16;
      temp -= 16;
    }
#endif
    while ((temp >> 3) > 0)
    {
      x = _mm_loadu_si128((const __m128i*)(&src[ i * srcStride + j]));
      _mm_storeu_si128((__m128i*)(&dst[ i * dstStride + j]), x);
      j += 8;
      temp -= 8;
    }
    while ((temp >> 2) > 0)
    {
      x = _mm_loadl_epi64((const __m128i*)(&src[i * srcStride + j]));
      _mm_storel_epi64((__m128i*)(&dst[i*dstStride + j]), x);
      j += 4;
      temp -= 4;
    }
    while (temp > 0)
    {
      dst[i * dstStride + j] = src[i * srcStride + j];
      j++;
      temp--;
    }
  }
}


template<X86_VEXT vext>
void paddingSimd(Pel *dst, int stride, int width, int height, int padSize)
{
  __m128i x;
#ifdef USE_AVX2
  __m256i x16;
#endif
  int temp, j;
  for (int i = 1; i <= padSize; i++)
  {
    j = 0;
    temp = width;
#ifdef USE_AVX2
    while ((temp >> 4) > 0)
    {

      x16 = _mm256_loadu_si256((const __m256i*)(&(dst[j])));
      _mm256_storeu_si256((__m256i*)(dst + j - i*stride), x16);
      x16 = _mm256_loadu_si256((const __m256i*)(dst + j + (height - 1)*stride));
      _mm256_storeu_si256((__m256i*)(dst + j + (height - 1 + i)*stride), x16);


      j = j + 16;
      temp = temp - 16;
    }
#endif
    while ((temp >> 3) > 0)
    {

      x = _mm_loadu_si128((const __m128i*)(&(dst[j])));
      _mm_storeu_si128((__m128i*)(dst + j - i*stride), x);
      x = _mm_loadu_si128((const __m128i*)(dst + j + (height - 1)*stride));
      _mm_storeu_si128((__m128i*)(dst + j + (height - 1 + i)*stride), x);

      j = j + 8;
      temp = temp - 8;
    }
    while ((temp >> 2) > 0)
    {
      x = _mm_loadl_epi64((const __m128i*)(&dst[j]));
      _mm_storel_epi64((__m128i*)(dst + j - i*stride), x);
      x = _mm_loadl_epi64((const __m128i*)(dst + j + (height - 1)*stride));
      _mm_storel_epi64((__m128i*)(dst + j + (height - 1 + i)*stride), x);

      j = j + 4;
      temp = temp - 4;
    }
    while (temp > 0)
    {
      dst[j - i*stride] = dst[j];
      dst[j + (height - 1 + i)*stride] = dst[j + (height - 1)*stride];
      j++;
      temp--;
    }
  }


  //Left and Right Padding
  Pel* ptr1 = dst - padSize*stride;
  Pel* ptr2 = dst - padSize*stride + width - 1;
  int offset = 0;
  for (int i = 0; i < height + 2 * padSize; i++)
  {
    offset = stride * i;
    for (int j = 1; j <= padSize; j++)
    {
      *(ptr1 - j + offset) = *(ptr1 + offset);
      *(ptr2 + j + offset) = *(ptr2 + offset);
    }

  }
}
#endif
template< X86_VEXT vext >
void addBIOAvg4_SSE(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
{
  __m128i mm_tmpx = _mm_unpacklo_epi64(_mm_set1_epi16(tmpx), _mm_set1_epi16(tmpy));
  __m128i mm_boffset = _mm_set1_epi32(1);
  __m128i mm_offset = _mm_set1_epi32(offset);
  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x += 4)
    {
      __m128i mm_a = _mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(gradX0 + x)), _mm_loadl_epi64((const __m128i *)(gradY0 + x)));
      __m128i mm_b = _mm_unpacklo_epi64(_mm_loadl_epi64((const __m128i *)(gradX1 + x)), _mm_loadl_epi64((const __m128i *)(gradY1 + x)));
      mm_a = _mm_sub_epi16(mm_a, mm_b);
      mm_b = _mm_mulhi_epi16(mm_a, mm_tmpx);
      mm_a = _mm_mullo_epi16(mm_a, mm_tmpx);

      __m128i mm_sum = _mm_add_epi32(_mm_unpacklo_epi16(mm_a, mm_b), _mm_unpackhi_epi16(mm_a, mm_b));
      mm_sum = _mm_srai_epi32(_mm_add_epi32(mm_sum, mm_boffset), 1);
      mm_a = _mm_cvtepi16_epi32(_mm_loadl_epi64((const __m128i *)(src0 + x)));
      mm_b = _mm_cvtepi16_epi32(_mm_loadl_epi64((const __m128i *)(src1 + x)));
      mm_sum = _mm_add_epi32(_mm_add_epi32(mm_sum, mm_a), _mm_add_epi32(mm_b, mm_offset));
      mm_sum = _mm_packs_epi32(_mm_srai_epi32(mm_sum, shift), mm_a);
      mm_sum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, mm_sum));
      _mm_storel_epi64((__m128i *)(dst + x), mm_sum);
    }
    dst += dstStride;       src0 += src0Stride;     src1 += src1Stride;
    gradX0 += gradStride; gradX1 += gradStride; gradY0 += gradStride; gradY1 += gradStride;
  }
}

template< X86_VEXT vext >
#if JVET_M0063_BDOF_FIX
void gradFilter_SSE(Pel* src, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth)
#else
void gradFilter_SSE(Pel* src, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY)
#endif
{
  __m128i vzero = _mm_setzero_si128();
  Pel* srcTmp = src + srcStride + 1;
  Pel* gradXTmp = gradX + gradStride + 1;
  Pel* gradYTmp = gradY + gradStride + 1;

  int widthInside = width - 2 * BIO_EXTEND_SIZE;
  int heightInside = height - 2 * BIO_EXTEND_SIZE;
#if JVET_M0063_BDOF_FIX
  int shift1 = std::max<int>(2, (14 - bitDepth));
#endif


  assert((widthInside & 3) == 0);

  for (int y = 0; y < heightInside; y++)
  {
    int x = 0;
    for (; x < widthInside; x += 4)
    {
      __m128i mmPixTop = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i*)(srcTmp + x - srcStride)));
      __m128i mmPixBottom = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i*)(srcTmp + x + srcStride)));
      __m128i mmPixLeft = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i*)(srcTmp + x - 1)));
      __m128i mmPixRight = _mm_cvtepi16_epi32(_mm_loadl_epi64((__m128i*)(srcTmp + x + 1)));

#if JVET_M0063_BDOF_FIX
      __m128i mmGradVer = _mm_sra_epi32(_mm_sub_epi32(mmPixBottom, mmPixTop), _mm_cvtsi32_si128(shift1));
      __m128i mmGradHor = _mm_sra_epi32(_mm_sub_epi32(mmPixRight, mmPixLeft), _mm_cvtsi32_si128(shift1));
#else
      __m128i mmGradVer = _mm_srai_epi32(_mm_sub_epi32(mmPixBottom, mmPixTop), 4);
      __m128i mmGradHor = _mm_srai_epi32(_mm_sub_epi32(mmPixRight, mmPixLeft), 4);
#endif
      mmGradVer = _mm_packs_epi32(mmGradVer, vzero);
      mmGradHor = _mm_packs_epi32(mmGradHor, vzero);

      _mm_storel_epi64((__m128i *)(gradYTmp + x), mmGradVer);
      _mm_storel_epi64((__m128i *)(gradXTmp + x), mmGradHor);
    }

    gradXTmp += gradStride;
    gradYTmp += gradStride;
    srcTmp += srcStride;
  }

  gradXTmp = gradX + gradStride + 1;
  gradYTmp = gradY + gradStride + 1;
  for (int y = 0; y < heightInside; y++)
  {
    gradXTmp[-1] = gradXTmp[0];
    gradXTmp[widthInside] = gradXTmp[widthInside - 1];
    gradXTmp += gradStride;

    gradYTmp[-1] = gradYTmp[0];
    gradYTmp[widthInside] = gradYTmp[widthInside - 1];
    gradYTmp += gradStride;
  }

  gradXTmp = gradX + gradStride;
  gradYTmp = gradY + gradStride;
  ::memcpy(gradXTmp - gradStride, gradXTmp, sizeof(Pel)*(width));
  ::memcpy(gradXTmp + heightInside*gradStride, gradXTmp + (heightInside - 1)*gradStride, sizeof(Pel)*(width));
  ::memcpy(gradYTmp - gradStride, gradYTmp, sizeof(Pel)*(width));
  ::memcpy(gradYTmp + heightInside*gradStride, gradYTmp + (heightInside - 1)*gradStride, sizeof(Pel)*(width));
}

template< X86_VEXT vext >
#if JVET_M0063_BDOF_FIX
void calcBIOPar_SSE(const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG, const int bitDepth)
#else
void calcBIOPar_SSE(const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG)
#endif
{
#if JVET_M0063_BDOF_FIX
  int shift4 = std::min<int>(8, (bitDepth - 4));
  int shift5 = std::min<int>(5, (bitDepth - 7));
#endif
  for (int y = 0; y < heightG; y++)
  {
    int x = 0;
    for (; x < ((widthG >> 3) << 3); x += 8)
    {
#if JVET_M0063_BDOF_FIX
      __m128i mmSrcY0Temp = _mm_sra_epi16(_mm_loadu_si128((__m128i*)(srcY0Temp + x)), _mm_cvtsi32_si128(shift4));
      __m128i mmSrcY1Temp = _mm_sra_epi16(_mm_loadu_si128((__m128i*)(srcY1Temp + x)), _mm_cvtsi32_si128(shift4));
#else
      __m128i mmSrcY0Temp = _mm_srai_epi16(_mm_loadu_si128((__m128i*)(srcY0Temp + x)), 6);
      __m128i mmSrcY1Temp = _mm_srai_epi16(_mm_loadu_si128((__m128i*)(srcY1Temp + x)), 6);
#endif
      __m128i mmGradX0 = _mm_loadu_si128((__m128i*)(gradX0 + x));
      __m128i mmGradX1 = _mm_loadu_si128((__m128i*)(gradX1 + x));
      __m128i mmGradY0 = _mm_loadu_si128((__m128i*)(gradY0 + x));
      __m128i mmGradY1 = _mm_loadu_si128((__m128i*)(gradY1 + x));

      __m128i mmTemp1 = _mm_sub_epi16(mmSrcY1Temp, mmSrcY0Temp);
#if JVET_M0063_BDOF_FIX
      __m128i mmTempX = _mm_sra_epi16(_mm_add_epi16(mmGradX0, mmGradX1), _mm_cvtsi32_si128(shift5));
      __m128i mmTempY = _mm_sra_epi16(_mm_add_epi16(mmGradY0, mmGradY1), _mm_cvtsi32_si128(shift5));
#else
      __m128i mmTempX = _mm_srai_epi16(_mm_add_epi16(mmGradX0, mmGradX1), 3);
      __m128i mmTempY = _mm_srai_epi16(_mm_add_epi16(mmGradY0, mmGradY1), 3);
#endif

      // m_piDotProductTemp1
      __m128i mm_b = _mm_mulhi_epi16(mmTempX, mmTempX);
      __m128i mm_a = _mm_mullo_epi16(mmTempX, mmTempX);

      __m128i mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
      __m128i mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp1 + x), mm_l);
      _mm_storeu_si128((__m128i *)(dotProductTemp1 + x + 4), mm_h);

      // m_piDotProductTemp2
      mm_b = _mm_mulhi_epi16(mmTempX, mmTempY);
      mm_a = _mm_mullo_epi16(mmTempX, mmTempY);

      mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
      mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp2 + x), mm_l);
      _mm_storeu_si128((__m128i *)(dotProductTemp2 + x + 4), mm_h);

      // m_piDotProductTemp3
      mm_b = _mm_mulhi_epi16(mmTempX, mmTemp1);
      mm_a = _mm_mullo_epi16(mmTempX, mmTemp1);

      mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
      mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp3 + x), mm_l);
      _mm_storeu_si128((__m128i *)(dotProductTemp3 + x + 4), mm_h);

      // m_piDotProductTemp5
      mm_b = _mm_mulhi_epi16(mmTempY, mmTempY);
      mm_a = _mm_mullo_epi16(mmTempY, mmTempY);

      mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
      mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp5 + x), mm_l);
      _mm_storeu_si128((__m128i *)(dotProductTemp5 + x + 4), mm_h);

      // m_piDotProductTemp6
      mm_b = _mm_mulhi_epi16(mmTempY, mmTemp1);
      mm_a = _mm_mullo_epi16(mmTempY, mmTemp1);

      mm_l = _mm_unpacklo_epi16(mm_a, mm_b);
      mm_h = _mm_unpackhi_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp6 + x), mm_l);
      _mm_storeu_si128((__m128i *)(dotProductTemp6 + x + 4), mm_h);
    }

    for (; x < ((widthG >> 2) << 2); x += 4)
    {
#if JVET_M0063_BDOF_FIX
      __m128i mmSrcY0Temp = _mm_sra_epi16(_mm_loadl_epi64((__m128i*)(srcY0Temp + x)), _mm_cvtsi32_si128(shift4));
      __m128i mmSrcY1Temp = _mm_sra_epi16(_mm_loadl_epi64((__m128i*)(srcY1Temp + x)), _mm_cvtsi32_si128(shift4));
#else
      __m128i mmSrcY0Temp = _mm_srai_epi16(_mm_loadl_epi64((__m128i*)(srcY0Temp + x)), 6);
      __m128i mmSrcY1Temp = _mm_srai_epi16(_mm_loadl_epi64((__m128i*)(srcY1Temp + x)), 6);
#endif
      __m128i mmGradX0 = _mm_loadl_epi64((__m128i*)(gradX0 + x));
      __m128i mmGradX1 = _mm_loadl_epi64((__m128i*)(gradX1 + x));
      __m128i mmGradY0 = _mm_loadl_epi64((__m128i*)(gradY0 + x));
      __m128i mmGradY1 = _mm_loadl_epi64((__m128i*)(gradY1 + x));

      __m128i mmTemp1 = _mm_sub_epi16(mmSrcY1Temp, mmSrcY0Temp);
#if JVET_M0063_BDOF_FIX
      __m128i mmTempX = _mm_sra_epi16(_mm_add_epi16(mmGradX0, mmGradX1), _mm_cvtsi32_si128(shift5));
      __m128i mmTempY = _mm_sra_epi16(_mm_add_epi16(mmGradY0, mmGradY1), _mm_cvtsi32_si128(shift5));
#else
      __m128i mmTempX = _mm_srai_epi16(_mm_add_epi16(mmGradX0, mmGradX1), 3);
      __m128i mmTempY = _mm_srai_epi16(_mm_add_epi16(mmGradY0, mmGradY1), 3);
#endif

      // m_piDotProductTemp1
      __m128i mm_b = _mm_mulhi_epi16(mmTempX, mmTempX);
      __m128i mm_a = _mm_mullo_epi16(mmTempX, mmTempX);
      __m128i mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp1 + x), mm_l);

      // m_piDotProductTemp2
      mm_b = _mm_mulhi_epi16(mmTempX, mmTempY);
      mm_a = _mm_mullo_epi16(mmTempX, mmTempY);
      mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp2 + x), mm_l);

      // m_piDotProductTemp3
      mm_b = _mm_mulhi_epi16(mmTempX, mmTemp1);
      mm_a = _mm_mullo_epi16(mmTempX, mmTemp1);
      mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp3 + x), mm_l);

      // m_piDotProductTemp5
      mm_b = _mm_mulhi_epi16(mmTempY, mmTempY);
      mm_a = _mm_mullo_epi16(mmTempY, mmTempY);
      mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp5 + x), mm_l);

      // m_piDotProductTemp6
      mm_b = _mm_mulhi_epi16(mmTempY, mmTemp1);
      mm_a = _mm_mullo_epi16(mmTempY, mmTemp1);
      mm_l = _mm_unpacklo_epi16(mm_a, mm_b);

      _mm_storeu_si128((__m128i *)(dotProductTemp6 + x), mm_l);
    }

    for (; x < widthG; x++)
    {
#if JVET_M0063_BDOF_FIX
      int temp = (srcY0Temp[x] >> shift4) - (srcY1Temp[x] >> shift4);
      int tempX = (gradX0[x] + gradX1[x]) >> shift5;
      int tempY = (gradY0[x] + gradY1[x]) >> shift5;
#else
      int temp = (srcY0Temp[x] >> 6) - (srcY1Temp[x] >> 6);
      int tempX = (gradX0[x] + gradX1[x]) >> 3;
      int tempY = (gradY0[x] + gradY1[x]) >> 3;
#endif
      dotProductTemp1[x] = tempX * tempX;
      dotProductTemp2[x] = tempX * tempY;
      dotProductTemp3[x] = -tempX * temp;
      dotProductTemp5[x] = tempY * tempY;
      dotProductTemp6[x] = -tempY * temp;
    }

    srcY0Temp += src0Stride;
    srcY1Temp += src1Stride;
    gradX0 += gradStride;
    gradX1 += gradStride;
    gradY0 += gradStride;
    gradY1 += gradStride;
    dotProductTemp1 += widthG;
    dotProductTemp2 += widthG;
    dotProductTemp3 += widthG;
    dotProductTemp5 += widthG;
    dotProductTemp6 += widthG;
  }
}

template< X86_VEXT vext >
void calcBlkGradient_SSE(int sx, int sy, int     *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize)
{
  int     *Gx2 = arraysGx2;
  int     *Gy2 = arraysGy2;
  int     *GxGy = arraysGxGy;
  int     *GxdI = arraysGxdI;
  int     *GydI = arraysGydI;

  // set to the above row due to JVET_K0485_BIO_EXTEND_SIZE
  Gx2 -= (BIO_EXTEND_SIZE*width);
  Gy2 -= (BIO_EXTEND_SIZE*width);
  GxGy -= (BIO_EXTEND_SIZE*width);
  GxdI -= (BIO_EXTEND_SIZE*width);
  GydI -= (BIO_EXTEND_SIZE*width);

  __m128i vzero = _mm_setzero_si128();
  __m128i mmGx2Total = _mm_setzero_si128();
  __m128i mmGy2Total = _mm_setzero_si128();
  __m128i mmGxGyTotal = _mm_setzero_si128();
  __m128i mmGxdITotal = _mm_setzero_si128();
  __m128i mmGydITotal = _mm_setzero_si128();

  for (int y = -BIO_EXTEND_SIZE; y < unitSize + BIO_EXTEND_SIZE; y++)
  {
    __m128i mmsGx2 = _mm_loadu_si128((__m128i*)(Gx2 - 1));   __m128i mmsGx2Sec = _mm_loadl_epi64((__m128i*)(Gx2 + 3));
    __m128i mmsGy2 = _mm_loadu_si128((__m128i*)(Gy2 - 1));   __m128i mmsGy2Sec = _mm_loadl_epi64((__m128i*)(Gy2 + 3));
    __m128i mmsGxGy = _mm_loadu_si128((__m128i*)(GxGy - 1));  __m128i mmsGxGySec = _mm_loadl_epi64((__m128i*)(GxGy + 3));
    __m128i mmsGxdI = _mm_loadu_si128((__m128i*)(GxdI - 1));  __m128i mmsGxdISec = _mm_loadl_epi64((__m128i*)(GxdI + 3));
    __m128i mmsGydI = _mm_loadu_si128((__m128i*)(GydI - 1));  __m128i mmsGydISec = _mm_loadl_epi64((__m128i*)(GydI + 3));

    mmsGx2 = _mm_add_epi32(mmsGx2, mmsGx2Sec);
    mmsGy2 = _mm_add_epi32(mmsGy2, mmsGy2Sec);
    mmsGxGy = _mm_add_epi32(mmsGxGy, mmsGxGySec);
    mmsGxdI = _mm_add_epi32(mmsGxdI, mmsGxdISec);
    mmsGydI = _mm_add_epi32(mmsGydI, mmsGydISec);


    mmGx2Total = _mm_add_epi32(mmGx2Total, mmsGx2);
    mmGy2Total = _mm_add_epi32(mmGy2Total, mmsGy2);
    mmGxGyTotal = _mm_add_epi32(mmGxGyTotal, mmsGxGy);
    mmGxdITotal = _mm_add_epi32(mmGxdITotal, mmsGxdI);
    mmGydITotal = _mm_add_epi32(mmGydITotal, mmsGydI);

    Gx2 += width;
    Gy2 += width;
    GxGy += width;
    GxdI += width;
    GydI += width;
  }

  mmGx2Total = _mm_hadd_epi32(_mm_hadd_epi32(mmGx2Total, vzero), vzero);
  mmGy2Total = _mm_hadd_epi32(_mm_hadd_epi32(mmGy2Total, vzero), vzero);
  mmGxGyTotal = _mm_hadd_epi32(_mm_hadd_epi32(mmGxGyTotal, vzero), vzero);
  mmGxdITotal = _mm_hadd_epi32(_mm_hadd_epi32(mmGxdITotal, vzero), vzero);
  mmGydITotal = _mm_hadd_epi32(_mm_hadd_epi32(mmGydITotal, vzero), vzero);

  sGx2 = _mm_cvtsi128_si32(mmGx2Total);
  sGy2 = _mm_cvtsi128_si32(mmGy2Total);
  sGxGy = _mm_cvtsi128_si32(mmGxGyTotal);
  sGxdI = _mm_cvtsi128_si32(mmGxdITotal);
  sGydI = _mm_cvtsi128_si32(mmGydITotal);
}

template< X86_VEXT vext, int W >
void reco_SSE( const int16_t* src0, int src0Stride, const int16_t* src1, int src1Stride, int16_t *dst, int dstStride, int width, int height, const ClpRng& clpRng )
{
  if( W == 8 )
  {
    if( vext >= AVX2 && ( width & 15 ) == 0 )
    {
#if USE_AVX2
      __m256i vbdmin = _mm256_set1_epi16( clpRng.min );
      __m256i vbdmax = _mm256_set1_epi16( clpRng.max );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 16 )
        {
          __m256i vdest = _mm256_lddqu_si256( ( const __m256i * )&src0[col] );
          __m256i vsrc1 = _mm256_lddqu_si256( ( const __m256i * )&src1[col] );

          vdest = _mm256_adds_epi16( vdest, vsrc1 );
          vdest = _mm256_min_epi16( vbdmax, _mm256_max_epi16( vbdmin, vdest ) );

          _mm256_storeu_si256( ( __m256i * )&dst[col], vdest );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  += dstStride;
      }
#endif
    }
    else
    {
      __m128i vbdmin = _mm_set1_epi16( clpRng.min );
      __m128i vbdmax = _mm_set1_epi16( clpRng.max );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 8 )
        {
          __m128i vdest = _mm_loadu_si128( ( const __m128i * )&src0[col] );
          __m128i vsrc1 = _mm_loadu_si128( ( const __m128i * )&src1[col] );

          vdest = _mm_adds_epi16( vdest, vsrc1 );
          vdest = _mm_min_epi16( vbdmax, _mm_max_epi16( vbdmin, vdest ) );

          _mm_storeu_si128( ( __m128i * )&dst[col], vdest );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  += dstStride;
      }
    }
  }
  else if( W == 4 )
  {
    __m128i vbdmin = _mm_set1_epi16( clpRng.min );
    __m128i vbdmax = _mm_set1_epi16( clpRng.max );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vsrc = _mm_loadl_epi64( ( const __m128i * )&src0[col] );
        __m128i vdst = _mm_loadl_epi64( ( const __m128i * )&src1[col] );

        vdst = _mm_adds_epi16( vdst, vsrc );
        vdst = _mm_min_epi16( vbdmax, _mm_max_epi16( vbdmin, vdst ) );

        _mm_storel_epi64( ( __m128i * )&dst[col], vdst );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
}

#if ENABLE_SIMD_OPT_GBI
template< X86_VEXT vext, int W >
void removeWeightHighFreq_SSE(int16_t* src0, int src0Stride, const int16_t* src1, int src1Stride, int width, int height, int shift, int gbiWeight)
{
  int normalizer = ((1 << 16) + (gbiWeight>0 ? (gbiWeight >> 1) : -(gbiWeight >> 1))) / gbiWeight;
  int weight0 = normalizer << g_GbiLog2WeightBase;
  int weight1 = (g_GbiWeightBase - gbiWeight)*normalizer;
  int offset = 1 << (shift - 1);
  if (W == 8)
  {
    __m128i vzero = _mm_setzero_si128();
    __m128i voffset = _mm_set1_epi32(offset);
    __m128i vw0 = _mm_set1_epi32(weight0);
    __m128i vw1 = _mm_set1_epi32(weight1);

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 8)
      {
        __m128i vsrc0 = _mm_load_si128((const __m128i *)&src0[col]);
        __m128i vsrc1 = _mm_load_si128((const __m128i *)&src1[col]);

        __m128i vtmp, vdst, vsrc;
        vdst = _mm_cvtepi16_epi32(vsrc0);
        vsrc = _mm_cvtepi16_epi32(vsrc1);
        vdst = _mm_mullo_epi32(vdst, vw0);
        vsrc = _mm_mullo_epi32(vsrc, vw1);
        vtmp = _mm_add_epi32(_mm_sub_epi32(vdst, vsrc), voffset);
        vtmp = _mm_srai_epi32(vtmp, shift);

        vsrc0 = _mm_unpackhi_epi64(vsrc0, vzero);
        vsrc1 = _mm_unpackhi_epi64(vsrc1, vzero);
        vdst = _mm_cvtepi16_epi32(vsrc0);
        vsrc = _mm_cvtepi16_epi32(vsrc1);
        vdst = _mm_mullo_epi32(vdst, vw0);
        vsrc = _mm_mullo_epi32(vsrc, vw1);
        vdst = _mm_add_epi32(_mm_sub_epi32(vdst, vsrc), voffset);
        vdst = _mm_srai_epi32(vdst, shift);
        vdst = _mm_packs_epi32(vtmp, vdst);

        _mm_store_si128((__m128i *)&src0[col], vdst);
      }
      src0 += src0Stride;
      src1 += src1Stride;
    }
  }
  else if (W == 4)
  {
    __m128i vzero = _mm_setzero_si128();
    __m128i voffset = _mm_set1_epi32(offset);
    __m128i vw0 = _mm_set1_epi32(weight0);
    __m128i vw1 = _mm_set1_epi32(weight1);

    for (int row = 0; row < height; row++)
    {
      __m128i vsum = _mm_loadl_epi64((const __m128i *)src0);
      __m128i vdst = _mm_loadl_epi64((const __m128i *)src1);

      vsum = _mm_cvtepi16_epi32(vsum);
      vdst = _mm_cvtepi16_epi32(vdst);
      vsum = _mm_mullo_epi32(vsum, vw0);
      vdst = _mm_mullo_epi32(vdst, vw1);
      vsum = _mm_add_epi32(_mm_sub_epi32(vsum, vdst), voffset);
      vsum = _mm_srai_epi32(vsum, shift);
      vsum = _mm_packs_epi32(vsum, vzero);

      _mm_storel_epi64((__m128i *)src0, vsum);

      src0 += src0Stride;
      src1 += src1Stride;
    }
  }
  else
  {
    THROW("Unsupported size");
  }
}

template< X86_VEXT vext, int W >
void removeHighFreq_SSE(int16_t* src0, int src0Stride, const int16_t* src1, int src1Stride, int width, int height)
{
  if (W == 8)
  {
    // TODO: AVX2 impl
    {
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col += 8)
        {
          __m128i vsrc0 = _mm_load_si128((const __m128i *)&src0[col]);
          __m128i vsrc1 = _mm_load_si128((const __m128i *)&src1[col]);

          vsrc0 = _mm_sub_epi16(_mm_slli_epi16(vsrc0, 1), vsrc1);
          _mm_store_si128((__m128i *)&src0[col], vsrc0);
        }

        src0 += src0Stride;
        src1 += src1Stride;
      }
    }
  }
  else if (W == 4)
  {
    for (int row = 0; row < height; row += 2)
    {
      __m128i vsrc0 = _mm_loadl_epi64((const __m128i *)src0);
      __m128i vsrc1 = _mm_loadl_epi64((const __m128i *)src1);
      __m128i vsrc0_2 = _mm_loadl_epi64((const __m128i *)(src0 + src0Stride));
      __m128i vsrc1_2 = _mm_loadl_epi64((const __m128i *)(src1 + src1Stride));

      vsrc0 = _mm_unpacklo_epi64(vsrc0, vsrc0_2);
      vsrc1 = _mm_unpacklo_epi64(vsrc1, vsrc1_2);

      vsrc0 = _mm_sub_epi16(_mm_slli_epi16(vsrc0, 1), vsrc1);
      _mm_storel_epi64((__m128i *)src0, vsrc0);
      _mm_storel_epi64((__m128i *)(src0 + src0Stride), _mm_unpackhi_epi64(vsrc0, vsrc0));

      src0 += (src0Stride << 1);
      src1 += (src1Stride << 1);
    }
  }
  else
  {
    THROW("Unsupported size");
  }
}
#endif

template<bool doShift, bool shiftR, typename T> static inline void do_shift( T &vreg, int num );
#if USE_AVX2
template<> inline void do_shift<true,  true , __m256i>( __m256i &vreg, int num ) { vreg = _mm256_srai_epi32( vreg, num ); }
template<> inline void do_shift<true,  false, __m256i>( __m256i &vreg, int num ) { vreg = _mm256_slli_epi32( vreg, num ); }
template<> inline void do_shift<false, true , __m256i>( __m256i &vreg, int num ) { }
template<> inline void do_shift<false, false, __m256i>( __m256i &vreg, int num ) { }
#endif
template<> inline void do_shift<true,  true , __m128i>( __m128i &vreg, int num ) { vreg = _mm_srai_epi32( vreg, num ); }
template<> inline void do_shift<true,  false, __m128i>( __m128i &vreg, int num ) { vreg = _mm_slli_epi32( vreg, num ); }
template<> inline void do_shift<false, true , __m128i>( __m128i &vreg, int num ) { }
template<> inline void do_shift<false, false, __m128i>( __m128i &vreg, int num ) { }

template<bool mult, typename T> static inline void do_mult( T& vreg, T& vmult );
template<> inline void do_mult<false, __m128i>( __m128i&, __m128i& ) { }
#if USE_AVX2
template<> inline void do_mult<false, __m256i>( __m256i&, __m256i& ) { }
#endif
template<> inline void do_mult<true,   __m128i>( __m128i& vreg, __m128i& vmult ) { vreg = _mm_mullo_epi32   ( vreg, vmult ); }
#if USE_AVX2
template<> inline void do_mult<true,   __m256i>( __m256i& vreg, __m256i& vmult ) { vreg = _mm256_mullo_epi32( vreg, vmult ); }
#endif

template<bool add, typename T> static inline void do_add( T& vreg, T& vadd );
template<> inline void do_add<false, __m128i>( __m128i&, __m128i& ) { }
#if USE_AVX2
template<> inline void do_add<false, __m256i>( __m256i&, __m256i& ) { }
#endif
template<> inline void do_add<true,  __m128i>( __m128i& vreg, __m128i& vadd ) { vreg = _mm_add_epi32( vreg, vadd ); }
#if USE_AVX2
template<> inline void do_add<true,  __m256i>( __m256i& vreg, __m256i& vadd ) { vreg = _mm256_add_epi32( vreg, vadd ); }
#endif

template<bool clip, typename T> static inline void do_clip( T& vreg, T& vbdmin, T& vbdmax );
template<> inline void do_clip<false, __m128i>( __m128i&, __m128i&, __m128i& ) { }
#if USE_AVX2
template<> inline void do_clip<false, __m256i>( __m256i&, __m256i&, __m256i& ) { }
#endif
template<> inline void do_clip<true,  __m128i>( __m128i& vreg, __m128i& vbdmin, __m128i& vbdmax ) { vreg = _mm_min_epi16   ( vbdmax, _mm_max_epi16   ( vbdmin, vreg ) ); }
#if USE_AVX2
template<> inline void do_clip<true,  __m256i>( __m256i& vreg, __m256i& vbdmin, __m256i& vbdmax ) { vreg = _mm256_min_epi16( vbdmax, _mm256_max_epi16( vbdmin, vreg ) ); }
#endif


template<X86_VEXT vext, int W, bool doAdd, bool mult, bool doShift, bool shiftR, bool clip>
void linTf_SSE( const Pel* src, int srcStride, Pel *dst, int dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng )
{
  if( vext >= AVX2 && ( width & 7 ) == 0 && W == 8 )
  {
#if USE_AVX2
    __m256i vzero    = _mm256_setzero_si256();
    __m256i vbdmin   = _mm256_set1_epi16( clpRng.min );
    __m256i vbdmax   = _mm256_set1_epi16( clpRng.max );
    __m256i voffset  = _mm256_set1_epi32( offset );
    __m256i vscale   = _mm256_set1_epi32( scale );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m256i val;
        val = _mm256_cvtepi16_epi32       (  _mm_loadu_si128( ( const __m128i * )&src[col] ) );
        do_mult<mult, __m256i>            ( val, vscale );
        do_shift<doShift, shiftR, __m256i>( val, shift );
        do_add<doAdd, __m256i>            ( val, voffset );
        val = _mm256_packs_epi32          ( val, vzero );
        do_clip<clip, __m256i>            ( val, vbdmin, vbdmax );
        val = _mm256_permute4x64_epi64    ( val, ( 0 << 0 ) + ( 2 << 2 ) + ( 1 << 4 ) + ( 1 << 6 ) );

        _mm_storeu_si128                  ( ( __m128i * )&dst[col], _mm256_castsi256_si128( val ) );
      }

      src += srcStride;
      dst += dstStride;
    }
#endif
  }
  else
  {
    __m128i vzero   = _mm_setzero_si128();
    __m128i vbdmin  = _mm_set1_epi16   ( clpRng.min );
    __m128i vbdmax  = _mm_set1_epi16   ( clpRng.max );
    __m128i voffset = _mm_set1_epi32   ( offset );
    __m128i vscale  = _mm_set1_epi32   ( scale );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i val;
        val = _mm_loadl_epi64             ( ( const __m128i * )&src[col] );
        val = _mm_cvtepi16_epi32          ( val );
        do_mult<mult, __m128i>            ( val, vscale );
        do_shift<doShift, shiftR, __m128i>( val, shift );
        do_add<doAdd, __m128i>            ( val, voffset );
        val = _mm_packs_epi32             ( val, vzero );
        do_clip<clip, __m128i>            ( val, vbdmin, vbdmax );

        _mm_storel_epi64                  ( ( __m128i * )&dst[col], val );
      }

      src += srcStride;
      dst += dstStride;
    }
  }
}

template<X86_VEXT vext, int W>
void linTf_SSE_entry( const Pel* src, int srcStride, Pel *dst, int dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng, bool clip )
{
  int fn = ( offset == 0 ? 16 : 0 ) + ( scale == 1 ? 8 : 0 ) + ( shift == 0 ? 4 : 0 ) + ( shift < 0 ? 2 : 0 ) + ( !clip ? 1 : 0 );

  switch( fn )
  {
  case  0: linTf_SSE<vext, W, true,  true,  true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  1: linTf_SSE<vext, W, true,  true,  true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  2: linTf_SSE<vext, W, true,  true,  true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  3: linTf_SSE<vext, W, true,  true,  true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  4: linTf_SSE<vext, W, true,  true,  false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  5: linTf_SSE<vext, W, true,  true,  false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  6: linTf_SSE<vext, W, true,  true,  false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  7: linTf_SSE<vext, W, true,  true,  false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case  8: linTf_SSE<vext, W, true,  false, true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case  9: linTf_SSE<vext, W, true,  false, true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 10: linTf_SSE<vext, W, true,  false, true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 11: linTf_SSE<vext, W, true,  false, true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 12: linTf_SSE<vext, W, true,  false, false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 13: linTf_SSE<vext, W, true,  false, false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 14: linTf_SSE<vext, W, true,  false, false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 15: linTf_SSE<vext, W, true,  false, false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 16: linTf_SSE<vext, W, false, true,  true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 17: linTf_SSE<vext, W, false, true,  true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 18: linTf_SSE<vext, W, false, true,  true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 19: linTf_SSE<vext, W, false, true,  true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 20: linTf_SSE<vext, W, false, true,  false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 21: linTf_SSE<vext, W, false, true,  false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 22: linTf_SSE<vext, W, false, true,  false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 23: linTf_SSE<vext, W, false, true,  false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 24: linTf_SSE<vext, W, false, false, true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 25: linTf_SSE<vext, W, false, false, true,  true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 26: linTf_SSE<vext, W, false, false, true,  false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 27: linTf_SSE<vext, W, false, false, true,  false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 28: linTf_SSE<vext, W, false, false, false, true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 29: linTf_SSE<vext, W, false, false, false, true,  false>( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng ); break;
  case 30: linTf_SSE<vext, W, false, false, false, false, true >( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  case 31: linTf_SSE<vext, W, false, false, false, false, false>( src, srcStride, dst, dstStride, width, height, scale, -shift, offset, clpRng ); break;
  default:
    THROW( "Unknown parametrization of the linear transformation" );
    break;
  }
}

template<X86_VEXT vext>
void PelBufferOps::_initPelBufOpsX86()
{
  addAvg8 = addAvg_SSE<vext, 8>;
  addAvg4 = addAvg_SSE<vext, 4>;

  addBIOAvg4      = addBIOAvg4_SSE<vext>;
  bioGradFilter   = gradFilter_SSE<vext>;
  calcBIOPar      = calcBIOPar_SSE<vext>;
  calcBlkGradient = calcBlkGradient_SSE<vext>;

#if JVET_M0147_DMVR
  copyBuffer = copyBufferSimd<vext>;
  padding    = paddingSimd<vext>;
#endif
  reco8 = reco_SSE<vext, 8>;
  reco4 = reco_SSE<vext, 4>;

  linTf8 = linTf_SSE_entry<vext, 8>;
  linTf4 = linTf_SSE_entry<vext, 4>;
#if ENABLE_SIMD_OPT_GBI
  removeWeightHighFreq8 = removeWeightHighFreq_SSE<vext, 8>;
  removeWeightHighFreq4 = removeWeightHighFreq_SSE<vext, 4>;
  removeHighFreq8 = removeHighFreq_SSE<vext, 8>;
  removeHighFreq4 = removeHighFreq_SSE<vext, 4>;
#endif
}

template void PelBufferOps::_initPelBufOpsX86<SIMDX86>();

#endif // TARGET_SIMD_X86
#endif
//! \}
