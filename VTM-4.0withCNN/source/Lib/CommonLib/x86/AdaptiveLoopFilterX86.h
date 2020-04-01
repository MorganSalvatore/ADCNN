/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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

/** \file     AdaptiveLoopFilterX86.h
    \brief    adaptive loop filter class
*/
#include "CommonDefX86.h"
#include "../AdaptiveLoopFilter.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86
#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

template<X86_VEXT vext>
static void simdDeriveClassificationBlk( AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blk, const int shift )
{
  const int img_stride = srcLuma.stride;
  const Pel* srcExt = srcLuma.buf;

  const int fl = 2;
  const int flplusOne = fl + 1;
  const int fl2plusTwo = 2 * fl + 2;
  const int var_max = 15;

  const int imgHExtended = blk.height + fl2plusTwo;
  const int imgWExtended = blk.width + fl2plusTwo;

  const int posX = blk.pos().x;
  const int posY = blk.pos().y;
  const int start_height1 = posY - flplusOne;

  static uint16_t _temp[( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 4 ) >> 1][AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 4];

  for( int i = 0; i < imgHExtended - 2; i += 2 )
  {
    int yoffset = ( i + 1 + start_height1 ) * img_stride - flplusOne;

    const Pel *p_imgY_pad_down = &srcExt[yoffset - img_stride];
    const Pel *p_imgY_pad = &srcExt[yoffset];
    const Pel *p_imgY_pad_up = &srcExt[yoffset + img_stride];
    const Pel *p_imgY_pad_up2 = &srcExt[yoffset + img_stride * 2];

    __m128i mmStore = _mm_setzero_si128();

    for( int j = 2; j < imgWExtended; j += 8 )
    {
      const int pixY = j - 1 + posX;

      const __m128i* pY = ( __m128i* )( p_imgY_pad + pixY - 1 );
      const __m128i* pYdown = ( __m128i* )( p_imgY_pad_down + pixY - 1 );
      const __m128i* pYup = ( __m128i* )( p_imgY_pad_up + pixY - 1 );
      const __m128i* pYup2 = ( __m128i* )( p_imgY_pad_up2 + pixY - 1 );

      const __m128i* pY_next = ( __m128i* )( p_imgY_pad + pixY + 7 );
      const __m128i* pYdown_next = ( __m128i* )( p_imgY_pad_down + pixY + 7 );
      const __m128i* pYup_next = ( __m128i* )( p_imgY_pad_up + pixY + 7 );
      const __m128i* pYup2_next = ( __m128i* )( p_imgY_pad_up2 + pixY + 7 );

      __m128i xmm0 = _mm_loadu_si128( pYdown );
      __m128i xmm1 = _mm_loadu_si128( pY );
      __m128i xmm2 = _mm_loadu_si128( pYup );
      __m128i xmm3 = _mm_loadu_si128( pYup2 );

      const __m128i xmm0_next = _mm_loadu_si128( pYdown_next );
      const __m128i xmm1_next = _mm_loadu_si128( pY_next );
      const __m128i xmm2_next = _mm_loadu_si128( pYup_next );
      const __m128i xmm3_next = _mm_loadu_si128( pYup2_next );

      __m128i xmm4 = _mm_slli_epi16( _mm_alignr_epi8( xmm1_next, xmm1, 2 ), 1 );
      __m128i xmm5 = _mm_slli_epi16( _mm_alignr_epi8( xmm2_next, xmm2, 2 ), 1 );

      __m128i xmm15 = _mm_setzero_si128();

      //dig0
      __m128i xmm6 = _mm_add_epi16( _mm_alignr_epi8( xmm2_next, xmm2, 4 ), xmm0 );
      xmm6 = _mm_sub_epi16( _mm_blend_epi16 ( xmm4, xmm15, 0xAA ), _mm_blend_epi16 ( xmm6, xmm15, 0xAA ) );
      __m128i xmm8 = _mm_add_epi16( _mm_alignr_epi8( xmm3_next, xmm3, 4 ), xmm1 );
      xmm8 = _mm_sub_epi16( _mm_blend_epi16 ( xmm5, xmm15, 0x55 ), _mm_blend_epi16 ( xmm8, xmm15, 0x55 ) );

      //dig1
      __m128i xmm9 = _mm_add_epi16( _mm_alignr_epi8( xmm0_next, xmm0, 4 ), xmm2 );
      xmm9 = _mm_sub_epi16( _mm_blend_epi16 ( xmm4, xmm15, 0xAA ), _mm_blend_epi16 ( xmm9, xmm15, 0xAA ) );
      __m128i xmm10 = _mm_add_epi16( _mm_alignr_epi8( xmm1_next, xmm1, 4 ), xmm3 );
      xmm10 = _mm_sub_epi16( _mm_blend_epi16 ( xmm5, xmm15, 0x55 ), _mm_blend_epi16 ( xmm10, xmm15, 0x55 ) );

      //hor
      __m128i xmm13 = _mm_add_epi16( _mm_alignr_epi8( xmm1_next, xmm1, 4 ), xmm1 );
      xmm13 = _mm_sub_epi16( _mm_blend_epi16 ( xmm4, xmm15, 0xAA ), _mm_blend_epi16 ( xmm13, xmm15, 0xAA ) );
      __m128i xmm14 = _mm_add_epi16( _mm_alignr_epi8( xmm2_next, xmm2, 4 ), xmm2 );
      xmm14 = _mm_sub_epi16( _mm_blend_epi16 ( xmm5, xmm15, 0x55 ), _mm_blend_epi16 ( xmm14, xmm15, 0x55 ) );

      //ver
      __m128i xmm11 = _mm_add_epi16( _mm_alignr_epi8( xmm0_next, xmm0, 2 ), _mm_alignr_epi8( xmm2_next, xmm2, 2 ) );
      xmm11 = _mm_sub_epi16( _mm_blend_epi16 ( xmm4, xmm15, 0xAA ), _mm_blend_epi16 ( xmm11, xmm15, 0xAA ) );
      __m128i xmm12 = _mm_add_epi16( _mm_alignr_epi8( xmm1_next, xmm1, 2 ), _mm_alignr_epi8( xmm3_next, xmm3, 2 ) );
      xmm12 = _mm_sub_epi16( _mm_blend_epi16 ( xmm5, xmm15, 0x55 ), _mm_blend_epi16 ( xmm12, xmm15, 0x55 ) );

      xmm6 = _mm_abs_epi16( xmm6 );
      xmm8 = _mm_abs_epi16( xmm8 );
      xmm9 = _mm_abs_epi16( xmm9 );
      xmm10 = _mm_abs_epi16( xmm10 );
      xmm11 = _mm_abs_epi16( xmm11 );
      xmm12 = _mm_abs_epi16( xmm12 );
      xmm13 = _mm_abs_epi16( xmm13 );
      xmm14 = _mm_abs_epi16( xmm14 );

      xmm6 = _mm_add_epi16( xmm6, xmm8 );
      xmm9 = _mm_add_epi16( xmm9, xmm10 );
      xmm11 = _mm_add_epi16( xmm11, xmm12 );
      xmm13 = _mm_add_epi16( xmm13, xmm14 );

      xmm6 = _mm_add_epi16( xmm6, _mm_srli_si128( xmm6, 2 ) );
      xmm9 = _mm_add_epi16( xmm9, _mm_slli_si128( xmm9, 2 ) );
      xmm11 = _mm_add_epi16( xmm11, _mm_srli_si128( xmm11, 2 ) );
      xmm13 = _mm_add_epi16( xmm13, _mm_slli_si128( xmm13, 2 ) );

      xmm6 = _mm_blend_epi16( xmm6, xmm9, 0xAA );
      xmm11 = _mm_blend_epi16( xmm11, xmm13, 0xAA );

      xmm6 = _mm_add_epi16( xmm6, _mm_slli_si128( xmm6, 4 ) );
      xmm11 = _mm_add_epi16( xmm11, _mm_srli_si128( xmm11, 4 ) );

      xmm6 = _mm_blend_epi16( xmm11, xmm6, 0xCC );

      xmm9 = _mm_srli_si128( xmm6, 8 );

      if( j > 2 )
      {
        _mm_storel_epi64( ( __m128i* )( &( _temp[i >> 1][j - 2 - 4] ) ), _mm_add_epi16( xmm6, mmStore ) );
      }

      xmm6 = _mm_add_epi16( xmm6, xmm9 );  //V H D0 D1
      _mm_storel_epi64( ( __m128i* )( &( _temp[i >> 1][j - 2] ) ), xmm6 );

      mmStore = xmm9;
    }
  }

  //const int offset = 8 << NO_VALS_LAGR_SHIFT;

  const __m128i mm_0 = _mm_setzero_si128();
  const __m128i mm_15 = _mm_set1_epi64x( 0x000000000000000F );
  const __m128i mm_th = _mm_set1_epi64x( 0x4333333332222210 );

  const __m128i xmm14 = _mm_set1_epi32( 1 ); //offset
  const __m128i xmm13 = _mm_set1_epi32( var_max );

  for( int i = 0; i < ( blk.height >> 1 ); i += 2 )
  {
    for( int j = 0; j < blk.width; j += 8 )
    {
      __m128i xmm0 = _mm_loadu_si128( ( __m128i* )( &( _temp[i + 0][j] ) ) );
      __m128i xmm1 = _mm_loadu_si128( ( __m128i* )( &( _temp[i + 1][j] ) ) );
      __m128i xmm2 = _mm_loadu_si128( ( __m128i* )( &( _temp[i + 2][j] ) ) );
      __m128i xmm3 = _mm_loadu_si128( ( __m128i* )( &( _temp[i + 3][j] ) ) );

      __m128i xmm4 = _mm_add_epi16( xmm0, xmm1 );
      __m128i xmm6 = _mm_add_epi16( xmm2, xmm3 );

      xmm0 = _mm_unpackhi_epi16( xmm4, mm_0 );
      xmm2 = _mm_unpackhi_epi16( xmm6, mm_0 );
      xmm0 = _mm_add_epi32( xmm0, xmm2 );

      xmm4 = _mm_unpacklo_epi16( xmm4, mm_0 );
      xmm6 = _mm_unpacklo_epi16( xmm6, mm_0 );
      xmm4 = _mm_add_epi32( xmm4, xmm6 );

      __m128i xmm12 = _mm_blend_epi16( xmm4, _mm_shuffle_epi32( xmm0, 0x40 ), 0xF0 );
      __m128i xmm10 = _mm_shuffle_epi32( xmm12, 0xB1 );
      xmm12 = _mm_add_epi32( xmm10, xmm12 );
      xmm12 = _mm_srai_epi32( xmm12, shift - 6 );
      xmm12 = _mm_min_epi32( xmm12, xmm13 );

      xmm12 = _mm_and_si128( xmm12, mm_15 );
      xmm12 = _mm_slli_epi32( xmm12, 2 );
      __m128i xmm11 = _mm_shuffle_epi32( xmm12, 0x0E ); //extracted from second half coz no different shifts are available
      xmm12 = _mm_srl_epi64( mm_th, xmm12 );
      xmm11 = _mm_srl_epi64( mm_th, xmm11 );
      xmm12 = _mm_blend_epi16( xmm12, xmm11, 0xF0 );
      xmm12 = _mm_and_si128( xmm12, mm_15 ); // avg_var in lower 4 bits of both halves

      xmm6 = _mm_shuffle_epi32( xmm4, 0xB1 );
      xmm2 = _mm_shuffle_epi32( xmm0, 0xB1 );

      __m128i xmm7 = _mm_set_epi32( 0, 2, 1, 3 );
      __m128i xmm9 = _mm_shuffle_epi32( xmm7, 0xB1 );

      __m128i xmm5 = _mm_cmplt_epi32( xmm6, xmm4 );
      __m128i xmm8 = _mm_cmplt_epi32( xmm2, xmm0 ); //2 masks coz 4 integers for every parts are compared

      xmm5 = _mm_shuffle_epi32( xmm5, 0xA0 );
      xmm8 = _mm_shuffle_epi32( xmm8, 0xA0 );

      xmm4 = _mm_or_si128( _mm_andnot_si128( xmm5, xmm4 ), _mm_and_si128( xmm5, xmm6 ) ); //HV + D
      xmm0 = _mm_or_si128( _mm_andnot_si128( xmm8, xmm0 ), _mm_and_si128( xmm8, xmm2 ) ); //HV + D <--second part

      xmm10 = _mm_or_si128( _mm_andnot_si128( xmm8, xmm7 ), _mm_and_si128( xmm8, xmm9 ) ); //dirTemp <-- second part
      xmm7 = _mm_or_si128( _mm_andnot_si128( xmm5, xmm7 ), _mm_and_si128( xmm5, xmm9 ) ); //dirTemp

      xmm3 = _mm_shuffle_epi32( xmm0, 0x1B );  // need higher part from this
      xmm6 = _mm_shuffle_epi32( xmm4, 0x1B );
      xmm8 = _mm_blend_epi16( xmm4, xmm3, 0xF0 ); // 0 or 3
      xmm6 = _mm_blend_epi16( xmm6, xmm0, 0xF0 );

      xmm6 = _mm_mullo_epi32( xmm8, xmm6 );
      xmm9 = _mm_shuffle_epi32( xmm6, 0xB1 );
      xmm5 = _mm_cmpgt_epi32( xmm6, xmm9 );
      xmm5 = _mm_shuffle_epi32( xmm5, 0xF0 ); //second mask is for all upper part

      xmm8 = _mm_shuffle_epi32( xmm4, 0x0E );
      xmm8 = _mm_blend_epi16( xmm8, xmm0, 0xF0 ); // (DL, DH in upepr part)
      xmm4 = _mm_blend_epi16( xmm4, _mm_shuffle_epi32( xmm0, 0x40 ), 0xF0 ); //(HVL, HVH) in upper part

      xmm7 = _mm_shuffle_epi32( xmm7, 0x08 ); // 2 -> 1
      xmm7 = _mm_blend_epi16( xmm7, _mm_shuffle_epi32( xmm10, 0x80 ), 0xF0 );
      xmm1 = _mm_shuffle_epi32( xmm7, 0xB1 ); // 1 -> 0, 0 -> 1

      xmm4 = _mm_or_si128( _mm_andnot_si128( xmm5, xmm4 ), _mm_and_si128( xmm5, xmm8 ) ); //HV_D
      xmm7 = _mm_or_si128( _mm_andnot_si128( xmm5, xmm7 ), _mm_and_si128( xmm5, xmm1 ) ); //main - secondary (upper halves are for second value)

                                                                                          //xmm7 not to mix

      xmm0 = _mm_shuffle_epi32( xmm4, 0xFA );
      xmm4 = _mm_shuffle_epi32( xmm4, 0x50 ); //low, low, high, high
      xmm6 = _mm_set_epi32( 2, 1, 9, 2 );

      xmm2 = _mm_mullo_epi32( xmm0, xmm6 );
      xmm6 = _mm_mullo_epi32( xmm4, xmm6 );
      xmm4 = _mm_shuffle_epi32( xmm6, 0x4E );
      xmm0 = _mm_shuffle_epi32( xmm2, 0x4E ); //p to xmm6
      xmm6 = _mm_blend_epi16( xmm6, xmm0, 0xF0 );
      xmm4 = _mm_blend_epi16( xmm4, xmm2, 0xF0 );

      xmm5 = _mm_cmpgt_epi32( xmm4, xmm6 );
      xmm4 = _mm_and_si128( xmm5, xmm14 ); // 1 + 1

      xmm8 = _mm_and_si128( xmm7, xmm14 );
      xmm8 = _mm_slli_epi32( xmm8, 1 );

      xmm5 = _mm_add_epi32( xmm4, _mm_shuffle_epi32( xmm4, 0xB1 ) ); //directionStrength
      xmm4 = _mm_cmpgt_epi32( xmm5, mm_0 ); //is a mask now
      xmm4 = _mm_and_si128( _mm_add_epi32( xmm8, xmm5 ), xmm4 );

      xmm4 = _mm_add_epi32( xmm4, _mm_slli_epi32( xmm4, 2 ) ); //x5
      xmm4 = _mm_add_epi32( xmm4, xmm12 ); //+=

      xmm9 = _mm_shuffle_epi32( xmm7, 0xB1 );// <--
      xmm7 = _mm_slli_epi32( xmm7, 1 );
      xmm9 = _mm_srai_epi32( xmm9, 1 );
      xmm7 = _mm_add_epi32( xmm7, xmm9 );

      //to write to struct
      const int t0 = _mm_extract_epi32( xmm7, 0 );
      const int t1 = _mm_extract_epi32( xmm7, 2 );
      const int c0 = _mm_extract_epi32( xmm4, 0 );
      const int c1 = _mm_extract_epi32( xmm4, 2 );

      const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int transposeIdx0 = transposeTable[t0];
      int transposeIdx1 = transposeTable[t1];
      int classIdx0 = c0;
      int classIdx1 = c1;

      const int yOffset = ( i << 1 ) + posY;
      const int xOffset = j + posX;

      AlfClassifier *cl0 = classifier[yOffset] + xOffset;
      AlfClassifier *cl1 = classifier[yOffset + 1] + xOffset;
      AlfClassifier *cl2 = classifier[yOffset + 2] + xOffset;
      AlfClassifier *cl3 = classifier[yOffset + 3] + xOffset;

      AlfClassifier *_cl0 = cl0 + 4;
      AlfClassifier *_cl1 = cl1 + 4;
      AlfClassifier *_cl2 = cl2 + 4;
      AlfClassifier *_cl3 = cl3 + 4;

      cl0[0] = cl0[1] = cl0[2] = cl0[3] = cl1[0] = cl1[1] = cl1[2] = cl1[3] = cl2[0] = cl2[1] = cl2[2] = cl2[3] = cl3[0] = cl3[1] = cl3[2] = cl3[3] = AlfClassifier( classIdx0, transposeIdx0 );
      _cl0[0] = _cl0[1] = _cl0[2] = _cl0[3] = _cl1[0] = _cl1[1] = _cl1[2] = _cl1[3] = _cl2[0] = _cl2[1] = _cl2[2] = _cl2[3] = _cl3[0] = _cl3[1] = _cl3[2] = _cl3[3] = AlfClassifier( classIdx1, transposeIdx1 );
    }
  }
}

template<X86_VEXT vext>
static void simdFilter5x5Blk( AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng )
{
  static const unsigned char mask05[16] = { 8, 9, 6, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  static const unsigned char mask03[16] = { 4, 5, 2, 3, 0, 1, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
  static const unsigned char mask_c[16] = { 0, 1, 8, 9, 4, 5, 14, 15, 2, 3, 10, 11, 12, 13, 6, 7 };

  const bool bChroma = isChroma( compId );

  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const int srcStride = srcLuma.stride;
  const int dstStride = dstLuma.stride;

  const Pel* srcExt = srcLuma.buf;
  Pel* dst = dstLuma.buf;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4, *pImgYPad5;

  short *coef = filterSet;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5;

  const int numBitsMinus1 = AdaptiveLoopFilter::m_NUM_BITS - 1;
  const int offset = ( 1 << ( AdaptiveLoopFilter::m_NUM_BITS - 2 ) );

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  Pel* imgYRecPost = dst;
  imgYRecPost += startHeight * dstStride;

  int transposeIdx = 0;

  const int clsSizeY = 4;
  const int clsSizeX = 4;

  VTMCHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  VTMCHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  VTMCHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  VTMCHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  const Pel* imgYRec = srcExt;

  Pel *pRec;
  AlfClassifier *pClass = nullptr;

  int srcStride2 = srcStride * clsSizeY;

  const __m128i mmOffset = _mm_set1_epi32( offset );
  const __m128i mmMin = _mm_set1_epi32( clpRng.min );
  const __m128i mmMax = _mm_set1_epi32( clpRng.max );

  const __m128i xmm10 = _mm_loadu_si128( ( __m128i* )mask03 );
  const __m128i mm_mask05 = _mm_loadu_si128( ( __m128i* )mask05 );

  pImgYPad0 = imgYRec + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  pImgYPad5 = pImgYPad3 + srcStride;

  pRec = imgYRecPost + startWidth;

  for( int i = 0; i < endHeight - startHeight; i += 4 )
  {
    pRec = imgYRecPost + startWidth + i * dstStride;

    if( !bChroma )
    {
      pClass = classifier[startHeight + i] + startWidth;
    }

    for( int j = 0; j < endWidth - startWidth; j += 4 )
    {
      if( !bChroma )
      {
        AlfClassifier& cl = pClass[j];
        transposeIdx = cl.transposeIdx;
        coef = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
      }

      __m128i c0, t0 = _mm_setzero_si128();

      c0 = _mm_loadu_si128( ( __m128i* )( coef + 0 ) );
      c0 = _mm_alignr_epi8( c0, c0, 2 );
      c0 = _mm_blend_epi16( c0, t0, 0x40 );

      if( transposeIdx & 1 )
      {
        c0 = _mm_shuffle_epi8( c0, _mm_loadu_si128( ( __m128i* )mask_c ) );
      }

      if( transposeIdx == 0 || transposeIdx == 1 )
      {
        c0 = _mm_shuffle_epi8( c0, xmm10 );
      }

      pImg0 = pImgYPad0 + j;
      pImg1 = pImgYPad1 + j;
      pImg2 = pImgYPad2 + j;
      pImg3 = pImgYPad3 + j;
      pImg4 = pImgYPad4 + j;
      pImg5 = pImgYPad5 + j;

      for( int k = 0; k < 4; k++ )
      {
        __m128i xmm4 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 ) );
        __m128i xmm2 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 - 1 ) );
        __m128i xmm0 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 2 ) );
        __m128i xmm1 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 - 1 - 1 ) );
        __m128i xmm3 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 - 0 - 2 ) );

        __m128i xmm7 = _mm_setzero_si128();

        __m128i xmm6 = _mm_shuffle_epi8( xmm0, mm_mask05 );
        __m128i xmm8 = _mm_shuffle_epi8( _mm_srli_si128( xmm0, 2 ), mm_mask05 );
        __m128i xmm9 = _mm_shuffle_epi8( _mm_srli_si128( xmm0, 4 ), mm_mask05 );
        __m128i xmm11 = _mm_shuffle_epi8( _mm_srli_si128( xmm0, 6 ), mm_mask05 );

        xmm6 = _mm_blend_epi16( xmm7, xmm6, 0x03 );
        xmm8 = _mm_blend_epi16( xmm7, xmm8, 0x03 );
        xmm9 = _mm_blend_epi16( xmm7, xmm9, 0x03 );
        xmm11 = _mm_blend_epi16( xmm7, xmm11, 0x03 );

        xmm6 = _mm_add_epi16( xmm6, xmm0 );
        xmm8 = _mm_add_epi16( xmm8, _mm_srli_si128( xmm0, 2 ) );
        xmm9 = _mm_add_epi16( xmm9, _mm_srli_si128( xmm0, 4 ) );
        xmm11 = _mm_add_epi16( xmm11, _mm_srli_si128( xmm0, 6 ) );

        xmm6 = _mm_slli_si128( xmm6, 6 );
        xmm8 = _mm_slli_si128( xmm8, 6 );
        xmm9 = _mm_slli_si128( xmm9, 6 );
        xmm11 = _mm_slli_si128( xmm11, 6 );

        xmm4 = _mm_add_epi16( xmm4, _mm_srli_si128( xmm3, 4 ) );
        xmm6 = _mm_blend_epi16( xmm6, _mm_slli_si128( xmm4, 14 ), 0x80 );
        xmm8 = _mm_blend_epi16( xmm8, _mm_slli_si128( xmm4, 12 ), 0x80 );
        xmm9 = _mm_blend_epi16( xmm9, _mm_slli_si128( xmm4, 10 ), 0x80 );
        xmm11 = _mm_blend_epi16( xmm11, _mm_slli_si128( xmm4, 8 ), 0x80 );

        __m128i xmm12 = _mm_shuffle_epi8( xmm2, xmm10 );
        __m128i xmm13 = _mm_shuffle_epi8( _mm_srli_si128( xmm2, 2 ), xmm10 );
        __m128i xmm14 = _mm_shuffle_epi8( _mm_srli_si128( xmm2, 4 ), xmm10 );
        __m128i xmm15 = _mm_shuffle_epi8( _mm_srli_si128( xmm2, 6 ), xmm10 );

        xmm12 = _mm_add_epi16( xmm12, _mm_srli_si128( xmm1, 2 ) );
        xmm13 = _mm_add_epi16( xmm13, _mm_srli_si128( xmm1, 4 ) );
        xmm14 = _mm_add_epi16( xmm14, _mm_srli_si128( xmm1, 6 ) );
        xmm15 = _mm_add_epi16( xmm15, _mm_srli_si128( xmm1, 8 ) );

        xmm6 = _mm_blend_epi16( xmm6, xmm12, 0x07 );
        xmm8 = _mm_blend_epi16( xmm8, xmm13, 0x07 );
        xmm9 = _mm_blend_epi16( xmm9, xmm14, 0x07 );
        xmm11 = _mm_blend_epi16( xmm11, xmm15, 0x07 );

        xmm6 = _mm_madd_epi16( xmm6, c0 );
        xmm8 = _mm_madd_epi16( xmm8, c0 );
        xmm9 = _mm_madd_epi16( xmm9, c0 );
        xmm11 = _mm_madd_epi16( xmm11, c0 );

        xmm12 = _mm_shuffle_epi32( xmm6, 0x1B );
        xmm13 = _mm_shuffle_epi32( xmm8, 0x1B );
        xmm14 = _mm_shuffle_epi32( xmm9, 0x1B );
        xmm15 = _mm_shuffle_epi32( xmm11, 0x1B );

        xmm6 = _mm_add_epi32( xmm6, xmm12 );
        xmm8 = _mm_add_epi32( xmm8, xmm13 );
        xmm9 = _mm_add_epi32( xmm9, xmm14 );
        xmm11 = _mm_add_epi32( xmm11, xmm15 );

        xmm6 = _mm_blend_epi16( xmm6, xmm8, 0xF0 );
        xmm9 = _mm_blend_epi16( xmm9, xmm11, 0xF0 );

        xmm12 = _mm_hadd_epi32( xmm6, xmm9 );

        xmm12 = _mm_add_epi32( xmm12, mmOffset );
        xmm12 = _mm_srai_epi32( xmm12, numBitsMinus1 );

        xmm12 = _mm_min_epi32( mmMax, _mm_max_epi32( xmm12, mmMin ) );

        xmm12 = _mm_packus_epi32( xmm12, xmm12 );

        _mm_storel_epi64( ( __m128i* )( pRec ), xmm12 );

        pRec += dstStride;

        pImg0 += srcStride;
        pImg1 += srcStride;
        pImg2 += srcStride;
        pImg3 += srcStride;
        pImg4 += srcStride;
        pImg5 += srcStride;

      } //<-- end of k-loop

      pRec -= ( 4 * dstStride );
      pRec += 4;
    }

    pRec += 4 * dstStride;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
    pImgYPad5 += srcStride2;
  }
}

template<X86_VEXT vext>
static void simdFilter7x7Blk( AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng )
{
  static const unsigned char mask0[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 6, 7, 4, 5, 2, 3 };
  static const unsigned char mask00[16] = { 2, 3, 0, 1, 0, 0, 0, 0, 8, 9, 0, 0, 0, 0, 0, 1 };
  static const unsigned char mask02[16] = { 0, 0, 0, 0, 2, 3, 10, 11, 0, 0, 10, 11, 2, 3, 0, 0 };
  static const unsigned char mask20[16] = { 0, 0, 4, 5, 0, 0, 0, 0, 0, 0, 6, 7, 0, 0, 0, 0 };
  static const unsigned char mask22[16] = { 14, 15, 0, 0, 6, 7, 4, 5, 12, 13, 0, 0, 8, 9, 0, 1 };
  static const unsigned char mask35[16] = { 4, 5, 2, 3, 0, 1, 14, 15, 12, 13, 10, 11, 8, 9, 6, 7 };

  const bool bChroma = isChroma( compId );

  if( bChroma )
  {
    VTMCHECK( 0, "Chroma doesn't support 7x7" );
  }

  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const int srcStride = srcLuma.stride;
  const int dstStride = dstLuma.stride;

  const Pel* srcExt = srcLuma.buf;
  Pel* dst = dstLuma.buf;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4, *pImgYPad5, *pImgYPad6;

  short *coef = filterSet;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4;
  const Pel *pImg5, *pImg6;

  const int numBitsMinus1 = AdaptiveLoopFilter::m_NUM_BITS - 1;
  const int offset = ( 1 << ( AdaptiveLoopFilter::m_NUM_BITS - 2 ) );

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  Pel* imgYRecPost = dst;
  imgYRecPost += startHeight * dstStride;

  int transposeIdx = 0;

  const int clsSizeY = 4;
  const int clsSizeX = 4;

  VTMCHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  VTMCHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  VTMCHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  VTMCHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  const Pel* imgYRec = srcExt;

  Pel *pRec;
  AlfClassifier *pClass = nullptr;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  const __m128i mmOffset = _mm_set1_epi32( offset );
  const __m128i mmMin = _mm_set1_epi32( clpRng.min );
  const __m128i mmMax = _mm_set1_epi32( clpRng.max );

  const __m128i xmm10 = _mm_loadu_si128( ( __m128i* )mask35 );

  pImgYPad0 = imgYRec + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  pImgYPad5 = pImgYPad3 + srcStride;
  pImgYPad6 = pImgYPad4 - srcStride;

  pRec = imgYRecPost + startWidth;

  for( int i = 0; i < endHeight - startHeight; i += 4 )
  {
    pRec = imgYRecPost + startWidth + i * dstStride;

    if( !bChroma )
    {
      pClass = classifier[startHeight + i] + startWidth;
    }

    for( int j = 0; j < endWidth - startWidth; j += 4 )
    {
      if( !bChroma )
      {
        AlfClassifier& cl = pClass[j];
        transposeIdx = cl.transposeIdx;
        coef = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
      }

      __m128i c0, c2, t1, t2;

      t1 = _mm_loadu_si128( ( __m128i* )( coef + 0 ) );
      t2 = _mm_loadu_si128( ( __m128i* )( coef + 1 ) );
      c2 = _mm_loadu_si128( ( __m128i* )( coef + 4 - 3 ) );
      c0 = _mm_loadu_si128( ( __m128i* )( coef + 9 - 1 ) );

      c0 = _mm_blend_epi16( c0, t1, 0x01 );
      c2 = _mm_blend_epi16( c2, t2, 0x07 );

      if( transposeIdx & 1 )
      {
        t1 = _mm_loadu_si128( ( __m128i* )mask00 );
        t2 = _mm_loadu_si128( ( __m128i* )mask02 );
        __m128i t3 = _mm_loadu_si128( ( __m128i* )mask20 );
        __m128i t4 = _mm_loadu_si128( ( __m128i* )mask22 );

        t1 = _mm_shuffle_epi8( c0, t1 );
        t2 = _mm_shuffle_epi8( c2, t2 );
        t3 = _mm_shuffle_epi8( c0, t3 );
        t4 = _mm_shuffle_epi8( c2, t4 );

        c0 = _mm_blend_epi16( t1, t2, 0x6C );
        c2 = _mm_blend_epi16( t4, t3, 0x22 );
      }
      else
      {
        c0 = _mm_shuffle_epi8( c0, _mm_loadu_si128( ( __m128i* )mask0 ) );
      }

      if( transposeIdx == 0 || transposeIdx == 3 )
      {
        c2 = _mm_shuffle_epi8( c2, xmm10 );
      }

      pImg0 = pImgYPad0 + j;
      pImg1 = pImgYPad1 + j;
      pImg2 = pImgYPad2 + j;
      pImg3 = pImgYPad3 + j;
      pImg4 = pImgYPad4 + j;
      pImg5 = pImgYPad5 + j;
      pImg6 = pImgYPad6 + j;

      for( int k = 0; k < 4; k++ )
      {
        __m128i xmm6 = _mm_lddqu_si128( ( __m128i* ) pImg6 );
        __m128i xmm4 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 - 1 ) );
        __m128i xmm2 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 - 2 ) );
        __m128i xmm0 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 3 ) );
        __m128i xmm11 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 5 ) );
        __m128i xmm1 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 - 2 - 1 ) );
        __m128i xmm8 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 5 ) );
        __m128i xmm3 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 - 2 ) );
        __m128i xmm5 = _mm_lddqu_si128( ( __m128i* ) ( pImg5 - 1 ) );

        xmm6 = _mm_add_epi16( xmm6, _mm_srli_si128( xmm5, 2 ) );

        __m128i xmm12 = _mm_blend_epi16( _mm_slli_si128( xmm0, 2 ), xmm6, 0x01 );
        __m128i xmm13 = _mm_blend_epi16( xmm0, _mm_srli_si128( xmm6, 2 ), 0x01 );

        __m128i xmm14 = _mm_blend_epi16( _mm_slli_si128( xmm2, 6 ), xmm4, 0x07 );
        __m128i xmm16 = _mm_blend_epi16( _mm_slli_si128( xmm1, 4 ), _mm_srli_si128( xmm3, 2 ), 0x07 );
        xmm14 = _mm_shuffle_epi8( xmm14, xmm10 );
        xmm14 = _mm_add_epi16( xmm14, xmm16 );
        __m128i xmm15 = _mm_blend_epi16( _mm_slli_si128( xmm2, 4 ), _mm_srli_si128( xmm4, 2 ), 0x07 );
        __m128i xmm17 = _mm_blend_epi16( _mm_slli_si128( xmm1, 2 ), _mm_srli_si128( xmm3, 4 ), 0x07 );
        xmm15 = _mm_shuffle_epi8( xmm15, xmm10 );
        xmm15 = _mm_add_epi16( xmm15, xmm17 );

        xmm12 = _mm_madd_epi16( xmm12, c0 );
        xmm13 = _mm_madd_epi16( xmm13, c0 );
        xmm14 = _mm_madd_epi16( xmm14, c2 );
        xmm15 = _mm_madd_epi16( xmm15, c2 );

        xmm12 = _mm_add_epi32( xmm12, xmm14 );
        xmm13 = _mm_add_epi32( xmm13, xmm15 );
        xmm14 = _mm_shuffle_epi32( xmm12, 0x1B );
        xmm15 = _mm_shuffle_epi32( xmm13, 0x1B );
        xmm12 = _mm_add_epi32( xmm12, xmm14 );
        xmm13 = _mm_add_epi32( xmm13, xmm15 );

        __m128i xmm7 = _mm_blend_epi16( xmm12, xmm13, 0xF0 );

        xmm12 = _mm_blend_epi16( _mm_alignr_epi8( xmm11, xmm0, 2 ), _mm_srli_si128( xmm6, 4 ), 0x01 );
        xmm13 = _mm_blend_epi16( _mm_alignr_epi8( xmm11, xmm0, 4 ), _mm_srli_si128( xmm6, 6 ), 0x01 );

        xmm14 = _mm_blend_epi16( _mm_slli_si128( xmm2, 2 ), _mm_srli_si128( xmm4, 4 ), 0x07 );
        xmm16 = _mm_blend_epi16( xmm1, _mm_srli_si128( xmm3, 6 ), 0x07 );
        xmm14 = _mm_shuffle_epi8( xmm14, xmm10 );
        xmm14 = _mm_add_epi16( xmm14, xmm16 );
        xmm15 = _mm_blend_epi16( xmm2, _mm_srli_si128( xmm4, 6 ), 0x07 );
        xmm8 = _mm_alignr_epi8( xmm8, xmm1, 2 );
        xmm17 = _mm_blend_epi16( xmm8, _mm_srli_si128( xmm3, 8 ), 0x07 );
        xmm15 = _mm_shuffle_epi8( xmm15, xmm10 );
        xmm15 = _mm_add_epi16( xmm15, xmm17 );

        xmm12 = _mm_madd_epi16( xmm12, c0 );
        xmm13 = _mm_madd_epi16( xmm13, c0 );
        xmm14 = _mm_madd_epi16( xmm14, c2 );
        xmm15 = _mm_madd_epi16( xmm15, c2 );

        xmm12 = _mm_add_epi32( xmm12, xmm14 );
        xmm13 = _mm_add_epi32( xmm13, xmm15 );
        xmm14 = _mm_shuffle_epi32( xmm12, 0x1B );
        xmm15 = _mm_shuffle_epi32( xmm13, 0x1B );
        xmm12 = _mm_add_epi32( xmm12, xmm14 );
        xmm13 = _mm_add_epi32( xmm13, xmm15 );

        __m128i xmm9 = _mm_blend_epi16( xmm12, xmm13, 0xF0 );

        xmm12 = _mm_hadd_epi32( xmm7, xmm9 );

        xmm12 = _mm_add_epi32( xmm12, mmOffset );
        xmm12 = _mm_srai_epi32( xmm12, numBitsMinus1 );

        xmm12 = _mm_min_epi32( mmMax, _mm_max_epi32( xmm12, mmMin ) );

        xmm12 = _mm_packus_epi32( xmm12, xmm12 );

        _mm_storel_epi64( ( __m128i* )( pRec ), xmm12 );

        pRec += dstStride;

        pImg0 += srcStride;
        pImg1 += srcStride;
        pImg2 += srcStride;
        pImg3 += srcStride;
        pImg4 += srcStride;
        pImg5 += srcStride;
        pImg6 += srcStride;
      }

      pRec -= ( 4 * dstStride );
      pRec += 4;
    }

    pRec += dstStride2;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
    pImgYPad5 += srcStride2;
    pImgYPad6 += srcStride2;
  }
}

template <X86_VEXT vext>
void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86()
{
  m_deriveClassificationBlk = simdDeriveClassificationBlk<vext>;
  m_filter5x5Blk = simdFilter5x5Blk<vext>;
  m_filter7x7Blk = simdFilter7x7Blk<vext>;
}

template void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86<SIMDX86>();
#endif //#ifdef TARGET_SIMD_X86
//! \}
