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

/** \file     AdaptiveLoopFilter.cpp
    \brief    adaptive loop filter class
*/

#include "AdaptiveLoopFilter.h"

#include "CodingStructure.h"
#include "Picture.h"

AdaptiveLoopFilter::AdaptiveLoopFilter()
  : m_classifier( nullptr )
{
  for( int i = 0; i < NUM_DIRECTIONS; i++ )
  {
    m_laplacian[i] = nullptr;
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = nullptr;
  }

  m_deriveClassificationBlk = deriveClassificationBlk;
  m_filter5x5Blk = filterBlk<ALF_FILTER_5>;
  m_filter7x7Blk = filterBlk<ALF_FILTER_7>;

#if ENABLE_SIMD_OPT_ALF
#ifdef TARGET_SIMD_X86
  initAdaptiveLoopFilterX86();
#endif
#endif
}

void AdaptiveLoopFilter::ALFProcess( CodingStructure& cs, AlfSliceParam& alfSliceParam )
{
  if( !alfSliceParam.enabledFlag[COMPONENT_Y] && !alfSliceParam.enabledFlag[COMPONENT_Cb] && !alfSliceParam.enabledFlag[COMPONENT_Cr] )
  {
    return;
  }

  // set available filter shapes
  alfSliceParam.filterShapes = m_filterShapes;

  // set clipping range
  m_clpRngs = cs.slice->getClpRngs();

  // set CTU enable flags
  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = cs.picture->getAlfCtuEnableFlag( compIdx );
  }
  reconstructCoeff( alfSliceParam, CHANNEL_TYPE_LUMA );
  reconstructCoeff( alfSliceParam, CHANNEL_TYPE_CHROMA );

  PelUnitBuf recYuv = cs.getRecoBuf();
  m_tempBuf.copyFrom( recYuv );
  PelUnitBuf tmpYuv = m_tempBuf.getBuf( cs.area );
  tmpYuv.extendBorderPel( MAX_ALF_FILTER_LENGTH >> 1 );

  const PreCalcValues& pcv = *cs.pcv;

  int ctuIdx = 0;
  for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
      const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;
      const UnitArea area( cs.area.chromaFormat, Area( xPos, yPos, width, height ) );
      if( m_ctuEnableFlag[COMPONENT_Y][ctuIdx] )
      {
        Area blk( xPos, yPos, width, height );
        deriveClassification( m_classifier, tmpYuv.get( COMPONENT_Y ), blk );
        m_filter7x7Blk(m_classifier, recYuv, tmpYuv, blk, COMPONENT_Y, m_coeffFinal, m_clpRngs.comp[COMPONENT_Y]);
      }

      for( int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
      {
        ComponentID compID = ComponentID( compIdx );
        const int chromaScaleX = getComponentScaleX( compID, tmpYuv.chromaFormat );
        const int chromaScaleY = getComponentScaleY( compID, tmpYuv.chromaFormat );

        if( m_ctuEnableFlag[compIdx][ctuIdx] )
        {
          Area blk( xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY );

          m_filter5x5Blk( m_classifier, recYuv, tmpYuv, blk, compID, alfSliceParam.chromaCoeff, m_clpRngs.comp[compIdx] );
        }
      }
      ctuIdx++;
    }
  }
}

void AdaptiveLoopFilter::reconstructCoeff( AlfSliceParam& alfSliceParam, ChannelType channel, const bool bRedo )
{
  int factor = ( 1 << ( m_NUM_BITS - 1 ) );
  AlfFilterType filterType = isLuma( channel ) ? ALF_FILTER_7 : ALF_FILTER_5;
  int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int numCoeff = filterType == ALF_FILTER_5 ? 7 : 13;
  int numCoeffMinus1 = numCoeff - 1;
  int numFilters = isLuma( channel ) ? alfSliceParam.numLumaFilters : 1;
  short* coeff = isLuma( channel ) ? alfSliceParam.lumaCoeff : alfSliceParam.chromaCoeff;

  if( alfSliceParam.alfLumaCoeffDeltaPredictionFlag && isLuma( channel ) )
  {
    for( int i = 1; i < numFilters; i++ )
    {
      for( int j = 0; j < numCoeffMinus1; j++ )
      {
        coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] += coeff[( i - 1 ) * MAX_NUM_ALF_LUMA_COEFF + j];
      }
    }
  }

  for( int filterIdx = 0; filterIdx < numFilters; filterIdx++ )
  {
    int sum = 0;
    for( int i = 0; i < numCoeffMinus1; i++ )
    {
      sum += ( coeff[filterIdx* MAX_NUM_ALF_LUMA_COEFF + i] << 1 );
    }
    coeff[filterIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = factor - sum;
  }

  if( isChroma( channel ) )
  {
    return;
  }

  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    int filterIdx = alfSliceParam.filterCoeffDeltaIdx[classIdx];
    memcpy( m_coeffFinal + classIdx * MAX_NUM_ALF_LUMA_COEFF, coeff + filterIdx * MAX_NUM_ALF_LUMA_COEFF, sizeof( int16_t ) * numCoeff );
  }

  if( bRedo && alfSliceParam.alfLumaCoeffDeltaPredictionFlag )
  {
    for( int i = numFilters - 1; i > 0; i-- )
    {
      for( int j = 0; j < numCoeffMinus1; j++ )
      {
        coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] = coeff[i * MAX_NUM_ALF_LUMA_COEFF + j] - coeff[( i - 1 ) * MAX_NUM_ALF_LUMA_COEFF + j];
      }
    }
  }
}

void AdaptiveLoopFilter::create( const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE] )
{
  std::memcpy( m_inputBitDepth, inputBitDepth, sizeof( m_inputBitDepth ) );
  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_maxCUWidth = maxCUWidth;
  m_maxCUHeight = maxCUHeight;
  m_maxCUDepth = maxCUDepth;
  m_chromaFormat = format;

  m_numCTUsInWidth = ( m_picWidth / m_maxCUWidth ) + ( ( m_picWidth % m_maxCUWidth ) ? 1 : 0 );
  m_numCTUsInHeight = ( m_picHeight / m_maxCUHeight ) + ( ( m_picHeight % m_maxCUHeight ) ? 1 : 0 );
  m_numCTUsInPic = m_numCTUsInHeight * m_numCTUsInWidth;
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back( AlfFilterShape( 7 ) );
  m_filterShapes[CHANNEL_TYPE_CHROMA].push_back( AlfFilterShape( 5 ) );

  m_tempBuf.destroy();
  m_tempBuf.create( format, Area( 0, 0, picWidth, picHeight ), maxCUWidth, MAX_ALF_FILTER_LENGTH >> 1, 0, false );

  // Laplacian based activity
  for( int i = 0; i < NUM_DIRECTIONS; i++ )
  {
    if ( m_laplacian[i] == nullptr )
    {
      m_laplacian[i] = new int*[m_CLASSIFICATION_BLK_SIZE + 5];

      for( int y = 0; y < m_CLASSIFICATION_BLK_SIZE + 5; y++ )
      {
        m_laplacian[i][y] = new int[m_CLASSIFICATION_BLK_SIZE + 5];
      }
    }
  }

  // Classification
  if ( m_classifier == nullptr )
  {
    m_classifier = new AlfClassifier*[picHeight];
    for( int i = 0; i < picHeight; i++ )
    {
      m_classifier[i] = new AlfClassifier[picWidth];
    }
  }
}

void AdaptiveLoopFilter::destroy()
{
  for( int i = 0; i < NUM_DIRECTIONS; i++ )
  {
    if( m_laplacian[i] )
    {
      for( int y = 0; y < m_CLASSIFICATION_BLK_SIZE + 5; y++ )
      {
        delete[] m_laplacian[i][y];
        m_laplacian[i][y] = nullptr;
      }

      delete[] m_laplacian[i];
      m_laplacian[i] = nullptr;
    }
  }

  if( m_classifier )
  {
    for( int i = 0; i < m_picHeight; i++ )
    {
      delete[] m_classifier[i];
      m_classifier[i] = nullptr;
    }

    delete[] m_classifier;
    m_classifier = nullptr;
  }

  m_tempBuf.destroy();
}

void AdaptiveLoopFilter::deriveClassification( AlfClassifier** classifier, const CPelBuf& srcLuma, const Area& blk )
{
  int height = blk.pos().y + blk.height;
  int width = blk.pos().x + blk.width;

  for( int i = blk.pos().y; i < height; i += m_CLASSIFICATION_BLK_SIZE )
  {
    int nHeight = std::min( i + m_CLASSIFICATION_BLK_SIZE, height ) - i;

    for( int j = blk.pos().x; j < width; j += m_CLASSIFICATION_BLK_SIZE )
    {
      int nWidth = std::min( j + m_CLASSIFICATION_BLK_SIZE, width ) - j;

      m_deriveClassificationBlk( classifier, m_laplacian, srcLuma, Area( j, i, nWidth, nHeight ), m_inputBitDepth[CHANNEL_TYPE_LUMA] + 4 );
    }
  }
}

void AdaptiveLoopFilter::deriveClassificationBlk( AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blk, const int shift )
{
  static const int th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
  const int stride = srcLuma.stride;
  const Pel* src = srcLuma.buf;
  const int maxActivity = 15;

  int fl = 2;
  int flP1 = fl + 1;
  int fl2 = 2 * fl;

  int mainDirection, secondaryDirection, dirTempHV, dirTempD;

  int pixY;
  int height = blk.height + fl2;
  int width = blk.width + fl2;
  int posX = blk.pos().x;
  int posY = blk.pos().y;
  int startHeight = posY - flP1;

  for( int i = 0; i < height; i += 2 )
  {
    int yoffset = ( i + 1 + startHeight ) * stride - flP1;
    const Pel *src0 = &src[yoffset - stride];
    const Pel *src1 = &src[yoffset];
    const Pel *src2 = &src[yoffset + stride];
    const Pel *src3 = &src[yoffset + stride * 2];

    int* pYver = laplacian[VER][i];
    int* pYhor = laplacian[HOR][i];
    int* pYdig0 = laplacian[DIAG0][i];
    int* pYdig1 = laplacian[DIAG1][i];

    for( int j = 0; j < width; j += 2 )
    {
      pixY = j + 1 + posX;
      const Pel *pY = src1 + pixY;
      const Pel* pYdown = src0 + pixY;
      const Pel* pYup = src2 + pixY;
      const Pel* pYup2 = src3 + pixY;

      const Pel y0 = pY[0] << 1;
      const Pel yup1 = pYup[1] << 1;

      pYver[j] = abs( y0 - pYdown[0] - pYup[0] ) + abs( yup1 - pY[1] - pYup2[1] );
      pYhor[j] = abs( y0 - pY[1] - pY[-1] ) + abs( yup1 - pYup[2] - pYup[0] );
      pYdig0[j] = abs( y0 - pYdown[-1] - pYup[1] ) + abs( yup1 - pY[0] - pYup2[2] );
      pYdig1[j] = abs( y0 - pYup[-1] - pYdown[1] ) + abs( yup1 - pYup2[0] - pY[2] );

      if( j > 4 && ( j - 6 ) % 4 == 0 )
      {
        int jM6 = j - 6;
        int jM4 = j - 4;
        int jM2 = j - 2;

        pYver[jM6] += pYver[jM4] + pYver[jM2] + pYver[j];
        pYhor[jM6] += pYhor[jM4] + pYhor[jM2] + pYhor[j];
        pYdig0[jM6] += pYdig0[jM4] + pYdig0[jM2] + pYdig0[j];
        pYdig1[jM6] += pYdig1[jM4] + pYdig1[jM2] + pYdig1[j];
      }
    }
  }

  // classification block size
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  for( int i = 0; i < blk.height; i += clsSizeY )
  {
    int* pYver = laplacian[VER][i];
    int* pYver2 = laplacian[VER][i + 2];
    int* pYver4 = laplacian[VER][i + 4];
    int* pYver6 = laplacian[VER][i + 6];

    int* pYhor = laplacian[HOR][i];
    int* pYhor2 = laplacian[HOR][i + 2];
    int* pYhor4 = laplacian[HOR][i + 4];
    int* pYhor6 = laplacian[HOR][i + 6];

    int* pYdig0 = laplacian[DIAG0][i];
    int* pYdig02 = laplacian[DIAG0][i + 2];
    int* pYdig04 = laplacian[DIAG0][i + 4];
    int* pYdig06 = laplacian[DIAG0][i + 6];

    int* pYdig1 = laplacian[DIAG1][i];
    int* pYdig12 = laplacian[DIAG1][i + 2];
    int* pYdig14 = laplacian[DIAG1][i + 4];
    int* pYdig16 = laplacian[DIAG1][i + 6];

    for( int j = 0; j < blk.width; j += clsSizeX )
    {
      int sumV = pYver[j] + pYver2[j] + pYver4[j] + pYver6[j];
      int sumH = pYhor[j] + pYhor2[j] + pYhor4[j] + pYhor6[j];
      int sumD0 = pYdig0[j] + pYdig02[j] + pYdig04[j] + pYdig06[j];
      int sumD1 = pYdig1[j] + pYdig12[j] + pYdig14[j] + pYdig16[j];

      int tempAct = sumV + sumH;
      int activity = (Pel)Clip3<int>( 0, maxActivity, ( tempAct * 64 ) >> shift );
      int classIdx = th[activity];

      int hv1, hv0, d1, d0, hvd1, hvd0;

      if( sumV > sumH )
      {
        hv1 = sumV;
        hv0 = sumH;
        dirTempHV = 1;
      }
      else
      {
        hv1 = sumH;
        hv0 = sumV;
        dirTempHV = 3;
      }
      if( sumD0 > sumD1 )
      {
        d1 = sumD0;
        d0 = sumD1;
        dirTempD = 0;
      }
      else
      {
        d1 = sumD1;
        d0 = sumD0;
        dirTempD = 2;
      }
      if( d1*hv0 > hv1*d0 )
      {
        hvd1 = d1;
        hvd0 = d0;
        mainDirection = dirTempD;
        secondaryDirection = dirTempHV;
      }
      else
      {
        hvd1 = hv1;
        hvd0 = hv0;
        mainDirection = dirTempHV;
        secondaryDirection = dirTempD;
      }

      int directionStrength = 0;
      if( hvd1 > 2 * hvd0 )
      {
        directionStrength = 1;
      }
      if( hvd1 * 2 > 9 * hvd0 )
      {
        directionStrength = 2;
      }

      if( directionStrength )
      {
        classIdx += ( ( ( mainDirection & 0x1 ) << 1 ) + directionStrength ) * 5;
      }

      static const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int transposeIdx = transposeTable[mainDirection * 2 + ( secondaryDirection >> 1 )];

      int yOffset = i + posY;
      int xOffset = j + posX;

      AlfClassifier *cl0 = classifier[yOffset] + xOffset;
      AlfClassifier *cl1 = classifier[yOffset + 1] + xOffset;
      AlfClassifier *cl2 = classifier[yOffset + 2] + xOffset;
      AlfClassifier *cl3 = classifier[yOffset + 3] + xOffset;
      cl0[0] = cl0[1] = cl0[2] = cl0[3] = cl1[0] = cl1[1] = cl1[2] = cl1[3] = cl2[0] = cl2[1] = cl2[2] = cl2[3] = cl3[0] = cl3[1] = cl3[2] = cl3[3] = AlfClassifier( classIdx, transposeIdx );
    }
  }
}

template<AlfFilterType filtType>
void AdaptiveLoopFilter::filterBlk( AlfClassifier** classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, short* filterSet, const ClpRng& clpRng )
{
  const bool bChroma = isChroma( compId );
  if( bChroma )
  {
    VTMCHECK( filtType != 0, "Chroma needs to have filtType == 0" );
  }

  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const int srcStride = srcLuma.stride;
  const int dstStride = dstLuma.stride;

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  const Pel* src = srcLuma.buf;
  Pel* dst = dstLuma.buf + startHeight * dstStride;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4, *pImgYPad5, *pImgYPad6;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;

  short *coef = filterSet;

  const int shift = m_NUM_BITS - 1;

  const int offset = 1 << ( shift - 1 );

  int transposeIdx = 0;
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  VTMCHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  VTMCHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  VTMCHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  VTMCHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  AlfClassifier *pClass = nullptr;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  std::vector<Pel> filterCoeff( MAX_NUM_ALF_LUMA_COEFF );

  pImgYPad0 = src + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  pImgYPad5 = pImgYPad3 + srcStride;
  pImgYPad6 = pImgYPad4 - srcStride;

  Pel* pRec0 = dst + startWidth;
  Pel* pRec1 = pRec0 + dstStride;

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    if( !bChroma )
    {
      pClass = classifier[startHeight + i] + startWidth;
    }

    for( int j = 0; j < endWidth - startWidth; j += clsSizeX )
    {
      if( !bChroma )
      {
        AlfClassifier& cl = pClass[j];
        transposeIdx = cl.transposeIdx;
        coef = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
      }

      if( filtType == ALF_FILTER_7 )
      {
        if( transposeIdx == 1 )
        {
          filterCoeff = { coef[9], coef[4], coef[10], coef[8], coef[1], coef[5], coef[11], coef[7], coef[3], coef[0], coef[2], coef[6], coef[12] };
        }
        else if( transposeIdx == 2 )
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[8], coef[7], coef[6], coef[5], coef[4], coef[9], coef[10], coef[11], coef[12] };
        }
        else if( transposeIdx == 3 )
        {
          filterCoeff = { coef[9], coef[8], coef[10], coef[4], coef[3], coef[7], coef[11], coef[5], coef[1], coef[0], coef[2], coef[6], coef[12] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7], coef[8], coef[9], coef[10], coef[11], coef[12] };
        }
      }
      else
      {
        if( transposeIdx == 1 )
        {
          filterCoeff = { coef[4], coef[1], coef[5], coef[3], coef[0], coef[2], coef[6] };
        }
        else if( transposeIdx == 2 )
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[4], coef[5], coef[6] };
        }
        else if( transposeIdx == 3 )
        {
          filterCoeff = { coef[4], coef[3], coef[5], coef[1], coef[0], coef[2], coef[6] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6] };
        }
      }

      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        pImg0 = pImgYPad0 + j + ii * srcStride;
        pImg1 = pImgYPad1 + j + ii * srcStride;
        pImg2 = pImgYPad2 + j + ii * srcStride;
        pImg3 = pImgYPad3 + j + ii * srcStride;
        pImg4 = pImgYPad4 + j + ii * srcStride;
        pImg5 = pImgYPad5 + j + ii * srcStride;
        pImg6 = pImgYPad6 + j + ii * srcStride;

        pRec1 = pRec0 + j + ii * dstStride;

        for( int jj = 0; jj < clsSizeX; jj++ )
        {
          int sum = 0;
          if( filtType == ALF_FILTER_7 )
          {
            sum += filterCoeff[0] * ( pImg5[0] + pImg6[0] );

            sum += filterCoeff[1] * ( pImg3[+1] + pImg4[-1] );
            sum += filterCoeff[2] * ( pImg3[+0] + pImg4[+0] );
            sum += filterCoeff[3] * ( pImg3[-1] + pImg4[+1] );

            sum += filterCoeff[4] * ( pImg1[+2] + pImg2[-2] );
            sum += filterCoeff[5] * ( pImg1[+1] + pImg2[-1] );
            sum += filterCoeff[6] * ( pImg1[+0] + pImg2[+0] );
            sum += filterCoeff[7] * ( pImg1[-1] + pImg2[+1] );
            sum += filterCoeff[8] * ( pImg1[-2] + pImg2[+2] );

            sum += filterCoeff[9] * ( pImg0[+3] + pImg0[-3] );
            sum += filterCoeff[10] * ( pImg0[+2] + pImg0[-2] );
            sum += filterCoeff[11] * ( pImg0[+1] + pImg0[-1] );
            sum += filterCoeff[12] * ( pImg0[+0] );
          }
          else
          {
            sum += filterCoeff[0] * ( pImg3[+0] + pImg4[+0] );

            sum += filterCoeff[1] * ( pImg1[+1] + pImg2[-1] );
            sum += filterCoeff[2] * ( pImg1[+0] + pImg2[+0] );
            sum += filterCoeff[3] * ( pImg1[-1] + pImg2[+1] );

            sum += filterCoeff[4] * ( pImg0[+2] + pImg0[-2] );
            sum += filterCoeff[5] * ( pImg0[+1] + pImg0[-1] );
            sum += filterCoeff[6] * ( pImg0[+0] );
          }

          sum = ( sum + offset ) >> shift;
          pRec1[jj] = ClipPel( sum, clpRng );

          pImg0++;
          pImg1++;
          pImg2++;
          pImg3++;
          pImg4++;
          pImg5++;
          pImg6++;
        }
      }
    }

    pRec0 += dstStride2;
    pRec1 += dstStride2;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
    pImgYPad5 += srcStride2;
    pImgYPad6 += srcStride2;
  }
}
