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

/** \file     EncAdaptiveLoopFilter.cpp
 \brief    estimation part of adaptive loop filter class
 */
#include "EncAdaptiveLoopFilter.h"

#include "CommonLib/Picture.h"
#include "CommonLib/CodingStructure.h"

#define AlfCtx(c) SubCtx( Ctx::ctbAlfFlag, c )
#if JVET_M0427_INLOOP_RESHAPER
std::vector<double> EncAdaptiveLoopFilter::m_lumaLevelToWeightPLUT;
#endif

EncAdaptiveLoopFilter::EncAdaptiveLoopFilter()
  : m_CABACEstimator( nullptr )
{
  for( int i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_alfCovariance[i] = nullptr;
  }
  for( int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_alfCovarianceFrame[i] = nullptr;
  }
  m_filterCoeffQuant = nullptr;
  m_filterCoeffSet = nullptr;
  m_diffFilterCoeff = nullptr;

#if JVET_M0427_INLOOP_RESHAPER
  m_alfWSSD = 0;
#endif
}

void EncAdaptiveLoopFilter::create( const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE] )
{
  AdaptiveLoopFilter::create( picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxCUDepth, inputBitDepth );

  for( int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++ )
  {
    ChannelType chType = (ChannelType)channelIdx;
    int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
    m_alfCovarianceFrame[chType] = new AlfCovariance*[m_filterShapes[chType].size()];
    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovarianceFrame[chType][i] = new AlfCovariance[numClasses];
      for( int k = 0; k < numClasses; k++ )
      {
        m_alfCovarianceFrame[chType][i][k].create( m_filterShapes[chType][i].numCoeff );
      }
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlagTmp[compIdx] = new uint8_t[m_numCTUsInPic];
    ChannelType chType = toChannelType( ComponentID( compIdx ) );
    int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

    m_alfCovariance[compIdx] = new AlfCovariance**[m_filterShapes[chType].size()];

    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovariance[compIdx][i] = new AlfCovariance*[m_numCTUsInPic];
      for( int j = 0; j < m_numCTUsInPic; j++ )
      {
        m_alfCovariance[compIdx][i][j] = new AlfCovariance[numClasses];
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovariance[compIdx][i][j][k].create( m_filterShapes[chType][i].numCoeff );
        }
      }
    }
  }

  for( int i = 0; i != m_filterShapes[COMPONENT_Y].size(); i++ )
  {
    for( int j = 0; j <= MAX_NUM_ALF_CLASSES; j++ )
    {
      m_alfCovarianceMerged[i][j].create( m_filterShapes[COMPONENT_Y][i].numCoeff );
    }
  }

  m_filterCoeffQuant = new int[MAX_NUM_ALF_LUMA_COEFF];
  m_filterCoeffSet = new int*[MAX_NUM_ALF_CLASSES];
  m_diffFilterCoeff = new int*[MAX_NUM_ALF_CLASSES];

  for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
  {
    m_filterCoeffSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_diffFilterCoeff[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
  }
}

void EncAdaptiveLoopFilter::destroy()
{
  for( int channelIdx = 0; channelIdx < MAX_NUM_CHANNEL_TYPE; channelIdx++ )
  {
    if( m_alfCovarianceFrame[channelIdx] )
    {
      ChannelType chType = (ChannelType)channelIdx;
      int numClasses = channelIdx ? 1 : MAX_NUM_ALF_CLASSES;
      for( int i = 0; i != m_filterShapes[chType].size(); i++ )
      {
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovarianceFrame[channelIdx][i][k].destroy();
        }
        delete[] m_alfCovarianceFrame[channelIdx][i];
        m_alfCovarianceFrame[channelIdx][i] = nullptr;
      }
      delete[] m_alfCovarianceFrame[channelIdx];
      m_alfCovarianceFrame[channelIdx] = nullptr;
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    if( m_ctuEnableFlagTmp[compIdx] )
    {
      delete[] m_ctuEnableFlagTmp[compIdx];
      m_ctuEnableFlagTmp[compIdx] = nullptr;
    }

    if( m_alfCovariance[compIdx] )
    {
      ChannelType chType = toChannelType( ComponentID( compIdx ) );
      int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

      for( int i = 0; i != m_filterShapes[chType].size(); i++ )
      {
        for( int j = 0; j < m_numCTUsInPic; j++ )
        {
          for( int k = 0; k < numClasses; k++ )
          {
            m_alfCovariance[compIdx][i][j][k].destroy();
          }
          delete[] m_alfCovariance[compIdx][i][j];
          m_alfCovariance[compIdx][i][j] = nullptr;

        }
        delete[] m_alfCovariance[compIdx][i];
        m_alfCovariance[compIdx][i] = nullptr;

      }
      delete[] m_alfCovariance[compIdx];
      m_alfCovariance[compIdx] = nullptr;
    }
  }

  for( int i = 0; i != m_filterShapes[COMPONENT_Y].size(); i++ )
  {
    for( int j = 0; j <= MAX_NUM_ALF_CLASSES; j++ )
    {
      m_alfCovarianceMerged[i][j].destroy();
    }
  }

  if( m_filterCoeffSet )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_filterCoeffSet[i];
      m_filterCoeffSet[i] = nullptr;
    }
    delete[] m_filterCoeffSet;
    m_filterCoeffSet = nullptr;
  }

  if( m_diffFilterCoeff )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_diffFilterCoeff[i];
      m_diffFilterCoeff[i] = nullptr;
    }
    delete[] m_diffFilterCoeff;
    m_diffFilterCoeff = nullptr;
  }

  delete[] m_filterCoeffQuant;
  m_filterCoeffQuant = nullptr;

  AdaptiveLoopFilter::destroy();
}

void EncAdaptiveLoopFilter::initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice )
{
  m_CABACEstimator = cabacEncoder->getCABACEstimator( pcSlice->getSPS() );
  m_CtxCache = ctxCache;
  m_CABACEstimator->initCtxModels( *pcSlice );
  m_CABACEstimator->resetBits();
}

void EncAdaptiveLoopFilter::ALFProcess( CodingStructure& cs, const double *lambdas,
#if ENABLE_QPA
                                        const double lambdaChromaWeight,
#endif
                                        AlfSliceParam& alfSliceParam )
{
  // set available filter shapes
  alfSliceParam.filterShapes = m_filterShapes;

  // set clipping range
  m_clpRngs = cs.slice->getClpRngs();

  // set CTU ALF enable flags, it was already reset before ALF process
  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = cs.picture->getAlfCtuEnableFlag( compIdx );
  }

  // reset ALF parameters
  alfSliceParam.reset();
  int shiftLuma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_LUMA]);
  int shiftChroma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_CHROMA]);
  m_lambda[COMPONENT_Y] = lambdas[COMPONENT_Y] * double(1 << shiftLuma);
  m_lambda[COMPONENT_Cb] = lambdas[COMPONENT_Cb] * double(1 << shiftChroma);
  m_lambda[COMPONENT_Cr] = lambdas[COMPONENT_Cr] * double(1 << shiftChroma);

  PelUnitBuf orgYuv = cs.getOrgBuf();

  m_tempBuf.copyFrom( cs.getRecoBuf() );
  PelUnitBuf recYuv = m_tempBuf.getBuf( cs.area );
  recYuv.extendBorderPel( MAX_ALF_FILTER_LENGTH >> 1 );

  // derive classification
  const CPelBuf& recLuma = recYuv.get( COMPONENT_Y );
  Area blk( 0, 0, recLuma.width, recLuma.height );
  deriveClassification( m_classifier, recLuma, blk );

  // get CTB stats for filtering
  deriveStatsForFiltering( orgYuv, recYuv );

  // derive filter (luma)
  alfEncoder( cs, alfSliceParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_LUMA
#if ENABLE_QPA
            , lambdaChromaWeight
#endif
            );

  // derive filter (chroma)
  if( alfSliceParam.enabledFlag[COMPONENT_Y] )
  {
    alfEncoder( cs, alfSliceParam, orgYuv, recYuv, cs.getRecoBuf(), CHANNEL_TYPE_CHROMA
#if ENABLE_QPA
              , lambdaChromaWeight
#endif
              );
  }
}

double EncAdaptiveLoopFilter::deriveCtbAlfEnableFlags( CodingStructure& cs, const int iShapeIdx, ChannelType channel,
#if ENABLE_QPA
                                                       const double chromaWeight,
#endif
                                                       const int numClasses, const int numCoeff, double& distUnfilter )
{
  TempCtx        ctxTempStart( m_CtxCache );
  TempCtx        ctxTempBest( m_CtxCache );
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;

  double cost = 0;
  distUnfilter = 0;

  setEnableFlag(m_alfSliceParamTemp, channel, true);
#if ENABLE_QPA
  VTMCHECK ((chromaWeight > 0.0) && (cs.slice->getSliceCurStartCtuTsAddr() != 0), "incompatible start CTU address, must be 0");
#endif

  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    for( int compID = compIDFirst; compID <= compIDLast; compID++ )
    {
      double distUnfilterCtu = getUnfilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses );

      ctxTempStart = AlfCtx( m_CABACEstimator->getCtx() );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 1;
      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfSliceParamTemp );
      double costOn = distUnfilterCtu + getFilteredDistortion( m_alfCovariance[compID][iShapeIdx][ctuIdx], numClasses, m_alfSliceParamTemp.numLumaFilters - 1, numCoeff );
#if ENABLE_QPA
      const double ctuLambda = chromaWeight > 0.0 ? (isLuma (channel) ? cs.picture->m_uEnerHpCtu[ctuIdx] : cs.picture->m_uEnerHpCtu[ctuIdx] / chromaWeight) : m_lambda[compID];
#else
      const double ctuLambda = m_lambda[compID];
#endif
      costOn += ctuLambda * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
      ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );

      m_CABACEstimator->getCtx() = AlfCtx( ctxTempStart );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 0;
      m_CABACEstimator->codeAlfCtuEnableFlag( cs, ctuIdx, compID, &m_alfSliceParamTemp);
      double costOff = distUnfilterCtu + ctuLambda * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();

      if( costOn < costOff )
      {
        cost += costOn;
        m_CABACEstimator->getCtx() = AlfCtx( ctxTempBest );
        m_ctuEnableFlag[compID][ctuIdx] = 1;
      }
      else
      {
        cost += costOff;
        m_ctuEnableFlag[compID][ctuIdx] = 0;
        distUnfilter += distUnfilterCtu;
      }
    }
  }

  if( isChroma( channel ) )
  {
    setEnableFlag(m_alfSliceParamTemp, channel, m_ctuEnableFlag);
    const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
    cost += lengthTruncatedUnary(alfChromaIdc, 3) * m_lambda[channel];
  }

  return cost;
}

void EncAdaptiveLoopFilter::alfEncoder( CodingStructure& cs, AlfSliceParam& alfSliceParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, const PelUnitBuf& recBuf, const ChannelType channel
#if ENABLE_QPA
                                      , const double lambdaChromaWeight // = 0.0
#endif
                                      )
{
  const TempCtx  ctxStart( m_CtxCache, AlfCtx( m_CABACEstimator->getCtx() ) );
  TempCtx        ctxBest( m_CtxCache );

  double costMin = MAX_DOUBLE;

  std::vector<AlfFilterShape>& alfFilterShape = alfSliceParam.filterShapes[channel];
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;
  const int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int uiCoeffBits = 0;

  for( int iShapeIdx = 0; iShapeIdx < alfFilterShape.size(); iShapeIdx++ )
  {
    m_alfSliceParamTemp = alfSliceParam;
    //1. get unfiltered distortion
    double cost = getUnfilteredDistortion( m_alfCovarianceFrame[channel][iShapeIdx], channel );
    cost /= 1.001; // slight preference for unfiltered choice

    if( cost < costMin )
    {
      costMin = cost;
      setEnableFlag( alfSliceParam, channel, false );
      // no CABAC signalling
      ctxBest = AlfCtx( ctxStart );
      setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 0 );
    }

    //2. all CTUs are on
    setEnableFlag( m_alfSliceParamTemp, channel, true );
    m_CABACEstimator->getCtx() = AlfCtx( ctxStart );
    setCtuEnableFlag( m_ctuEnableFlag, channel, 1 );
    cost = getFilterCoeffAndCost( cs, 0, channel, false, iShapeIdx, uiCoeffBits );

    if( cost < costMin )
    {
      costMin = cost;
      copyAlfSliceParam( alfSliceParam, m_alfSliceParamTemp, channel );
      ctxBest = AlfCtx( m_CABACEstimator->getCtx() );
      setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 1 );
    }

    //3. CTU decision
    double distUnfilter = 0;
    const int iterNum = isLuma(channel) ? (2 * 4 + 1) : (2 * 2 + 1);

    for( int iter = 0; iter < iterNum; iter++ )
    {
      if ((iter & 0x01) == 0)
      {
        m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
        cost = m_lambda[channel] * uiCoeffBits;
        cost += deriveCtbAlfEnableFlags(cs, iShapeIdx, channel,
#if ENABLE_QPA
                                        lambdaChromaWeight,
#endif
                                        numClasses, alfFilterShape[iShapeIdx].numCoeff, distUnfilter);
        if (cost < costMin)
        {
          costMin = cost;
          ctxBest = AlfCtx(m_CABACEstimator->getCtx());
          copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
          copyAlfSliceParam(alfSliceParam, m_alfSliceParamTemp, channel);
        }
      }
      else
      {
        // unfiltered distortion is added due to some CTBs may not use filter
        cost = getFilterCoeffAndCost(cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits);
      }
    }//for iter
  }//for shapeIdx
  m_CABACEstimator->getCtx() = AlfCtx( ctxBest );
  copyCtuEnableFlag( m_ctuEnableFlag, m_ctuEnableFlagTmp, channel );

  //filtering
  reconstructCoeff( alfSliceParam, channel, isLuma( channel ) );

  for( int compIdx = compIDFirst; compIdx <= compIDLast; compIdx++ )
  {
    ComponentID compID = (ComponentID)compIdx;
    if( alfSliceParam.enabledFlag[compID] )
    {
      const PreCalcValues& pcv = *cs.pcv;
      int ctuIdx = 0;
      const int chromaScaleX = getComponentScaleX( compID, recBuf.chromaFormat );
      const int chromaScaleY = getComponentScaleY( compID, recBuf.chromaFormat );
      AlfFilterType filterType = isLuma( compID ) ? ALF_FILTER_7 : ALF_FILTER_5;
      short* coeff = isLuma( compID ) ? m_coeffFinal : alfSliceParam.chromaCoeff;

      for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
      {
        for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
        {
          const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
          const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;
          Area blk( xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY );

          if( m_ctuEnableFlag[compID][ctuIdx] )
          {
            if( filterType == ALF_FILTER_5 )
            {
              m_filter5x5Blk( m_classifier, recBuf, recExtBuf, blk, compID, coeff, m_clpRngs.comp[compIdx] );
            }
            else if( filterType == ALF_FILTER_7 )
            {
              m_filter7x7Blk( m_classifier, recBuf, recExtBuf, blk, compID, coeff, m_clpRngs.comp[compIdx] );
            }
            else
            {
              VTMCHECK( 0, "Wrong ALF filter type" );
            }
          }
          ctuIdx++;
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyAlfSliceParam( AlfSliceParam& alfSliceParamDst, AlfSliceParam& alfSliceParamSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( &alfSliceParamDst, &alfSliceParamSrc, sizeof( AlfSliceParam ) );
  }
  else
  {
    alfSliceParamDst.enabledFlag[COMPONENT_Cb] = alfSliceParamSrc.enabledFlag[COMPONENT_Cb];
    alfSliceParamDst.enabledFlag[COMPONENT_Cr] = alfSliceParamSrc.enabledFlag[COMPONENT_Cr];
    memcpy( alfSliceParamDst.chromaCoeff, alfSliceParamSrc.chromaCoeff, sizeof( alfSliceParamDst.chromaCoeff ) );
  }
}
double EncAdaptiveLoopFilter::getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits )
{
  //collect stat based on CTU decision
  if( bReCollectStat )
  {
    getFrameStats( channel, iShapeIdx );
  }

  double dist = distUnfilter;
  uiCoeffBits = 0;
  int uiSliceFlag = 0;
  AlfFilterShape& alfFilterShape = m_alfSliceParamTemp.filterShapes[channel][iShapeIdx];
  //get filter coeff
  if( isLuma( channel ) )
  {
    //distortion
    dist += mergeFiltersAndCost( m_alfSliceParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], uiCoeffBits );
  }
  else
  {
    //distortion
    dist += m_alfCovarianceFrame[channel][iShapeIdx][0].pixAcc + deriveCoeffQuant( m_filterCoeffQuant, m_alfCovarianceFrame[channel][iShapeIdx][0].E, m_alfCovarianceFrame[channel][iShapeIdx][0].y, alfFilterShape.numCoeff, alfFilterShape.weights, m_NUM_BITS, true );
    memcpy( m_filterCoeffSet[0], m_filterCoeffQuant, sizeof( *m_filterCoeffQuant ) * alfFilterShape.numCoeff );
    //setEnableFlag( m_alfSliceParamTemp, channel, m_ctuEnableFlag );
    const int alfChromaIdc = m_alfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_alfSliceParamTemp.enabledFlag[COMPONENT_Cr];
    for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
    {
      m_alfSliceParamTemp.chromaCoeff[i] = m_filterCoeffQuant[i];
    }
    uiCoeffBits += getCoeffRate( m_alfSliceParamTemp, true );
    uiSliceFlag = lengthTruncatedUnary(alfChromaIdc, 3);
  }

  double rate = uiCoeffBits + uiSliceFlag;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->codeAlfCtuEnableFlags( cs, channel, &m_alfSliceParamTemp);
  rate += FracBitsScale * (double)m_CABACEstimator->getEstFracBits();
  return dist + m_lambda[channel] * rate;
}

int EncAdaptiveLoopFilter::getCoeffRate( AlfSliceParam& alfSliceParam, bool isChroma )
{
  int iBits = 0;
  if( !isChroma )
  {
    iBits++;                                               // alf_coefficients_delta_flag
    if( !alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      if( alfSliceParam.numLumaFilters > 1 )
      {
        iBits++;                                           // coeff_delta_pred_mode_flag
      }
    }
  }

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
  AlfFilterShape alfShape( isChroma ? 5 : 7 );
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
  const short* coeff = isChroma ? alfSliceParam.chromaCoeff : alfSliceParam.lumaCoeff;
  const int numFilters = isChroma ? 1 : alfSliceParam.numLumaFilters;

  // vlc for all
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( isChroma || !alfSliceParam.alfLumaCoeffDeltaFlag || alfSliceParam.alfLumaCoeffFlag[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        int coeffVal = abs( coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] );

        for( int k = 1; k < 15; k++ )
        {
          m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
        }
      }
    }
  }

  int kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  // Golomb parameters
  iBits += lengthUvlc( kMin - 1 );  // "min_golomb_order"
  int golombOrderIncreaseFlag = 0;

  for( int idx = 0; idx < maxGolombIdx; idx++ )
  {
    golombOrderIncreaseFlag = ( m_kMinTab[idx] != kMin ) ? 1 : 0;
    VTMCHECK( !( m_kMinTab[idx] <= kMin + 1 ), "ALF Golomb parameter not consistent" );
    iBits += golombOrderIncreaseFlag;                           //golomb_order_increase_flag
    kMin = m_kMinTab[idx];
  }

  if( !isChroma )
  {
    if( alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      iBits += numFilters;             //filter_coefficient_flag[i]
    }
  }

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !isChroma && !alfSliceParam.alfLumaCoeffFlag[ind] && alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      continue;
    }

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      iBits += lengthGolomb( coeff[ind* MAX_NUM_ALF_LUMA_COEFF + i], m_kMinTab[alfShape.golombIdx[i]] );  // alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
    }
  }
  return iBits;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, ChannelType channel )
{
  double dist = 0;
  if( isLuma( channel ) )
  {
    dist = getUnfilteredDistortion( cov, MAX_NUM_ALF_CLASSES );
  }
  else
  {
    dist = getUnfilteredDistortion( cov, 1 ) + lengthTruncatedUnary( 0, 3 ) * m_lambda[COMPONENT_Cb];
  }
  return dist;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, const int numClasses )
{
  double dist = 0;
  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    dist += cov[classIdx].pixAcc;
  }
  return dist;
}

double EncAdaptiveLoopFilter::getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff )
{
  double dist = 0;

  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    int filterIdx = numClasses == 1 ? 0 : m_filterIndices[numFiltersMinus1][classIdx];
    dist += calcErrorForCoeffs( cov[classIdx].E, cov[classIdx].y, m_filterCoeffSet[filterIdx], numCoeff, m_NUM_BITS );
  }

  return dist;
}

double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfSliceParam& alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int& uiCoeffBits )
{
  int numFiltersBest = 0;
  int numFilters = MAX_NUM_ALF_CLASSES;
  static bool codedVarBins[MAX_NUM_ALF_CLASSES];
  static double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  double cost, cost0, dist, distForce0, costMin = MAX_DOUBLE;
  int predMode = 0, bestPredMode = 0, coeffBits, coeffBitsForce0;

  mergeClasses( covFrame, covMerged, MAX_NUM_ALF_CLASSES, m_filterIndices );

  while( numFilters >= 1 )
  {
    dist = deriveFilterCoeffs( covFrame, covMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab );
    // filter coeffs are stored in m_filterCoeffSet
    distForce0 = getDistForce0( alfShape, numFilters, errorForce0CoeffTab, codedVarBins );
    coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters, predMode );
    coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFilters, codedVarBins );

    cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
    cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

    if( cost0 < cost )
    {
      cost = cost0;
    }

    if( cost <= costMin )
    {
      costMin = cost;
      numFiltersBest = numFilters;
      bestPredMode = predMode;
    }
    numFilters--;
  }

  dist = deriveFilterCoeffs( covFrame, covMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab );
  coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest, predMode );
  distForce0 = getDistForce0( alfShape, numFiltersBest, errorForce0CoeffTab, codedVarBins );
  coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFiltersBest, codedVarBins );

  cost = dist + m_lambda[COMPONENT_Y] * coeffBits;
  cost0 = distForce0 + m_lambda[COMPONENT_Y] * coeffBitsForce0;

  alfSliceParam.numLumaFilters = numFiltersBest;
  double distReturn;
  if (cost <= cost0)
  {
    distReturn = dist;
    alfSliceParam.alfLumaCoeffDeltaFlag = 0;
    uiCoeffBits = coeffBits;
    alfSliceParam.alfLumaCoeffDeltaPredictionFlag = bestPredMode;
  }
  else
  {
    distReturn = distForce0;
    alfSliceParam.alfLumaCoeffDeltaFlag = 1;
    uiCoeffBits = coeffBitsForce0;
    memcpy( alfSliceParam.alfLumaCoeffFlag, codedVarBins, sizeof( codedVarBins ) );
    alfSliceParam.alfLumaCoeffDeltaPredictionFlag = 0;

    for( int varInd = 0; varInd < numFiltersBest; varInd++ )
    {
      if( codedVarBins[varInd] == 0 )
      {
        memset( m_filterCoeffSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
      }
    }
  }

  for( int ind = 0; ind < alfSliceParam.numLumaFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff; i++ )
    {
      if( alfSliceParam.alfLumaCoeffDeltaPredictionFlag )
      {
        alfSliceParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_diffFilterCoeff[ind][i];
      }
      else
      {
        alfSliceParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      }
    }
  }

  memcpy( alfSliceParam.filterCoeffDeltaIdx, m_filterIndices[numFiltersBest - 1], sizeof( short ) * MAX_NUM_ALF_CLASSES );
  uiCoeffBits += getNonFilterCoeffRate( alfSliceParam );
  return distReturn;
}

int EncAdaptiveLoopFilter::getNonFilterCoeffRate( AlfSliceParam& alfSliceParam )
{
  int len = 1   // alf_coefficients_delta_flag
          + lengthTruncatedUnary( 0, 3 )    // chroma_idc = 0, it is signalled when ALF is enabled for luma
          + getTBlength( alfSliceParam.numLumaFilters - 1, MAX_NUM_ALF_CLASSES );   //numLumaFilters

  if( alfSliceParam.numLumaFilters > 1 )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      len += getTBlength( (int)alfSliceParam.filterCoeffDeltaIdx[i], alfSliceParam.numLumaFilters );  //filter_coeff_delta[i]
    }
  }
  return len;
}

int EncAdaptiveLoopFilter::lengthTruncatedUnary( int symbol, int maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return 0;
  }

  bool codeLast = ( maxSymbol > symbol );
  int bins = 0;
  int numBins = 0;
  while( symbol-- )
  {
    bins <<= 1;
    bins++;
    numBins++;
  }
  if( codeLast )
  {
    bins <<= 1;
    numBins++;
  }

  return numBins;
}

int EncAdaptiveLoopFilter::getTBlength( int uiSymbol, const int uiMaxSymbol )
{
  int uiThresh;
  if( uiMaxSymbol > 256 )
  {
    int uiThreshVal = 1 << 8;
    uiThresh = 8;
    while( uiThreshVal <= uiMaxSymbol )
    {
      uiThresh++;
      uiThreshVal <<= 1;
    }
    uiThresh--;
  }
  else
  {
    uiThresh = g_tbMax[uiMaxSymbol];
  }

  int uiVal = 1 << uiThresh;
  assert( uiVal <= uiMaxSymbol );
  assert( ( uiVal << 1 ) > uiMaxSymbol );
  assert( uiSymbol < uiMaxSymbol );
  int b = uiMaxSymbol - uiVal;
  assert( b < uiVal );
  if( uiSymbol < uiVal - b )
  {
    return uiThresh;
  }
  else
  {
    return uiThresh + 1;
  }
}

int EncAdaptiveLoopFilter::getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins )
{
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );
  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !codedVarBins[ind] )
    {
      continue;
    }
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      int coeffVal = abs( pDiffQFilterCoeffIntPP[ind][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
      }
    }
  }

  int kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  // Coding parameters
  int len = kMin           //min_golomb_order
          + maxGolombIdx   //golomb_order_increase_flag
          + numFilters;    //filter_coefficient_flag[i]

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( codedVarBins[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        len += lengthGolomb( abs( pDiffQFilterCoeffIntPP[ind][i] ), m_kMinTab[alfShape.golombIdx[i]] ); // alf_coeff_luma_delta[i][j]
      }
    }
  }

  return len;
}

int EncAdaptiveLoopFilter::deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, int& predMode )
{
  int ratePredMode0 = getCostFilterCoeff( alfShape, filterSet, numFilters );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( ind == 0 )
    {
      memcpy( filterCoeffDiff[ind], filterSet[ind], sizeof( int ) * alfShape.numCoeff );
    }
    else
    {
      for( int i = 0; i < alfShape.numCoeff; i++ )
      {
        filterCoeffDiff[ind][i] = filterSet[ind][i] - filterSet[ind - 1][i];
      }
    }
  }

  int ratePredMode1 = getCostFilterCoeff( alfShape, filterCoeffDiff, numFilters );

  predMode = ( ratePredMode1 < ratePredMode0 && numFilters > 1 ) ? 1 : 0;

  return ( numFilters > 1 ? 1 : 0 )        // coeff_delta_pred_mode_flag
       + ( predMode ? ratePredMode1 : ratePredMode0 ); // min_golomb_order, golomb_order_increase_flag, alf_coeff_luma_delta
}

int EncAdaptiveLoopFilter::getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters )
{
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      int coeffVal = abs( pDiffQFilterCoeffIntPP[ind][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
      }
    }
  }

  int kMin = getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  // Coding parameters
  int len = kMin           //min_golomb_order
          + maxGolombIdx;  //golomb_order_increase_flag

  // Filter coefficients
  len += lengthFilterCoeffs( alfShape, numFilters, pDiffQFilterCoeffIntPP, m_kMinTab );  // alf_coeff_luma_delta[i][j]

  return len;
}

int EncAdaptiveLoopFilter::lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab )
{
  int bitCnt = 0;

  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      bitCnt += lengthGolomb( abs( FilterCoeff[ind][i] ), kMinTab[alfShape.golombIdx[i]] );
    }
  }
  return bitCnt;
}

double EncAdaptiveLoopFilter::getDistForce0( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins )
{
  static int bitsVarBin[MAX_NUM_ALF_CLASSES];

  memset( m_bitsCoeffScan, 0, sizeof( m_bitsCoeffScan ) );
  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      int coeffVal = abs( m_filterCoeffSet[ind][i] );
      for( int k = 1; k < 15; k++ )
      {
        m_bitsCoeffScan[alfShape.golombIdx[i]][k] += lengthGolomb( coeffVal, k );
      }
    }
  }

  getGolombKMin( alfShape, numFilters, m_kMinTab, m_bitsCoeffScan );

  for( int ind = 0; ind < numFilters; ++ind )
  {
    bitsVarBin[ind] = 0;
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      bitsVarBin[ind] += lengthGolomb( abs( m_filterCoeffSet[ind][i] ), m_kMinTab[alfShape.golombIdx[i]] );
    }
  }

  double distForce0 = getDistCoeffForce0( codedVarBins, errorTabForce0Coeff, bitsVarBin, numFilters );

  return distForce0;
}

int EncAdaptiveLoopFilter::getGolombKMin( AlfFilterShape& alfShape, const int numFilters, int kMinTab[MAX_NUM_ALF_LUMA_COEFF], int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB] )
{
  int kStart;
  const int maxGolombIdx = getMaxGolombIdx( alfShape.filterType );

  int minBitsKStart = MAX_INT;
  int minKStart = -1;

  for( int k = 1; k < 8; k++ )
  {
    int bitsKStart = 0; kStart = k;
    for( int scanPos = 0; scanPos < maxGolombIdx; scanPos++ )
    {
      int kMin = kStart;
      int minBits = bitsCoeffScan[scanPos][kMin];

      if( bitsCoeffScan[scanPos][kStart + 1] < minBits )
      {
        kMin = kStart + 1;
        minBits = bitsCoeffScan[scanPos][kMin];
      }
      kStart = kMin;
      bitsKStart += minBits;
    }
    if( bitsKStart < minBitsKStart )
    {
      minBitsKStart = bitsKStart;
      minKStart = k;
    }
  }

  kStart = minKStart;
  for( int scanPos = 0; scanPos < maxGolombIdx; scanPos++ )
  {
    int kMin = kStart;
    int minBits = bitsCoeffScan[scanPos][kMin];

    if( bitsCoeffScan[scanPos][kStart + 1] < minBits )
    {
      kMin = kStart + 1;
      minBits = bitsCoeffScan[scanPos][kMin];
    }

    kMinTab[scanPos] = kMin;
    kStart = kMin;
  }

  return minKStart;
}

double EncAdaptiveLoopFilter::getDistCoeffForce0( bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, const int numFilters )
{
  double distForce0 = 0;
  std::memset( codedVarBins, 0, sizeof( *codedVarBins ) * MAX_NUM_ALF_CLASSES );

  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    double costDiff = errorForce0CoeffTab[filtIdx][0] - ( errorForce0CoeffTab[filtIdx][1] + m_lambda[COMPONENT_Y] * bitsVarBin[filtIdx] );
    codedVarBins[filtIdx] = costDiff > 0 ? true : false;
    distForce0 += errorForce0CoeffTab[filtIdx][codedVarBins[filtIdx] ? 1 : 0];
  }

  return distForce0;
}

int EncAdaptiveLoopFilter::lengthUvlc( int uiCode )
{
  int uiLength = 1;
  int uiTemp = ++uiCode;

  VTMCHECK( !uiTemp, "Integer overflow" );

  while( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  return ( uiLength >> 1 ) + ( ( uiLength + 1 ) >> 1 );
}

int EncAdaptiveLoopFilter::lengthGolomb( int coeffVal, int k )
{
  int m = 2 << ( k - 1 );
  int q = coeffVal / m;
  if( coeffVal != 0 )
  {
    return q + 2 + k;
  }
  else
  {
    return q + 1 + k;
  }
}

double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2] )
{
  double error = 0.0;
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    tmpCov.reset();
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
    {
      if( filterIndices[classIdx] == filtIdx )
      {
        tmpCov += cov[classIdx];
      }
    }

    // Find coeffcients
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant( m_filterCoeffQuant, tmpCov.E, tmpCov.y, alfShape.numCoeff, alfShape.weights, m_NUM_BITS );
    errorTabForce0Coeff[filtIdx][0] = tmpCov.pixAcc;
    error += errorTabForce0Coeff[filtIdx][1];

    // store coeff
    memcpy( m_filterCoeffSet[filtIdx], m_filterCoeffQuant, sizeof( int )*alfShape.numCoeff );
  }
  return error;
}

double EncAdaptiveLoopFilter::deriveCoeffQuant( int *filterCoeffQuant, double **E, double *y, const int numCoeff, std::vector<int>& weights, const int bitDepth, const bool bChroma )
{
  const int factor = 1 << ( bitDepth - 1 );
  static int filterCoeffQuantMod[MAX_NUM_ALF_LUMA_COEFF];
  static double filterCoeff[MAX_NUM_ALF_LUMA_COEFF];

  gnsSolveByChol( E, y, filterCoeff, numCoeff );
  roundFiltCoeff( filterCoeffQuant, filterCoeff, numCoeff, factor );
  const int targetCoeffSumInt = 0;
  int quantCoeffSum = 0;
  for( int i = 0; i < numCoeff; i++ )
  {
    quantCoeffSum += weights[i] * filterCoeffQuant[i];
  }

  int count = 0;
  while( quantCoeffSum != targetCoeffSumInt && count < 10 )
  {
    int sign = quantCoeffSum > targetCoeffSumInt ? 1 : -1;
    int diff = ( quantCoeffSum - targetCoeffSumInt ) * sign;

    double errMin = MAX_DOUBLE;
    int minInd = -1;

    for( int k = 0; k < numCoeff; k++ )
    {
      if( weights[k] <= diff )
      {
        memcpy( filterCoeffQuantMod, filterCoeffQuant, sizeof( int ) * numCoeff );

        filterCoeffQuantMod[k] -= sign;
        double error = calcErrorForCoeffs( E, y, filterCoeffQuantMod, numCoeff, bitDepth );

        if( error < errMin )
        {
          errMin = error;
          minInd = k;
        }
      }
    }

    if( minInd != -1 )
    {
      filterCoeffQuant[minInd] -= sign;
    }

    quantCoeffSum = 0;
    for( int i = 0; i < numCoeff; i++ )
    {
      quantCoeffSum += weights[i] * filterCoeffQuant[i];
    }
    ++count;
  }
  if( count == 10 )
  {
    memset( filterCoeffQuant, 0, sizeof( int ) * numCoeff );
  }

  int max_value = factor - 1;
  int min_value = -factor;

  for ( int i = 0; i < numCoeff - 1; i++ )
  {
    filterCoeffQuant[i] = std::min( max_value, std::max( min_value, filterCoeffQuant[i] ) );
    filterCoeff[i] = filterCoeffQuant[i] / double( factor );
  }

  quantCoeffSum = 0;
  for ( int i = 0; i < numCoeff - 1; i++ )
  {
    quantCoeffSum += weights[i] * filterCoeffQuant[i];
    filterCoeff[i] = filterCoeffQuant[i] / double(factor);
  }
  filterCoeffQuant[numCoeff - 1] = -quantCoeffSum;
  filterCoeff[numCoeff - 1] = filterCoeffQuant[numCoeff - 1] / double(factor);


  //Restrict the range of the center coefficient
  int max_value_center = (2 * factor - 1) - factor;
  int min_value_center = 0 - factor;

  filterCoeffQuant[numCoeff - 1] = std::min(max_value_center, std::max(min_value_center, filterCoeffQuant[numCoeff - 1]));
  filterCoeff[numCoeff - 1] = filterCoeffQuant[numCoeff - 1] / double(factor);

  int coeffQuantAdjust[MAX_NUM_ALF_LUMA_COEFF];
  int adjustedTotalCoeff = (numCoeff - 1) << 1;

  count = 0;
  quantCoeffSum += filterCoeffQuant[numCoeff - 1];
  while (quantCoeffSum != targetCoeffSumInt && count < 15)
  {
    int sign = quantCoeffSum > targetCoeffSumInt ? 1 : -1;
    int diff = (quantCoeffSum - targetCoeffSumInt) * sign;

    if (diff > 4 * adjustedTotalCoeff)     sign = sign * 8;
    else if (diff > 2 * adjustedTotalCoeff)     sign = sign * 4;
    else if (diff >     adjustedTotalCoeff)     sign = sign * 2;

    double errMin = MAX_DOUBLE;
    int    minInd = -1;

    for (int k = 0; k < numCoeff - 1; k++)
    {
      memcpy(coeffQuantAdjust, filterCoeffQuant, sizeof(int) * numCoeff);

      coeffQuantAdjust[k] -= sign;

      if (coeffQuantAdjust[k] <= max_value && coeffQuantAdjust[k] >= min_value)
      {
        double error = calcErrorForCoeffs(E, y, coeffQuantAdjust, numCoeff, bitDepth);

        if (error < errMin)
        {
          errMin = error;
          minInd = k;
        }
      }
    }

    if (minInd != -1)
    {
      filterCoeffQuant[minInd] -= sign;
      quantCoeffSum -= (weights[minInd] * sign);
    }

    ++count;
  }

  if (quantCoeffSum != targetCoeffSumInt)
  {
    memset(filterCoeffQuant, 0, sizeof(int) * numCoeff);
  }

  for (int i = 0; i < numCoeff - 1; i++)
  {
    VTMCHECK(filterCoeffQuant[i] > max_value || filterCoeffQuant[i] < min_value, "filterCoeffQuant[i]>max_value || filterCoeffQuant[i]<min_value");
    filterCoeff[i] = filterCoeffQuant[i] / double(factor);
  }
  VTMCHECK(filterCoeffQuant[numCoeff - 1] > max_value_center || filterCoeffQuant[numCoeff - 1] < min_value_center, "filterCoeffQuant[numCoeff-1]>max_value_center || filterCoeffQuant[numCoeff-1]<min_value_center");
  filterCoeff[numCoeff - 1] = filterCoeffQuant[numCoeff - 1] / double(factor);


  double error = calcErrorForCoeffs( E, y, filterCoeffQuant, numCoeff, bitDepth );
  return error;
}

double EncAdaptiveLoopFilter::calcErrorForCoeffs( double **E, double *y, int *coeff, const int numCoeff, const int bitDepth )
{
  double factor = 1 << ( bitDepth - 1 );
  double error = 0;

  for( int i = 0; i < numCoeff; i++ )   //diagonal
  {
    double sum = 0;
    for( int j = i + 1; j < numCoeff; j++ )
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E[i][j] * coeff[j];
    }
    error += ( ( E[i][i] * coeff[i] + sum * 2 ) / factor - 2 * y[i] ) * coeff[i];
  }

  return error / factor;
}

void EncAdaptiveLoopFilter::roundFiltCoeff( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor )
{
  for( int i = 0; i < numCoeff; i++ )
  {
    int sign = filterCoeff[i] > 0 ? 1 : -1;
    filterCoeffQuant[i] = int( filterCoeff[i] * sign * factor + 0.5 ) * sign;
  }
}

void EncAdaptiveLoopFilter::mergeClasses( AlfCovariance* cov, AlfCovariance* covMerged, const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] )
{
  static bool availableClass[MAX_NUM_ALF_CLASSES];
  static uint8_t indexList[MAX_NUM_ALF_CLASSES];
  static uint8_t indexListTemp[MAX_NUM_ALF_CLASSES];
  int numRemaining = numClasses;

  memset( filterIndices, 0, sizeof( short ) * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES );

  for( int i = 0; i < numClasses; i++ )
  {
    filterIndices[numRemaining - 1][i] = i;
    indexList[i] = i;
    availableClass[i] = true;
    covMerged[i] = cov[i];
  }

  // Try merging different covariance matrices

  // temporal AlfCovariance structure is allocated as the last element in covMerged array, the size of covMerged is MAX_NUM_ALF_CLASSES + 1
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];

  while( numRemaining > 2 )
  {
    double errorMin = std::numeric_limits<double>::max();
    int bestToMergeIdx1 = 0, bestToMergeIdx2 = 1;

    for( int i = 0; i < numClasses - 1; i++ )
    {
      if( availableClass[i] )
      {
        for( int j = i + 1; j < numClasses; j++ )
        {
          if( availableClass[j] )
          {
            double error1 = calculateError( covMerged[i] );
            double error2 = calculateError( covMerged[j] );

            tmpCov.add( covMerged[i], covMerged[j] );
            double error = calculateError( tmpCov ) - error1 - error2;

            if( error < errorMin )
            {
              errorMin = error;
              bestToMergeIdx1 = i;
              bestToMergeIdx2 = j;
            }
          }
        }
      }
    }

    covMerged[bestToMergeIdx1] += covMerged[bestToMergeIdx2];
    availableClass[bestToMergeIdx2] = false;

    for( int i = 0; i < numClasses; i++ )
    {
      if( indexList[i] == bestToMergeIdx2 )
      {
        indexList[i] = bestToMergeIdx1;
      }
    }

    numRemaining--;
    if( numRemaining <= numClasses )
    {
      std::memcpy( indexListTemp, indexList, sizeof( uint8_t ) * numClasses );

      bool exist = false;
      int ind = 0;

      for( int j = 0; j < numClasses; j++ )
      {
        exist = false;
        for( int i = 0; i < numClasses; i++ )
        {
          if( indexListTemp[i] == j )
          {
            exist = true;
            break;
          }
        }

        if( exist )
        {
          for( int i = 0; i < numClasses; i++ )
          {
            if( indexListTemp[i] == j )
            {
              filterIndices[numRemaining - 1][i] = ind;
              indexListTemp[i] = -1;
            }
          }
          ind++;
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::getFrameStats( ChannelType channel, int iShapeIdx )
{
  int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  for( int i = 0; i < numClasses; i++ )
  {
    m_alfCovarianceFrame[channel][iShapeIdx][i].reset();
  }
  if( isLuma( channel ) )
  {
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_LUMA][iShapeIdx], m_alfCovariance[COMPONENT_Y][iShapeIdx], m_ctuEnableFlag[COMPONENT_Y], numClasses );
  }
  else
  {
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cb][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cb], numClasses );
    getFrameStat( m_alfCovarianceFrame[CHANNEL_TYPE_CHROMA][iShapeIdx], m_alfCovariance[COMPONENT_Cr][iShapeIdx], m_ctuEnableFlag[COMPONENT_Cr], numClasses );
  }
}

void EncAdaptiveLoopFilter::getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, const int numClasses )
{
  for( int i = 0; i < m_numCTUsInPic; i++ )
  {
    if( ctbEnableFlags[i] )
    {
      for( int j = 0; j < numClasses; j++ )
      {
        frameCov[j] += ctbCov[i][j];
      }
    }
  }
}

void EncAdaptiveLoopFilter::deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv )
{
  int ctuRsAddr = 0;
  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );

  // init CTU stats buffers
  for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

    for( int shape = 0; shape != m_filterShapes[toChannelType( compID )].size(); shape++ )
    {
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
        for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
        {
          m_alfCovariance[compIdx][shape][ctuIdx][classIdx].reset();
        }
      }
    }
  }

  // init Frame stats buffers
  const int numberOfChannels = getNumberValidChannels( m_chromaFormat );
  for( int channelIdx = 0; channelIdx < numberOfChannels; channelIdx++ )
  {
    const ChannelType channelID = ChannelType( channelIdx );
    const int numClasses = isLuma( channelID ) ? MAX_NUM_ALF_CLASSES : 1;

    for( int shape = 0; shape != m_filterShapes[channelIdx].size(); shape++ )
    {
      for( int classIdx = 0; classIdx < numClasses; classIdx++ )
      {
        m_alfCovarianceFrame[channelIdx][shape][classIdx].reset();
      }
    }
  }

  for( int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight )
  {
    for( int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth )
    {
      const int width = ( xPos + m_maxCUWidth > m_picWidth ) ? ( m_picWidth - xPos ) : m_maxCUWidth;
      const int height = ( yPos + m_maxCUHeight > m_picHeight ) ? ( m_picHeight - yPos ) : m_maxCUHeight;
      const UnitArea area( m_chromaFormat, Area( xPos, yPos, width, height ) );

      for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
      {
        const ComponentID compID = ComponentID( compIdx );
        const CompArea& compArea = area.block( compID );

        int  recStride = recYuv.get( compID ).stride;
        Pel* rec = recYuv.get( compID ).bufAt( compArea );

        int  orgStride = orgYuv.get( compID ).stride;
        Pel* org = orgYuv.get( compID ).bufAt( compArea );

        ChannelType chType = toChannelType( compID );

        for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
        {
          getBlkStats( m_alfCovariance[compIdx][shape][ctuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : m_classifier, org, orgStride, rec, recStride, compArea );

          const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

          for( int classIdx = 0; classIdx < numClasses; classIdx++ )
          {
            m_alfCovarianceFrame[chType][shape][classIdx] += m_alfCovariance[compIdx][shape][ctuRsAddr][classIdx];
          }
        }
      }
      ctuRsAddr++;
    }
  }
}

void EncAdaptiveLoopFilter::getBlkStats( AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& area )
{
  static int ELocal[MAX_NUM_ALF_LUMA_COEFF];

  int transposeIdx = 0;
  int classIdx = 0;

  for( int i = 0; i < area.height; i++ )
  {
    for( int j = 0; j < area.width; j++ )
    {
      std::memset( ELocal, 0, shape.numCoeff * sizeof( int ) );
      if( classifier )
      {
        AlfClassifier& cl = classifier[area.y + i][area.x + j];
        transposeIdx = cl.transposeIdx;
        classIdx = cl.classIdx;
      }

#if JVET_M0427_INLOOP_RESHAPER
      double weight = 1.0;
      if (m_alfWSSD)
      {
        weight = m_lumaLevelToWeightPLUT[org[j]];
      }
#endif
      int yLocal = org[j] - rec[j];
      calcCovariance( ELocal, rec + j, recStride, shape.pattern.data(), shape.filterLength >> 1, transposeIdx );
      for( int k = 0; k < shape.numCoeff; k++ )
      {
        for( int l = k; l < shape.numCoeff; l++ )
        {
#if JVET_M0427_INLOOP_RESHAPER
          if (m_alfWSSD)
          {
            alfCovariace[classIdx].E[k][l] += weight * (double)(ELocal[k] * ELocal[l]);
          }
          else
#endif
          alfCovariace[classIdx].E[k][l] += ELocal[k] * ELocal[l];
        }
#if JVET_M0427_INLOOP_RESHAPER
        if (m_alfWSSD)
        {
          alfCovariace[classIdx].y[k] += weight * (double)(ELocal[k] * yLocal);
        }
        else
#endif
        alfCovariace[classIdx].y[k] += ELocal[k] * yLocal;
      }
#if JVET_M0427_INLOOP_RESHAPER
      if (m_alfWSSD)
      {
        alfCovariace[classIdx].pixAcc += weight * (double)(yLocal * yLocal);
      }
      else
#endif
      alfCovariace[classIdx].pixAcc += yLocal * yLocal;
    }
    org += orgStride;
    rec += recStride;
  }

  int numClasses = classifier ? MAX_NUM_ALF_CLASSES : 1;
  for( classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    for( int k = 1; k < shape.numCoeff; k++ )
    {
      for( int l = 0; l < k; l++ )
      {
        alfCovariace[classIdx].E[k][l] = alfCovariace[classIdx].E[l][k];
      }
    }
  }
}

void EncAdaptiveLoopFilter::calcCovariance( int *ELocal, const Pel *rec, const int stride, const int *filterPattern, const int halfFilterLength, const int transposeIdx )
{
  int k = 0;

  if( transposeIdx == 0 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;

      for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++ )
      {
        ELocal[filterPattern[k++]] += rec0[j] + rec1[-j];
      }
    }
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      ELocal[filterPattern[k++]] += rec[j] + rec[-j];
    }
  }
  else if( transposeIdx == 1 )
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

      for( int i = -halfFilterLength - j; i <= halfFilterLength + j; i++ )
      {
        ELocal[filterPattern[k++]] += rec0[i * stride] + rec1[-i * stride];
      }
    }
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      ELocal[filterPattern[k++]] += rec[i*stride] + rec[-i * stride];
    }
  }
  else if( transposeIdx == 2 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + i * stride;
      const Pel* rec1 = rec - i * stride;

      for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j-- )
      {
        ELocal[filterPattern[k++]] += rec0[j] + rec1[-j];
      }
    }
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      ELocal[filterPattern[k++]] += rec[j] + rec[-j];
    }
  }
  else
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;

      for( int i = halfFilterLength + j; i >= -halfFilterLength - j; i-- )
      {
        ELocal[filterPattern[k++]] += rec0[i * stride] + rec1[-i * stride];
      }
    }
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      ELocal[filterPattern[k++]] += rec[i*stride] + rec[-i * stride];
    }
  }
  ELocal[filterPattern[k++]] += rec[0];
}



double EncAdaptiveLoopFilter::calculateError( AlfCovariance& cov )
{
  static double c[MAX_NUM_ALF_COEFF];

  gnsSolveByChol( cov.E, cov.y, c, cov.numCoeff );

  double sum = 0;
  for( int i = 0; i < cov.numCoeff; i++ )
  {
    sum += c[i] * cov.y[i];
  }

  return cov.pixAcc - sum;
}

//********************************
// Cholesky decomposition
//********************************

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#define REG              0.0001
#define REG_SQR          0.0000001

//Find filter coeff related
int EncAdaptiveLoopFilter::gnsCholeskyDec( double **inpMatr, double outMatr[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], int numEq )
{
  static double invDiag[MAX_NUM_ALF_COEFF];  /* Vector of the inverse of diagonal entries of outMatr */

  for( int i = 0; i < numEq; i++ )
  {
    for( int j = i; j < numEq; j++ )
    {
      /* Compute the scaling factor */
      double scale = inpMatr[i][j];
      if( i > 0 )
      {
        for( int k = i - 1; k >= 0; k-- )
        {
          scale -= outMatr[k][j] * outMatr[k][i];
        }
      }

      /* Compute i'th row of outMatr */
      if( i == j )
      {
        if( scale <= REG_SQR ) // if(scale <= 0 )  /* If inpMatr is singular */
        {
          return 0;
        }
        else              /* Normal operation */
          invDiag[i] = 1.0 / ( outMatr[i][i] = sqrt( scale ) );
      }
      else
      {
        outMatr[i][j] = scale * invDiag[i]; /* Upper triangular part          */
        outMatr[j][i] = 0.0;              /* Lower triangular part set to 0 */
      }
    }
  }
  return 1; /* Signal that Cholesky factorization is successfully performed */
}

void EncAdaptiveLoopFilter::gnsTransposeBacksubstitution( double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* rhs, double* x, int order )
{
  /* Backsubstitution starts */
  x[0] = rhs[0] / U[0][0];               /* First row of U'                   */
  for( int i = 1; i < order; i++ )
  {         /* For the rows 1..order-1           */

    double sum = 0; //Holds backsubstitution from already handled rows

    for( int j = 0; j < i; j++ ) /* Backsubst already solved unknowns */
    {
      sum += x[j] * U[j][i];
    }

    x[i] = ( rhs[i] - sum ) / U[i][i];       /* i'th component of solution vect.  */
  }
}

void EncAdaptiveLoopFilter::gnsBacksubstitution( double R[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* z, int size, double* A )
{
  size--;
  A[size] = z[size] / R[size][size];

  for( int i = size - 1; i >= 0; i-- )
  {
    double sum = 0;

    for( int j = i + 1; j <= size; j++ )
    {
      sum += R[i][j] * A[j];
    }

    A[i] = ( z[i] - sum ) / R[i][i];
  }
}

int EncAdaptiveLoopFilter::gnsSolveByChol( double **LHS, double *rhs, double *x, int numEq )
{
  static double aux[MAX_NUM_ALF_COEFF];     /* Auxiliary vector */
  static double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF];    /* Upper triangular Cholesky factor of LHS */
  int res = 1;  // Signal that Cholesky factorization is successfully performed

                /* The equation to be solved is LHSx = rhs */

                /* Compute upper triangular U such that U'*U = LHS */
  if( gnsCholeskyDec( LHS, U, numEq ) ) /* If Cholesky decomposition has been successful */
  {
    /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
    * Solve U'*aux = rhs for aux
    */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
    gnsBacksubstitution( U, aux, numEq, x );

  }
  else /* LHS was singular */
  {
    res = 0;

    /* Regularize LHS */
    for( int i = 0; i < numEq; i++ )
    {
      LHS[i][i] += REG;
    }

    /* Compute upper triangular U such that U'*U = regularized LHS */
    res = gnsCholeskyDec( LHS, U, numEq );

    if( !res )
    {
      std::memset( x, 0, sizeof( double )*numEq );
      return 0;
    }

    /* Solve  U'*aux = rhs for aux */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* Solve U*x = aux for x */
    gnsBacksubstitution( U, aux, numEq, x );
  }
  return res;
}
//////////////////////////////////////////////////////////////////////////////////////////
void EncAdaptiveLoopFilter::setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, bool val )
{
  if( channel == CHANNEL_TYPE_LUMA )
  {
    alfSlicePara.enabledFlag[COMPONENT_Y] = val;
  }
  else
  {
    alfSlicePara.enabledFlag[COMPONENT_Cb] = alfSlicePara.enabledFlag[COMPONENT_Cr] = val;
  }
}

void EncAdaptiveLoopFilter::setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags )
{
  const ComponentID compIDFirst = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMPONENT_Y : COMPONENT_Cr;
  for( int compId = compIDFirst; compId <= compIDLast; compId++ )
  {
    alfSlicePara.enabledFlag[compId] = false;
    for( int i = 0; i < m_numCTUsInPic; i++ )
    {
      if( ctuFlags[compId][i] )
      {
        alfSlicePara.enabledFlag[compId] = true;
        break;
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyCtuEnableFlag( uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( ctuFlagsDst[COMPONENT_Y], ctuFlagsSrc[COMPONENT_Y], sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memcpy( ctuFlagsDst[COMPONENT_Cb], ctuFlagsSrc[COMPONENT_Cb], sizeof( uint8_t ) * m_numCTUsInPic );
    memcpy( ctuFlagsDst[COMPONENT_Cr], ctuFlagsSrc[COMPONENT_Cr], sizeof( uint8_t ) * m_numCTUsInPic );
  }
}

void EncAdaptiveLoopFilter::setCtuEnableFlag( uint8_t** ctuFlags, ChannelType channel, uint8_t val )
{
  if( isLuma( channel ) )
  {
    memset( ctuFlags[COMPONENT_Y], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memset( ctuFlags[COMPONENT_Cb], val, sizeof( uint8_t ) * m_numCTUsInPic );
    memset( ctuFlags[COMPONENT_Cr], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
}

