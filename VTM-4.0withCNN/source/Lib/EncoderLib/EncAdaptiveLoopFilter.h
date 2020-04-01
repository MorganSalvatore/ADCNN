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

/** \file     EncAdaptiveLoopFilter.h
 \brief    estimation part of adaptive loop filter class (header)
 */

#ifndef __ENCADAPTIVELOOPFILTER__
#define __ENCADAPTIVELOOPFILTER__

#include "CommonLib/AdaptiveLoopFilter.h"

#include "CABACWriter.h"

struct AlfCovariance
{
  int numCoeff;
  double *y;
  double **E;
  double pixAcc;

  AlfCovariance() {}
  ~AlfCovariance() {}

  void create( int size )
  {
    numCoeff = size;

    y = new double[numCoeff];
    E = new double*[numCoeff];

    for( int i = 0; i < numCoeff; i++ )
    {
      E[i] = new double[numCoeff];
    }
  }

  void destroy()
  {
    for( int i = 0; i < numCoeff; i++ )
    {
      delete[] E[i];
      E[i] = nullptr;
    }

    delete[] E;
    E = nullptr;

    delete[] y;
    y = nullptr;
  }

  void reset()
  {
    pixAcc = 0;
    std::memset( y, 0, sizeof( *y ) * numCoeff );
    for( int i = 0; i < numCoeff; i++ )
    {
      std::memset( E[i], 0, sizeof( *E[i] ) * numCoeff );
    }
  }

  const AlfCovariance& operator=( const AlfCovariance& src )
  {
    for( int i = 0; i < numCoeff; i++ )
    {
      std::memcpy( E[i], src.E[i], sizeof( *E[i] ) * numCoeff );
    }
    std::memcpy( y, src.y, sizeof( *y ) * numCoeff );
    pixAcc = src.pixAcc;

    return *this;
  }

  void add( const AlfCovariance& lhs, const AlfCovariance& rhs )
  {
    for( int j = 0; j < numCoeff; j++ )
    {
      for( int i = 0; i < numCoeff; i++ )
      {
        E[j][i] = lhs.E[j][i] + rhs.E[j][i];
      }
      y[j] = lhs.y[j] + rhs.y[j];
    }
    pixAcc = lhs.pixAcc + rhs.pixAcc;
  }

  const AlfCovariance& operator+= ( const AlfCovariance& src )
  {
    for( int j = 0; j < numCoeff; j++ )
    {
      for( int i = 0; i < numCoeff; i++ )
      {
        E[j][i] += src.E[j][i];
      }
      y[j] += src.y[j];
    }
    pixAcc += src.pixAcc;

    return *this;
  }

  const AlfCovariance& operator-= ( const AlfCovariance& src )
  {
    for( int j = 0; j < numCoeff; j++ )
    {
      for( int i = 0; i < numCoeff; i++ )
      {
        E[j][i] -= src.E[j][i];
      }
      y[j] -= src.y[j];
    }
    pixAcc -= src.pixAcc;

    return *this;
  }
};

class EncAdaptiveLoopFilter : public AdaptiveLoopFilter
{
public:
  static constexpr int   m_MAX_SCAN_VAL = 11;
  static constexpr int   m_MAX_EXP_GOLOMB = 16;
#if JVET_M0427_INLOOP_RESHAPER
  int m_alfWSSD;
  inline void           setAlfWSSD(int alfWSSD) { m_alfWSSD = alfWSSD; }
  static std::vector<double>  m_lumaLevelToWeightPLUT;
  inline std::vector<double>& getLumaLevelWeightTable() { return m_lumaLevelToWeightPLUT; }
#endif

private:
  AlfCovariance***       m_alfCovariance[MAX_NUM_COMPONENT];          // [compIdx][shapeIdx][ctbAddr][classIdx]
  AlfCovariance**        m_alfCovarianceFrame[MAX_NUM_CHANNEL_TYPE];   // [CHANNEL][shapeIdx][classIdx]
  uint8_t*                 m_ctuEnableFlagTmp[MAX_NUM_COMPONENT];

  //for RDO
  AlfSliceParam          m_alfSliceParamTemp;
  AlfCovariance          m_alfCovarianceMerged[ALF_NUM_OF_FILTER_TYPES][MAX_NUM_ALF_CLASSES + 1];
  CABACWriter*           m_CABACEstimator;
  CtxCache*              m_CtxCache;
  double                 m_lambda[MAX_NUM_COMPONENT];
  const double           FracBitsScale = 1.0 / double( 1 << SCALE_BITS );

  int*                   m_filterCoeffQuant;
  int**                  m_filterCoeffSet;
  int**                  m_diffFilterCoeff;
  int                    m_kMinTab[MAX_NUM_ALF_LUMA_COEFF];
  int                    m_bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB];
  short                  m_filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES];

public:
  EncAdaptiveLoopFilter();
  virtual ~EncAdaptiveLoopFilter() {}

  void ALFProcess( CodingStructure& cs, const double *lambdas,
#if ENABLE_QPA
                   const double lambdaChromaWeight,
#endif
                   AlfSliceParam& alfSliceParam );
  void initCABACEstimator( CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice );
  void create( const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE], const int internalBitDepth[MAX_NUM_CHANNEL_TYPE] );
  void destroy();
  static int lengthGolomb( int coeffVal, int k );
  static int getGolombKMin( AlfFilterShape& alfShape, const int numFilters, int kMinTab[MAX_NUM_ALF_LUMA_COEFF], int bitsCoeffScan[m_MAX_SCAN_VAL][m_MAX_EXP_GOLOMB] );

private:
  void   alfEncoder( CodingStructure& cs, AlfSliceParam& alfSliceParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& recExtBuf, const PelUnitBuf& recBuf, const ChannelType channel
#if ENABLE_QPA
                   , const double lambdaChromaWeight = 0.0
#endif
                   );

  void   copyAlfSliceParam( AlfSliceParam& alfSliceParamDst, AlfSliceParam& alfSliceParamSrc, ChannelType channel );
  double mergeFiltersAndCost( AlfSliceParam& alfSliceParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int& uiCoeffBits );

  void   getFrameStats( ChannelType channel, int iShapeIdx );
  void   getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, const int numClasses );
  void   deriveStatsForFiltering( PelUnitBuf& orgYuv, PelUnitBuf& recYuv );
  void   getBlkStats( AlfCovariance* alfCovariace, const AlfFilterShape& shape, AlfClassifier** classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& area );
  void   calcCovariance( int *ELocal, const Pel *rec, const int stride, const int *filterPattern, const int halfFilterLength, const int transposeIdx );
  void   mergeClasses( AlfCovariance* cov, AlfCovariance* covMerged, const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] );

  double calculateError( AlfCovariance& cov );
  double calcErrorForCoeffs( double **E, double *y, int *coeff, const int numCoeff, const int bitDepth );
  double getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits );
  double deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, AlfFilterShape& alfShape, short* filterIndices, int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2] );
  int    deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters, int& predMode );
  double deriveCoeffQuant( int *filterCoeffQuant, double **E, double *y, const int numCoeff, std::vector<int>& weights, const int bitDepth, const bool bChroma = false );
  double deriveCtbAlfEnableFlags( CodingStructure& cs, const int iShapeIdx, ChannelType channel,
#if ENABLE_QPA
                                  const double chromaWeight,
#endif
                                  const int numClasses, const int numCoeff, double& distUnfilter );
  void   roundFiltCoeff( int *filterCoeffQuant, double *filterCoeff, const int numCoeff, const int factor );

  double getDistCoeffForce0( bool* codedVarBins, double errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, const int numFilters );
  int    lengthTruncatedUnary( int symbol, int maxSymbol );
  int    lengthUvlc( int uiCode );
  int    getNonFilterCoeffRate( AlfSliceParam& alfSliceParam );
  int    getTBlength( int uiSymbol, const int uiMaxSymbol );

  int    getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins );
  int    getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters );
  int    lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff, int* kMinTab );
  double getDistForce0( AlfFilterShape& alfShape, const int numFilters, double errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins );
  int    getCoeffRate( AlfSliceParam& alfSliceParam, bool isChroma );

  double getUnfilteredDistortion( AlfCovariance* cov, ChannelType channel );
  double getUnfilteredDistortion( AlfCovariance* cov, const int numClasses );
  double getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff );

  // Cholesky decomposition
  int  gnsSolveByChol( double **LHS, double *rhs, double *x, int numEq );
  void gnsBacksubstitution( double R[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* z, int size, double* A );
  void gnsTransposeBacksubstitution( double U[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], double* rhs, double* x, int order );
  int  gnsCholeskyDec( double **inpMatr, double outMatr[MAX_NUM_ALF_COEFF][MAX_NUM_ALF_COEFF], int numEq );

  void setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, bool val );
  void setEnableFlag( AlfSliceParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags );
  void setCtuEnableFlag( uint8_t** ctuFlags, ChannelType channel, uint8_t val );
  void copyCtuEnableFlag( uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel );
};



#endif
