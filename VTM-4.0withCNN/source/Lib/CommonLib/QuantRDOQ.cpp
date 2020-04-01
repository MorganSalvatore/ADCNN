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

/** \file     QuantRDOQ.cpp
    \brief    transform and quantization class
*/

#include "QuantRDOQ.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "CrossCompPrediction.h"

#include "dtrace_next.h"
#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>


struct coeffGroupRDStats
{
  int    iNNZbeforePos0;
  double d64CodedLevelandDist; // distortion and level cost only
  double d64UncodedDist;    // all zero coded block distortion
  double d64SigCost;
  double d64SigCost_0;
};


//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================


// ====================================================================================================================
// Static functions
// ====================================================================================================================

// ====================================================================================================================
// QuantRDOQ class member functions
// ====================================================================================================================


QuantRDOQ::QuantRDOQ( const Quant* other ) : Quant( other )
{

  const QuantRDOQ *rdoq = dynamic_cast<const QuantRDOQ*>( other );
  VTMCHECK( other && !rdoq, "The RDOQ cast must be successfull!" );
#if HEVC_USE_SCALING_LISTS
  xInitScalingList( rdoq );
#endif
}

QuantRDOQ::~QuantRDOQ()
{
#if HEVC_USE_SCALING_LISTS
  xDestroyScalingList();
#endif
}




/** Get the best level in RD sense
 *
 * \returns best quantized transform level for given scan position
 *
 * This method calculates the best quantized transform level for a given scan position.
 */
inline uint32_t QuantRDOQ::xGetCodedLevel( double&            rd64CodedCost,
                                       double&            rd64CodedCost0,
                                       double&            rd64CodedCostSig,
                                       Intermediate_Int   lLevelDouble,
                                       uint32_t               uiMaxAbsLevel,
                                       const BinFracBits* fracBitsSig,
                                       const BinFracBits& fracBitsPar,
                                       const BinFracBits& fracBitsGt1,
                                       const BinFracBits& fracBitsGt2,
                                       const int          remGt2Bins,
                                       const int          remRegBins,
                                       unsigned           goRiceZero,
                                       uint16_t             ui16AbsGoRice,
                                       int                iQBits,
                                       double             errorScale,
                                       bool               bLast,
                                       bool               useLimitedPrefixLength,
                                       const int          maxLog2TrDynamicRange
                                     ) const
{
  double dCurrCostSig   = 0;
  uint32_t   uiBestAbsLevel = 0;

  if( !bLast && uiMaxAbsLevel < 3 )
  {
    rd64CodedCostSig    = xGetRateSigCoef( *fracBitsSig, 0 );
    rd64CodedCost       = rd64CodedCost0 + rd64CodedCostSig;
    if( uiMaxAbsLevel == 0 )
    {
      return uiBestAbsLevel;
    }
  }
  else
  {
    rd64CodedCost       = MAX_DOUBLE;
  }

  if( !bLast )
  {
    dCurrCostSig        = xGetRateSigCoef( *fracBitsSig, 1 );
  }

  uint32_t uiMinAbsLevel    = ( uiMaxAbsLevel > 1 ? uiMaxAbsLevel - 1 : 1 );
  for( int uiAbsLevel  = uiMaxAbsLevel; uiAbsLevel >= uiMinAbsLevel ; uiAbsLevel-- )
  {
    double dErr         = double( lLevelDouble  - ( Intermediate_Int(uiAbsLevel) << iQBits ) );

#if JVET_M0470
    double dCurrCost    = dErr * dErr * errorScale + xGetICost( xGetICRate( uiAbsLevel, fracBitsPar, fracBitsGt1, fracBitsGt2, remGt2Bins, remRegBins, goRiceZero, ui16AbsGoRice, true, maxLog2TrDynamicRange ) );
#else
    double dCurrCost    = dErr * dErr * errorScale + xGetICost( xGetICRate( uiAbsLevel, fracBitsPar, fracBitsGt1, fracBitsGt2, remGt2Bins, remRegBins, goRiceZero, ui16AbsGoRice, useLimitedPrefixLength, maxLog2TrDynamicRange ) );
#endif
    dCurrCost          += dCurrCostSig;

    if( dCurrCost < rd64CodedCost )
    {
      uiBestAbsLevel    = uiAbsLevel;
      rd64CodedCost     = dCurrCost;
      rd64CodedCostSig  = dCurrCostSig;
    }
  }

  return uiBestAbsLevel;
}

/** Calculates the cost for specific absolute transform level
 * \param uiAbsLevel scaled quantized level
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16AbsGoRice Rice parameter for coeff_abs_level_minus3
 * \param c1Idx
 * \param c2Idx
 * \param useLimitedPrefixLength
 * \param maxLog2TrDynamicRange
 * \returns cost of given absolute transform level
 */
inline int QuantRDOQ::xGetICRate( const uint32_t         uiAbsLevel,
                                  const BinFracBits& fracBitsPar,
                                  const BinFracBits& fracBitsGt1,
                                  const BinFracBits& fracBitsGt2,
                                  const int          remGt2Bins,
                                  const int          remRegBins,
                                  unsigned           goRiceZero,
                                  const uint16_t       ui16AbsGoRice,
                                  const bool         useLimitedPrefixLength,
                                  const int          maxLog2TrDynamicRange  ) const
{
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  if( remRegBins < 4 )
#else
  if( remRegBins < 3 )
#endif
  {
    int       iRate   = int( xGetIEPRate() ); // cost of sign bit
    uint32_t  symbol  = ( uiAbsLevel == 0 ? goRiceZero : uiAbsLevel <= goRiceZero ? uiAbsLevel-1 : uiAbsLevel );
    uint32_t  length;
#if JVET_M0470
    const int threshold = COEF_REMAIN_BIN_REDUCTION;
#else
    const int threshold = g_auiGoRiceRange[ui16AbsGoRice];
#endif
    if( symbol < ( threshold << ui16AbsGoRice ) )
    {
      length = symbol >> ui16AbsGoRice;
      iRate += ( length + 1 + ui16AbsGoRice ) << SCALE_BITS;
    }
    else if( useLimitedPrefixLength )
    {
      const uint32_t maximumPrefixLength = ( 32 - ( COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange ) );

      uint32_t prefixLength = 0;
      uint32_t suffix = ( symbol >> ui16AbsGoRice ) - COEF_REMAIN_BIN_REDUCTION;

      while( ( prefixLength < maximumPrefixLength ) && ( suffix > ( ( 2 << prefixLength ) - 2 ) ) )
      {
        prefixLength++;
      }

      const uint32_t suffixLength = ( prefixLength == maximumPrefixLength ) ? ( maxLog2TrDynamicRange - ui16AbsGoRice ) : ( prefixLength + 1/*separator*/ );

      iRate += ( COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ui16AbsGoRice ) << SCALE_BITS;
    }
    else
    {
      length = ui16AbsGoRice;
      symbol = symbol - ( threshold << ui16AbsGoRice );
      while( symbol >= ( 1 << length ) )
      {
        symbol -= ( 1 << ( length++ ) );
      }
      iRate += ( threshold + length + 1 - ui16AbsGoRice + length ) << SCALE_BITS;
    }
    return iRate;
  }

  int iRate = int( xGetIEPRate() ); // cost of sign bit
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  const uint32_t cthres = 4;
#else
  const uint32_t cthres = ( remGt2Bins ? 4 : 2 );
#endif
  if( uiAbsLevel >= cthres )
  {
    uint32_t symbol = ( uiAbsLevel - cthres ) >> 1;
    uint32_t length;
#if JVET_M0470
    const int threshold = COEF_REMAIN_BIN_REDUCTION;
#else
    const int threshold = g_auiGoRiceRange[ui16AbsGoRice];
#endif
    if( symbol < ( threshold << ui16AbsGoRice ) )
    {
      length = symbol >> ui16AbsGoRice;
      iRate += ( length + 1 + ui16AbsGoRice ) << SCALE_BITS;
    }
    else if( useLimitedPrefixLength )
    {
      const uint32_t maximumPrefixLength = ( 32 - ( COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange ) );

      uint32_t prefixLength = 0;
      uint32_t suffix = ( symbol >> ui16AbsGoRice ) - COEF_REMAIN_BIN_REDUCTION;

      while( ( prefixLength < maximumPrefixLength ) && ( suffix > ( ( 2 << prefixLength ) - 2 ) ) )
      {
        prefixLength++;
      }

      const uint32_t suffixLength = ( prefixLength == maximumPrefixLength ) ? ( maxLog2TrDynamicRange - ui16AbsGoRice ) : ( prefixLength + 1/*separator*/ );

      iRate += ( COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ui16AbsGoRice ) << SCALE_BITS;
    }
    else
    {
      length = ui16AbsGoRice;
      symbol = symbol - ( threshold << ui16AbsGoRice );
      while( symbol >= ( 1 << length ) )
      {
        symbol -= ( 1 << ( length++ ) );
      }
      iRate += ( threshold + length + 1 - ui16AbsGoRice + length ) << SCALE_BITS;
    }

    iRate += fracBitsGt1.intBits[1];
    iRate += fracBitsPar.intBits[( uiAbsLevel - 2 ) & 1];
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
    iRate += fracBitsGt2.intBits[1];
#else
    if( remGt2Bins )
    {
      iRate += fracBitsGt2.intBits[1];
    }
#endif
  }
  else if( uiAbsLevel == 1 )
  {
    iRate += fracBitsGt1.intBits[0];
  }
  else if( uiAbsLevel == 2 )
  {
    iRate += fracBitsGt1.intBits[1];
    iRate += fracBitsPar.intBits[0];
    iRate += fracBitsGt2.intBits[0];
  }
  else if( uiAbsLevel == 3 )
  {
    iRate += fracBitsGt1.intBits[1];
    iRate += fracBitsPar.intBits[1];
    iRate += fracBitsGt2.intBits[0];
  }
  else
  {
    iRate = 0;
  }
  return  iRate;
}

inline double QuantRDOQ::xGetRateSigCoeffGroup( const BinFracBits& fracBitsSigCG, unsigned uiSignificanceCoeffGroup ) const
{
  return xGetICost( fracBitsSigCG.intBits[uiSignificanceCoeffGroup] );
}

/** Calculates the cost of signaling the last significant coefficient in the block
 * \param uiPosX X coordinate of the last significant coefficient
 * \param uiPosY Y coordinate of the last significant coefficient
 * \param component colour component ID
 * \returns cost of last significant coefficient
 */
/*
 * \param uiWidth width of the transform unit (TU)
*/
inline double QuantRDOQ::xGetRateLast( const int* lastBitsX, const int* lastBitsY, unsigned PosX, unsigned PosY ) const
{
  uint32_t    CtxX  = g_uiGroupIdx[PosX];
  uint32_t    CtxY  = g_uiGroupIdx[PosY];
  double  Cost  = lastBitsX[ CtxX ] + lastBitsY[ CtxY ];
  if( CtxX > 3 )
  {
    Cost += xGetIEPRate() * ((CtxX-2)>>1);
  }
  if( CtxY > 3 )
  {
    Cost += xGetIEPRate() * ((CtxY-2)>>1);
  }
  return xGetICost( Cost );
}


inline double QuantRDOQ::xGetRateSigCoef( const BinFracBits& fracBitsSig, unsigned uiSignificance ) const
{
  return xGetICost( fracBitsSig.intBits[uiSignificance] );
}

/** Get the cost for a specific rate
 * \param dRate rate of a bit
 * \returns cost at the specific rate
 */
inline double QuantRDOQ::xGetICost        ( double                          dRate         ) const
{
  return m_dLambda * dRate;
}

/** Get the cost of an equal probable bit
 * \returns cost of equal probable bit
 */
inline double QuantRDOQ::xGetIEPRate      (                                               ) const
{
  return 32768;
}



#if HEVC_USE_SCALING_LISTS
/** set quantized matrix coefficient for encode
 * \param scalingList            quantized matrix address
 * \param format                 chroma format
 * \param maxLog2TrDynamicRange
 * \param bitDepths              reference to bit depth array for all channels
 */
void QuantRDOQ::setScalingList(ScalingList *scalingList, const int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths)
{
  Quant::setScalingList( scalingList, maxLog2TrDynamicRange, bitDepths );

  const int minimumQp = 0;
  const int maximumQp = SCALING_LIST_REM_NUM;

  for(uint32_t size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(uint32_t list = 0; list < SCALING_LIST_NUM; list++)
    {
      for(int qp = minimumQp; qp < maximumQp; qp++)
      {
//         xSetScalingListEnc(scalingList,list,size,qp);
//         xSetScalingListDec(*scalingList,list,size,qp);
        xSetErrScaleCoeff(list,size, size,qp,maxLog2TrDynamicRange, bitDepths);
      }
    }
  }
}



#if HM_QTBT_AS_IN_JEM_QUANT
#endif
#else

#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
double QuantRDOQ::xGetErrScaleCoeff( const bool needsSqrt2, SizeType width, SizeType height, int qp, const int maxLog2TrDynamicRange, const int channelBitDepth )
#else
double QuantRDOQ::xGetErrScaleCoeff( SizeType width, SizeType height, int qp, const int maxLog2TrDynamicRange, const int channelBitDepth )
#endif
{
  const int iTransformShift = getTransformShift(channelBitDepth, Size(width, height), maxLog2TrDynamicRange);
#if HM_QTBT_AS_IN_JEM_QUANT
  double    dErrScale       = (double)( 1 << SCALE_BITS );                                // Compensate for scaling of bitcount in Lagrange cost function
#if !JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
  bool      needsSqrt2      = TU::needsBlockSizeTrafoScale( Size(width, height) );// ( ( (sizeX+sizeY) & 1 ) !=0 );
#endif
  double    dTransShift     = (double)iTransformShift + ( needsSqrt2 ? -0.5 : 0.0 );
  dErrScale                 = dErrScale*pow( 2.0, ( -2.0*dTransShift ) );                     // Compensate for scaling through forward transform
  int       QStep           = ( needsSqrt2 ? ( ( g_quantScales[qp] * 181 ) >> 7 ) : g_quantScales[qp] );
  double    finalErrScale = dErrScale / QStep / QStep / (1 << (DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth) << 1));
#else
  int errShift = SCALE_BITS - ((iTransformShift + DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth)) << 1);
  double    dErrScale       = exp2( double( errShift ) );
  double    finalErrScale   = dErrScale / double( g_quantScales[qp] * g_quantScales[qp] );
#endif
  return    finalErrScale;
}
#endif



#if HEVC_USE_SCALING_LISTS
/** set error scale coefficients
 * \param list                   list ID
 * \param size
 * \param qp                     quantization parameter
 * \param maxLog2TrDynamicRange
 * \param bitDepths              reference to bit depth array for all channels
 */
void QuantRDOQ::xSetErrScaleCoeff( uint32_t list, uint32_t sizeX, uint32_t sizeY, int qp, const int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths )
{
  const int width = g_scalingListSizeX[sizeX];
  const int height = g_scalingListSizeX[sizeY];
  const ChannelType channelType = ( ( list == 0 ) || ( list == MAX_NUM_COMPONENT ) ) ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA;
  const int channelBitDepth = bitDepths.recon[channelType];
  const int iTransformShift = getTransformShift( channelBitDepth, Size( g_scalingListSizeX[sizeX], g_scalingListSizeX[sizeY] ), maxLog2TrDynamicRange[channelType] );  // Represents scaling through forward transform

  uint32_t i, uiMaxNumCoeff = width * height;
  int *piQuantcoeff;
  double *pdErrScale;
  piQuantcoeff = getQuantCoeff( list, qp, sizeX, sizeY );
  pdErrScale   = xGetErrScaleCoeff( list, sizeX, sizeY, qp );

#if HM_QTBT_AS_IN_JEM_QUANT
  double dErrScale = (double)( 1 << SCALE_BITS );                                // Compensate for scaling of bitcount in Lagrange cost function

  bool   needsSqrt2 = TU::needsBlockSizeTrafoScale( Size( g_scalingListSizeX[sizeX], g_scalingListSizeX[sizeY] ) );// ( ( (sizeX+sizeY) & 1 ) !=0 );
  double dTransShift = (double)iTransformShift + ( needsSqrt2 ? -0.5 : 0.0 );
  dErrScale = dErrScale*pow( 2.0, ( -2.0*dTransShift ) );                     // Compensate for scaling through forward transform

  for( i = 0; i < uiMaxNumCoeff; i++ )
  {
    pdErrScale[i] = dErrScale / piQuantcoeff[i] / piQuantcoeff[i]
                    / (1 << (DISTORTION_PRECISION_ADJUSTMENT(bitDepths.recon[channelType]) << 1));
  }

  int QStep = ( needsSqrt2 ? ( ( g_quantScales[qp] * 181 ) >> 7 ) : g_quantScales[qp] );

  xGetErrScaleCoeffNoScalingList(list, sizeX, sizeY, qp) =
    dErrScale / QStep / QStep / (1 << (DISTORTION_PRECISION_ADJUSTMENT(bitDepths.recon[channelType]) << 1));
#else
  int errShift = SCALE_BITS - ((iTransformShift + DISTORTION_PRECISION_ADJUSTMENT(bitDepths.recon[channelType])) << 1);
  double dErrScale = exp2( double( errShift ) );
  for( i = 0; i < uiMaxNumCoeff; i++ )
  {
    pdErrScale[i] = dErrScale / double( piQuantcoeff[i] * piQuantcoeff[i] );
  }
  xGetErrScaleCoeffNoScalingList( list, sizeX, sizeY, qp ) = dErrScale / double( g_quantScales[qp] * g_quantScales[qp] );
#endif
}

/** set flat matrix value to quantized coefficient
 */
void QuantRDOQ::setFlatScalingList(const int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths)
{
  Quant::setFlatScalingList( maxLog2TrDynamicRange, bitDepths );

  const int minimumQp = 0;
  const int maximumQp = SCALING_LIST_REM_NUM;

  for(uint32_t sizeX = 0; sizeX < SCALING_LIST_SIZE_NUM; sizeX++)
  {
    for(uint32_t sizeY = 0; sizeY < SCALING_LIST_SIZE_NUM; sizeY++)
    {
      for(uint32_t list = 0; list < SCALING_LIST_NUM; list++)
      {
        for(int qp = minimumQp; qp < maximumQp; qp++)
        {
          xSetErrScaleCoeff( list, sizeX, sizeY, qp, maxLog2TrDynamicRange, bitDepths );
        }
      }
    }
  }
}

/** initialization process of scaling list array
 */
void QuantRDOQ::xInitScalingList( const QuantRDOQ* other )
{
  m_isErrScaleListOwner = other == nullptr;

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
        {
          if( m_isErrScaleListOwner )
          {
            m_errScale[sizeIdX][sizeIdY][listId][qp] = new double[g_scalingListSizeX[sizeIdX] * g_scalingListSizeX[sizeIdY]];
          }
          else
          {
            m_errScale[sizeIdX][sizeIdY][listId][qp] = other->m_errScale[sizeIdX][sizeIdY][listId][qp];
          }
        } // listID loop
      }
    }
  }
}

/** destroy quantization matrix array
 */
void QuantRDOQ::xDestroyScalingList()
{
  if( !m_isErrScaleListOwner ) return;

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
      {
        for(uint32_t qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
        {
          if(m_errScale[sizeIdX][sizeIdY][listId][qp])
          {
            delete [] m_errScale[sizeIdX][sizeIdY][listId][qp];
          }
        }
      }
    }
  }
//   Quant::destroyScalingList();
}
#endif


void QuantRDOQ::quant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx)
{
  const CompArea &rect      = tu.blocks[compID];
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;

  const CCoeffBuf &piCoef   = pSrc;
        CoeffBuf   piQCoef  = tu.getCoeffs(compID);

#if JVET_M0464_UNI_MTS
  const bool useTransformSkip      = tu.mtsIdx==1;
#else
  const bool useTransformSkip      = tu.transformSkip[compID];
#endif

  bool useRDOQ = useTransformSkip ? m_useRDOQTS : m_useRDOQ;

#if JVET_M0102_INTRA_SUBPARTITIONS
  if( !tu.cu->ispMode || !isLuma(compID) )
#endif
  {
    useRDOQ &= uiWidth > 2;
    useRDOQ &= uiHeight > 2;
  }

  if (useRDOQ && (isLuma(compID) || RDOQ_CHROMA))
  {
#if T0196_SELECTIVE_RDOQ
    if (!m_useSelectiveRDOQ || xNeedRDOQ(tu, compID, piCoef, cQP))
    {
#endif
      xRateDistOptQuant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
#if T0196_SELECTIVE_RDOQ
    }
    else
    {
      piQCoef.fill(0);
      uiAbsSum = 0;
    }
#endif
  }
  else
  {
    Quant::quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
  }
}



void QuantRDOQ::xRateDistOptQuant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx)
{
  const FracBitsAccess& fracBits = ctx.getFracBitsAcess();

  const SPS &sps            = *tu.cs->sps;
  const CompArea &rect      = tu.blocks[compID];
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;
  const ChannelType chType  = toChannelType(compID);
  const int channelBitDepth = sps.getBitDepth( chType );

  const bool extendedPrecision     = sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
  const int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);

#if JVET_M0102_INTRA_SUBPARTITIONS
  const bool useIntraSubPartitions = tu.cu->ispMode && isLuma(compID);
#endif
  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
  * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
  * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
  * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
  */

  // Represents scaling through forward transform
  int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

#if JVET_M0464_UNI_MTS
  if (tu.mtsIdx==1 && extendedPrecision)
#else
  if (tu.transformSkip[compID] && extendedPrecision)
#endif
  {
    iTransformShift = std::max<int>(0, iTransformShift);
  }

  double     d64BlockUncodedCost               = 0;
  const uint32_t uiLog2BlockWidth                  = g_aucLog2[uiWidth];
#if HEVC_USE_SCALING_LISTS
  const uint32_t uiLog2BlockHeight                 = g_aucLog2[uiHeight];
#endif
  const uint32_t uiMaxNumCoeff                     = rect.area();

  VTMCHECK(compID >= MAX_NUM_TBLOCKS, "Invalid component ID");

#if HEVC_USE_SCALING_LISTS
  int scalingListType = getScalingListType(tu.cu->predMode, compID);

  VTMCHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
#endif

  const TCoeff *plSrcCoeff = pSrc.buf;
        TCoeff *piDstCoeff = tu.getCoeffs(compID).buf;

  double *pdCostCoeff  = m_pdCostCoeff;
  double *pdCostSig    = m_pdCostSig;
  double *pdCostCoeff0 = m_pdCostCoeff0;
#if HEVC_USE_SIGN_HIDING
  int    *rateIncUp    = m_rateIncUp;
  int    *rateIncDown  = m_rateIncDown;
  int    *sigRateDelta = m_sigRateDelta;
  TCoeff *deltaU       = m_deltaU;
#endif

  memset( m_pdCostCoeff,  0, sizeof( double ) *  uiMaxNumCoeff );
  memset( m_pdCostSig,    0, sizeof( double ) *  uiMaxNumCoeff );
#if HEVC_USE_SIGN_HIDING
  memset( m_rateIncUp,    0, sizeof( int    ) *  uiMaxNumCoeff );
  memset( m_rateIncDown,  0, sizeof( int    ) *  uiMaxNumCoeff );
  memset( m_sigRateDelta, 0, sizeof( int    ) *  uiMaxNumCoeff );
  memset( m_deltaU,       0, sizeof( TCoeff ) *  uiMaxNumCoeff );
#endif


  const int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;                   // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits

#if HEVC_USE_SCALING_LISTS
  const double *const pdErrScale = xGetErrScaleCoeff(scalingListType, (uiLog2BlockWidth-1), (uiLog2BlockHeight-1), cQP.rem);
  const int    *const piQCoef    = getQuantCoeff(scalingListType, cQP.rem, (uiLog2BlockWidth-1), (uiLog2BlockHeight-1));

  const bool   enableScalingLists             = getUseScalingList(uiWidth, uiHeight, tu.transformSkip[compID]);
#if HM_QTBT_AS_IN_JEM_QUANT
#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
  const int    defaultQuantisationCoefficient = ( TU::needsSqrt2Scale( rect, tu.transformSkip[compID] ) ? ( g_quantScales[cQP.rem] * 181 ) >> 7 : g_quantScales[cQP.rem] );
  const double defaultErrorScale              = xGetErrScaleCoeffNoScalingList(scalingListType, (uiLog2BlockWidth-1), (uiLog2BlockHeight-1), cQP.rem);
#else
  const int    defaultQuantisationCoefficient = ( TU::needsSqrt2Scale( rect ) ? ( g_quantScales[cQP.rem] * 181 ) >> 7 : g_quantScales[cQP.rem] );
  const double defaultErrorScale              = xGetErrScaleCoeffNoScalingList(scalingListType, (uiLog2BlockWidth-1), (uiLog2BlockHeight-1), cQP.rem);
#endif
#else
  const double blkErrScale                    = ( TU::needsQP3Offset( tu, compID ) ? 2.0 : 1.0 );
  const int    defaultQuantisationCoefficient = g_quantScales[cQP.rem];
  const double defaultErrorScale              = blkErrScale * xGetErrScaleCoeffNoScalingList( scalingListType, ( uiLog2BlockWidth - 1 ), ( uiLog2BlockHeight - 1 ), cQP.rem );
#endif
#else //HEVC_USE_SCALING_LISTS
#if HM_QTBT_AS_IN_JEM_QUANT
#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
  const int    quantisationCoefficient = ( TU::needsSqrt2Scale( tu, compID ) ? ( g_quantScales[cQP.rem] * 181 ) >> 7 : g_quantScales[cQP.rem] );
  const double errorScale              = xGetErrScaleCoeff( TU::needsSqrt2Scale( tu, compID ), uiWidth, uiHeight, cQP.rem, maxLog2TrDynamicRange, channelBitDepth );
#else
  const int    quantisationCoefficient = ( TU::needsSqrt2Scale( rect ) ? ( g_quantScales[cQP.rem] * 181 ) >> 7 : g_quantScales[cQP.rem] );
  const double errorScale              = xGetErrScaleCoeff( uiWidth, uiHeight, cQP.rem, maxLog2TrDynamicRange, channelBitDepth );
#endif
#else
  const double blkErrScale             = ( TU::needsQP3Offset( tu, compID ) ? 2.0 : 1.0 );
  const int    quantisationCoefficient = g_quantScales[cQP.rem];
  const double errorScale              = blkErrScale * xGetErrScaleCoeff( uiWidth, uiHeight, cQP.rem, maxLog2TrDynamicRange, channelBitDepth );
#endif
#endif//HEVC_USE_SCALING_LISTS


#if HEVC_USE_SIGN_HIDING
  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
#endif
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

#if HEVC_USE_SIGN_HIDING
  CoeffCodingContext cctx(tu, compID, tu.cs->slice->getSignDataHidingEnabledFlag());
#else
  CoeffCodingContext cctx(tu, compID);
#endif
  const int    iCGSizeM1      = (1 << cctx.log2CGSize()) - 1;

  int     iCGLastScanPos      = -1;
  double  d64BaseCost         = 0;
  int     iLastScanPos        = -1;

  bool      is2x2subblock = ( iCGSizeM1 == 3 );
  int       remGt2Bins    = ( is2x2subblock ? MAX_NUM_GT2_BINS_2x2SUBBLOCK : MAX_NUM_GT2_BINS_4x4SUBBLOCK );
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  int       remRegBins    = ( is2x2subblock ? MAX_NUM_REG_BINS_2x2SUBBLOCK : MAX_NUM_REG_BINS_4x4SUBBLOCK );
#else
  int       remRegBins = (is2x2subblock ? MAX_NUM_REG_BINS_2x2SUBBLOCK : MAX_NUM_REG_BINS_4x4SUBBLOCK) - remGt2Bins;
#endif
  uint32_t  goRiceParam   = 0;

  double *pdCostCoeffGroupSig = m_pdCostCoeffGroupSig;
  memset( pdCostCoeffGroupSig, 0, ( uiMaxNumCoeff >> cctx.log2CGSize() ) * sizeof( double ) );
#if JVET_M0257
  const int iCGNum = std::min<int>(JVET_C0024_ZERO_OUT_TH, uiWidth) * std::min<int>(JVET_C0024_ZERO_OUT_TH, uiHeight) >> cctx.log2CGSize();
#else
  const int iCGNum  = uiWidth * uiHeight >> cctx.log2CGSize();
#endif
  int iScanPos;
  coeffGroupRDStats rdStats;

#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_RDOQ, "%d: %3d, %3d, %dx%d, comp=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), rect.x, rect.y, rect.width, rect.height, compID );
#endif



  for (int subSetId = iCGNum - 1; subSetId >= 0; subSetId--)
  {
    cctx.initSubblock( subSetId );

    memset( &rdStats, 0, sizeof (coeffGroupRDStats));

    for (int iScanPosinCG = iCGSizeM1; iScanPosinCG >= 0; iScanPosinCG--)
    {
      iScanPos = cctx.minSubPos() + iScanPosinCG;
      //===== quantization =====
      uint32_t    uiBlkPos          = cctx.blockPos(iScanPos);

      // set coeff
#if HEVC_USE_SCALING_LISTS
      const int    quantisationCoefficient = (enableScalingLists) ? piQCoef   [uiBlkPos]               : defaultQuantisationCoefficient;
#if HM_QTBT_AS_IN_JEM_QUANT
      const double errorScale              = (enableScalingLists) ? pdErrScale[uiBlkPos]               : defaultErrorScale;
#else
      const double errorScale              = (enableScalingLists) ? pdErrScale[uiBlkPos] * blkErrScale : defaultErrorScale;
#endif
#endif
      const int64_t  tmpLevel                = int64_t(abs(plSrcCoeff[ uiBlkPos ])) * quantisationCoefficient;

      const Intermediate_Int lLevelDouble  = (Intermediate_Int)std::min<int64_t>(tmpLevel, std::numeric_limits<Intermediate_Int>::max() - (Intermediate_Int(1) << (iQBits - 1)));

      uint32_t uiMaxAbsLevel        = std::min<uint32_t>(uint32_t(entropyCodingMaximum), uint32_t((lLevelDouble + (Intermediate_Int(1) << (iQBits - 1))) >> iQBits));

      const double dErr         = double( lLevelDouble );
      pdCostCoeff0[ iScanPos ]  = dErr * dErr * errorScale;
      d64BlockUncodedCost      += pdCostCoeff0[ iScanPos ];
      piDstCoeff[ uiBlkPos ]    = uiMaxAbsLevel;

      if ( uiMaxAbsLevel > 0 && iLastScanPos < 0 )
      {
        iLastScanPos            = iScanPos;
        iCGLastScanPos          = cctx.subSetId();
      }

      if ( iLastScanPos >= 0 )
      {

#if ENABLE_TRACING
        uint32_t uiCGPosY = cctx.cgPosX();
        uint32_t uiCGPosX = cctx.cgPosY();
        uint32_t uiPosY = cctx.posY( iScanPos );
        uint32_t uiPosX = cctx.posX( iScanPos );
        DTRACE( g_trace_ctx, D_RDOQ, "%d [%d][%d][%2d:%2d][%2d:%2d]", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), iScanPos, uiBlkPos, uiCGPosX, uiCGPosY, uiPosX, uiPosY );
#endif
        //===== coefficient level estimation =====
        unsigned ctxIdSig = 0;
        if( iScanPos != iLastScanPos )
        {
          ctxIdSig = cctx.sigCtxIdAbs( iScanPos, piDstCoeff, 0 );
        }
        uint32_t    uiLevel;
        uint8_t ctxOffset     = cctx.ctxOffsetAbs     ();
        uint32_t    uiParCtx      = cctx.parityCtxIdAbs   ( ctxOffset );
        uint32_t    uiGt1Ctx      = cctx.greater1CtxIdAbs ( ctxOffset );
        uint32_t    uiGt2Ctx      = cctx.greater2CtxIdAbs ( ctxOffset );
        uint32_t    goRiceZero    = 0;
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
        if( remRegBins < 4 )
#else
        if( remRegBins < 3 )
#endif
        {
          unsigned  sumAbs        = cctx.templateAbsSum( iScanPos, piDstCoeff );
          goRiceParam             = g_auiGoRiceParsCoeff   [ sumAbs ];
          goRiceZero              = g_auiGoRicePosCoeff0[0][ sumAbs ];
        }

        const BinFracBits fracBitsPar = fracBits.getFracBitsArray( uiParCtx );
        const BinFracBits fracBitsGt1 = fracBits.getFracBitsArray( uiGt1Ctx );
        const BinFracBits fracBitsGt2 = fracBits.getFracBitsArray( uiGt2Ctx );

        if( iScanPos == iLastScanPos )
        {
          uiLevel = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                    lLevelDouble, uiMaxAbsLevel, nullptr, fracBitsPar, fracBitsGt1, fracBitsGt2, remGt2Bins, remRegBins, goRiceZero, goRiceParam, iQBits, errorScale, 1, extendedPrecision, maxLog2TrDynamicRange );
        }
        else
        {
          DTRACE_COND( ( uiMaxAbsLevel != 0 ), g_trace_ctx, D_RDOQ_MORE, " uiCtxSig=%d", ctxIdSig );

          const BinFracBits fracBitsSig = fracBits.getFracBitsArray( ctxIdSig );
          uiLevel = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                    lLevelDouble, uiMaxAbsLevel, &fracBitsSig, fracBitsPar, fracBitsGt1, fracBitsGt2, remGt2Bins, remRegBins, goRiceZero, goRiceParam, iQBits, errorScale, 0, extendedPrecision, maxLog2TrDynamicRange );
#if HEVC_USE_SIGN_HIDING
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
          sigRateDelta[ uiBlkPos ] = ( remRegBins < 4 ? 0 : fracBitsSig.intBits[1] - fracBitsSig.intBits[0] );
#else
          sigRateDelta[ uiBlkPos ] = ( remRegBins < 3 ? 0 : fracBitsSig.intBits[1] - fracBitsSig.intBits[0] );
#endif
#endif
        }

        DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", uiLevel );
        DTRACE_COND( ( uiMaxAbsLevel != 0 ), g_trace_ctx, D_RDOQ, " CostC0=%d\n", (int64_t)( pdCostCoeff0[iScanPos] ) );
        DTRACE_COND( ( uiMaxAbsLevel != 0 ), g_trace_ctx, D_RDOQ, " CostC =%d\n", (int64_t)( pdCostCoeff[iScanPos] ) );

#if HEVC_USE_SIGN_HIDING
        deltaU[ uiBlkPos ]        = TCoeff((lLevelDouble - (Intermediate_Int(uiLevel) << iQBits)) >> (iQBits-8));

        if( uiLevel > 0 )
        {
          int rateNow              = xGetICRate( uiLevel,   fracBitsPar, fracBitsGt1, fracBitsGt2, remGt2Bins, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange );
          rateIncUp   [ uiBlkPos ] = xGetICRate( uiLevel+1, fracBitsPar, fracBitsGt1, fracBitsGt2, remGt2Bins, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;
          rateIncDown [ uiBlkPos ] = xGetICRate( uiLevel-1, fracBitsPar, fracBitsGt1, fracBitsGt2, remGt2Bins, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;
        }
        else // uiLevel == 0
        {
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
          if( remRegBins < 4 )
#else
          if( remRegBins < 3 )
#endif
          {
            int rateNow            = xGetICRate( uiLevel,   fracBitsPar, fracBitsGt1, fracBitsGt2, remGt2Bins, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange );
            rateIncUp [ uiBlkPos ] = xGetICRate( uiLevel+1, fracBitsPar, fracBitsGt1, fracBitsGt2, remGt2Bins, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;
          }
          else
          {
            rateIncUp [ uiBlkPos ] = fracBitsGt1.intBits[ 0 ];
          }
        }
#endif
        piDstCoeff[ uiBlkPos ] = uiLevel;
        d64BaseCost           += pdCostCoeff [ iScanPos ];

        if( ( (iScanPos & iCGSizeM1) == 0 ) && ( iScanPos > 0 ) )
        {
          remGt2Bins    = ( is2x2subblock ? MAX_NUM_GT2_BINS_2x2SUBBLOCK : MAX_NUM_GT2_BINS_4x4SUBBLOCK );
          remRegBins    = ( is2x2subblock ? MAX_NUM_REG_BINS_2x2SUBBLOCK : MAX_NUM_REG_BINS_4x4SUBBLOCK ) - remGt2Bins;
          goRiceParam   = 0;
        }
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
        else if( remRegBins >= 4 )
#else
        else if( remRegBins >= 3 )
#endif
        {
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
          const uint32_t baseLevel = 4;
#else
          const uint32_t baseLevel = ( remGt2Bins ? 4 : 2 );
#endif
          if( goRiceParam < 3 && ((uiLevel-baseLevel)>>1) > (3<<goRiceParam)-1 )
          {
            goRiceParam++;
          }
#if !JVET_M0173_MOVE_GT2_TO_FIRST_PASS
          if( uiLevel >= 2 && remGt2Bins )
          {
            remGt2Bins--;
          }
#endif
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
          remRegBins -= (uiLevel < 2 ? uiLevel : 3) + (iScanPos != iLastScanPos);
#else
          remRegBins -= std::min<int>( uiLevel, 2 ) + (iScanPos != iLastScanPos);
#endif
        }
      }
      else
      {
        d64BaseCost    += pdCostCoeff0[ iScanPos ];
      }
      rdStats.d64SigCost += pdCostSig[ iScanPos ];
      if (iScanPosinCG == 0 )
      {
        rdStats.d64SigCost_0 = pdCostSig[ iScanPos ];
      }
      if (piDstCoeff[ uiBlkPos ] )
      {
        cctx.setSigGroup();
        rdStats.d64CodedLevelandDist += pdCostCoeff[ iScanPos ] - pdCostSig[ iScanPos ];
        rdStats.d64UncodedDist += pdCostCoeff0[ iScanPos ];
        if ( iScanPosinCG != 0 )
        {
          rdStats.iNNZbeforePos0++;
        }
      }
    } //end for (iScanPosinCG)

    if (iCGLastScanPos >= 0)
    {
      if( cctx.subSetId() )
      {
        if( !cctx.isSigGroup() )
        {
          const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray( cctx.sigGroupCtxId() );
          d64BaseCost += xGetRateSigCoeffGroup(fracBitsSigGroup, 0) - rdStats.d64SigCost;
          pdCostCoeffGroupSig[ cctx.subSetId() ] = xGetRateSigCoeffGroup(fracBitsSigGroup, 0);
        }
        else
        {
          if (cctx.subSetId() < iCGLastScanPos) //skip the last coefficient group, which will be handled together with last position below.
          {
            if ( rdStats.iNNZbeforePos0 == 0 )
            {
              d64BaseCost -= rdStats.d64SigCost_0;
              rdStats.d64SigCost -= rdStats.d64SigCost_0;
            }
            // rd-cost if SigCoeffGroupFlag = 0, initialization
            double d64CostZeroCG = d64BaseCost;

            const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray( cctx.sigGroupCtxId() );

            if (cctx.subSetId() < iCGLastScanPos)
            {
              d64BaseCost  += xGetRateSigCoeffGroup(fracBitsSigGroup,1);
              d64CostZeroCG += xGetRateSigCoeffGroup(fracBitsSigGroup,0);
              pdCostCoeffGroupSig[ cctx.subSetId() ] = xGetRateSigCoeffGroup(fracBitsSigGroup,1);
            }

            // try to convert the current coeff group from non-zero to all-zero
            d64CostZeroCG += rdStats.d64UncodedDist;  // distortion for resetting non-zero levels to zero levels
            d64CostZeroCG -= rdStats.d64CodedLevelandDist;   // distortion and level cost for keeping all non-zero levels
            d64CostZeroCG -= rdStats.d64SigCost;     // sig cost for all coeffs, including zero levels and non-zerl levels

                                                     // if we can save cost, change this block to all-zero block
            if ( d64CostZeroCG < d64BaseCost )
            {
              cctx.resetSigGroup();
              d64BaseCost = d64CostZeroCG;
              if (cctx.subSetId() < iCGLastScanPos)
              {
                pdCostCoeffGroupSig[ cctx.subSetId() ] = xGetRateSigCoeffGroup(fracBitsSigGroup,0);
              }
              // reset coeffs to 0 in this block
              for (int iScanPosinCG = iCGSizeM1; iScanPosinCG >= 0; iScanPosinCG--)
              {
                iScanPos      = cctx.minSubPos() + iScanPosinCG;
                uint32_t uiBlkPos = cctx.blockPos( iScanPos );

                if (piDstCoeff[ uiBlkPos ])
                {
                  piDstCoeff [ uiBlkPos ] = 0;
                  pdCostCoeff[ iScanPos ] = pdCostCoeff0[ iScanPos ];
                  pdCostSig  [ iScanPos ] = 0;
                }
              }
            } // end if ( d64CostAllZeros < d64BaseCost )
          }
        } // end if if (uiSigCoeffGroupFlag[ uiCGBlkPos ] == 0)
      }
      else
      {
        cctx.setSigGroup();
      }
    }
  } //end for (cctx.subSetId)


  //===== estimate last position =====
  if ( iLastScanPos < 0 )
  {
    return;
  }

  double  d64BestCost         = 0;
  int     iBestLastIdxP1      = 0;


  if( !CU::isIntra( *tu.cu ) && isLuma( compID ) && tu.depth == 0 )
  {
    const BinFracBits fracBitsQtRootCbf = fracBits.getFracBitsArray( Ctx::QtRootCbf() );
    d64BestCost  = d64BlockUncodedCost + xGetICost( fracBitsQtRootCbf.intBits[ 0 ] );
    d64BaseCost += xGetICost( fracBitsQtRootCbf.intBits[ 1 ] );
  }
  else
  {
#if JVET_M0102_INTRA_SUBPARTITIONS
    bool previousCbf       = tu.cbf[COMPONENT_Cb];
    bool lastCbfIsInferred = false;
    if( useIntraSubPartitions )
    {
      bool rootCbfSoFar       = false;
      bool isLastSubPartition = CU::isISPLast(*tu.cu, tu.Y(), compID);
      uint32_t nTus = tu.cu->ispMode == HOR_INTRA_SUBPARTITIONS ? tu.cu->lheight() >> g_aucLog2[tu.lheight()] : tu.cu->lwidth() >> g_aucLog2[tu.lwidth()];
      if( isLastSubPartition )
      {
        TransformUnit* tuPointer = tu.cu->firstTU;
        for( int tuIdx = 0; tuIdx < nTus - 1; tuIdx++ )
        {
          rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMPONENT_Y, tu.depth);
          tuPointer     = tuPointer->next;
        }
        if( !rootCbfSoFar )
        {
          lastCbfIsInferred = true;
        }
      }
      if( !lastCbfIsInferred )
      {
        previousCbf = TU::getPrevTuCbfAtDepth(tu, compID, tu.depth);
      }
    }
    BinFracBits fracBitsQtCbf = fracBits.getFracBitsArray( Ctx::QtCbf[compID]( DeriveCtx::CtxQtCbf( rect.compID, tu.depth, previousCbf, useIntraSubPartitions ) ) );

    if( !lastCbfIsInferred )
    {
      d64BestCost  = d64BlockUncodedCost + xGetICost(fracBitsQtCbf.intBits[0]);
      d64BaseCost += xGetICost(fracBitsQtCbf.intBits[1]);
    }
    else
    {
      d64BestCost  = d64BlockUncodedCost;
    }
#else
    BinFracBits fracBitsQtCbf = fracBits.getFracBitsArray( Ctx::QtCbf[compID]( DeriveCtx::CtxQtCbf( rect.compID, tu.depth, tu.cbf[COMPONENT_Cb] ) ) );

    d64BestCost  = d64BlockUncodedCost + xGetICost( fracBitsQtCbf.intBits[0] );
    d64BaseCost += xGetICost( fracBitsQtCbf.intBits[1] );
#endif
  }

  int lastBitsX[LAST_SIGNIFICANT_GROUPS] = { 0 };
  int lastBitsY[LAST_SIGNIFICANT_GROUPS] = { 0 };
  {
#if HEVC_USE_MDCS
    int dim1  = ( cctx.scanType() == SCAN_VER ? uiHeight : uiWidth  );
    int dim2  = ( cctx.scanType() == SCAN_VER ? uiWidth  : uiHeight );
#else
#if JVET_M0257
    int dim1 = std::min<int>(JVET_C0024_ZERO_OUT_TH, uiWidth);
    int dim2 = std::min<int>(JVET_C0024_ZERO_OUT_TH, uiHeight);
#else
    int dim1  = uiWidth;
    int dim2  = uiHeight;
#endif
#endif
    int bitsX = 0;
    int bitsY = 0;
    int ctxId;
    //X-coordinate
    for ( ctxId = 0; ctxId < g_uiGroupIdx[dim1-1]; ctxId++)
    {
      const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastXCtxId(ctxId) );
      lastBitsX[ ctxId ]   = bitsX + fB.intBits[ 0 ];
      bitsX               +=         fB.intBits[ 1 ];
    }
    lastBitsX[ctxId] = bitsX;
    //Y-coordinate
    for ( ctxId = 0; ctxId < g_uiGroupIdx[dim2-1]; ctxId++)
    {
      const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastYCtxId(ctxId) );
      lastBitsY[ ctxId ]   = bitsY + fB.intBits[ 0 ];
      bitsY               +=         fB.intBits[ 1 ];
    }
    lastBitsY[ctxId] = bitsY;
  }


  bool bFoundLast = false;
  for (int iCGScanPos = iCGLastScanPos; iCGScanPos >= 0; iCGScanPos--)
  {
    d64BaseCost -= pdCostCoeffGroupSig [ iCGScanPos ];
    if (cctx.isSigGroup( iCGScanPos ) )
    {
      for (int iScanPosinCG = iCGSizeM1; iScanPosinCG >= 0; iScanPosinCG--)
      {
        iScanPos = iCGScanPos * (iCGSizeM1 + 1) + iScanPosinCG;

        if (iScanPos > iLastScanPos)
        {
          continue;
        }
        uint32_t   uiBlkPos     = cctx.blockPos( iScanPos );

        if( piDstCoeff[ uiBlkPos ] )
        {
          uint32_t   uiPosY = uiBlkPos >> uiLog2BlockWidth;
          uint32_t   uiPosX = uiBlkPos - ( uiPosY << uiLog2BlockWidth );
#if HEVC_USE_MDCS
          double d64CostLast  = ( cctx.scanType() == SCAN_VER ? xGetRateLast( lastBitsX, lastBitsY, uiPosY, uiPosX ) : xGetRateLast( lastBitsX, lastBitsY, uiPosX, uiPosY ) );
#else
          double d64CostLast  = xGetRateLast( lastBitsX, lastBitsY, uiPosX, uiPosY );
#endif

          double totalCost = d64BaseCost + d64CostLast - pdCostSig[ iScanPos ];

          if( totalCost < d64BestCost )
          {
            iBestLastIdxP1  = iScanPos + 1;
            d64BestCost     = totalCost;
          }
          if( piDstCoeff[ uiBlkPos ] > 1 )
          {
            bFoundLast = true;
            break;
          }
          d64BaseCost      -= pdCostCoeff[ iScanPos ];
          d64BaseCost      += pdCostCoeff0[ iScanPos ];
        }
        else
        {
          d64BaseCost      -= pdCostSig[ iScanPos ];
        }
      } //end for
      if (bFoundLast)
      {
        break;
      }
    } // end if (uiSigCoeffGroupFlag[ uiCGBlkPos ])
    DTRACE( g_trace_ctx, D_RDOQ_COST, "%d: %3d, %3d, %dx%d, comp=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ_COST ), rect.x, rect.y, rect.width, rect.height, compID );
    DTRACE( g_trace_ctx, D_RDOQ_COST, "Uncoded=%d\n", (int64_t)( d64BlockUncodedCost ) );
    DTRACE( g_trace_ctx, D_RDOQ_COST, "Coded  =%d\n", (int64_t)( d64BaseCost ) );

  } // end for


  for ( int scanPos = 0; scanPos < iBestLastIdxP1; scanPos++ )
  {
    int blkPos = cctx.blockPos( scanPos );
    TCoeff level = piDstCoeff[ blkPos ];
    uiAbsSum += level;
    piDstCoeff[ blkPos ] = ( plSrcCoeff[ blkPos ] < 0 ) ? -level : level;
  }

  //===== clean uncoded coefficients =====
  for ( int scanPos = iBestLastIdxP1; scanPos <= iLastScanPos; scanPos++ )
  {
    piDstCoeff[ cctx.blockPos( scanPos ) ] = 0;
  }

#if HEVC_USE_SIGN_HIDING
  if( cctx.signHiding() && uiAbsSum>=2)
  {
    const double inverseQuantScale = double(g_invQuantScales[cQP.rem]);
    int64_t rdFactor = (int64_t)(inverseQuantScale * inverseQuantScale * (1 << (2 * cQP.per)) / m_dLambda / 16
                               / (1 << (2 * DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth)))
#if HM_QTBT_AS_IN_JEM_QUANT
#else
                              * blkErrScale
#endif
                             + 0.5);

    int lastCG = -1;
    int absSum = 0 ;
    int n ;
#if JVET_M0257
    for (int subSet = iCGNum - 1; subSet >= 0; subSet--)
#else
    for( int subSet = (uiWidth*uiHeight-1) >> cctx.log2CGSize(); subSet >= 0; subSet-- )
#endif
    {
      int  subPos         = subSet << cctx.log2CGSize();
      int  firstNZPosInCG = iCGSizeM1 + 1, lastNZPosInCG = -1;
      absSum = 0 ;

      for( n = iCGSizeM1; n >= 0; --n )
      {
        if( piDstCoeff[ cctx.blockPos( n + subPos )] )
        {
          lastNZPosInCG = n;
          break;
        }
      }

      for( n = 0; n <= iCGSizeM1; n++ )
      {
        if( piDstCoeff[ cctx.blockPos( n + subPos )] )
        {
          firstNZPosInCG = n;
          break;
        }
      }

      for( n = firstNZPosInCG; n <= lastNZPosInCG; n++ )
      {
        absSum += int(piDstCoeff[ cctx.blockPos( n + subPos )]);
      }

      if(lastNZPosInCG>=0 && lastCG==-1)
      {
        lastCG = 1;
      }

      if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
      {
        uint32_t signbit = (piDstCoeff[cctx.blockPos(subPos+firstNZPosInCG)]>0?0:1);
        if( signbit!=(absSum&0x1) )  // hide but need tune
        {
          // calculate the cost
          int64_t minCostInc = std::numeric_limits<int64_t>::max(), curCost = std::numeric_limits<int64_t>::max();
          int minPos = -1, finalChange = 0, curChange = 0;

          for( n = (lastCG == 1 ? lastNZPosInCG : iCGSizeM1); n >= 0; --n )
          {
            uint32_t uiBlkPos   = cctx.blockPos( n + subPos );
            if(piDstCoeff[ uiBlkPos ] != 0 )
            {
              int64_t costUp   = rdFactor * ( - deltaU[uiBlkPos] ) + rateIncUp[uiBlkPos];
              int64_t costDown = rdFactor * (   deltaU[uiBlkPos] ) + rateIncDown[uiBlkPos]
                -   ((abs(piDstCoeff[uiBlkPos]) == 1) ? sigRateDelta[uiBlkPos] : 0);

              if(lastCG==1 && lastNZPosInCG==n && abs(piDstCoeff[uiBlkPos])==1)
              {
                costDown -= (4<<SCALE_BITS);
              }

              if(costUp<costDown)
              {
                curCost = costUp;
                curChange =  1;
              }
              else
              {
                curChange = -1;
                if(n==firstNZPosInCG && abs(piDstCoeff[uiBlkPos])==1)
                {
                  curCost = std::numeric_limits<int64_t>::max();
                }
                else
                {
                  curCost = costDown;
                }
              }
            }
            else
            {
              curCost = rdFactor * ( - (abs(deltaU[uiBlkPos])) ) + (1<<SCALE_BITS) + rateIncUp[uiBlkPos] + sigRateDelta[uiBlkPos] ;
              curChange = 1 ;

              if(n<firstNZPosInCG)
              {
                uint32_t thissignbit = (plSrcCoeff[uiBlkPos]>=0?0:1);
                if(thissignbit != signbit )
                {
                  curCost = std::numeric_limits<int64_t>::max();
                }
              }
            }

            if( curCost<minCostInc)
            {
              minCostInc = curCost;
              finalChange = curChange;
              minPos = uiBlkPos;
            }
          }

          if(piDstCoeff[minPos] == entropyCodingMaximum || piDstCoeff[minPos] == entropyCodingMinimum)
          {
            finalChange = -1;
          }

          if(plSrcCoeff[minPos]>=0)
          {
            piDstCoeff[minPos] += finalChange ;
          }
          else
          {
            piDstCoeff[minPos] -= finalChange ;
          }
        }
      }

      if(lastCG==1)
      {
        lastCG=0 ;
      }
    }
  }
#endif
}

//! \}
