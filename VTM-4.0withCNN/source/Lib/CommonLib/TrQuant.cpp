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

/** \file     TrQuant.cpp
    \brief    transform and quantization class
*/

#include "TrQuant.h"
#include "TrQuant_EMT.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "CrossCompPrediction.h"


#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>

#include "QuantRDOQ.h"
#include "DepQuant.h"

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

struct coeffGroupRDStats
{
  int    iNNZbeforePos0;
  double d64CodedLevelandDist; // distortion and level cost only
  double d64UncodedDist;    // all zero coded block distortion
  double d64SigCost;
  double d64SigCost_0;
};

FwdTrans *fastFwdTrans[NUM_TRANS_TYPE][g_numTransformMatrixSizes] =
{
  { fastForwardDCT2_B2, fastForwardDCT2_B4, fastForwardDCT2_B8, fastForwardDCT2_B16, fastForwardDCT2_B32, fastForwardDCT2_B64 },
  { nullptr,            fastForwardDCT8_B4, fastForwardDCT8_B8, fastForwardDCT8_B16, fastForwardDCT8_B32, nullptr },
  { nullptr,            fastForwardDST7_B4, fastForwardDST7_B8, fastForwardDST7_B16, fastForwardDST7_B32, nullptr },
};

InvTrans *fastInvTrans[NUM_TRANS_TYPE][g_numTransformMatrixSizes] =
{
  { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32, fastInverseDCT2_B64 },
  { nullptr,            fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32, nullptr },
  { nullptr,            fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32, nullptr },
};

//! \ingroup CommonLib
//! \{


// ====================================================================================================================
// TrQuant class member functions
// ====================================================================================================================
TrQuant::TrQuant() : m_quant( nullptr )
{
  // allocate temporary buffers
  m_plTempCoeff   = (TCoeff*) xMalloc( TCoeff, MAX_CU_SIZE * MAX_CU_SIZE );
#if JVET_M0464_UNI_MTS
  m_mtsCoeffs = new TCoeff*[ NUM_TRAFO_MODES_MTS ];
  for( int i = 0; i < NUM_TRAFO_MODES_MTS; i++ )
  {
    m_mtsCoeffs[i] = (TCoeff*) xMalloc( TCoeff, MAX_CU_SIZE * MAX_CU_SIZE );
  }
#endif
}

TrQuant::~TrQuant()
{
  if( m_quant )
  {
    delete m_quant;
    m_quant = nullptr;
  }

  // delete temporary buffers
  if ( m_plTempCoeff )
  {
    xFree( m_plTempCoeff );
    m_plTempCoeff = nullptr;
  }
#if JVET_M0464_UNI_MTS
  if( m_mtsCoeffs )
  {
    for( int i = 0; i < NUM_TRAFO_MODES_MTS; i++ )
    {
      xFree( m_mtsCoeffs[i] );
      m_mtsCoeffs[i] = nullptr;
    }
    delete[] m_mtsCoeffs;
    m_mtsCoeffs = nullptr;
  }
#endif
}

#if ENABLE_SPLIT_PARALLELISM
void TrQuant::copyState( const TrQuant& other )
{
  m_quant->copyState( *other.m_quant );
}
#endif

void TrQuant::xDeQuant(const TransformUnit &tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  m_quant->dequant( tu, dstCoeff, compID, cQP );
}

void TrQuant::init( const Quant* otherQuant,
                    const uint32_t uiMaxTrSize,
                    const bool bUseRDOQ,
                    const bool bUseRDOQTS,
#if T0196_SELECTIVE_RDOQ
                    const bool useSelectiveRDOQ,
#endif
                    const bool bEnc,
                    const bool useTransformSkipFast
)
{
  m_uiMaxTrSize          = uiMaxTrSize;
  m_bEnc                 = bEnc;
  m_useTransformSkipFast = useTransformSkipFast;

  delete m_quant;
  m_quant = nullptr;

  if( bUseRDOQ || !bEnc )
  {
    m_quant = new DepQuant( otherQuant, bEnc );
  }
  else
    m_quant = new Quant( otherQuant );

  if( m_quant )
  {
    m_quant->init( uiMaxTrSize, bUseRDOQ, bUseRDOQTS, useSelectiveRDOQ );
  }
}



void TrQuant::invTransformNxN( TransformUnit &tu, const ComponentID &compID, PelBuf &pResi, const QpParam &cQP )
{
  const CompArea &area    = tu.blocks[compID];
  const uint32_t uiWidth      = area.width;
  const uint32_t uiHeight     = area.height;

  VTMCHECK( uiWidth > tu.cs->sps->getMaxTrSize() || uiHeight > tu.cs->sps->getMaxTrSize(), "Maximal allowed transformation size exceeded!" );

  if (tu.cu->transQuantBypass)
  {
    // where should this logic go?
    const bool rotateResidual = TU::isNonTransformedResidualRotated(tu, compID);
    const CCoeffBuf pCoeff    = tu.getCoeffs(compID);

    for (uint32_t y = 0, coefficientIndex = 0; y < uiHeight; y++)
    {
      for (uint32_t x = 0; x < uiWidth; x++, coefficientIndex++)
      {
        pResi.at(x, y) = rotateResidual ? pCoeff.at(pCoeff.width - x - 1, pCoeff.height - y - 1) : pCoeff.at(x, y);
      }
    }
  }
  else
  {
    CoeffBuf tempCoeff = CoeffBuf( m_plTempCoeff, area );
    xDeQuant( tu, tempCoeff, compID, cQP );

    DTRACE_COEFF_BUF( D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID );

#if JVET_M0464_UNI_MTS
    if( isLuma(compID) && tu.mtsIdx == 1 )
#else
    if( tu.transformSkip[compID] )
#endif
    {
      xITransformSkip( tempCoeff, pResi, tu, compID );
    }
    else
    {
      xIT( tu, compID, tempCoeff, pResi );
    }
  }

  //DTRACE_BLOCK_COEFF(tu.getCoeffs(compID), tu, tu.cu->predMode, compID);
  DTRACE_PEL_BUF( D_RESIDUALS, pResi, tu, tu.cu->predMode, compID);
  invRdpcmNxN(tu, compID, pResi);
}

void TrQuant::invRdpcmNxN(TransformUnit& tu, const ComponentID &compID, PelBuf &pcResidual)
{
  const CompArea &area    = tu.blocks[compID];

#if JVET_M0464_UNI_MTS
  if (CU::isRDPCMEnabled(*tu.cu) && (tu.mtsIdx==1 || tu.cu->transQuantBypass))
#else
  if (CU::isRDPCMEnabled(*tu.cu) && ((tu.transformSkip[compID] != 0) || tu.cu->transQuantBypass))
#endif
  {
    const uint32_t uiWidth  = area.width;
    const uint32_t uiHeight = area.height;

    RDPCMMode rdpcmMode = RDPCM_OFF;

    if (tu.cu->predMode == MODE_INTRA)
    {
      const ChannelType chType = toChannelType(compID);
      const uint32_t uiChFinalMode = PU::getFinalIntraMode(*tu.cs->getPU(area.pos(), chType), chType);

      if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
      {
        rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;
      }
    }
    else  // not intra case
    {
      rdpcmMode = RDPCMMode(tu.rdpcm[compID]);
    }

    const TCoeff pelMin = (TCoeff) std::numeric_limits<Pel>::min();
    const TCoeff pelMax = (TCoeff) std::numeric_limits<Pel>::max();

    if (rdpcmMode == RDPCM_VER)
    {
      for (uint32_t uiX = 0; uiX < uiWidth; uiX++)
      {
        TCoeff accumulator = pcResidual.at(uiX, 0); // 32-bit accumulator

        for (uint32_t uiY = 1; uiY < uiHeight; uiY++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
    else if (rdpcmMode == RDPCM_HOR)
    {
      for (uint32_t uiY = 0; uiY < uiHeight; uiY++)
      {
        TCoeff accumulator = pcResidual.at(0, uiY);

        for (uint32_t uiX = 1; uiX < uiWidth; uiX++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

void TrQuant::getTrTypes ( TransformUnit tu, const ComponentID compID, int &trTypeHor, int &trTypeVer )
{
#if JVET_M0464_UNI_MTS
  bool mtsActivated = CU::isIntra( *tu.cu ) ? tu.cs->sps->getUseIntraMTS() : tu.cs->sps->getUseInterMTS();
#else
  bool emtActivated = CU::isIntra( *tu.cu ) ? tu.cs->sps->getUseIntraEMT() : tu.cs->sps->getUseInterEMT();
#endif

#if JVET_M0303_IMPLICIT_MTS
  bool mtsImplicit  = CU::isIntra( *tu.cu ) && tu.cs->sps->getUseImplicitMTS() && compID == COMPONENT_Y;
#endif

  trTypeHor = DCT2;
  trTypeVer = DCT2;

#if JVET_M0102_INTRA_SUBPARTITIONS
  if (tu.cu->ispMode && isLuma(compID))
  {
    TU::getTransformTypeISP(tu, compID, trTypeHor, trTypeVer);
    return;
}
#endif
#if JVET_M0140_SBT
  if( tu.cu->sbtInfo && compID == COMPONENT_Y )
  {
    uint8_t sbtIdx = tu.cu->getSbtIdx();
    uint8_t sbtPos = tu.cu->getSbtPos();

    if( sbtIdx == SBT_VER_HALF || sbtIdx == SBT_VER_QUAD )
    {
      assert( tu.lwidth() <= MTS_INTER_MAX_CU_SIZE );
      if( tu.lheight() > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if( sbtPos == SBT_POS0 )  { trTypeHor = DCT8;  trTypeVer = DST7; }
        else                      { trTypeHor = DST7;  trTypeVer = DST7; }
      }
    }
    else
    {
      assert( tu.lheight() <= MTS_INTER_MAX_CU_SIZE );
      if( tu.lwidth() > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if( sbtPos == SBT_POS0 )  { trTypeHor = DST7;  trTypeVer = DCT8; }
        else                      { trTypeHor = DST7;  trTypeVer = DST7; }
      }
    }
    return;
  }
#endif

#if JVET_M0464_UNI_MTS
  if ( mtsActivated )
#else
  if ( emtActivated )
#endif
  {
    if( compID == COMPONENT_Y )
    {
#if JVET_M0464_UNI_MTS
      if ( tu.mtsIdx > 1 )
      {
        int indHor = ( tu.mtsIdx - 2 ) &  1;
        int indVer = ( tu.mtsIdx - 2 ) >> 1;
#else
      if ( tu.cu->emtFlag )
      {
        int indHor = tu.emtIdx &  1;
        int indVer = tu.emtIdx >> 1;
#endif

        trTypeHor = indHor ? DCT8 : DST7;
        trTypeVer = indVer ? DCT8 : DST7;
      }
    }
  }
#if JVET_M0303_IMPLICIT_MTS
  else if ( mtsImplicit )
  {
    int  width       = tu.blocks[compID].width;
    int  height      = tu.blocks[compID].height;
    bool widthDstOk  = width  >= 4 && width  <= 16;
    bool heightDstOk = height >= 4 && height <= 16;

    if ( width < height && widthDstOk )
      trTypeHor = DST7;
    else if ( height < width && heightDstOk )
      trTypeVer = DST7;
    else if ( width == height && widthDstOk )
      trTypeHor = trTypeVer = DST7;
  }
#endif
}

void TrQuant::xT( const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, CoeffBuf &dstCoeff, const int width, const int height )
{
  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cs->sps->getBitDepth(              toChannelType( compID ) );
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];
#if !JVET_M0102_INTRA_SUBPARTITIONS
  const int      shift_1st              = ((g_aucLog2[width ]) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
  const int      shift_2nd              =  (g_aucLog2[height])            + TRANSFORM_MATRIX_SHIFT                          + COM16_C806_TRANS_PREC;
#endif
  const uint32_t transformWidthIndex    = g_aucLog2[width ] - 1;  // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = g_aucLog2[height] - 1;  // nLog2HeightMinus1, since transform start from 2-point
#if !JVET_M0297_32PT_MTS_ZERO_OUT
  const int      skipWidth              = width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  const int      skipHeight             = height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#endif

#if !JVET_M0102_INTRA_SUBPARTITIONS
  VTMCHECK( shift_1st < 0, "Negative shift" );
  VTMCHECK( shift_2nd < 0, "Negative shift" );
#endif

  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

  getTrTypes ( tu, compID, trTypeHor, trTypeVer );

#if JVET_M0297_32PT_MTS_ZERO_OUT
  const int      skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  const int      skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#endif

#if RExt__DECODER_DEBUG_TOOL_STATISTICS
  if ( trTypeHor != DCT2 )
  {
    CodingStatistics::IncrementStatisticTool( CodingStatisticsClassType{ STATS__TOOL_EMT, uint32_t( width ), uint32_t( height ), compID } );
  }
#endif

  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, TCoeff block[MAX_TU_SIZE * MAX_TU_SIZE] );

  const Pel *resiBuf    = resi.buf;
  const int  resiStride = resi.stride;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      block[( y * width ) + x] = resiBuf[( y * resiStride ) + x];
    }
  }

#if JVET_M0102_INTRA_SUBPARTITIONS
  if( width > 1 && height > 1 ) // 2-D transform
  {
    const int      shift_1st              = ((g_aucLog2[width ]) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    const int      shift_2nd              =  (g_aucLog2[height])            + TRANSFORM_MATRIX_SHIFT                          + COM16_C806_TRANS_PREC;
    VTMCHECK( shift_1st < 0, "Negative shift" );
    VTMCHECK( shift_2nd < 0, "Negative shift" );
#endif
  TCoeff *tmp = ( TCoeff * ) alloca( width * height * sizeof( TCoeff ) );

  fastFwdTrans[trTypeHor][transformWidthIndex ](block,        tmp, shift_1st, height,        0, skipWidth);
  fastFwdTrans[trTypeVer][transformHeightIndex](tmp, dstCoeff.buf, shift_2nd, width, skipWidth, skipHeight);
#if JVET_M0102_INTRA_SUBPARTITIONS
  }
  else if( height == 1 ) //1-D horizontal transform
  {
    const int      shift              = ((g_aucLog2[width ]) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    VTMCHECK( shift < 0, "Negative shift" );
    CHECKD( ( transformWidthIndex < 0 ), "There is a problem with the width." );
    fastFwdTrans[trTypeHor][transformWidthIndex]( block, dstCoeff.buf, shift, 1, 0, skipWidth );
  }
  else //if (iWidth == 1) //1-D vertical transform
  {
    int shift = ( ( g_aucLog2[height] ) + bitDepth + TRANSFORM_MATRIX_SHIFT ) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    VTMCHECK( shift < 0, "Negative shift" );
    CHECKD( ( transformHeightIndex < 0 ), "There is a problem with the height." );
    fastFwdTrans[trTypeVer][transformHeightIndex]( block, dstCoeff.buf, shift, 1, 0, skipHeight );
  }
#endif
}

void TrQuant::xIT( const TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual )
{
  const int      width                  = pCoeff.width;
  const int      height                 = pCoeff.height;
  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cs->sps->getBitDepth(              toChannelType( compID ) );
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const TCoeff   clipMinimum            = -( 1 << maxLog2TrDynamicRange );
  const TCoeff   clipMaximum            =  ( 1 << maxLog2TrDynamicRange ) - 1;
#if !JVET_M0102_INTRA_SUBPARTITIONS
  const int      shift_1st              =   TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; // 1 has been added to shift_1st at the expense of shift_2nd
  const int      shift_2nd              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
#endif
  const uint32_t transformWidthIndex    = g_aucLog2[width ] - 1;                                // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = g_aucLog2[height] - 1;                                // nLog2HeightMinus1, since transform start from 2-point
#if !JVET_M0297_32PT_MTS_ZERO_OUT
  const int      skipWidth              = width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  const int      skipHeight             = height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#endif

#if !JVET_M0102_INTRA_SUBPARTITIONS
  VTMCHECK( shift_1st < 0, "Negative shift" );
  VTMCHECK( shift_2nd < 0, "Negative shift" );
#endif

  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

  getTrTypes ( tu, compID, trTypeHor, trTypeVer );

#if JVET_M0297_32PT_MTS_ZERO_OUT
  const int      skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  const int      skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;
#endif

#if !JVET_M0102_INTRA_SUBPARTITIONS
  TCoeff *tmp   = ( TCoeff * ) alloca( width * height * sizeof( TCoeff ) );
#endif
  TCoeff *block = ( TCoeff * ) alloca( width * height * sizeof( TCoeff ) );

#if JVET_M0102_INTRA_SUBPARTITIONS
  if( width > 1 && height > 1 ) //2-D transform
  {
    const int      shift_1st              =   TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; // 1 has been added to shift_1st at the expense of shift_2nd
    const int      shift_2nd              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    VTMCHECK( shift_1st < 0, "Negative shift" );
    VTMCHECK( shift_2nd < 0, "Negative shift" );
    TCoeff *tmp = ( TCoeff * ) alloca( width * height * sizeof( TCoeff ) );
#endif
  fastInvTrans[trTypeVer][transformHeightIndex](pCoeff.buf, tmp, shift_1st, width, skipWidth, skipHeight, clipMinimum, clipMaximum);
  fastInvTrans[trTypeHor][transformWidthIndex] (tmp,      block, shift_2nd, height,         0, skipWidth, clipMinimum, clipMaximum);
#if JVET_M0102_INTRA_SUBPARTITIONS
  }
  else if( width == 1 ) //1-D vertical transform
  {
    int shift = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    VTMCHECK( shift < 0, "Negative shift" );
    VTMCHECK( ( transformHeightIndex < 0 ), "There is a problem with the height." );
    fastInvTrans[trTypeVer][transformHeightIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipHeight, clipMinimum, clipMaximum );
  }
  else //if(iHeight == 1) //1-D horizontal transform
  {
    const int      shift              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    VTMCHECK( shift < 0, "Negative shift" );
    VTMCHECK( ( transformWidthIndex < 0 ), "There is a problem with the width." );
    fastInvTrans[trTypeHor][transformWidthIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipWidth, clipMinimum, clipMaximum );
  }
#endif

  Pel *resiBuf    = pResidual.buf;
  int  resiStride = pResidual.stride;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      resiBuf[( y * resiStride ) + x] = Pel( block[( y * width ) + x] );
    }
  }
}

/** Wrapper function between HM interface and core NxN transform skipping
 */
void TrQuant::xITransformSkip(const CCoeffBuf     &pCoeff,
                                    PelBuf        &pResidual,
                              const TransformUnit &tu,
                              const ComponentID   &compID)
{
  const CompArea &area      = tu.blocks[compID];
  const int width           = area.width;
  const int height          = area.height;
  const int maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const int channelBitDepth = tu.cs->sps->getBitDepth(toChannelType(compID));

  int iTransformShift = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);
  if( tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag() )
  {
    iTransformShift = std::max<int>( 0, iTransformShift );
  }

  int iWHScale = 1;
#if HM_QTBT_AS_IN_JEM_QUANT && !JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
  if( TU::needsBlockSizeTrafoScale( area ) )
  {
    iTransformShift += ADJ_QUANT_SHIFT;
    iWHScale = 181;
  }
#endif

  const bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );

  if( iTransformShift >= 0 )
  {
    const TCoeff offset = iTransformShift == 0 ? 0 : ( 1 << ( iTransformShift - 1 ) );

    for( uint32_t y = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++ )
      {
        pResidual.at( x, y ) = Pel( ( ( rotateResidual ? pCoeff.at( pCoeff.width - x - 1, pCoeff.height - y - 1 ) : pCoeff.at( x, y ) ) * iWHScale + offset ) >> iTransformShift );
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;

    for( uint32_t y = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++ )
      {
        pResidual.at( x, y ) = Pel( ( rotateResidual ? pCoeff.at( pCoeff.width - x - 1, pCoeff.height - y - 1 ) : pCoeff.at( x, y ) )  * iWHScale << iTransformShift );
      }
    }
  }
}

void TrQuant::xQuant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx)
{
  m_quant->quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
}

#if JVET_M0464_UNI_MTS
#if JVET_M0102_INTRA_SUBPARTITIONS
void TrQuant::transformNxN( TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, std::vector<TrMode>* trModes, const int maxCand, double* diagRatio, double* horVerRatio )
#else
void TrQuant::transformNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, std::vector<TrMode>* trModes, const int maxCand)
#endif
{
        CodingStructure &cs = *tu.cs;
  const SPS &sps            = *cs.sps;
  const CompArea &rect      = tu.blocks[compID];
  const uint32_t width      = rect.width;
  const uint32_t height     = rect.height;

  const CPelBuf  resiBuf    = cs.getResiBuf(rect);

  VTMCHECK( sps.getMaxTrSize() < width, "Unsupported transformation size" );

  int pos = 0;
  std::vector<TrCost> trCosts;
  std::vector<TrMode>::iterator it = trModes->begin();
  const double facBB[] = { 1.2, 1.3, 1.3, 1.4, 1.5 };
  while( it != trModes->end() )
  {
    tu.mtsIdx = it->first;
    CoeffBuf tempCoeff( m_mtsCoeffs[tu.mtsIdx], rect );
#if JVET_M0140_SBT
    if( tu.noResidual )
    {
      int sumAbs = 0;
      trCosts.push_back( TrCost( sumAbs, pos++ ) );
      it++;
      continue;
    }
#endif

    if( isLuma(compID) && tu.mtsIdx == 1 )
    {
      xTransformSkip( tu, compID, resiBuf, tempCoeff.buf );
    }
    else
    {
      xT( tu, compID, resiBuf, tempCoeff, width, height );
    }

    int sumAbs = 0;
    for( int pos = 0; pos < width*height; pos++ )
    {
      sumAbs += abs( tempCoeff.buf[pos] );
    }

#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
    double scaleSAD=1.0;
    if (isLuma(compID) && tu.mtsIdx==1 && ((g_aucLog2[width] + g_aucLog2[height]) & 1) == 1 )
    {
      scaleSAD=1.0/1.414213562; // compensate for not scaling transform skip coefficients by 1/sqrt(2)
    }
    trCosts.push_back( TrCost( int(sumAbs*scaleSAD), pos++ ) );
#else
    trCosts.push_back( TrCost( sumAbs, pos++ ) );
#endif
    it++;
  }

#if JVET_M0102_INTRA_SUBPARTITIONS
  // it gets the distribution of the DCT-II coefficients energy, which will be useful to discard ISP tests
  CoeffBuf coeffsDCT( m_mtsCoeffs[0], rect );
  xGetCoeffEnergy( tu, compID, coeffsDCT, diagRatio, horVerRatio );
#endif
  int numTests = 0;
  std::vector<TrCost>::iterator itC = trCosts.begin();
  const double fac   = facBB[g_aucLog2[std::max(width, height)]-2];
  const double thr   = fac * trCosts.begin()->first;
  const double thrTS = trCosts.begin()->first;
  while( itC != trCosts.end() )
  {
    const bool testTr = itC->first <= ( itC->second == 1 ? thrTS : thr ) && numTests <= maxCand;
    trModes->at( itC->second ).second = testTr;
    numTests += testTr;
    itC++;
  }
}
#endif

#if JVET_M0464_UNI_MTS
#if JVET_M0102_INTRA_SUBPARTITIONS
void TrQuant::transformNxN( TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx, const bool loadTr, double* diagRatio, double* horVerRatio )
#else
void TrQuant::transformNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx, const bool loadTr)
#endif
#else
#if JVET_M0102_INTRA_SUBPARTITIONS
void TrQuant::transformNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx, double* diagRatio, double* horVerRatio)
#else
void TrQuant::transformNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx)
#endif
#endif
{
        CodingStructure &cs = *tu.cs;
  const SPS &sps            = *cs.sps;
  const CompArea &rect      = tu.blocks[compID];
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;

  const CPelBuf resiBuf     = cs.getResiBuf(rect);
        CoeffBuf rpcCoeff   = tu.getCoeffs(compID);

#if JVET_M0140_SBT
  if( tu.noResidual )
  {
    uiAbsSum = 0;
    TU::setCbfAtDepth( tu, compID, tu.depth, uiAbsSum > 0 );
    return;
  }
#endif

  RDPCMMode rdpcmMode = RDPCM_OFF;
  rdpcmNxN(tu, compID, cQP, uiAbsSum, rdpcmMode);

  if (rdpcmMode == RDPCM_OFF)
  {
    uiAbsSum = 0;

    // transform and quantize
    if (CU::isLosslessCoded(*tu.cu))
    {
      const bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );

      for( uint32_t y = 0; y < uiHeight; y++ )
      {
        for( uint32_t x = 0; x < uiWidth; x++ )
        {
          const Pel currentSample = resiBuf.at( x, y );

          if( rotateResidual )
          {
            rpcCoeff.at( uiWidth - x - 1, uiHeight - y - 1 ) = currentSample;
          }
          else
          {
            rpcCoeff.at( x, y ) = currentSample;
          }

          uiAbsSum += TCoeff( abs( currentSample ) );
        }
      }
    }
    else
    {
      VTMCHECK( sps.getMaxTrSize() < uiWidth, "Unsupported transformation size" );

#if JVET_M0464_UNI_MTS
      CoeffBuf tempCoeff( loadTr ? m_mtsCoeffs[tu.mtsIdx] : m_plTempCoeff, rect );
#else
      CoeffBuf tempCoeff( m_plTempCoeff, rect );
#endif

      DTRACE_PEL_BUF( D_RESIDUALS, resiBuf, tu, tu.cu->predMode, compID );

#if JVET_M0464_UNI_MTS
      if( !loadTr )
      {
        if( isLuma(compID) && tu.mtsIdx == 1 )
#else
      if( tu.transformSkip[compID] )
#endif
      {
        xTransformSkip( tu, compID, resiBuf, tempCoeff.buf );
      }
      else
      {
        xT( tu, compID, resiBuf, tempCoeff, uiWidth, uiHeight );
      }
#if JVET_M0464_UNI_MTS
      }
#endif

#if JVET_M0102_INTRA_SUBPARTITIONS
      //we do this only with the DCT-II coefficients
      if( isLuma(compID) &&
#if JVET_M0464_UNI_MTS
        !loadTr && tu.mtsIdx == 0
#else
        !tu.cu->emtFlag
#endif
        )
      {
        //it gets the distribution of the coefficients energy, which will be useful to discard ISP tests
        xGetCoeffEnergy( tu, compID, tempCoeff, diagRatio, horVerRatio );
      }
#endif
      DTRACE_COEFF_BUF( D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID );

      xQuant( tu, compID, tempCoeff, uiAbsSum, cQP, ctx );

      DTRACE_COEFF_BUF( D_TCOEFF, tu.getCoeffs( compID ), tu, tu.cu->predMode, compID );
    }
  }

  // set coded block flag (CBF)
  TU::setCbfAtDepth (tu, compID, tu.depth, uiAbsSum > 0);
}

#if JVET_M0102_INTRA_SUBPARTITIONS
void TrQuant::xGetCoeffEnergy( TransformUnit &tu, const ComponentID &compID, const CoeffBuf& coeffs, double* diagRatio, double* horVerRatio )
{
  if( nullptr == diagRatio || nullptr == horVerRatio ) return;

  if( tu.cu->predMode == MODE_INTRA && !tu.cu->ispMode && isLuma( compID ) && CU::canUseISPSplit( *tu.cu, compID ) != NOT_INTRA_SUBPARTITIONS )
  {
    const int width   = tu.cu->blocks[compID].width;
    const int height  = tu.cu->blocks[compID].height;
    const int log2Sl  = width <= height ? g_aucLog2[height >> g_aucLog2[width]] : g_aucLog2[width >> g_aucLog2[height]];
    const int diPos1  = width <= height ? width  : height;
    const int diPos2  = width <= height ? height : width;
    const int ofsPos1 = width <= height ? 1 : coeffs.stride;
    const int ofsPos2 = width <= height ? coeffs.stride : 1;

    int wdtE = 0, hgtE = 0, diaE = 0;
    int* gtE = width <= height ? &wdtE : &hgtE;
    int* stE = width <= height ? &hgtE : &wdtE;

    for( int pos1 = 0; pos1 < diPos1; pos1++ )
    {
      const int posN = pos1 << log2Sl;
      for( int pos2 = 0; pos2 < diPos2; pos2++ )
      {
        const int blkP = pos1 * ofsPos1 + pos2 * ofsPos2;
        if( posN  > pos2 ) *gtE += abs( coeffs.buf[ blkP ] );
        if( posN  < pos2 ) *stE += abs( coeffs.buf[ blkP ] );
        if( posN == pos2 ) diaE += abs( coeffs.buf[ blkP ] );
      }
    }

    *horVerRatio = 0 == wdtE && 0 == hgtE ? 1 : double( wdtE ) / double( hgtE );
    *diagRatio   = 0 == wdtE && 0 == hgtE && 0 == diaE ? 1 : double( diaE ) / double( wdtE + hgtE );
  }
}
#endif

void TrQuant::applyForwardRDPCM(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, const RDPCMMode &mode)
{
  const bool bLossless      = tu.cu->transQuantBypass;
  const uint32_t uiWidth        = tu.blocks[compID].width;
  const uint32_t uiHeight       = tu.blocks[compID].height;
  const bool rotateResidual = TU::isNonTransformedResidualRotated(tu, compID);
  const uint32_t uiSizeMinus1   = (uiWidth * uiHeight) - 1;

  const CPelBuf pcResidual  = tu.cs->getResiBuf(tu.blocks[compID]);
  const CoeffBuf pcCoeff    = tu.getCoeffs(compID);

  uint32_t uiX = 0;
  uint32_t uiY = 0;

  uint32_t &majorAxis            = (mode == RDPCM_VER) ? uiX      : uiY;
  uint32_t &minorAxis            = (mode == RDPCM_VER) ? uiY      : uiX;
  const uint32_t  majorAxisLimit = (mode == RDPCM_VER) ? uiWidth  : uiHeight;
  const uint32_t  minorAxisLimit = (mode == RDPCM_VER) ? uiHeight : uiWidth;

  const bool bUseHalfRoundingPoint = (mode != RDPCM_OFF);

  uiAbsSum = 0;

  for (majorAxis = 0; majorAxis < majorAxisLimit; majorAxis++)
  {
    TCoeff accumulatorValue = 0; // 32-bit accumulator

    for (minorAxis = 0; minorAxis < minorAxisLimit; minorAxis++)
    {
      const uint32_t sampleIndex        = (uiY * uiWidth) + uiX;
      const uint32_t coefficientIndex   = (rotateResidual ? (uiSizeMinus1-sampleIndex) : sampleIndex);
      const Pel  currentSample      = pcResidual.at(uiX, uiY);
      const TCoeff encoderSideDelta = TCoeff(currentSample) - accumulatorValue;

      Pel reconstructedDelta;

      if (bLossless)
      {
        pcCoeff.buf[coefficientIndex] = encoderSideDelta;
        reconstructedDelta            = (Pel) encoderSideDelta;
      }
      else
      {
        m_quant->transformSkipQuantOneSample(tu, compID, encoderSideDelta, pcCoeff.buf[coefficientIndex],   coefficientIndex, cQP, bUseHalfRoundingPoint);
        m_quant->invTrSkipDeQuantOneSample  (tu, compID, pcCoeff.buf[coefficientIndex], reconstructedDelta, coefficientIndex, cQP);
      }

      uiAbsSum += abs(pcCoeff.buf[coefficientIndex]);

      if (mode != RDPCM_OFF)
      {
        accumulatorValue += reconstructedDelta;
      }
    }
  }
}

void TrQuant::rdpcmNxN(TransformUnit &tu, const ComponentID &compID, const QpParam &cQP, TCoeff &uiAbsSum, RDPCMMode &rdpcmMode)
{
#if JVET_M0464_UNI_MTS
  if (!CU::isRDPCMEnabled(*tu.cu) || (tu.mtsIdx!=1 && !tu.cu->transQuantBypass))
#else
  if (!CU::isRDPCMEnabled(*tu.cu) || (!tu.transformSkip[compID] && !tu.cu->transQuantBypass))
#endif
  {
    rdpcmMode = RDPCM_OFF;
  }
  else if (CU::isIntra(*tu.cu))
  {
    const ChannelType chType = toChannelType(compID);
    const uint32_t uiChFinalMode = PU::getFinalIntraMode(*tu.cs->getPU(tu.blocks[compID].pos(), chType), chType);

    if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
    {
      rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;

      applyForwardRDPCM(tu, compID, cQP, uiAbsSum, rdpcmMode);
    }
    else
    {
      rdpcmMode = RDPCM_OFF;
    }
  }
  else // not intra, need to select the best mode
  {
    const CompArea &area = tu.blocks[compID];
    const uint32_t uiWidth   = area.width;
    const uint32_t uiHeight  = area.height;

    RDPCMMode bestMode = NUMBER_OF_RDPCM_MODES;
    TCoeff    bestAbsSum = std::numeric_limits<TCoeff>::max();
    TCoeff    bestCoefficients[MAX_TU_SIZE * MAX_TU_SIZE];

    for (uint32_t modeIndex = 0; modeIndex < NUMBER_OF_RDPCM_MODES; modeIndex++)
    {
      const RDPCMMode mode = RDPCMMode(modeIndex);

      TCoeff currAbsSum = 0;

      applyForwardRDPCM(tu, compID, cQP, uiAbsSum, rdpcmMode);

      if (currAbsSum < bestAbsSum)
      {
        bestMode = mode;
        bestAbsSum = currAbsSum;

        if (mode != RDPCM_OFF)
        {
          CoeffBuf(bestCoefficients, uiWidth, uiHeight).copyFrom(tu.getCoeffs(compID));
        }
      }
    }

    rdpcmMode = bestMode;
    uiAbsSum = bestAbsSum;

    if (rdpcmMode != RDPCM_OFF) //the TU is re-transformed and quantized if DPCM_OFF is returned, so there is no need to preserve it here
    {
      tu.getCoeffs(compID).copyFrom(CoeffBuf(bestCoefficients, uiWidth, uiHeight));
    }
  }

  tu.rdpcm[compID] = rdpcmMode;
}

void TrQuant::xTransformSkip(const TransformUnit &tu, const ComponentID &compID, const CPelBuf &resi, TCoeff* psCoeff)
{
  const SPS &sps            = *tu.cs->sps;
  const CompArea &rect      = tu.blocks[compID];
  const uint32_t width          = rect.width;
  const uint32_t height         = rect.height;
  const ChannelType chType  = toChannelType(compID);
  const int channelBitDepth = sps.getBitDepth(chType);
  const int maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);
  int iTransformShift       = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  if( sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() )
  {
    iTransformShift = std::max<int>( 0, iTransformShift );
  }

  int iWHScale = 1;
#if HM_QTBT_AS_IN_JEM_QUANT && !JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
  if( TU::needsBlockSizeTrafoScale( rect ) )
  {
    iTransformShift -= ADJ_DEQUANT_SHIFT;
    iWHScale = 181;
  }
#endif

  const bool rotateResidual = TU::isNonTransformedResidualRotated( tu, compID );
  const uint32_t uiSizeMinus1 = ( width * height ) - 1;

  if( iTransformShift >= 0 )
  {
    for( uint32_t y = 0, coefficientIndex = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++, coefficientIndex++ )
      {
        psCoeff[rotateResidual ? uiSizeMinus1 - coefficientIndex : coefficientIndex] = ( TCoeff( resi.at( x, y ) ) * iWHScale ) << iTransformShift;
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;
    const TCoeff offset = 1 << ( iTransformShift - 1 );

    for( uint32_t y = 0, coefficientIndex = 0; y < height; y++ )
    {
      for( uint32_t x = 0; x < width; x++, coefficientIndex++ )
      {
        psCoeff[rotateResidual ? uiSizeMinus1 - coefficientIndex : coefficientIndex] = ( TCoeff( resi.at( x, y ) ) * iWHScale + offset ) >> iTransformShift;
      }
    }
  }
}

//! \}
