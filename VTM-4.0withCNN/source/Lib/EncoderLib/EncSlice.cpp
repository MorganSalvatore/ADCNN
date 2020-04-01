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

/** \file     EncSlice.cpp
    \brief    slice encoder class
*/

#include "EncSlice.h"

#include "EncLib.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/Picture.h"
#if K0149_BLOCK_STATISTICS
#include "CommonLib/dtrace_blockstatistics.h"
#endif

#if ENABLE_WPP_PARALLELISM
#include <mutex>
extern recursive_mutex g_cache_mutex;
#endif

#include <math.h>

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncSlice::EncSlice()
 : m_encCABACTableIdx(I_SLICE)
#if ENABLE_QPA
 , m_adaptedLumaQP(-1)
#endif
{
}

EncSlice::~EncSlice()
{
  destroy();
}

void EncSlice::create( int iWidth, int iHeight, ChromaFormat chromaFormat, uint32_t iMaxCUWidth, uint32_t iMaxCUHeight, uint8_t uhTotalDepth )
{
}

void EncSlice::destroy()
{
  // free lambda and QP arrays
  m_vdRdPicLambda.clear();
  m_vdRdPicQp.clear();
  m_viRdPicQp.clear();
}

void EncSlice::init( EncLib* pcEncLib, const SPS& sps )
{
  m_pcCfg             = pcEncLib;
  m_pcLib             = pcEncLib;
  m_pcListPic         = pcEncLib->getListPic();

  m_pcGOPEncoder      = pcEncLib->getGOPEncoder();
  m_pcCuEncoder       = pcEncLib->getCuEncoder();
  m_pcInterSearch     = pcEncLib->getInterSearch();
  m_CABACWriter       = pcEncLib->getCABACEncoder()->getCABACWriter   (&sps);
  m_CABACEstimator    = pcEncLib->getCABACEncoder()->getCABACEstimator(&sps);
  m_pcTrQuant         = pcEncLib->getTrQuant();
  m_pcRdCost          = pcEncLib->getRdCost();

  // create lambda and QP arrays
  m_vdRdPicLambda.resize(m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_vdRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_viRdPicQp.resize(    m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_pcRateCtrl        = pcEncLib->getRateCtrl();
}

void
EncSlice::setUpLambda( Slice* slice, const double dLambda, int iQP)
{
  // store lambda
  m_pcRdCost ->setLambda( dLambda, slice->getSPS()->getBitDepths() );

  // for RDO
  // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
  double dLambdas[MAX_NUM_COMPONENT] = { dLambda };
  for( uint32_t compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    int chromaQPOffset       = slice->getPPS()->getQpOffset( compID ) + slice->getSliceChromaQpDelta( compID );
    int qpc                  = ( iQP + chromaQPOffset < 0 ) ? iQP : getScaledChromaQP( iQP + chromaQPOffset, m_pcCfg->getChromaFormatIdc() );
    double tmpWeight         = pow( 2.0, ( iQP - qpc ) / 3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
    if( m_pcCfg->getDepQuantEnabledFlag() )
    {
      tmpWeight *= ( m_pcCfg->getGOPSize() >= 8 ? pow( 2.0, 0.1/3.0 ) : pow( 2.0, 0.2/3.0 ) );  // increase chroma weight for dependent quantization (in order to reduce bit rate shift from chroma to luma)
    }
    m_pcRdCost->setDistortionWeight( compID, tmpWeight );
#if ENABLE_WPP_PARALLELISM
    for( int jId = 1; jId < ( m_pcLib->getNumWppThreads() + m_pcLib->getNumWppExtraLines() ); jId++ )
    {
      m_pcLib->getRdCost( slice->getPic()->scheduler.getWppDataId( jId ) )->setDistortionWeight( compID, tmpWeight );
    }
#endif
    dLambdas[compIdx] = dLambda / tmpWeight;
  }

#if RDOQ_CHROMA_LAMBDA
  // for RDOQ
  m_pcTrQuant->setLambdas( dLambdas );
#else
  m_pcTrQuant->setLambda( dLambda );
#endif

  // for SAO
  slice->setLambdas( dLambdas );
}

#if ENABLE_QPA

static inline int apprI3Log2 (const double d) // rounded 3*log2(d)
{
  return d < 1.5e-13 ? -128 : int (floor (3.0 * log (d) / log (2.0) + 0.5));
}

static void filterAndCalculateAverageEnergies (const Pel* pSrc, const int  iSrcStride,
                                               double &hpEner,  const int  iHeight,    const int iWidth,
                                               const uint32_t uBitDepth /* luma bit-depth (4-16) */)
{
  uint64_t saAct = 0;

  // skip first row as there may be a black border frame
  pSrc += iSrcStride;
  // center rows
  for (int y = 1; y < iHeight - 1; y++)
  {
    // skip column as there may be a black border frame

    for (int x = 1; x < iWidth - 1; x++) // and columns
    {
      const int f = 12 * (int)pSrc[x  ] - 2 * ((int)pSrc[x-1] + (int)pSrc[x+1] + (int)pSrc[x  -iSrcStride] + (int)pSrc[x  +iSrcStride])
                       - (int)pSrc[x-1-iSrcStride] - (int)pSrc[x+1-iSrcStride] - (int)pSrc[x-1+iSrcStride] - (int)pSrc[x+1+iSrcStride];
      saAct += abs (f);
    }
    // skip column as there may be a black border frame
    pSrc += iSrcStride;
  }
  // skip last row as there may be a black border frame

  hpEner = double(saAct) / double((iWidth - 2) * (iHeight - 2));

  // lower limit, compensate for highpass amplification
  if (hpEner < double(1 << (uBitDepth - 4))) hpEner = double(1 << (uBitDepth - 4));
}

#ifndef GLOBAL_AVERAGING
  #define GLOBAL_AVERAGING 1 // "global" averaging of a_k across a set instead of one picture
#endif

#if GLOBAL_AVERAGING
static double getAveragePictureEnergy (const CPelBuf picOrig, const uint32_t uBitDepth)
{
  const double hpEnerPic = 16.0 * sqrt ((3840.0 * 2160.0) / double(picOrig.width * picOrig.height)) * double(1 << uBitDepth);

  return sqrt (hpEnerPic); // square-root of a_pic value
}
#endif

static int applyQPAdaptationChroma (Picture* const pcPic, Slice* const pcSlice, EncCfg* const pcEncCfg, const int sliceQP)
{
  double hpEner[MAX_NUM_COMPONENT] = {0.0, 0.0, 0.0};
  int    optSliceChromaQpOffset[2] = {0, 0};
  int    savedLumaQP               = -1;

  for (uint32_t comp = 0; comp < getNumberValidComponents (pcPic->chromaFormat); comp++)
  {
    const ComponentID compID = (ComponentID)comp;
    const CPelBuf    picOrig = pcPic->getOrigBuf (pcPic->block (compID));

    filterAndCalculateAverageEnergies (picOrig.buf, picOrig.stride, hpEner[comp], picOrig.height, picOrig.width,
                                       pcSlice->getSPS()->getBitDepth (toChannelType (compID)) - (isChroma (compID) ? 1 : 0));
    if (isChroma (compID))
    {
      const int  adaptChromaQPOffset = 2.0 * hpEner[comp] <= hpEner[0] ? 0 : apprI3Log2 (2.0 * hpEner[comp] / hpEner[0]);
   #if GLOBAL_AVERAGING
      int       averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP + apprI3Log2 (hpEner[0] / getAveragePictureEnergy (pcPic->getOrigBuf().Y(), pcSlice->getSPS()->getBitDepth (CH_L))));
   #else
      int       averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP); // mean slice QP
   #endif
   #if SHARP_LUMA_DELTA_QP

      // change mean picture QP index based on picture's average luma value (Sharp)
      if (pcEncCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES)
      {
        const CPelBuf picLuma = pcPic->getOrigBuf().Y();
        uint64_t uAvgLuma = 0;

        for (SizeType y = 0; y < picLuma.height; y++)
        {
          for (SizeType x = 0; x < picLuma.width; x++)
          {
            uAvgLuma += (uint64_t)picLuma.at (x, y);
          }
        }
        uAvgLuma = (uAvgLuma + (picLuma.area() >> 1)) / picLuma.area();

        averageAdaptedLumaQP = Clip3 (0, MAX_QP, averageAdaptedLumaQP + 1 - int((3 * uAvgLuma * uAvgLuma) >> uint64_t (2 * pcSlice->getSPS()->getBitDepth (CH_L) - 1)));
      }
   #endif
      const int lumaChromaMappingDQP = averageAdaptedLumaQP - getScaledChromaQP (averageAdaptedLumaQP, pcEncCfg->getChromaFormatIdc());

      optSliceChromaQpOffset[comp-1] = std::min (3 + lumaChromaMappingDQP, adaptChromaQPOffset + lumaChromaMappingDQP);

      if (savedLumaQP < 0) savedLumaQP = averageAdaptedLumaQP; // save it for later
    }
  }

  pcEncCfg->setSliceChromaOffsetQpIntraOrPeriodic (pcEncCfg->getSliceChromaOffsetQpPeriodicity(), optSliceChromaQpOffset);

  return savedLumaQP;
}

#endif // ENABLE_QPA

/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pcPic         picture class
 \param pocLast       POC of last picture
 \param pocCurr       current POC
 \param iNumPicRcvd   number of received pictures
 \param iGOPid        POC offset for hierarchical structure
 \param rpcSlice      slice header class
 \param isField       true for field coding
 */
void EncSlice::initEncSlice(Picture* pcPic, const int pocLast, const int pocCurr, const int iGOPid, Slice*& rpcSlice, const bool isField
  , bool isEncodeLtRef
)
{
  double dQP;
  double dLambda;

  rpcSlice = pcPic->slices[0];
  rpcSlice->setSliceBits(0);
  rpcSlice->setPic( pcPic );
  rpcSlice->initSlice();
  int multipleFactor = pcPic->cs->sps->getUseCompositeRef() ? 2 : 1;
  if (pcPic->cs->sps->getUseCompositeRef() && isEncodeLtRef)
  {
    rpcSlice->setPicOutputFlag(false);
  }
  else
  {
    rpcSlice->setPicOutputFlag(true);
  }
  rpcSlice->setPOC( pocCurr );
  rpcSlice->setDepQuantEnabledFlag( m_pcCfg->getDepQuantEnabledFlag() );
#if HEVC_USE_SIGN_HIDING
  rpcSlice->setSignDataHidingEnabledFlag( m_pcCfg->getSignDataHidingEnabledFlag() );
#endif

#if SHARP_LUMA_DELTA_QP
  pcPic->fieldPic = isField;
  m_gopID = iGOPid;
#endif

  // depth computation based on GOP size
  int depth;
  {
    int poc = rpcSlice->getPOC();
    if(isField)
    {
      poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
    }
    else
    {
      poc = poc % (m_pcCfg->getGOPSize() * multipleFactor);
    }

    if ( poc == 0 )
    {
      depth = 0;
    }
    else
    {
      int step = m_pcCfg->getGOPSize() * multipleFactor;
      depth    = 0;
      for( int i=step>>1; i>=1; i>>=1 )
      {
        for (int j = i; j<(m_pcCfg->getGOPSize() * multipleFactor); j += step)
        {
          if ( j == poc )
          {
            i=0;
            break;
          }
        }
        step >>= 1;
        depth++;
      }
    }

    if(m_pcCfg->getHarmonizeGopFirstFieldCoupleEnabled() && poc != 0)
    {
      if (isField && ((rpcSlice->getPOC() % 2) == 1))
      {
        depth++;
      }
    }
  }

  // slice type
  SliceType eSliceType;

  eSliceType=B_SLICE;
  if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    if(m_pcCfg->getDecodingRefreshType() == 3)
    {
      eSliceType = (pocLast == 0 || pocCurr % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
    else
    {
      eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
  }

  rpcSlice->setSliceType    ( eSliceType );

  // ------------------------------------------------------------------------------------------------------------------
  // Non-referenced frame marking
  // ------------------------------------------------------------------------------------------------------------------

  if(pocLast == 0)
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(false);
  }
  else
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);
  }
  pcPic->referenced = true;

  // ------------------------------------------------------------------------------------------------------------------
  // QP setting
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  dQP = m_pcCfg->getQPForPicture(iGOPid, rpcSlice);
#else
  dQP = m_pcCfg->getBaseQP();
  if(eSliceType!=I_SLICE)
  {
#if SHARP_LUMA_DELTA_QP
    if (!(( m_pcCfg->getMaxDeltaQP() == 0) && (!m_pcCfg->getLumaLevelToDeltaQPMapping().isEnabled()) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnabledFlag())))
#else
    if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnabledFlag())))
#endif
    {
      dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;
    }
  }

  // modify QP
  const int* pdQPs = m_pcCfg->getdQPs();
  if ( pdQPs )
  {
    dQP += pdQPs[ rpcSlice->getPOC() ];
  }

  if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)
  {
    dQP=LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
    m_pcCfg->setDeltaQpRD(0);
  }
#endif

  // ------------------------------------------------------------------------------------------------------------------
  // Lambda computation
  // ------------------------------------------------------------------------------------------------------------------

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
#if !SHARP_LUMA_DELTA_QP
  const std::vector<double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif
#endif
  int iQP;
  double dOrigQP = dQP;

  // pre-compute lambda and QP values for all possible QP candidates
  for ( int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )
  {
    // compute QP value
    dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);
#if SHARP_LUMA_DELTA_QP
    dLambda = calculateLambda(rpcSlice, iGOPid, depth, dQP, dQP, iQP );
#else
    // compute lambda value
    int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
    int    SHIFT_QP = 12;

    int    bitdepth_luma_qp_scale =
      6
      * (rpcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
         - DISTORTION_PRECISION_ADJUSTMENT(rpcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));
    double qp_temp = (double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
    double qp_temp_orig = (double) dQP - SHIFT_QP;
#endif
    // Case #1: I or P-slices (key-frame)
    double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;
    if ( eSliceType==I_SLICE )
    {
      if (m_pcCfg->getIntraQpFactor()>=0.0 && m_pcCfg->getGOPEntry(iGOPid).m_sliceType != I_SLICE)
      {
        dQPFactor=m_pcCfg->getIntraQpFactor();
      }
      else
      {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
        if(m_pcCfg->getLambdaFromQPEnable())
        {
          dQPFactor=0.57;
        }
        else
        {
#endif
        double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? NumberBFrames/2 : NumberBFrames) );

        dQPFactor=0.57*dLambda_scale;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
        }
#endif
      }
    }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
    else if( m_pcCfg->getLambdaFromQPEnable() )
    {
      dQPFactor=0.57;
    }
#endif

    dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
    if(!m_pcCfg->getLambdaFromQPEnable() && depth>0)
#else
    if ( depth>0 )
#endif
    {
#if FULL_NBIT
        dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
        dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#endif
    }

    // if hadamard is used in ME process
    if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )
    {
      dLambda *= 0.95;
    }

#if X0038_LAMBDA_FROM_QP_CAPABILITY
    double lambdaModifier;
    if( rpcSlice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
    {
      lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
    }
    else
    {
      lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
    }
    dLambda *= lambdaModifier;
#endif

    iQP = Clip3( -rpcSlice->getSPS()->getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, (int) floor( dQP + 0.5 ) );
#endif

    m_vdRdPicLambda[iDQpIdx] = dLambda;
    m_vdRdPicQp    [iDQpIdx] = dQP;
    m_viRdPicQp    [iDQpIdx] = iQP;
  }

  // obtain dQP = 0 case
  dLambda = m_vdRdPicLambda[0];
  dQP     = m_vdRdPicQp    [0];
  iQP     = m_viRdPicQp    [0];

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  const int temporalId=m_pcCfg->getGOPEntry(iGOPid).m_temporalId;
  const std::vector<double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif

#if W0038_CQP_ADJ
 #if ENABLE_QPA
  m_adaptedLumaQP = -1;

  if ((m_pcCfg->getUsePerceptQPA() || m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0) && !m_pcCfg->getUseRateCtrl() && rpcSlice->getPPS()->getSliceChromaQpFlag() &&
      (rpcSlice->isIntra() || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0)))
  {
    m_adaptedLumaQP = applyQPAdaptationChroma (pcPic, rpcSlice, m_pcCfg, iQP);
  }
 #endif
  if(rpcSlice->getPPS()->getSliceChromaQpFlag())
  {
#if JVET_M0483_IBC
    const bool bUseIntraOrPeriodicOffset = (rpcSlice->isIntra() && !rpcSlice->getSPS()->getIBCFlag()) || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0);
#else
    const bool bUseIntraOrPeriodicOffset = rpcSlice->isIntra() || (m_pcCfg->getSliceChromaOffsetQpPeriodicity() > 0 && (rpcSlice->getPOC() % m_pcCfg->getSliceChromaOffsetQpPeriodicity()) == 0);
#endif
    int cbQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(false) : m_pcCfg->getGOPEntry(iGOPid).m_CbQPoffset;
    int crQP = bUseIntraOrPeriodicOffset ? m_pcCfg->getSliceChromaOffsetQpIntraOrPeriodic(true)  : m_pcCfg->getGOPEntry(iGOPid).m_CrQPoffset;

    cbQP = Clip3( -12, 12, cbQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb);
    crQP = Clip3( -12, 12, crQP + rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr) ) - rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr);
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cb, Clip3( -12, 12, cbQP));
    VTMCHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cb)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cb)>=-12), "Unspecified error");
    rpcSlice->setSliceChromaQpDelta(COMPONENT_Cr, Clip3( -12, 12, crQP));
    VTMCHECK(!(rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)<=12 && rpcSlice->getSliceChromaQpDelta(COMPONENT_Cr)+rpcSlice->getPPS()->getQpOffset(COMPONENT_Cr)>=-12), "Unspecified error");
  }
  else
  {
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
    rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
  }
#endif

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
  double lambdaModifier;
  if( rpcSlice->getSliceType( ) != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }

  dLambda *= lambdaModifier;
#endif

  setUpLambda(rpcSlice, dLambda, iQP);

#if WCG_EXT
  // cost = Distortion + Lambda*R,
  // when QP is adjusted by luma, distortion is changed, so we have to adjust lambda to match the distortion, then the cost function becomes
  // costA = Distortion + AdjustedLambda * R          -- currently, costA is still used when calculating intermediate cost of using SAD, HAD, resisual etc.
  // an alternative way is to weight the distortion to before the luma QP adjustment, then the cost function becomes
  // costB = weightedDistortion + Lambda * R          -- currently, costB is used to calculat final cost, and when DF_FUNC is DF_DEFAULT
  m_pcRdCost->saveUnadjustedLambda();
#endif

  if (m_pcCfg->getFastMEForGenBLowDelayEnabled())
  {
    // restore original slice type

    if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if(m_pcCfg->getDecodingRefreshType() == 3)
      {
        eSliceType = (pocLast == 0 || (pocCurr) % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
      else
      {
        eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % (m_pcCfg->getIntraPeriod() * multipleFactor) == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
    }

    rpcSlice->setSliceType        ( eSliceType );
  }

  if (m_pcCfg->getUseRecalculateQPAccordingToLambda())
  {
    dQP = xGetQPValueAccordingToLambda( dLambda );
    iQP = Clip3( -rpcSlice->getSPS()->getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, (int) floor( dQP + 0.5 ) );
  }

  rpcSlice->setSliceQp           ( iQP );
  rpcSlice->setSliceQpDelta      ( 0 );
#if !W0038_CQP_ADJ
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );
#endif
  rpcSlice->setUseChromaQpAdj( rpcSlice->getPPS()->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag() );
  rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
  rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);

  if ( m_pcCfg->getDeblockingFilterMetric() )
  {
    rpcSlice->setDeblockingFilterOverrideFlag(true);
    rpcSlice->setDeblockingFilterDisable(false);
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }
  else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
  {
    rpcSlice->setDeblockingFilterOverrideFlag( rpcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() );
    rpcSlice->setDeblockingFilterDisable( rpcSlice->getPPS()->getPPSDeblockingFilterDisabledFlag() );
    if ( !rpcSlice->getDeblockingFilterDisable())
    {
      if ( rpcSlice->getDeblockingFilterOverrideFlag() && eSliceType!=I_SLICE)
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
      }
      else
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
      }
    }
  }
  else
  {
    rpcSlice->setDeblockingFilterOverrideFlag( false );
    rpcSlice->setDeblockingFilterDisable( false );
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
  }

  rpcSlice->setDepth            ( depth );

  pcPic->layer =  temporalId;
  if(eSliceType==I_SLICE)
  {
    pcPic->layer = 0;
  }
  rpcSlice->setTLayer( pcPic->layer );

  rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );
  rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );
#if HEVC_DEPENDENT_SLICES
  rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );
  rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );
#endif
  rpcSlice->setMaxNumMergeCand      ( m_pcCfg->getMaxNumMergeCand()      );
  rpcSlice->setMaxNumAffineMergeCand( m_pcCfg->getMaxNumAffineMergeCand() );
  rpcSlice->setSplitConsOverrideFlag(false);
  rpcSlice->setMinQTSize( rpcSlice->getSPS()->getMinQTSize(eSliceType));
  rpcSlice->setMaxBTDepth( rpcSlice->isIntra() ? rpcSlice->getSPS()->getMaxBTDepthI() : rpcSlice->getSPS()->getMaxBTDepth() );
  rpcSlice->setMaxBTSize( rpcSlice->isIntra() ? rpcSlice->getSPS()->getMaxBTSizeI() : rpcSlice->getSPS()->getMaxBTSize() );
  rpcSlice->setMaxTTSize( rpcSlice->isIntra() ? rpcSlice->getSPS()->getMaxTTSizeI() : rpcSlice->getSPS()->getMaxTTSize() );
  if ( eSliceType == I_SLICE && rpcSlice->getSPS()->getUseDualITree() )
  {
    rpcSlice->setMinQTSizeIChroma( rpcSlice->getSPS()->getMinQTSize(eSliceType, CHANNEL_TYPE_CHROMA) );
    rpcSlice->setMaxBTDepthIChroma( rpcSlice->getSPS()->getMaxBTDepthIChroma() );
    rpcSlice->setMaxBTSizeIChroma( rpcSlice->getSPS()->getMaxBTSizeIChroma() );
    rpcSlice->setMaxTTSizeIChroma( rpcSlice->getSPS()->getMaxTTSizeIChroma() );
  }
}


#if SHARP_LUMA_DELTA_QP
double EncSlice::calculateLambda( const Slice*     slice,
                                  const int        GOPid, // entry in the GOP table
                                  const int        depth, // slice GOP hierarchical depth.
                                  const double     refQP, // initial slice-level QP
                                  const double     dQP,   // initial double-precision QP
                                        int       &iQP )  // returned integer QP.
{
  enum   SliceType eSliceType    = slice->getSliceType();
  const  bool      isField       = slice->getPic()->fieldPic;
  const  int       NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
  const  int       SHIFT_QP      = 12;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  const int temporalId=m_pcCfg->getGOPEntry(GOPid).m_temporalId;
  const std::vector<double> &intraLambdaModifiers=m_pcCfg->getIntraLambdaModifier();
#endif

  int bitdepth_luma_qp_scale = 6
                               * (slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
                                  - DISTORTION_PRECISION_ADJUSTMENT(slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));
  double qp_temp = dQP + bitdepth_luma_qp_scale - SHIFT_QP;
  // Case #1: I or P-slices (key-frame)
  double dQPFactor = m_pcCfg->getGOPEntry(GOPid).m_QPFactor;
  if ( eSliceType==I_SLICE )
  {
    if (m_pcCfg->getIntraQpFactor()>=0.0 && m_pcCfg->getGOPEntry(GOPid).m_sliceType != I_SLICE)
    {
      dQPFactor=m_pcCfg->getIntraQpFactor();
    }
    else
    {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      if(m_pcCfg->getLambdaFromQPEnable())
      {
        dQPFactor=0.57;
      }
      else
      {
#endif
        double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? NumberBFrames/2 : NumberBFrames) );
        dQPFactor=0.57*dLambda_scale;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      }
#endif
    }
  }
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  else if( m_pcCfg->getLambdaFromQPEnable() )
  {
    dQPFactor=0.57;
  }
#endif

  double dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  if( !(m_pcCfg->getLambdaFromQPEnable()) && depth>0 )
#else
  if ( depth>0 )
#endif
  {
    double qp_temp_ref = refQP + bitdepth_luma_qp_scale - SHIFT_QP;
    dLambda *= Clip3(2.00, 4.00, (qp_temp_ref / 6.0));   // (j == B_SLICE && p_cur_frm->layer != 0 )
  }

  // if hadamard is used in ME process
  if ( !m_pcCfg->getUseHADME() && slice->getSliceType( ) != I_SLICE )
  {
    dLambda *= 0.95;
  }

#if X0038_LAMBDA_FROM_QP_CAPABILITY
  double lambdaModifier;
  if( eSliceType != I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcCfg->getLambdaModifier( temporalId );
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }
  dLambda *= lambdaModifier;
#endif

  iQP = Clip3( -slice->getSPS()->getQpBDOffset( CHANNEL_TYPE_LUMA ), MAX_QP, (int) floor( dQP + 0.5 ) );

  if( m_pcCfg->getDepQuantEnabledFlag() )
  {
    dLambda *= pow( 2.0, 0.25/3.0 ); // slight lambda adjustment for dependent quantization (due to different slope of quantizer)
  }

  // NOTE: the lambda modifiers that are sometimes applied later might be best always applied in here.
  return dLambda;
}
#endif

void EncSlice::resetQP( Picture* pic, int sliceQP, double lambda )
{
  Slice* slice = pic->slices[0];

  // store lambda
  slice->setSliceQp( sliceQP );
  setUpLambda(slice, lambda, sliceQP);
}

#if ENABLE_QPA
static bool applyQPAdaptation (Picture* const pcPic,     Slice* const pcSlice,        const PreCalcValues& pcv,
                               const uint32_t startAddr, const uint32_t boundingAddr, const bool useSharpLumaDQP,
                               const double hpEnerAvg,   const double hpEnerMax,      const bool useFrameWiseQPA, const int previouslyAdaptedLumaQP = -1)
{
  const int  iBitDepth   = pcSlice->getSPS()->getBitDepth (CHANNEL_TYPE_LUMA);
  const int  iQPIndex    = pcSlice->getSliceQp(); // initial QP index for current slice, used in following loops
#if HEVC_TILES_WPP
  const TileMap& tileMap = *pcPic->tileMap;
#endif
  bool   sliceQPModified = false;
#if GLOBAL_AVERAGING
  const double hpEnerPic = 1.0 / getAveragePictureEnergy (pcPic->getOrigBuf().Y(), iBitDepth); // inverse, speed
#else
  const double hpEnerPic = 1.0 / hpEnerAvg; // speedup: multiply instead of divide in loop below; 1.0 for tuning
#endif

  if (useFrameWiseQPA || (iQPIndex >= MAX_QP))
  {
    int iQPFixed;

    if (useFrameWiseQPA)
    {
      iQPFixed = (previouslyAdaptedLumaQP < 0) ? Clip3 (0, MAX_QP, iQPIndex + apprI3Log2 (hpEnerAvg * hpEnerPic)) : previouslyAdaptedLumaQP; // average-activity slice QP
    }
    else
    {
      iQPFixed = Clip3 (0, MAX_QP, iQPIndex + ((apprI3Log2 (hpEnerAvg * hpEnerPic) + apprI3Log2 (hpEnerMax * hpEnerPic) + 1) >> 1)); // adapted slice QP = (mean(QP) + max(QP)) / 2
    }
#if SHARP_LUMA_DELTA_QP

    // change new fixed QP based on average CTU luma value (Sharp)
    if (useSharpLumaDQP && (iQPIndex < MAX_QP) && (previouslyAdaptedLumaQP < 0))
    {
      uint64_t uAvgLuma = 0;

      for (uint32_t ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
      {
#if HEVC_TILES_WPP
        const uint32_t ctuRsAddr = tileMap.getCtuTsToRsAddrMap (ctuTsAddr);
#else
        const uint32_t ctuRsAddr = ctuTsAddr;
#endif

        uAvgLuma += (uint64_t)pcPic->m_iOffsetCtu[ctuRsAddr];
      }
      uAvgLuma = (uAvgLuma + ((boundingAddr - startAddr) >> 1)) / (boundingAddr - startAddr);

      iQPFixed = Clip3 (0, MAX_QP, iQPFixed + 1 - int((3 * uAvgLuma * uAvgLuma) >> uint64_t(2 * iBitDepth - 1)));
    }
#endif

    if (iQPIndex >= MAX_QP) iQPFixed = MAX_QP;
    else
    if (iQPFixed != iQPIndex)
    {
      const double* oldLambdas = pcSlice->getLambdas();
      const double  corrFactor = pow (2.0, double(iQPFixed - iQPIndex) / 3.0);
      const double  newLambdas[MAX_NUM_COMPONENT] = {oldLambdas[0] * corrFactor, oldLambdas[1] * corrFactor, oldLambdas[2] * corrFactor};

      VTMCHECK (iQPIndex != pcSlice->getSliceQpBase(), "Invalid slice QP!");
      pcSlice->setLambdas (newLambdas);
      pcSlice->setSliceQp (iQPFixed); // update the slice/base QPs
      pcSlice->setSliceQpBase (iQPFixed);

      sliceQPModified = true;
    }

    for (uint32_t ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
    {
#if HEVC_TILES_WPP
      const uint32_t ctuRsAddr = tileMap.getCtuTsToRsAddrMap (ctuTsAddr);
#else
      const uint32_t ctuRsAddr = ctuTsAddr;
#endif

      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPFixed; // fixed QPs
    }
  }
  else
  {
    for (uint32_t ctuTsAddr = startAddr; ctuTsAddr < boundingAddr; ctuTsAddr++)
    {
#if HEVC_TILES_WPP
      const uint32_t ctuRsAddr = tileMap.getCtuTsToRsAddrMap (ctuTsAddr);
#else
      const uint32_t ctuRsAddr = ctuTsAddr;
#endif

      int iQPAdapt = Clip3 (0, MAX_QP, iQPIndex + apprI3Log2 (pcPic->m_uEnerHpCtu[ctuRsAddr] * hpEnerPic));

      if (pcv.widthInCtus > 1) // try to enforce CTU SNR greater than zero dB
      {
        const Pel      dcOffset   = pcPic->m_iOffsetCtu[ctuRsAddr];
#if SHARP_LUMA_DELTA_QP

        // change adaptive QP based on mean CTU luma value (Sharp)
        if (useSharpLumaDQP)
        {
          const uint64_t uAvgLuma   = (uint64_t)dcOffset;

          iQPAdapt = std::max (0, iQPAdapt + 1 - int((3 * uAvgLuma * uAvgLuma) >> uint64_t(2 * iBitDepth - 1)));
        }

#endif
        const uint32_t uRefScale  = g_invQuantScales[iQPAdapt % 6] << ((iQPAdapt / 6) + iBitDepth - 4);
        const CompArea subArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area ((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
        const Pel*     pSrc       = pcPic->getOrigBuf (subArea).buf;
        const SizeType iSrcStride = pcPic->getOrigBuf (subArea).stride;
        const SizeType iSrcHeight = pcPic->getOrigBuf (subArea).height;
        const SizeType iSrcWidth  = pcPic->getOrigBuf (subArea).width;
        uint32_t uAbsDCless = 0;

        // compute sum of absolute DC-less (high-pass) luma values
        for (SizeType h = 0; h < iSrcHeight; h++)
        {
          for (SizeType w = 0; w < iSrcWidth; w++)
          {
            uAbsDCless += (uint32_t)abs (pSrc[w] - dcOffset);
          }
          pSrc += iSrcStride;
        }

        if (iSrcHeight >= 64 || iSrcWidth >= 64)  // normalization
        {
          const uint64_t blockSize = uint64_t(iSrcWidth * iSrcHeight);

          uAbsDCless = uint32_t((uint64_t(uAbsDCless) * 64*64 + (blockSize >> 1)) / blockSize);
        }

        if (uAbsDCless < 64*64) uAbsDCless = 64*64;  // limit to 1

        // reduce QP index if CTU would be fully quantized to zero
        if (uAbsDCless < uRefScale)
        {
          const int limit  = std::min (0, ((iQPIndex + 4) >> 3) - 6);
          const int redVal = std::max (limit, apprI3Log2 ((double)uAbsDCless / (double)uRefScale));

          iQPAdapt = std::max (0, iQPAdapt + redVal);
        }
#if SHARP_LUMA_DELTA_QP

        if (iQPAdapt > MAX_QP) iQPAdapt = MAX_QP;
#endif
      }

      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt; // adapted QPs

      if (pcv.widthInCtus > 1) // try to reduce local bitrate peaks via minimum smoothing of the adapted QPs
      {
        iQPAdapt = ctuRsAddr % pcv.widthInCtus; // horizontal offset
        if (iQPAdapt == 0)
        {
          iQPAdapt = (ctuRsAddr > 1) ? pcPic->m_iOffsetCtu[ctuRsAddr - 2] : 0;
        }
        else // iQPAdapt >= 1
        {
          iQPAdapt = (iQPAdapt > 1) ? std::min (pcPic->m_iOffsetCtu[ctuRsAddr - 2], pcPic->m_iOffsetCtu[ctuRsAddr]) : pcPic->m_iOffsetCtu[ctuRsAddr];
        }
        if (ctuRsAddr > pcv.widthInCtus)
        {
          iQPAdapt = std::min (iQPAdapt, (int)pcPic->m_iOffsetCtu[ctuRsAddr - 1 - pcv.widthInCtus]);
        }
        if ((ctuRsAddr > 0) && (pcPic->m_iOffsetCtu[ctuRsAddr - 1] < (Pel)iQPAdapt))
        {
          pcPic->m_iOffsetCtu[ctuRsAddr - 1] = (Pel)iQPAdapt;
        }
        if ((ctuTsAddr == boundingAddr - 1) && (ctuRsAddr > pcv.widthInCtus)) // last CTU in the given slice
        {
          iQPAdapt = std::min (pcPic->m_iOffsetCtu[ctuRsAddr - 1], pcPic->m_iOffsetCtu[ctuRsAddr - pcv.widthInCtus]);
          if (pcPic->m_iOffsetCtu[ctuRsAddr] < (Pel)iQPAdapt)
          {
            pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iQPAdapt;
          }
        }
      }
    } // end iteration over all CTUs in current slice
  }

  return sliceQPModified;
}
#endif // ENABLE_QPA

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

//! set adaptive search range based on poc difference
void EncSlice::setSearchRange( Slice* pcSlice )
{
  int iCurrPOC = pcSlice->getPOC();
  int iRefPOC;
  int iGOPSize = m_pcCfg->getGOPSize();
  int iOffset = (iGOPSize >> 1);
  int iMaxSR = m_pcCfg->getSearchRange();
  int iNumPredDir = pcSlice->isInterP() ? 1 : 2;

  for (int iDir = 0; iDir < iNumPredDir; iDir++)
  {
    RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for (int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)
    {
      iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();
      int newSearchRange = Clip3(m_pcCfg->getMinSearchWindow(), iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
      m_pcInterSearch->setAdaptiveSearchRange(iDir, iRefIdx, newSearchRange);
#if ENABLE_WPP_PARALLELISM
      for( int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++ )
      {
        m_pcLib->getInterSearch( jId )->setAdaptiveSearchRange( iDir, iRefIdx, newSearchRange );
      }
#endif
    }
  }
}

/**
 Multi-loop slice encoding for different slice QP

 \param pcPic    picture class
 */
void EncSlice::precompressSlice( Picture* pcPic )
{
  // if deltaQP RD is not used, simply return
  if ( m_pcCfg->getDeltaQpRD() == 0 )
  {
    return;
  }

  if ( m_pcCfg->getUseRateCtrl() )
  {
    THROW("\nMultiple QP optimization is not allowed when rate control is enabled." );
  }

  Slice* pcSlice        = pcPic->slices[getSliceSegmentIdx()];

#if HEVC_DEPENDENT_SLICES
  if (pcSlice->getDependentSliceSegmentFlag())
  {
    // if this is a dependent slice segment, then it was optimised
    // when analysing the entire slice.
    return;
  }
#endif

  if (pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES)
  {
    // TODO: investigate use of average cost per CTU so that this Slice Mode can be used.
    THROW( "Unable to optimise Slice-level QP if Slice Mode is set to FIXED_NUMBER_OF_BYTES\n" );
  }

  double     dPicRdCostBest = MAX_DOUBLE;
  uint32_t       uiQpIdxBest = 0;

  double dFrameLambda;
  int SHIFT_QP = 12
                 + 6
                     * (pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
                        - DISTORTION_PRECISION_ADJUSTMENT(pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));

  // set frame lambda
  if (m_pcCfg->getGOPSize() > 1)
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
  }
  else
  {
    dFrameLambda = 0.68 * pow (2, (m_viRdPicQp[0] - SHIFT_QP) / 3.0);
  }

  // for each QP candidate
  for ( uint32_t uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )
  {
    pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdx] );
    setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdx], m_viRdPicQp    [uiQpIdx]);

    // try compress
    compressSlice   ( pcPic, true, m_pcCfg->getFastDeltaQp());

    uint64_t uiPicDist        = m_uiPicDist; // Distortion, as calculated by compressSlice.
    // NOTE: This distortion is the chroma-weighted SSE distortion for the slice.
    //       Previously a standard SSE distortion was calculated (for the entire frame).
    //       Which is correct?
#if W0038_DB_OPT
    // TODO: Update loop filter, SAO and distortion calculation to work on one slice only.
    // uiPicDist = m_pcGOPEncoder->preLoopFilterPicAndCalcDist( pcPic );
#endif
    // compute RD cost and choose the best
    double dPicRdCost = double( uiPicDist ) + dFrameLambda * double( m_uiPicTotalBits );

    if ( dPicRdCost < dPicRdCostBest )
    {
      uiQpIdxBest    = uiQpIdx;
      dPicRdCostBest = dPicRdCost;
    }
  }

  // set best values
  pcSlice       ->setSliceQp             ( m_viRdPicQp    [uiQpIdxBest] );
  setUpLambda(pcSlice, m_vdRdPicLambda[uiQpIdxBest], m_viRdPicQp    [uiQpIdxBest]);
}

void EncSlice::calCostSliceI(Picture* pcPic) // TODO: this only analyses the first slice segment. What about the others?
{
  double         iSumHadSlice      = 0;
  Slice * const  pcSlice           = pcPic->slices[getSliceSegmentIdx()];
#if HEVC_TILES_WPP
  const TileMap &tileMap           = *pcPic->tileMap;
#endif
  const PreCalcValues& pcv         = *pcPic->cs->pcv;
  const SPS     &sps               = *(pcSlice->getSPS());
  const int      shift             = sps.getBitDepth(CHANNEL_TYPE_LUMA)-8;
  const int      offset            = (shift>0)?(1<<(shift-1)):0;

#if HEVC_DEPENDENT_SLICES
  pcSlice->setSliceSegmentBits(0);
#endif

  uint32_t startCtuTsAddr, boundingCtuTsAddr;
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );

#if HEVC_TILES_WPP
  for( uint32_t ctuTsAddr = startCtuTsAddr, ctuRsAddr = tileMap.getCtuTsToRsAddrMap( startCtuTsAddr);
       ctuTsAddr < boundingCtuTsAddr;
       ctuRsAddr = tileMap.getCtuTsToRsAddrMap(++ctuTsAddr) )
#else
  for( uint32_t ctuTsAddr = startCtuTsAddr, ctuRsAddr = startCtuTsAddr;
       ctuTsAddr < boundingCtuTsAddr;
       ctuRsAddr = ++ctuTsAddr )
#endif
  {
    Position pos( (ctuRsAddr % pcv.widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUHeight);

    const int height  = std::min( pcv.maxCUHeight, pcv.lumaHeight - pos.y );
    const int width   = std::min( pcv.maxCUWidth,  pcv.lumaWidth  - pos.x );
    const CompArea blk( COMPONENT_Y, pcv.chrFormat, pos, Size( width, height));
    int iSumHad = m_pcCuEncoder->updateCtuDataISlice( pcPic->getOrigBuf( blk ) );

    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra=(iSumHad+offset)>>shift;
    iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra;

  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);
}

/** \param pcPic   picture class
 */
void EncSlice::compressSlice( Picture* pcPic, const bool bCompressEntireSlice, const bool bFastDeltaQP )
{
  // if bCompressEntireSlice is true, then the entire slice (not slice segment) is compressed,
  //   effectively disabling the slice-segment-mode.

  Slice* const pcSlice    = pcPic->slices[getSliceSegmentIdx()];
#if HEVC_TILES_WPP
  const TileMap&  tileMap = *pcPic->tileMap;
#endif
  uint32_t  startCtuTsAddr;
  uint32_t  boundingCtuTsAddr;

#if HEVC_DEPENDENT_SLICES
  pcSlice->setSliceSegmentBits(0);
#endif
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );
  if (bCompressEntireSlice)
  {
    boundingCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr();
#if HEVC_DEPENDENT_SLICES
    pcSlice->setSliceSegmentCurEndCtuTsAddr(boundingCtuTsAddr);
#endif
  }

  // initialize cost values - these are used by precompressSlice (they should be parameters).
  m_uiPicTotalBits  = 0;
  m_uiPicDist       = 0;

  pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

  m_CABACEstimator->initCtxModels( *pcSlice );

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  for( int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++ )
  {
    CABACWriter* cw = m_pcLib->getCABACEncoder( jId )->getCABACEstimator( pcSlice->getSPS() );
    cw->initCtxModels( *pcSlice );
  }

#endif
  m_pcCuEncoder->getModeCtrl()->setFastDeltaQp(bFastDeltaQP);


  //------------------------------------------------------------------------------
  //  Weighted Prediction parameters estimation.
  //------------------------------------------------------------------------------
  // calculate AC/DC values for current picture
  if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )
  {
    xCalcACDCParamSlice(pcSlice);
  }

  const bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());

  if ( bWp_explicit )
  {
    //------------------------------------------------------------------------------
    //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
    //------------------------------------------------------------------------------
#if HEVC_DEPENDENT_SLICES
    if ( pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES || pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES )
#else
    if(pcSlice->getSliceMode() == FIXED_NUMBER_OF_BYTES)
#endif
    {
      EXIT("Weighted Prediction is not yet supported with slice mode determined by max number of bins.");
    }

    xEstimateWPParamSlice( pcSlice, m_pcCfg->getWeightedPredictionMethod() );
    pcSlice->initWpScaling(pcSlice->getSPS());

    // check WP on/off
    xCheckWPEnable( pcSlice );
  }


#if HEVC_DEPENDENT_SLICES
#if HEVC_TILES_WPP
  // Adjust initial state if this is the start of a dependent slice.
  {
    const uint32_t      ctuRsAddr               = tileMap.getCtuTsToRsAddrMap( startCtuTsAddr);
    const uint32_t      currentTileIdx          = tileMap.getTileIdxMap(ctuRsAddr);
    const Tile&     currentTile             = tileMap.tiles[currentTileIdx];
    const uint32_t      firstCtuRsAddrOfTile    = currentTile.getFirstCtuRsAddr();
    if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile )
    {
      // This will only occur if dependent slice-segments (m_entropyCodingSyncContextState=true) are being used.
      if( currentTile.getTileWidthInCtus() >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag() )
      {
        m_CABACEstimator->getCtx() = m_lastSliceSegmentEndContextState;
        m_CABACEstimator->start();
      }
    }
  }
#else
  // KJS: not sure if this works (but both dep slices and tiles shall be removed in VTM, so this code should not be used)
  if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != startCtuTsAddr )
  {
    if( pcPic->cs->pcv->widthInCtus >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag() )
    {
      m_CABACEstimator->getCtx() = m_lastSliceSegmentEndContextState;
      m_CABACEstimator->start();
    }
#endif
#endif

#if HEVC_DEPENDENT_SLICES
  if( !pcSlice->getDependentSliceSegmentFlag() )
  {
#endif
    pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
  }
#endif

  VTMCHECK( pcPic->m_prevQP[0] == std::numeric_limits<int>::max(), "Invalid previous QP" );

  CodingStructure&  cs          = *pcPic->cs;
#if ENABLE_QPA || ENABLE_WPP_PARALLELISM
  const PreCalcValues& pcv      = *cs.pcv;
  const uint32_t    widthInCtus = pcv.widthInCtus;
#endif

  cs.slice = pcSlice;

#if JVET_M0055_DEBUG_CTU
  if( startCtuTsAddr == 0 && ( pcSlice->getPOC() != m_pcCfg->getSwitchPOC() || -1 == m_pcCfg->getDebugCTU() ) )
#else
  if( startCtuTsAddr == 0 )
#endif
  {
    cs.initStructData (pcSlice->getSliceQp(), pcSlice->getPPS()->getTransquantBypassEnabledFlag());
  }

#if ENABLE_QPA
  double hpEnerMax     = 1.0;
  double hpEnerPic     = 0.0;
  int    iSrcOffset;

  if (m_pcCfg->getUsePerceptQPA() && !m_pcCfg->getUseRateCtrl() && (boundingCtuTsAddr > startCtuTsAddr))
  {
    for (uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++)
    {
 #if HEVC_TILES_WPP
      const uint32_t ctuRsAddr  = tileMap.getCtuTsToRsAddrMap (ctuTsAddr);
 #else
      const uint32_t ctuRsAddr  = ctuTsAddr;
 #endif
      const Position pos ((ctuRsAddr % widthInCtus) * pcv.maxCUWidth, (ctuRsAddr / widthInCtus) * pcv.maxCUHeight);
      const CompArea subArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight)), pcPic->Y());
      const CompArea fltArea    = clipArea (CompArea (COMPONENT_Y, pcPic->chromaFormat, Area (pos.x > 0 ? pos.x - 1 : 0, pos.y > 0 ? pos.y - 1 : 0, pcv.maxCUWidth + (pos.x > 0 ? 2 : 1), pcv.maxCUHeight + (pos.y > 0 ? 2 : 1))), pcPic->Y());
      const SizeType iSrcStride = pcPic->getOrigBuf (subArea).stride;
      const Pel*     pSrc       = pcPic->getOrigBuf (subArea).buf;
      const SizeType iSrcHeight = pcPic->getOrigBuf (subArea).height;
      const SizeType iSrcWidth  = pcPic->getOrigBuf (subArea).width;
      const SizeType iFltHeight = pcPic->getOrigBuf (fltArea).height;
      const SizeType iFltWidth  = pcPic->getOrigBuf (fltArea).width;
      double hpEner = 0.0;

      DTRACE_UPDATE (g_trace_ctx, std::make_pair ("ctu", ctuRsAddr));

      // compute DC offset to be subtracted from luma values
      iSrcOffset = 0;
      for (SizeType h = 0; h < iSrcHeight; h++)
      {
        for (SizeType w = 0; w < iSrcWidth; w++)
        {
          iSrcOffset += pSrc[w];
        }
        pSrc += iSrcStride;
      }
      VTMCHECK (iSrcOffset < 0, "DC offset cannot be negative!");

      int x = iSrcHeight * iSrcWidth;
      iSrcOffset = (iSrcOffset + (x >> 1)) / x; // slow division

      filterAndCalculateAverageEnergies (pcPic->getOrigBuf (fltArea).buf, iSrcStride,
                                         hpEner, iFltHeight, iFltWidth,
                                         pcSlice->getSPS()->getBitDepth (CHANNEL_TYPE_LUMA));

      if (hpEner > hpEnerMax) hpEnerMax = hpEner;
      hpEnerPic += hpEner;
      pcPic->m_uEnerHpCtu[ctuRsAddr] = hpEner;
      pcPic->m_iOffsetCtu[ctuRsAddr] = (Pel)iSrcOffset;
    } // end iteration over all CTUs in current slice

    const double hpEnerAvg = hpEnerPic / double(boundingCtuTsAddr - startCtuTsAddr);

    if (applyQPAdaptation (pcPic, pcSlice, pcv, startCtuTsAddr, boundingCtuTsAddr, m_pcCfg->getLumaLevelToDeltaQPMapping().mode == LUMALVL_TO_DQP_NUM_MODES,
                           hpEnerAvg, hpEnerMax, (m_pcCfg->getBaseQP() >= 38) || (m_pcCfg->getSourceWidth() <= 512 && m_pcCfg->getSourceHeight() <= 320), m_adaptedLumaQP))
    {
      m_CABACEstimator->initCtxModels (*pcSlice);
  #if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
      for (int jId = 1; jId < m_pcLib->getNumCuEncStacks(); jId++)
      {
        CABACWriter* cw = m_pcLib->getCABACEncoder (jId)->getCABACEstimator (pcSlice->getSPS());
        cw->initCtxModels (*pcSlice);
      }
  #endif
#if HEVC_DEPENDENT_SLICES
      if (!pcSlice->getDependentSliceSegmentFlag())
      {
#endif
        pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
      }
#endif
      if (startCtuTsAddr == 0)
      {
        cs.currQP[0] = cs.currQP[1] = pcSlice->getSliceQp(); // cf code above
      }
    }
  }
#endif // ENABLE_QPA

  cs.pcv      = pcSlice->getPPS()->pcv;
  cs.fracBits = 0;


#if ENABLE_WPP_PARALLELISM
  bool bUseThreads = m_pcCfg->getNumWppThreads() > 1;
  if( bUseThreads )
  {
    VTMCHECK( startCtuTsAddr != 0 || boundingCtuTsAddr != pcPic->cs->pcv->sizeInCtus, "not intended" );

    pcPic->cs->allocateVectorsAtPicLevel();

    omp_set_num_threads( m_pcCfg->getNumWppThreads() + m_pcCfg->getNumWppExtraLines() );

    #pragma omp parallel for schedule(static,1) if(bUseThreads)
    for( int ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr += widthInCtus )
    {
      // wpp thread start
      pcPic->scheduler.setWppThreadId();
#if ENABLE_SPLIT_PARALLELISM
      pcPic->scheduler.setSplitThreadId( 0 );
#endif
      encodeCtus( pcPic, bCompressEntireSlice, bFastDeltaQP, ctuTsAddr, ctuTsAddr + widthInCtus, m_pcLib );
      // wpp thread stop
    }
  }
  else
#endif
#if K0149_BLOCK_STATISTICS
  const SPS *sps = pcSlice->getSPS();
  VTMCHECK(sps == 0, "No SPS present");
  writeBlockStatisticsHeader(sps);
#endif
  m_pcInterSearch->resetAffineMVList();
  encodeCtus( pcPic, bCompressEntireSlice, bFastDeltaQP, startCtuTsAddr, boundingCtuTsAddr, m_pcLib );

#if HEVC_DEPENDENT_SLICES
  // store context state at the end of this slice-segment, in case the next slice is a dependent slice and continues using the CABAC contexts.
  if( pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag() )
  {
    m_lastSliceSegmentEndContextState = m_CABACEstimator->getCtx();//ctx end of dep.slice
  }
#endif

}

#if JVET_M0255_FRACMMVD_SWITCH
void EncSlice::checkDisFracMmvd( Picture* pcPic, uint32_t startCtuTsAddr, uint32_t boundingCtuTsAddr )
{
  CodingStructure&  cs            = *pcPic->cs;
  Slice* pcSlice                  = cs.slice;
  const PreCalcValues& pcv        = *cs.pcv;
  const uint32_t    widthInCtus   = pcv.widthInCtus;
#if HEVC_TILES_WPP
  const TileMap&  tileMap         = *pcPic->tileMap;
#endif
  const uint32_t hashThreshold    = 20;
  uint32_t totalCtu               = 0;
  uint32_t hashRatio              = 0;

  if ( !pcSlice->getSPS()->getDisFracMmvdEnabledFlag() )
  {
    return;
  }

  for ( uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
#if HEVC_TILES_WPP
    const uint32_t ctuRsAddr = tileMap.getCtuTsToRsAddrMap( ctuTsAddr );
#else
    const uint32_t ctuRsAddr = ctuTsAddr;
#endif
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    const Position pos ( ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight );
    const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight ) );

    hashRatio += m_pcCuEncoder->getIbcHashMap().getHashHitRatio( ctuArea.Y() );
    totalCtu++;
  }

  if ( hashRatio > totalCtu * hashThreshold )
  {
    pcSlice->setDisFracMMVD( true );
  }
#if JVET_M0854_FRACMMVD_SWITCH_FOR_UHD
  if (!pcSlice->getDisFracMMVD()) {
    bool useIntegerMVD = (pcPic->lwidth()*pcPic->lheight() > 1920 * 1080);
    pcSlice->setDisFracMMVD( useIntegerMVD );
  }
#endif
}
#endif

void EncSlice::encodeCtus( Picture* pcPic, const bool bCompressEntireSlice, const bool bFastDeltaQP, uint32_t startCtuTsAddr, uint32_t boundingCtuTsAddr, EncLib* pEncLib )
{
  //PROF_ACCUM_AND_START_NEW_SET( getProfilerCTU( pcPic, 0, 0 ), P_PIC_LEVEL );
  //PROF_START( getProfilerCTU( cs.slice->isIntra(), pcPic->scheduler.getWppThreadId() ), P_PIC_LEVEL, toWSizeIdx( cs.pcv->maxCUWidth ), toHSizeIdx( cs.pcv->maxCUHeight ) );
  CodingStructure&  cs            = *pcPic->cs;
  Slice* pcSlice                  = cs.slice;
  const PreCalcValues& pcv        = *cs.pcv;
  const uint32_t        widthInCtus   = pcv.widthInCtus;
#if HEVC_TILES_WPP
  const TileMap&  tileMap         = *pcPic->tileMap;
#endif
#if ENABLE_QPA
  const int iQPIndex              = pcSlice->getSliceQpBase();
#endif

#if ENABLE_WPP_PARALLELISM
  const int       dataId          = pcPic->scheduler.getWppDataId();
#elif ENABLE_SPLIT_PARALLELISM
  const int       dataId          = 0;
#endif
  CABACWriter*    pCABACWriter    = pEncLib->getCABACEncoder( PARL_PARAM0( dataId ) )->getCABACEstimator( pcSlice->getSPS() );
  TrQuant*        pTrQuant        = pEncLib->getTrQuant( PARL_PARAM0( dataId ) );
  RdCost*         pRdCost         = pEncLib->getRdCost( PARL_PARAM0( dataId ) );
  EncCfg*         pCfg            = pEncLib;
  RateCtrl*       pRateCtrl       = pEncLib->getRateCtrl();
#if ENABLE_WPP_PARALLELISM
  // first version dont use ctx from above
  pCABACWriter->initCtxModels( *pcSlice );
#endif
#if RDOQ_CHROMA_LAMBDA
  pTrQuant    ->setLambdas( pcSlice->getLambdas() );
#else
  pTrQuant    ->setLambda ( pcSlice->getLambdas()[0] );
#endif
  pRdCost     ->setLambda ( pcSlice->getLambdas()[0], pcSlice->getSPS()->getBitDepths() );

  int prevQP[2];
  int currQP[2];
  prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
  currQP[0] = currQP[1] = pcSlice->getSliceQp();

#if HEVC_DEPENDENT_SLICES
  if( !pcSlice->getDependentSliceSegmentFlag() )
  {
#endif
    prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
  }
#endif
#if JVET_M0255_FRACMMVD_SWITCH
  if ( pcSlice->getSPS()->getDisFracMmvdEnabledFlag() ||
#if JVET_M0483_IBC
      (pcSlice->getSPS()->getIBCFlag() && m_pcCuEncoder->getEncCfg()->getIBCHashSearch()))
#else
      ( pcSlice->getSPS()->getIBCMode() && m_pcCuEncoder->getEncCfg()->getIBCHashSearch() ) )
#endif
  {
#if JVET_M0427_INLOOP_RESHAPER
#if JVET_M0483_IBC
    if (pcSlice->getSPS()->getUseReshaper() && m_pcLib->getReshaper()->getCTUFlag() && pcSlice->getSPS()->getIBCFlag())
#else
    if (pcSlice->getSPS()->getUseReshaper() && m_pcLib->getReshaper()->getCTUFlag() && pcSlice->getSPS()->getIBCMode())
#endif
      cs.picture->getOrigBuf(COMPONENT_Y).rspSignal(m_pcLib->getReshaper()->getFwdLUT());
#endif
    m_pcCuEncoder->getIbcHashMap().rebuildPicHashMap( cs.picture->getOrigBuf() );
#if JVET_M0427_INLOOP_RESHAPER
#if JVET_M0483_IBC
    if (pcSlice->getSPS()->getUseReshaper() && m_pcLib->getReshaper()->getCTUFlag() && pcSlice->getSPS()->getIBCFlag())
#else
    if (pcSlice->getSPS()->getUseReshaper() && m_pcLib->getReshaper()->getCTUFlag() && pcSlice->getSPS()->getIBCMode())
#endif
      cs.picture->getOrigBuf().copyFrom(cs.picture->getTrueOrigBuf());
#endif
  }
  checkDisFracMmvd( pcPic, startCtuTsAddr, boundingCtuTsAddr );
#endif
  // for every CTU in the slice segment (may terminate sooner if there is a byte limit on the slice-segment)
  for( uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
#if JVET_M0055_DEBUG_CTU
 #if HEVC_TILES_WPP
    const int32_t ctuRsAddr = tileMap.getCtuTsToRsAddrMap( ctuTsAddr );
 #else
    const int32_t ctuRsAddr = ctuTsAddr;
#endif
#else
#if HEVC_TILES_WPP
    const uint32_t ctuRsAddr = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
#else
    const uint32_t ctuRsAddr = ctuTsAddr;
#endif
#endif

#if HEVC_TILES_WPP
    // update CABAC state
    const uint32_t firstCtuRsAddrOfTile = tileMap.tiles[tileMap.getTileIdxMap(ctuRsAddr)].getFirstCtuRsAddr();
    const uint32_t tileXPosInCtus       = firstCtuRsAddrOfTile % widthInCtus;
#endif
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight ) );
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

#if JVET_M0055_DEBUG_CTU
    if( pCfg->getSwitchPOC() != pcPic->poc || -1 == pCfg->getDebugCTU() )
#endif
    if ( pcSlice->getSliceType() != I_SLICE && ctuXPosInCtus == 0)
    {
      pcSlice->resetMotionLUTs();
    }

#if ENABLE_WPP_PARALLELISM
    pcPic->scheduler.wait( ctuXPosInCtus, ctuYPosInCtus );
#endif

#if HEVC_TILES_WPP
    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      pCABACWriter->initCtxModels( *pcSlice );
      prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
    }
    else if (ctuXPosInCtus == tileXPosInCtus && pEncLib->getEntropyCodingSyncEnabledFlag())
    {
      // reset and then update contexts to the state at the end of the top-right CTU (if within current slice and tile).
      pCABACWriter->initCtxModels( *pcSlice );
      if( cs.getCURestricted( pos.offset(pcv.maxCUWidth, -1), pcSlice->getIndependentSliceIdx(), tileMap.getTileIdxMap( pos ), CH_L ) )
      {
        // Top-right is available, we use it.
        pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextState;
      }
      prevQP[0] = prevQP[1] = pcSlice->getSliceQp();
    }
#endif

#if ENABLE_WPP_PARALLELISM
    if( ctuXPosInCtus == 0 && ctuYPosInCtus > 0 && widthInCtus > 1 && ( pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual() ) )
    {
      pCABACWriter->getCtx() = pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus-1];  // last line
    }
#else
#endif

#if RDOQ_CHROMA_LAMBDA && ENABLE_QPA
    double oldLambdaArray[MAX_NUM_COMPONENT] = {0.0};
#endif
    const double oldLambda = pRdCost->getLambda();
    if ( pCfg->getUseRateCtrl() )
    {
      int estQP        = pcSlice->getSliceQp();
      double estLambda = -1.0;
      double bpp       = -1.0;

      if( ( pcPic->slices[0]->isIRAP() && pCfg->getForceIntraQP() ) || !pCfg->getLCULevelRC() )
      {
        estQP = pcSlice->getSliceQp();
      }
      else
      {
        bpp = pRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->isIRAP());
        if ( pcPic->slices[0]->isIRAP())
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);
        }
        else
        {
          estLambda = pRateCtrl->getRCPic()->getLCUEstLambda( bpp );
          estQP     = pRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );
        }

        estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );

        pRdCost->setLambda(estLambda, pcSlice->getSPS()->getBitDepths());

#if RDOQ_CHROMA_LAMBDA
        // set lambda for RDOQ
        const double chromaLambda = estLambda / pRdCost->getChromaWeight();
        const double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
        pTrQuant->setLambdas( lambdaArray );
#else
        pTrQuant->setLambda( estLambda );
#endif
      }

      pRateCtrl->setRCQP( estQP );
    }
#if ENABLE_QPA
    else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
      const int adaptedQP    = pcPic->m_iOffsetCtu[ctuRsAddr];
      const double newLambda = oldLambda * pow (2.0, double (adaptedQP - iQPIndex) / 3.0);
      pcPic->m_uEnerHpCtu[ctuRsAddr] = newLambda;
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->getLambdas (oldLambdaArray); // save the old lambdas
      const double chromaLambda = newLambda / pRdCost->getChromaWeight();
      const double lambdaArray[MAX_NUM_COMPONENT] = {newLambda, chromaLambda, chromaLambda};
      pTrQuant->setLambdas (lambdaArray);
#else
      pTrQuant->setLambda (newLambda);
#endif
      pRdCost->setLambda (newLambda, pcSlice->getSPS()->getBitDepths());
      currQP[0] = currQP[1] = adaptedQP;
    }
#endif

    bool updateGbiCodingOrder = cs.slice->getSliceType() == B_SLICE && ctuTsAddr == startCtuTsAddr;
    if( updateGbiCodingOrder )
    {
      resetGbiCodingOrder(false, cs);
      m_pcInterSearch->initWeightIdxBits();
    }
#if JVET_M0427_INLOOP_RESHAPER && REUSE_CU_RESULTS
    if (pcSlice->getSPS()->getUseReshaper())
    {
      m_pcCuEncoder->setDecCuReshaperInEncCU(m_pcLib->getReshaper(), pcSlice->getSPS()->getChromaFormatIdc());
    }
#endif

#if JVET_M0055_DEBUG_CTU
  if (pCfg->getSwitchPOC() != pcPic->poc || ctuRsAddr >= pCfg->getDebugCTU())
#endif
#if ENABLE_WPP_PARALLELISM
    pEncLib->getCuEncoder( dataId )->compressCtu( cs, ctuArea, ctuRsAddr, prevQP, currQP );
#else
    m_pcCuEncoder->compressCtu( cs, ctuArea, ctuRsAddr, prevQP, currQP );
#endif

#if K0149_BLOCK_STATISTICS
    getAndStoreBlockStatistics(cs, ctuArea);
#endif

    pCABACWriter->resetBits();
    pCABACWriter->coding_tree_unit( cs, ctuArea, prevQP, ctuRsAddr, true );
    const int numberOfWrittenBits = int( pCABACWriter->getEstFracBits() >> SCALE_BITS );

    // Calculate if this CTU puts us over slice bit size.
    // cannot terminate if current slice/slice-segment would be 0 Ctu in size,
    const uint32_t validEndOfSliceCtuTsAddr = ctuTsAddr + (ctuTsAddr == startCtuTsAddr ? 1 : 0);
    // Set slice end parameter
    if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceBits()+numberOfWrittenBits > (pcSlice->getSliceArgument()<<3))
    {
#if HEVC_DEPENDENT_SLICES
      pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
#endif
      pcSlice->setSliceCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
      boundingCtuTsAddr=validEndOfSliceCtuTsAddr;
    }
#if HEVC_DEPENDENT_SLICES
    else if((!bCompressEntireSlice) && pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+numberOfWrittenBits > (pcSlice->getSliceSegmentArgument()<<3))
    {
      pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
      boundingCtuTsAddr=validEndOfSliceCtuTsAddr;
    }
#endif
    if (boundingCtuTsAddr <= ctuTsAddr)
    {
      break;
    }

#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
    pcSlice->setSliceBits( ( uint32_t ) ( pcSlice->getSliceBits() + numberOfWrittenBits ) );
#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#pragma omp critical
#endif
#if HEVC_DEPENDENT_SLICES
    pcSlice->setSliceSegmentBits( pcSlice->getSliceSegmentBits() + numberOfWrittenBits );
#endif

#if HEVC_TILES_WPP
    // Store probabilities of second CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
    if( ctuXPosInCtus == tileXPosInCtus + 1 && pEncLib->getEntropyCodingSyncEnabledFlag() )
    {
      pEncLib->m_entropyCodingSyncContextState = pCABACWriter->getCtx();
    }
#endif
#if ENABLE_WPP_PARALLELISM
    if( ctuXPosInCtus == 1 && ( pEncLib->getNumWppThreads() > 1 || pEncLib->getEnsureWppBitEqual() ) )
    {
      pEncLib->m_entropyCodingSyncContextStateVec[ctuYPosInCtus] = pCABACWriter->getCtx();
    }
#endif

#if !ENABLE_WPP_PARALLELISM
    int actualBits = int(cs.fracBits >> SCALE_BITS);
    actualBits    -= (int)m_uiPicTotalBits;
#endif
    if ( pCfg->getUseRateCtrl() )
    {
#if ENABLE_WPP_PARALLELISM
      int actualBits      = int( cs.fracBits >> SCALE_BITS );
      actualBits         -= (int)m_uiPicTotalBits;
#endif
      int actualQP        = g_RCInvalidQPValue;
      double actualLambda = pRdCost->getLambda();
      int numberOfEffectivePixels    = 0;

      for( auto &cu : cs.traverseCUs( ctuArea, CH_L ) )
      {
        if( !cu.skip || cu.rootCbf )
        {
          numberOfEffectivePixels += cu.lumaSize().area();
          break;
        }
      }

      CodingUnit* cu = cs.getCU( ctuArea.lumaPos(), CH_L );

      if ( numberOfEffectivePixels == 0 )
      {
        actualQP = g_RCInvalidQPValue;
      }
      else
      {
        actualQP = cu->qp;
      }
      pRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());
      pRateCtrl->getRCPic()->updateAfterCTU( pRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
                                             pcSlice->isIRAP() ? 0 : pCfg->getLCULevelRC() );
    }
#if ENABLE_QPA
    else if (pCfg->getUsePerceptQPA() && pcSlice->getPPS()->getUseDQP())
    {
#if RDOQ_CHROMA_LAMBDA
      pTrQuant->setLambdas (oldLambdaArray);
#else
      pTrQuant->setLambda (oldLambda);
#endif
      pRdCost->setLambda (oldLambda, pcSlice->getSPS()->getBitDepths());
    }
#endif

#if !ENABLE_WPP_PARALLELISM
    m_uiPicTotalBits += actualBits;
    m_uiPicDist       = cs.dist;
#endif
#if ENABLE_WPP_PARALLELISM
    pcPic->scheduler.setReady( ctuXPosInCtus, ctuYPosInCtus );
#endif
  }

  // this is wpp exclusive section

//  m_uiPicTotalBits += actualBits;
//  m_uiPicDist       = cs.dist;

}

void EncSlice::encodeSlice   ( Picture* pcPic, OutputBitstream* pcSubstreams, uint32_t &numBinsCoded )
{

  Slice *const pcSlice               = pcPic->slices[getSliceSegmentIdx()];
#if HEVC_TILES_WPP
  const TileMap& tileMap             = *pcPic->tileMap;
#endif
#if HEVC_DEPENDENT_SLICES
  const uint32_t startCtuTsAddr          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
  const uint32_t boundingCtuTsAddr       = pcSlice->getSliceSegmentCurEndCtuTsAddr();
  const bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();
#else
  const uint32_t startCtuTsAddr          = pcSlice->getSliceCurStartCtuTsAddr();
  const uint32_t boundingCtuTsAddr       = pcSlice->getSliceCurEndCtuTsAddr();
#endif
#if HEVC_TILES_WPP
  const bool wavefrontsEnabled       = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();
#endif


  // setup coding structure
  CodingStructure& cs = *pcPic->cs;
  cs.slice            = pcSlice;
  // initialise entropy coder for the slice
  m_CABACWriter->initCtxModels( *pcSlice );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", pcSlice->getPOC() );

#if HEVC_DEPENDENT_SLICES
  if (depSliceSegmentsEnabled)
  {
#if HEVC_TILES_WPP
    // modify initial contexts with previous slice segment if this is a dependent slice.
    const uint32_t ctuRsAddr            = tileMap.getCtuTsToRsAddrMap( startCtuTsAddr );
    const uint32_t currentTileIdx       = tileMap.getTileIdxMap(ctuRsAddr);
    const Tile& currentTile         = tileMap.tiles[currentTileIdx];
    const uint32_t firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();

    if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile )
    {
      if( currentTile.getTileWidthInCtus() >= 2 || !wavefrontsEnabled )
      {
        m_CABACWriter->getCtx() = m_lastSliceSegmentEndContextState;
      }
    }
#else
  // KJS: not sure if this works (but both dep slices and tiles shall be removed in VTM, so this code should not be used)
  if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != startCtuTsAddr )
  {
    if( pcPic->cs->pcv->widthInCtus >= 2 || !m_pcCfg->getEntropyCodingSyncEnabledFlag() )
    {
        m_CABACWriter->getCtx() = m_lastSliceSegmentEndContextState;
    }
#endif
  }

  if( !pcSlice->getDependentSliceSegmentFlag() )
  {
#endif
    pcPic->m_prevQP[0] = pcPic->m_prevQP[1] = pcSlice->getSliceQp();
#if HEVC_DEPENDENT_SLICES
  }
#endif

  const PreCalcValues& pcv = *cs.pcv;
  const uint32_t widthInCtus   = pcv.widthInCtus;

  // for every CTU in the slice segment...

  for( uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
#if HEVC_TILES_WPP
    const uint32_t ctuRsAddr            = tileMap.getCtuTsToRsAddrMap(ctuTsAddr);
    const Tile& currentTile         = tileMap.tiles[tileMap.getTileIdxMap(ctuRsAddr)];
    const uint32_t firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();
    const uint32_t tileXPosInCtus       = firstCtuRsAddrOfTile % widthInCtus;
    const uint32_t tileYPosInCtus       = firstCtuRsAddrOfTile / widthInCtus;
#else
    const uint32_t ctuRsAddr            = ctuTsAddr;
#endif
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;
#if HEVC_TILES_WPP
    const uint32_t uiSubStrm            = tileMap.getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice);
#else
    const uint32_t uiSubStrm            = 0;
#endif

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    const Position pos (ctuXPosInCtus * pcv.maxCUWidth, ctuYPosInCtus * pcv.maxCUHeight);
    const UnitArea ctuArea (cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUWidth, pcv.maxCUHeight));
    m_CABACWriter->initBitstream( &pcSubstreams[uiSubStrm] );

#if HEVC_TILES_WPP
    // set up CABAC contexts' state for this CTU
    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter->initCtxModels( *pcSlice );
      }
    }
    else if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter->initCtxModels( *pcSlice );
      }
      if( cs.getCURestricted( pos.offset( pcv.maxCUWidth, -1 ), pcSlice->getIndependentSliceIdx(), tileMap.getTileIdxMap( pos ), CH_L ) )
      {
        // Top-right is available, so use it.
        m_CABACWriter->getCtx() = m_entropyCodingSyncContextState;
      }
    }
#endif

    bool updateGbiCodingOrder = cs.slice->getSliceType() == B_SLICE && ctuTsAddr == startCtuTsAddr;
    if( updateGbiCodingOrder )
    {
      resetGbiCodingOrder(false, cs);
    }

    m_CABACWriter->coding_tree_unit( cs, ctuArea, pcPic->m_prevQP, ctuRsAddr );

#if HEVC_TILES_WPP
    // store probabilities of second CTU in line into buffer
    if( ctuXPosInCtus == tileXPosInCtus + 1 && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = m_CABACWriter->getCtx();
    }
#endif

    // terminate the sub-stream, if required (end of slice-segment, end of tile, end of wavefront-CTU-row):
#if HEVC_TILES_WPP
    if( ctuTsAddr + 1 == boundingCtuTsAddr ||
         (  ctuXPosInCtus + 1 == tileXPosInCtus + currentTile.getTileWidthInCtus () &&
          ( ctuYPosInCtus + 1 == tileYPosInCtus + currentTile.getTileHeightInCtus() || wavefrontsEnabled )
         )
       )
#else
    if( ctuTsAddr + 1 == boundingCtuTsAddr )
#endif
    {
      m_CABACWriter->end_of_slice();

      // Byte-alignment in slice_data() when new tile
      pcSubstreams[uiSubStrm].writeByteAlignment();

      // write sub-stream size
      if( ctuTsAddr + 1 != boundingCtuTsAddr )
      {
        pcSlice->addSubstreamSize( (pcSubstreams[uiSubStrm].getNumberOfWrittenBits() >> 3) + pcSubstreams[uiSubStrm].countStartCodeEmulations() );
      }
    }
  } // CTU-loop

#if HEVC_DEPENDENT_SLICES
  if( depSliceSegmentsEnabled )
  {
    m_lastSliceSegmentEndContextState = m_CABACWriter->getCtx();//ctx end of dep.slice
  }
#endif

#if HEVC_DEPENDENT_SLICES
  if (pcSlice->getPPS()->getCabacInitPresentFlag() && !pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
#else
  if(pcSlice->getPPS()->getCabacInitPresentFlag())
#endif
  {
    m_encCABACTableIdx = m_CABACWriter->getCtxInitId( *pcSlice );
  }
  else
  {
    m_encCABACTableIdx = pcSlice->getSliceType();
  }
  numBinsCoded = m_CABACWriter->getNumBins();

}

#if HEVC_TILES_WPP
void EncSlice::calculateBoundingCtuTsAddrForSlice(uint32_t &startCtuTSAddrSlice, uint32_t &boundingCtuTSAddrSlice, bool &haveReachedTileBoundary,
                                                   Picture* pcPic, const int sliceMode, const int sliceArgument)
#else
void EncSlice::calculateBoundingCtuTsAddrForSlice(uint32_t &startCtuTSAddrSlice, uint32_t &boundingCtuTSAddrSlice,
                                                   Picture* pcPic, const int sliceMode, const int sliceArgument)
#endif
{
#if HEVC_TILES_WPP
  Slice* pcSlice = pcPic->slices[getSliceSegmentIdx()];
  const TileMap& tileMap = *( pcPic->tileMap );
  const PPS &pps         = *( pcSlice->getPPS() );
#endif
  const uint32_t numberOfCtusInFrame = pcPic->cs->pcv->sizeInCtus;
  boundingCtuTSAddrSlice=0;
#if HEVC_TILES_WPP
  haveReachedTileBoundary=false;
#endif

  switch (sliceMode)
  {
    case FIXED_NUMBER_OF_CTU:
      {
        uint32_t ctuAddrIncrement    = sliceArgument;
        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
    case FIXED_NUMBER_OF_BYTES:
      boundingCtuTSAddrSlice  = numberOfCtusInFrame; // This will be adjusted later if required.
      break;
#if HEVC_TILES_WPP
    case FIXED_NUMBER_OF_TILES:
      {
        const uint32_t tileIdx        = tileMap.getTileIdxMap( tileMap.getCtuTsToRsAddrMap(startCtuTSAddrSlice) );
        const uint32_t tileTotalCount = (pps.getNumTileColumnsMinus1()+1) * (pps.getNumTileRowsMinus1()+1);
        uint32_t ctuAddrIncrement   = 0;

        for(uint32_t tileIdxIncrement = 0; tileIdxIncrement < sliceArgument; tileIdxIncrement++)
        {
          if((tileIdx + tileIdxIncrement) < tileTotalCount)
          {
            uint32_t tileWidthInCtus    = tileMap.tiles[tileIdx + tileIdxIncrement].getTileWidthInCtus();
            uint32_t tileHeightInCtus   = tileMap.tiles[tileIdx + tileIdxIncrement].getTileHeightInCtus();
            ctuAddrIncrement       += (tileWidthInCtus * tileHeightInCtus);
          }
        }

        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;
      }
      break;
#endif
    default:
      boundingCtuTSAddrSlice    = numberOfCtusInFrame;
      break;
  }

#if HEVC_TILES_WPP
  // Adjust for tiles and wavefronts.
  const bool wavefrontsAreEnabled = pps.getEntropyCodingSyncEnabledFlag();

  if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) &&
      (pps.getNumTileRowsMinus1() > 0 || pps.getNumTileColumnsMinus1() > 0))
  {
    const uint32_t ctuRsAddr                   = tileMap.getCtuTsToRsAddrMap(startCtuTSAddrSlice);
    const uint32_t startTileIdx                = tileMap.getTileIdxMap(ctuRsAddr);
    const Tile& startingTile               = tileMap.tiles[startTileIdx];
    const uint32_t  tileStartTsAddr            = tileMap.getCtuRsToTsAddrMap(startingTile.getFirstCtuRsAddr());
    const uint32_t  tileStartWidth             = startingTile.getTileWidthInCtus();
    const uint32_t  tileStartHeight            = startingTile.getTileHeightInCtus();
    const uint32_t tileLastTsAddr_excl        = tileStartTsAddr + tileStartWidth*tileStartHeight;
    const uint32_t tileBoundingCtuTsAddrSlice = tileLastTsAddr_excl;
    const uint32_t ctuColumnOfStartingTile     = ((startCtuTSAddrSlice-tileStartTsAddr)%tileStartWidth);
    if (wavefrontsAreEnabled && ctuColumnOfStartingTile!=0)
    {
      // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
      const uint32_t numberOfCTUsToEndOfRow            = tileStartWidth - ctuColumnOfStartingTile;
      const uint32_t wavefrontTileBoundingCtuAddrSlice = startCtuTSAddrSlice + numberOfCTUsToEndOfRow;
      if (wavefrontTileBoundingCtuAddrSlice < boundingCtuTSAddrSlice)
      {
        boundingCtuTSAddrSlice = wavefrontTileBoundingCtuAddrSlice;
      }
    }

    if (tileBoundingCtuTsAddrSlice < boundingCtuTSAddrSlice)
    {
      boundingCtuTSAddrSlice = tileBoundingCtuTsAddrSlice;
      haveReachedTileBoundary = true;
    }
  }
  else if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) && wavefrontsAreEnabled && ((startCtuTSAddrSlice % pcPic->cs->pcv->widthInCtus) != 0))
  {
    // Adjust for wavefronts (no tiles).
    // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
    boundingCtuTSAddrSlice = std::min(boundingCtuTSAddrSlice, startCtuTSAddrSlice - (startCtuTSAddrSlice % pcPic->cs->pcv->widthInCtus) + (pcPic->cs->pcv->widthInCtus));
  }
#endif
}

/** Determines the starting and bounding CTU address of current slice / dependent slice
 * \param [out] startCtuTsAddr
 * \param [out] boundingCtuTsAddr
 * \param [in]  pcPic

 * Updates startCtuTsAddr, boundingCtuTsAddr with appropriate CTU address
 */
void EncSlice::xDetermineStartAndBoundingCtuTsAddr  ( uint32_t& startCtuTsAddr, uint32_t& boundingCtuTsAddr, Picture* pcPic )
{
  Slice* pcSlice                 = pcPic->slices[getSliceSegmentIdx()];

  // Non-dependent slice
  uint32_t startCtuTsAddrSlice           = pcSlice->getSliceCurStartCtuTsAddr();
#if HEVC_TILES_WPP
  bool haveReachedTileBoundarySlice  = false;
#endif
  uint32_t boundingCtuTsAddrSlice;
#if HEVC_TILES_WPP
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSlice, boundingCtuTsAddrSlice, haveReachedTileBoundarySlice, pcPic,
                                     m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument());
#else
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSlice, boundingCtuTsAddrSlice, pcPic,
                                     m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument());
#endif
  pcSlice->setSliceCurEndCtuTsAddr(   boundingCtuTsAddrSlice );
  pcSlice->setSliceCurStartCtuTsAddr( startCtuTsAddrSlice    );

#if HEVC_DEPENDENT_SLICES
  // Dependent slice
  uint32_t startCtuTsAddrSliceSegment          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
#if HEVC_TILES_WPP
  bool haveReachedTileBoundarySliceSegment = false;
#endif
  uint32_t boundingCtuTsAddrSliceSegment;
#if HEVC_TILES_WPP
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSliceSegment, boundingCtuTsAddrSliceSegment, haveReachedTileBoundarySliceSegment, pcPic,
                                     m_pcCfg->getSliceSegmentMode(), m_pcCfg->getSliceSegmentArgument());
#else
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSliceSegment, boundingCtuTsAddrSliceSegment, pcPic,
                                     m_pcCfg->getSliceSegmentMode(), m_pcCfg->getSliceSegmentArgument());
#endif
  if (boundingCtuTsAddrSliceSegment>boundingCtuTsAddrSlice)
  {
    boundingCtuTsAddrSliceSegment = boundingCtuTsAddrSlice;
  }
  pcSlice->setSliceSegmentCurEndCtuTsAddr( boundingCtuTsAddrSliceSegment );
  pcSlice->setSliceSegmentCurStartCtuTsAddr(startCtuTsAddrSliceSegment);

  // Make a joint decision based on reconstruction and dependent slice bounds
  startCtuTsAddr    = std::max(startCtuTsAddrSlice, startCtuTsAddrSliceSegment);
  boundingCtuTsAddr = boundingCtuTsAddrSliceSegment;
#else
  startCtuTsAddr = startCtuTsAddrSlice;
  boundingCtuTsAddr = boundingCtuTsAddrSlice;
#endif
}

double EncSlice::xGetQPValueAccordingToLambda ( double lambda )
{
  return 4.2005*log(lambda) + 13.7122;
}

//! \}
