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

/** \file     IntraSearch.h
    \brief    intra search class (header)
*/

#ifndef __INTRASEARCH__
#define __INTRASEARCH__

// Include files

#include "CABACWriter.h"
#include "EncCfg.h"

#include "CommonLib/IntraPrediction.h"
#include "CommonLib/CrossCompPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/RdCost.h"
#if JVET_M0427_INLOOP_RESHAPER
#include "EncReshape.h"
#endif

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class EncModeCtrl;

/// encoder search class
class IntraSearch : public IntraPrediction, CrossComponentPrediction
{
private:
#if JVET_M0102_INTRA_SUBPARTITIONS
  EncModeCtrl    *m_modeCtrl;
#endif
  Pel*            m_pSharedPredTransformSkip[MAX_NUM_TBLOCKS];

  XUCache         m_unitCache;

  CodingStructure ****m_pSplitCS;
  CodingStructure ****m_pFullCS;

  CodingStructure ***m_pTempCS;
  CodingStructure ***m_pBestCS;

  CodingStructure **m_pSaveCS;

  //cost variables for the EMT algorithm and new modes list
#if !JVET_M0464_UNI_MTS
  double m_bestModeCostStore[4];                                    // RD cost of the best mode for each PU using DCT2
  double m_modeCostStore    [4][NUM_LUMA_MODE];                         // RD cost of each mode for each PU using DCT2
  uint32_t   m_savedRdModeList  [4][NUM_LUMA_MODE], m_savedNumRdModes[4];
  int        m_savedExtendRefList[4][NUM_LUMA_MODE];
#endif
#if JVET_M0102_INTRA_SUBPARTITIONS
  static_vector<uint32_t, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrl;
  static_vector<uint32_t, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrlHor;
  static_vector<uint32_t, FAST_UDI_MAX_RDMODE_NUM> m_rdModeListWithoutMrlVer;

  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_intraModeDiagRatio;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> m_intraModeHorVerRatio;
  static_vector<int,    FAST_UDI_MAX_RDMODE_NUM> m_intraModeTestedNormalIntra;
#endif
#if JVET_M0427_INLOOP_RESHAPER
  PelStorage      m_tmpStorageLCU;
#endif
protected:
  // interface to option
  EncCfg*         m_pcEncCfg;

  // interface to classes
  TrQuant*        m_pcTrQuant;
  RdCost*         m_pcRdCost;
#if JVET_M0427_INLOOP_RESHAPER
  EncReshape*     m_pcReshape;
#endif

  // RD computation
  CABACWriter*    m_CABACEstimator;
  CtxCache*       m_CtxCache;

  bool            m_isInitialized;

public:

  IntraSearch();
  ~IntraSearch();

  void init                       ( EncCfg*        pcEncCfg,
                                    TrQuant*       pcTrQuant,
                                    RdCost*        pcRdCost,
                                    CABACWriter*   CABACEstimator,
                                    CtxCache*      ctxCache,
                                    const uint32_t     maxCUWidth,
                                    const uint32_t     maxCUHeight,
                                    const uint32_t     maxTotalCUDepth
#if JVET_M0427_INLOOP_RESHAPER
                                  , EncReshape*   m_pcReshape
#endif
                                  );

  void destroy                    ();

  CodingStructure****getSplitCSBuf() { return m_pSplitCS; }
  CodingStructure****getFullCSBuf () { return m_pFullCS; }
  CodingStructure  **getSaveCSBuf () { return m_pSaveCS; }

#if JVET_M0102_INTRA_SUBPARTITIONS
  void setModeCtrl                ( EncModeCtrl *modeCtrl ) { m_modeCtrl = modeCtrl; }
#endif

public:

#if JVET_M0102_INTRA_SUBPARTITIONS
  void estIntraPredLumaQT         ( CodingUnit &cu, Partitioner& pm, const double bestCostSoFar  = MAX_DOUBLE );
  void estIntraPredChromaQT       ( CodingUnit &cu, Partitioner& pm, const double maxCostAllowed = MAX_DOUBLE );
#else
  void estIntraPredLumaQT         ( CodingUnit &cu, Partitioner& pm );
  void estIntraPredChromaQT       (CodingUnit &cu, Partitioner& pm);
#endif
  void IPCMSearch                 (CodingStructure &cs, Partitioner& partitioner);
  uint64_t xFracModeBitsIntra     (PredictionUnit &pu, const uint32_t &uiMode, const ChannelType &compID);

protected:

  // -------------------------------------------------------------------------------------------------------------------
  // T & Q & Q-1 & T-1
  // -------------------------------------------------------------------------------------------------------------------

  void xEncPCM                    (CodingStructure &cs, Partitioner& partitioner, const ComponentID &compID);

  // -------------------------------------------------------------------------------------------------------------------
  // Intra search
  // -------------------------------------------------------------------------------------------------------------------

#if JVET_M0102_INTRA_SUBPARTITIONS
  void     xEncIntraHeader                         ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1 );
  void     xEncSubdivCbfQT                         ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );
  uint64_t xGetIntraFracBitsQT                     ( CodingStructure &cs, Partitioner& pm, const bool &luma, const bool &chroma, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );
  uint64_t xGetIntraFracBitsQTSingleChromaComponent( CodingStructure &cs, Partitioner& pm, const ComponentID compID );
#else
  void xEncIntraHeader            (CodingStructure &cs, Partitioner& pm, const bool &bLuma, const bool &bChroma);
  void xEncSubdivCbfQT            (CodingStructure &cs, Partitioner& pm, const bool &bLuma, const bool &bChroma);
  uint64_t xGetIntraFracBitsQT      (CodingStructure &cs, Partitioner& pm, const bool &bLuma, const bool &bChroma);
#endif

  uint64_t xGetIntraFracBitsQTChroma(TransformUnit& tu, const ComponentID &compID);
#if JVET_M0102_INTRA_SUBPARTITIONS
  void xEncCoeffQT                                 ( CodingStructure &cs, Partitioner& pm, const ComponentID compID, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );
#else
  void xEncCoeffQT                (CodingStructure &cs, Partitioner& pm, const ComponentID &compID);
#endif


#if JVET_M0464_UNI_MTS
  void xIntraCodingTUBlock        (TransformUnit &tu, const ComponentID &compID, const bool &checkCrossCPrediction, Distortion& ruiDist, const int &default0Save1Load2 = 0, uint32_t* numSig = nullptr, std::vector<TrMode>* trModes=nullptr, const bool loadTr=false );
#else
  void xIntraCodingTUBlock        (TransformUnit &tu, const ComponentID &compID, const bool &checkCrossCPrediction, Distortion& ruiDist, const int &default0Save1Load2 = 0, uint32_t* numSig = nullptr );
#endif

#if JVET_M0102_INTRA_SUBPARTITIONS
  ChromaCbfs xRecurIntraChromaCodingQT( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE,                          const PartSplit ispType = TU_NO_ISP );
  void       xRecurIntraCodingLumaQT  ( CodingStructure &cs, Partitioner& pm, const double bestCostSoFar = MAX_DOUBLE, const int subTuIdx = -1, const PartSplit ispType = TU_NO_ISP );
#else
  ChromaCbfs xRecurIntraChromaCodingQT  (CodingStructure &cs, Partitioner& pm);

  void xRecurIntraCodingLumaQT    ( CodingStructure &cs, Partitioner& pm );
#endif


  void encPredIntraDPCM( const ComponentID &compID, PelBuf &pOrg, PelBuf &pDst, const uint32_t &uiDirMode );
  static bool useDPCMForFirstPassIntraEstimation( const PredictionUnit &pu, const uint32_t &uiDirMode );
};// END CLASS DEFINITION EncSearch

//! \}

#endif // __ENCSEARCH__
