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

/** \file     UnitTool.h
 *  \brief    defines operations for basic units
 */

#ifndef __UNITTOOLS__
#define __UNITTOOLS__

#include "Unit.h"
#include "UnitPartitioner.h"
#include "ContextModelling.h"
#include "InterPrediction.h"

// CS tools
namespace CS
{
  uint64_t getEstBits                   ( const CodingStructure &cs );
  UnitArea getArea                    ( const CodingStructure &cs, const UnitArea &area, const ChannelType chType );
  bool   isDualITree                  ( const CodingStructure &cs );
#if JVET_M0147_DMVR
  void   setRefinedMotionField(CodingStructure &cs);
#endif
}


// CU tools
namespace CU
{
  bool isIntra                        (const CodingUnit &cu);
  bool isInter                        (const CodingUnit &cu);
#if JVET_M0483_IBC
  bool isIBC                          (const CodingUnit &cu);
#endif
  bool isRDPCMEnabled                 (const CodingUnit &cu);
  bool isLosslessCoded                (const CodingUnit &cu);
  uint32_t getIntraSizeIdx                (const CodingUnit &cu);

  bool isSameCtu                      (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSlice                    (const CodingUnit &cu, const CodingUnit &cu2);
#if HEVC_TILES_WPP
  bool isSameTile                     (const CodingUnit &cu, const CodingUnit &cu2);
  bool isSameSliceAndTile             (const CodingUnit &cu, const CodingUnit &cu2);
#endif
  bool isLastSubCUOfCtu               (const CodingUnit &cu);
  uint32_t getCtuAddr                     (const CodingUnit &cu);

  int  predictQP                      (const CodingUnit& cu, const int prevQP );
  bool isQGStart                      (const CodingUnit& cu, Partitioner& partitioner ); // check if start of a Quantization Group

  uint32_t getNumPUs                      (const CodingUnit& cu);
  void addPUs                         (      CodingUnit& cu);

  PartSplit getSplitAtDepth           (const CodingUnit& cu, const unsigned depth);

  bool hasNonTsCodedBlock             (const CodingUnit& cu);
  uint32_t getNumNonZeroCoeffNonTs        (const CodingUnit& cu);

  bool  isGBiIdxCoded                 (const CodingUnit& cu);
  uint8_t getValidGbiIdx              (const CodingUnit& cu);
  void  setGbiIdx                     (CodingUnit& cu, uint8_t uh);
  uint8_t deriveGbiIdx                (uint8_t gbiLO, uint8_t gbiL1);

#if JVET_M0102_INTRA_SUBPARTITIONS
  bool      divideTuInRows            ( const CodingUnit &cu );
  bool      firstTestISPHorSplit      ( const int width, const int height,            const ComponentID compID, const CodingUnit *cuLeft = nullptr, const CodingUnit *cuAbove = nullptr );
  PartSplit getISPType                ( const CodingUnit &cu,                         const ComponentID compID );
  bool      isISPLast                 ( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID );
  bool      isISPFirst                ( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID );
  ISPType   canUseISPSplit            ( const CodingUnit &cu,                         const ComponentID compID );
  ISPType   canUseISPSplit            ( const int width, const int height, const int maxTrSize = MAX_TU_SIZE );
  uint32_t  getISPSplitDim            ( const int width, const int height, const PartSplit ispType );
#endif

  PUTraverser traversePUs             (      CodingUnit& cu);
  TUTraverser traverseTUs             (      CodingUnit& cu);
  cPUTraverser traversePUs            (const CodingUnit& cu);
  cTUTraverser traverseTUs            (const CodingUnit& cu);

  bool  hasSubCUNonZeroMVd            (const CodingUnit& cu);
#if JVET_M0246_AFFINE_AMVR
  bool  hasSubCUNonZeroAffineMVd      ( const CodingUnit& cu );
#endif
  int   getMaxNeighboriMVCandNum      (const CodingStructure& cs, const Position& pos);
  void  resetMVDandMV2Int             (      CodingUnit& cu, InterPrediction *interPred );

#if JVET_M0140_SBT
  uint8_t getSbtInfo                  (uint8_t idx, uint8_t pos);
  uint8_t getSbtIdx                   (const uint8_t sbtInfo);
  uint8_t getSbtPos                   (const uint8_t sbtInfo);
  uint8_t getSbtMode                  (const uint8_t sbtIdx, const uint8_t sbtPos);
  uint8_t getSbtIdxFromSbtMode        (const uint8_t sbtMode);
  uint8_t getSbtPosFromSbtMode        (const uint8_t sbtMode);
  uint8_t targetSbtAllowed            (uint8_t idx, uint8_t sbtAllowed);
  uint8_t numSbtModeRdo               (uint8_t sbtAllowed);
  bool    isMtsMode                   (const uint8_t sbtInfo);
  bool    isSbtMode                   (const uint8_t sbtInfo);
  bool    isSameSbtSize               (const uint8_t sbtInfo1, const uint8_t sbtInfo2);
#endif
}
// PU tools
namespace PU
{
  int  getLMSymbolList(const PredictionUnit &pu, int *pModeList);
  int  getIntraMPMs(const PredictionUnit &pu, unsigned *mpm, const ChannelType &channelType = CHANNEL_TYPE_LUMA);
  void getIntraChromaCandModes        (const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE]);
  uint32_t getFinalIntraMode              (const PredictionUnit &pu, const ChannelType &chType);
  void getInterMergeCandidates        (const PredictionUnit &pu, MergeCtx& mrgCtx,
    int mmvdList,
    const int& mrgCandIdx = -1 );
#if JVET_M0483_IBC
  void getIBCMergeCandidates          (const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1);
#endif
  void getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx = -1);
  int getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC);
  bool isDiffMER                      (const PredictionUnit &pu, const PredictionUnit &pu2);
  bool getColocatedMVP                (const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &pos, Mv& rcMv, const int &refIdx);
  void fillMvpCand                    (      PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo );
#if JVET_M0483_IBC
  void fillIBCMvpCand                 (PredictionUnit &pu, AMVPInfo &amvpInfo);
  bool addIBCMVPCand                  (const PredictionUnit &pu, const Position &pos, const MvpDir &eDir, AMVPInfo &amvpInfo);
#endif
  void fillAffineMvpCand              (      PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo);
  bool addMVPCandUnscaled             (const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &amvpInfo);
  bool addMVPCandWithScaling          (const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &amvpInfo);
  void xInheritedAffineMv             ( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] );
  bool xCheckSimilarMotion(const int mergeCandIndex, const int prevCnt, const MergeCtx mergeCandList, bool hasPruned[MRG_MAX_NUM_CANDS]);
#if JVET_L0090_PAIR_AVG
  bool addMergeHMVPCand(const Slice &slice, MergeCtx& mrgCtx, bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos
#if JVET_M0483_IBC==0
    , int mmvdList
#endif
#if JVET_M0483_IBC
    , bool ibcFlag
#endif
#if JVET_M0170_MRG_SHARELIST
    , bool isShared
#endif
  );
#else
  bool addMergeHMVPCand(const Slice &slice, MergeCtx& mrgCtx, bool isCandInter[MRG_MAX_NUM_CANDS], bool canFastExit, const int& mrgCandIdx, const uint32_t maxNumMergeCandMin1, int &cnt, const int prevCnt, bool isAvailableSubPu, unsigned subPuMvpPos
#if JVET_M0483_IBC==0
    , int mmvdList
#endif
  );
#endif
  void addAMVPHMVPCand(const PredictionUnit &pu, const RefPicList eRefPicList, const RefPicList eRefPicList2nd, const int currRefPOC, AMVPInfo &info, uint8_t imv);
  bool addAffineMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAmvpInfo );
  bool isBipredRestriction            (const PredictionUnit &pu);
  void spanMotionInfo                 (      PredictionUnit &pu, const MergeCtx &mrgCtx = MergeCtx() );
  void applyImv                       (      PredictionUnit &pu, MergeCtx &mrgCtx, InterPrediction *interPred = NULL );
  void getAffineControlPointCand( const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int modelIdx, int verNum, AffineMergeCtx& affMrgCtx );
  void getAffineMergeCand( const PredictionUnit &pu, AffineMergeCtx& affMrgCtx, const int mrgCandIdx = -1 );
  void setAllAffineMvField            (      PredictionUnit &pu, MvField *mvField, RefPicList eRefList );
  void setAllAffineMv                 (      PredictionUnit &pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList
    , bool setHighPrec = false
  );
  bool getInterMergeSubPuMvpCand(const PredictionUnit &pu, MergeCtx &mrgCtx, bool& LICFlag, const int count
    , int mmvdList
#if !JVET_M0409_ATMVP_FIX
#if JVET_M0483_IBC==0
    , const int countIBC
#endif
#endif
  );
  bool getInterMergeSubPuRecurCand(const PredictionUnit &pu, MergeCtx &mrgCtx, const int count);
  bool isBiPredFromDifferentDir       (const PredictionUnit &pu);
#if JVET_M0147_DMVR
  bool isBiPredFromDifferentDirEqDistPoc(const PredictionUnit &pu);
#endif
  void restrictBiPredMergeCands       (const PredictionUnit &pu, MergeCtx& mrgCtx);
#if JVET_M0068_M0171_MMVD_CLEANUP
  void restrictBiPredMergeCandsOne    (PredictionUnit &pu);
#endif

  bool isLMCMode                      (                          unsigned mode);
  bool isLMCModeEnabled               (const PredictionUnit &pu, unsigned mode);
  bool isChromaIntraModeCrossCheckMode(const PredictionUnit &pu);
  int  getMHIntraMPMs                 (const PredictionUnit &pu, unsigned *mpm, const ChannelType &channelType = CHANNEL_TYPE_LUMA, const bool isChromaMDMS = false, const unsigned startIdx = 0);
  int  getNarrowShape                 (const int width, const int height);
  void getTriangleMergeCandidates     (const PredictionUnit &pu, MergeCtx &triangleMrgCtx);
  bool isUniqueTriangleCandidates     (const PredictionUnit &pu, MergeCtx &triangleMrgCtx);
#if !JVET_M0328_KEEP_ONE_WEIGHT_GROUP
  bool getTriangleWeights             (const PredictionUnit &pu, MergeCtx &triangleMrgCtx, const uint8_t candIdx0, const uint8_t candIdx1);
#endif
#if JVET_M0883_TRIANGLE_SIGNALING
  void spanTriangleMotionInfo         (      PredictionUnit &pu, MergeCtx &triangleMrgCtx, const bool splitDir, const uint8_t candIdx0, const uint8_t candIdx1);
#else
  void spanTriangleMotionInfo         (      PredictionUnit &pu, MergeCtx &triangleMrgCtx, const uint8_t mergeIdx, const bool splitDir, const uint8_t candIdx0, const uint8_t candIdx1);
#endif
  int32_t mappingRefPic               (const PredictionUnit &pu, int32_t refPicPoc, bool targetRefPicList);
  void getIbcMVPsEncOnly(PredictionUnit &pu, Mv* MvPred, int& nbPred);
  bool getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv);
  bool isBlockVectorValid(PredictionUnit& pu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xStartInCU, int yStartInCU, int xBv, int yBv, int ctuSize);
#if JVET_M0147_DMVR
  bool checkDMVRCondition(const PredictionUnit& pu);
#endif
}

// TU tools
namespace TU
{
  uint32_t getNumNonZeroCoeffsNonTS       (const TransformUnit &tu, const bool bLuma = true, const bool bChroma = true);
  bool isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID);
  bool getCbf                         (const TransformUnit &tu, const ComponentID &compID);
  bool getCbfAtDepth                  (const TransformUnit &tu, const ComponentID &compID, const unsigned &depth);
  void setCbfAtDepth                  (      TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf);
#if JVET_M0464_UNI_MTS
  bool isTSAllowed                    (const TransformUnit &tu, const ComponentID  compID);
  bool isMTSAllowed                   (const TransformUnit &tu, const ComponentID  compID);
#else
  bool hasTransformSkipFlag           (const CodingStructure& cs, const CompArea& area);
#endif
  uint32_t getGolombRiceStatisticsIndex   (const TransformUnit &tu, const ComponentID &compID);
#if HEVC_USE_MDCS
  uint32_t getCoefScanIdx                 (const TransformUnit &tu, const ComponentID &compID);
#endif
  bool hasCrossCompPredInfo           (const TransformUnit &tu, const ComponentID &compID);


#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
  bool needsSqrt2Scale                ( const TransformUnit &tu, const ComponentID &compID );
#else
  bool needsSqrt2Scale                ( const Size& size );
#endif
#if HM_QTBT_AS_IN_JEM_QUANT
#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
  bool needsBlockSizeTrafoScale       ( const TransformUnit &tu, const ComponentID &compID );
#else
  bool needsBlockSizeTrafoScale       ( const Size& size );
#endif
#else
  bool needsQP3Offset                 (const TransformUnit &tu, const ComponentID &compID);
#endif
#if JVET_M0102_INTRA_SUBPARTITIONS
  TransformUnit* getPrevTU          ( const TransformUnit &tu, const ComponentID compID );
  bool           getPrevTuCbfAtDepth( const TransformUnit &tu, const ComponentID compID, const int trDepth );
  void           getTransformTypeISP( const TransformUnit &tu, const ComponentID compID, int &typeH, int &typeV );
#endif
}

uint32_t getCtuAddr        (const Position& pos, const PreCalcValues &pcv);

template<typename T, size_t N>
uint32_t updateCandList(T uiMode, double uiCost, static_vector<T, N>& candModeList, static_vector<double, N>& candCostList
  , static_vector<int, N>& extendRefList, int extendRef
  , size_t uiFastCandNum = N, int* iserttPos = nullptr)
{
  VTMCHECK( std::min( uiFastCandNum, candModeList.size() ) != std::min( uiFastCandNum, candCostList.size() ), "Sizes do not match!" );
  VTMCHECK( uiFastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!" );

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min( uiFastCandNum, candCostList.size() );

  while( shift < uiFastCandNum && shift < currSize && uiCost < candCostList[currSize - 1 - shift] )
  {
    shift++;
  }

  if( candModeList.size() >= uiFastCandNum && shift != 0 )
  {
    for( i = 1; i < shift; i++ )
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
      if (extendRef != -1)
      {
        extendRefList[currSize - i] = extendRefList[currSize - 1 - i];
      }
    }
    candModeList[currSize - shift] = uiMode;
    candCostList[currSize - shift] = uiCost;
    if (extendRef != -1)
    {
      extendRefList[currSize - shift] = extendRef;
    }
    if (iserttPos != nullptr)
    {
      *iserttPos = int(currSize - shift);
    }
    return 1;
  }
  else if( currSize < uiFastCandNum )
  {
    candModeList.insert( candModeList.end() - shift, uiMode );
    candCostList.insert( candCostList.end() - shift, uiCost );
    if (extendRef != -1)
    {
      extendRefList.insert(extendRefList.end() - shift, extendRef);
    }
    if (iserttPos != nullptr)
    {
      *iserttPos = int(candModeList.size() - shift - 1);
    }
    return 1;
  }
  if (iserttPos != nullptr)
  {
    *iserttPos = -1;
  }
  return 0;
}

template<typename T, size_t N>
uint32_t updateDoubleCandList(T mode, double cost, static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, static_vector<T, N>& candModeList2, T mode2, size_t fastCandNum = N, int* iserttPos = nullptr)
{
  VTMCHECK(std::min(fastCandNum, candModeList.size()) != std::min(fastCandNum, candCostList.size()), "Sizes do not match!");
  VTMCHECK(fastCandNum > candModeList.capacity(), "The vector is to small to hold all the candidates!");

  size_t i;
  size_t shift = 0;
  size_t currSize = std::min(fastCandNum, candCostList.size());

  while (shift < fastCandNum && shift < currSize && cost < candCostList[currSize - 1 - shift])
  {
    shift++;
  }

  if (candModeList.size() >= fastCandNum && shift != 0)
  {
    for (i = 1; i < shift; i++)
    {
      candModeList[currSize - i] = candModeList[currSize - 1 - i];
      candModeList2[currSize - i] = candModeList2[currSize - 1 - i];
      candCostList[currSize - i] = candCostList[currSize - 1 - i];
    }
    candModeList[currSize - shift] = mode;
    candModeList2[currSize - shift] = mode2;
    candCostList[currSize - shift] = cost;
    if (iserttPos != nullptr)
    {
      *iserttPos = int(currSize - shift);
    }
    return 1;
  }
  else if (currSize < fastCandNum)
  {
    candModeList.insert(candModeList.end() - shift, mode);
    candModeList2.insert(candModeList2.end() - shift, mode2);
    candCostList.insert(candCostList.end() - shift, cost);
    if (iserttPos != nullptr)
    {
      *iserttPos = int(candModeList.size() - shift - 1);
    }
    return 1;
  }

  if (iserttPos != nullptr)
  {
    *iserttPos = -1;
  }
  return 0;
}


#endif
