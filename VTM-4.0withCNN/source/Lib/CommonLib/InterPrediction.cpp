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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "InterPrediction.h"

#include "Buffer.h"
#include "UnitTools.h"

#include <memory.h>
#include <algorithm>

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
, m_maxCompIDToPred ( MAX_NUM_COMPONENT )
, m_pcRdCost        ( nullptr )
, m_storedMv        ( nullptr )
, m_gradX0(nullptr)
, m_gradY0(nullptr)
, m_gradX1(nullptr)
, m_gradY1(nullptr)
, m_subPuMC(false)
{
  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
    {
      m_acYuvPred[refList][ch] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
      {
        m_filteredBlock[i][j][c] = nullptr;
      }

      m_filteredBlockTmp[i][c] = nullptr;
    }
  }
#if JVET_M0147_DMVR
  m_cYuvPredTempDMVRL1 = nullptr;
  m_cYuvPredTempDMVRL0 = nullptr;
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    m_cRefSamplesDMVRL0[ch] = nullptr;
    m_cRefSamplesDMVRL1[ch] = nullptr;
  }
#endif
}

InterPrediction::~InterPrediction()
{
  destroy();
}

void InterPrediction::destroy()
{
  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      xFree( m_acYuvPred[i][c] );
      m_acYuvPred[i][c] = nullptr;
    }
  }

  for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
      {
        xFree( m_filteredBlock[i][j][c] );
        m_filteredBlock[i][j][c] = nullptr;
      }

      xFree( m_filteredBlockTmp[i][c] );
      m_filteredBlockTmp[i][c] = nullptr;
    }
  }

  m_triangleBuf.destroy();

  if (m_storedMv != nullptr)
  {
    delete[]m_storedMv;
    m_storedMv = nullptr;
  }

  xFree(m_gradX0);   m_gradX0 = nullptr;
  xFree(m_gradY0);   m_gradY0 = nullptr;
  xFree(m_gradX1);   m_gradX1 = nullptr;
  xFree(m_gradY1);   m_gradY1 = nullptr;
#if JVET_M0147_DMVR
  xFree(m_cYuvPredTempDMVRL0);
  m_cYuvPredTempDMVRL0 = nullptr;
  xFree(m_cYuvPredTempDMVRL1);
  m_cYuvPredTempDMVRL1 = nullptr;
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    xFree(m_cRefSamplesDMVRL0[ch]);
    m_cRefSamplesDMVRL0[ch] = nullptr;
    xFree(m_cRefSamplesDMVRL1[ch]);
    m_cRefSamplesDMVRL1[ch] = nullptr;
  }
#endif
}

void InterPrediction::init( RdCost* pcRdCost, ChromaFormat chromaFormatIDC )
{
  m_pcRdCost = pcRdCost;


  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] != nullptr && m_currChromaFormat != chromaFormatIDC )
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;
  if( m_acYuvPred[REF_PIC_LIST_0][COMPONENT_Y] == nullptr ) // check if first is null (in which case, nothing initialised yet)
  {
    for( uint32_t c = 0; c < MAX_NUM_COMPONENT; c++ )
    {
      int extWidth = MAX_CU_SIZE + (2 * BIO_EXTEND_SIZE + 2) + 16;
      int extHeight = MAX_CU_SIZE + (2 * BIO_EXTEND_SIZE + 2) + 1;
#if JVET_M0147_DMVR
      extWidth = extWidth > (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 16) ? extWidth : MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 16;
      extHeight = extHeight > (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 1) ? extHeight : MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 1;
#endif
      for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
      {
        m_filteredBlockTmp[i][c] = ( Pel* ) xMalloc( Pel, ( extWidth + 4 ) * ( extHeight + 7 + 4 ) );

        for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
        {
          m_filteredBlock[i][j][c] = ( Pel* ) xMalloc( Pel, extWidth * extHeight );
        }
      }

      // new structure
      for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        m_acYuvPred[i][c] = ( Pel* ) xMalloc( Pel, MAX_CU_SIZE * MAX_CU_SIZE );
      }
    }

    m_triangleBuf.create(UnitArea(chromaFormatIDC, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));

    m_iRefListIdx = -1;

    m_gradX0 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradY0 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradX1 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
    m_gradY1 = (Pel*)xMalloc(Pel, BIO_TEMP_BUFFER_SIZE);
  }

#if JVET_M0147_DMVR
  m_cYuvPredTempDMVRL0 = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)));
  m_cYuvPredTempDMVRL1 = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)));
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    m_cRefSamplesDMVRL0[ch] = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA));
    m_cRefSamplesDMVRL1[ch] = (Pel*)xMalloc(Pel, (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA));
  }
#endif
#if !JVET_J0090_MEMORY_BANDWITH_MEASURE
  m_if.initInterpolationFilter( true );
#endif

  if (m_storedMv == nullptr)
  {
    const int MVBUFFER_SIZE = MAX_CU_SIZE / MIN_PU_SIZE;
    m_storedMv = new Mv[MVBUFFER_SIZE*MVBUFFER_SIZE];
  }
}

bool checkIdenticalMotion( const PredictionUnit &pu, bool checkAffine )
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
        if( !pu.cu->affine )
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
        else
        {
          VTMCHECK( !checkAffine, "In this case, checkAffine should be on." );
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]) && (pu.mvAffi[0][2] == pu.mvAffi[1][2])) )
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion( const PredictionUnit &pu )
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
        if( !pu.cu->affine )
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
        else
        {
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]) && (pu.mvAffi[0][2] == pu.mvAffi[1][2])) )
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

void InterPrediction::xSubPuMC( PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/ )
{

  // compute the location of the current PU
  Position puPos    = pu.lumaPos();
  Size puSize       = pu.lumaSize();

  int numPartLine, numPartCol, puHeight, puWidth;
  {
    numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
    numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
    puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
    puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
  }

  PredictionUnit subPu;

  subPu.cs        = pu.cs;
  subPu.cu        = pu.cu;
  subPu.mergeType = MRG_TYPE_DEFAULT_N;

  bool isAffine = pu.cu->affine;
  subPu.cu->affine = false;

  // join sub-pus containing the same motion
  bool verMC = puSize.height > puSize.width;
  int  fstStart = (!verMC ? puPos.y : puPos.x);
  int  secStart = (!verMC ? puPos.x : puPos.y);
  int  fstEnd = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int  secEnd = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int  fstStep = (!verMC ? puHeight : puWidth);
  int  secStep = (!verMC ? puWidth : puHeight);

  m_subPuMC = true;

  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = pu.getMotionInfo(Position{ x, y });

      int length = secStep;
      int later  = secDim + secStep;

      while (later < secEnd)
      {
        const MotionInfo &laterMi = !verMC ? pu.getMotionInfo(Position{ later, fstDim }) : pu.getMotionInfo(Position{ fstDim, later });
        if (laterMi == curMi)
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }
      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));
#if JVET_M0823_MMVD_ENCOPT
      subPu.mmvdEncOptMode = 0;
#endif
#if JVET_M0147_DMVR
      subPu.mvRefine = false;
#endif
      motionCompensation(subPu, subPredBuf, eRefPicList);
      secDim = later - secStep;
    }
  }
  m_subPuMC = false;

  pu.cu->affine = isAffine;
}

void InterPrediction::xChromaMC(PredictionUnit &pu, PelUnitBuf& pcYuvPred)
{
  // separated tree, chroma
  const CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
  PredictionUnit subPu;
  subPu.cs = pu.cs;
  subPu.cu = pu.cu;

  Picture * refPic = pu.cu->slice->getPic();
  for (int y = lumaArea.y; y < lumaArea.y + lumaArea.height; y += MIN_PU_SIZE)
  {
    for (int x = lumaArea.x; x < lumaArea.x + lumaArea.width; x += MIN_PU_SIZE)
    {
      const MotionInfo &curMi = pu.cs->picture->cs->getMotionInfo(Position{ x, y });

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, MIN_PU_SIZE, MIN_PU_SIZE)));
      PelUnitBuf subPredBuf = pcYuvPred.subBuf(UnitAreaRelative(pu, subPu));

      xPredInterBlk(COMPONENT_Cb, subPu, refPic, curMi.mv[0], subPredBuf, false, pu.cu->slice->clpRng(COMPONENT_Cb)
                    , false
                    , true);
      xPredInterBlk(COMPONENT_Cr, subPu, refPic, curMi.mv[0], subPredBuf, false, pu.cu->slice->clpRng(COMPONENT_Cr)
                    , false
                    , true);
    }
  }
}


void InterPrediction::xPredInterUni(const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi
                                   , const bool& bioApplied
                                   , const bool luma, const bool chroma
)
{
  const SPS &sps = *pu.cs->sps;

  int iRefIdx = pu.refIdx[eRefPicList];
  Mv mv[3];
  bool isIBC = false;
#if JVET_M0483_IBC
  if (CU::isIBC(*pu.cu))
#else
  if (pu.cs->slice->getRefPic(eRefPicList, iRefIdx)->getPOC() == pu.cs->slice->getPOC())
#endif
  {
    isIBC = true;
  }
  if( pu.cu->affine )
  {
    VTMCHECK( iRefIdx < 0, "iRefIdx incorrect." );

    mv[0] = pu.mvAffi[eRefPicList][0];
    mv[1] = pu.mvAffi[eRefPicList][1];
    mv[2] = pu.mvAffi[eRefPicList][2];
  }
  else
  {
    mv[0] = pu.mv[eRefPicList];
  }
  if ( !pu.cu->affine )
  clipMv(mv[0], pu.cu->lumaPos(),
         pu.cu->lumaSize(),
         sps);

  for( uint32_t comp = COMPONENT_Y; comp < pcYuvPred.bufs.size() && comp <= m_maxCompIDToPred; comp++ )
  {
    const ComponentID compID = ComponentID( comp );
    if (compID == COMPONENT_Y && !luma)
      continue;
    if (compID != COMPONENT_Y && !chroma)
      continue;
    if ( pu.cu->affine )
    {
      VTMCHECK( bioApplied, "BIO is not allowed with affine" );
      xPredAffineBlk( compID, pu, pu.cu->slice->getRefPic( eRefPicList, iRefIdx ), mv, pcYuvPred, bi, pu.cu->slice->clpRng( compID ) );
    }
    else
    {
#if JVET_M0483_IBC
      if (isIBC)
      {
        xPredInterBlk(compID, pu, pu.cu->slice->getPic(), mv[0], pcYuvPred, bi, pu.cu->slice->clpRng(compID)
          , bioApplied
          , isIBC
        );
      }
      else
      {
        xPredInterBlk(compID, pu, pu.cu->slice->getRefPic(eRefPicList, iRefIdx), mv[0], pcYuvPred, bi, pu.cu->slice->clpRng(compID)
          , bioApplied
          , isIBC
        );
      }
#else
      xPredInterBlk( compID, pu, pu.cu->slice->getRefPic( eRefPicList, iRefIdx ), mv[0], pcYuvPred, bi, pu.cu->slice->clpRng( compID )
                    , bioApplied
                    , isIBC
                    );
#endif
    }
  }
}

void InterPrediction::xPredInterBi(PredictionUnit& pu, PelUnitBuf &pcYuvPred)
{
  const PPS   &pps   = *pu.cs->pps;
  const Slice &slice = *pu.cs->slice;

  bool bioApplied = false;
  if (pu.cs->sps->getBDOFEnabledFlag())
  {
    if (pu.cu->affine || m_subPuMC)
    {
      bioApplied = false;
    }
    else
    {
      const bool biocheck0 = !(pps.getWPBiPred() && slice.getSliceType() == B_SLICE);
      const bool biocheck1 = !(pps.getUseWP() && slice.getSliceType() == P_SLICE);
      if (biocheck0
        && biocheck1
        && PU::isBiPredFromDifferentDir(pu)
        && !(pu.Y().height == 4 || (pu.Y().width == 4 && pu.Y().height == 8))
       )
      {
        bioApplied = true;
      }
    }

#if JVET_M0444_SMVD
    if (bioApplied && pu.cu->smvdMode)
    {
      bioApplied = false;
    }
#endif

    if (pu.cu->cs->sps->getUseGBi() && bioApplied && pu.cu->GBiIdx != GBI_DEFAULT)
    {
      bioApplied = false;
    }
  }
#if JVET_M0823_MMVD_ENCOPT
  if (pu.mmvdEncOptMode == 2 && pu.mmvdMergeFlag) {
    bioApplied = false;
  }
#endif
#if JVET_M0147_DMVR
  bool dmvrApplied = false;
  dmvrApplied = (pu.mvRefine) && PU::checkDMVRCondition(pu);
#endif
  for (uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if( pu.refIdx[refList] < 0)
    {
      continue;
    }

    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

#if JVET_M0483_IBC
    VTMCHECK(CU::isIBC(*pu.cu) && eRefPicList != REF_PIC_LIST_0, "Invalid interdir for ibc mode");
    VTMCHECK(CU::isIBC(*pu.cu) && pu.refIdx[refList] != MAX_NUM_REF, "Invalid reference index for ibc mode");
    VTMCHECK((CU::isInter(*pu.cu) && pu.refIdx[refList] >= slice.getNumRefIdx(eRefPicList)), "Invalid reference index");
#else
    VTMCHECK( pu.refIdx[refList] >= slice.getNumRefIdx( eRefPicList ), "Invalid reference index" );
#endif
    m_iRefListIdx = refList;

    PelUnitBuf pcMbBuf = ( pu.chromaFormat == CHROMA_400 ?
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y())) :
                           PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[refList][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[refList][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[refList][2], pcYuvPred.Cr())) );

    if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
    {
#if JVET_M0147_DMVR
      if (dmvrApplied)
        continue; // mc will happen in processDMVR
#endif
      xPredInterUni ( pu, eRefPicList, pcMbBuf, true
        , bioApplied
        , true, true
      );
    }
    else
    {
      if( ( (pps.getUseWP() && slice.getSliceType() == P_SLICE) || (pps.getWPBiPred() && slice.getSliceType() == B_SLICE) ) )
      {
        xPredInterUni ( pu, eRefPicList, pcMbBuf, true
          , bioApplied
          , true, true
        );
      }
      else
      {
        xPredInterUni( pu, eRefPicList, pcMbBuf, pu.cu->triangle
          , bioApplied
          , true, true
        );
      }
    }
  }
#if JVET_M0147_DMVR
  if (dmvrApplied)
  {
    xProcessDMVR(pu, pcYuvPred, slice.clpRngs(), bioApplied);
  }
#endif


  CPelUnitBuf srcPred0 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[0][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvPred.Cr())) );
  CPelUnitBuf srcPred1 = ( pu.chromaFormat == CHROMA_400 ?
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y())) :
                           CPelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvPred.Y()), PelBuf(m_acYuvPred[1][1], pcYuvPred.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvPred.Cr())) );
  if( pps.getWPBiPred() && slice.getSliceType() == B_SLICE )
  {
    xWeightedPredictionBi( pu, srcPred0, srcPred1, pcYuvPred, m_maxCompIDToPred );
  }
  else if( pps.getUseWP() && slice.getSliceType() == P_SLICE )
  {
    xWeightedPredictionUni( pu, srcPred0, REF_PIC_LIST_0, pcYuvPred, -1, m_maxCompIDToPred );
  }
  else
  {
#if JVET_M0147_DMVR
    if (dmvrApplied == false)
    {
#endif
    xWeightedAverage( pu, srcPred0, srcPred1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs(), bioApplied );
#if JVET_M0147_DMVR
    }
#endif
  }
}

#if JVET_M0147_DMVR
void InterPrediction::xPredInterBlk ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng
                                     , const bool& bioApplied
                                     , bool isIBC
                                     , SizeType dmvrWidth
                                     , SizeType dmvrHeight
                                     , bool bilinearMC
                                     , Pel *srcPadBuf
                                     , int32_t srcPadStride
                                    )
#else
void InterPrediction::xPredInterBlk ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng
                                     , const bool& bioApplied
                                     , bool isIBC
                                    )
#endif
{
  JVET_J0090_SET_REF_PICTURE( refPic, compID );
  const ChromaFormat  chFmt = pu.chromaFormat;
  const bool          rndRes = !bi;

  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleX(compID, chFmt);
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleY(compID, chFmt);

  int xFrac = _mv.hor & ((1 << shiftHor) - 1);
  int yFrac = _mv.ver & ((1 << shiftVer) - 1);
  if (isIBC)
  {
    xFrac = yFrac = 0;
    JVET_J0090_SET_CACHE_ENABLE( false );
  }

  PelBuf &dstBuf  = dstPic.bufs[compID];
  unsigned width  = dstBuf.width;
  unsigned height = dstBuf.height;

  CPelBuf refBuf;
  {
    Position offset = pu.blocks[compID].pos().offset( _mv.getHor() >> shiftHor, _mv.getVer() >> shiftVer );
#if JVET_M0147_DMVR
    if (dmvrWidth)
    {
      refBuf = refPic->getRecoBuf(CompArea(compID, chFmt, offset, Size(dmvrWidth, dmvrHeight)));
    }
    else
#endif
    refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, offset, pu.blocks[compID].size() ) );
  }

#if JVET_M0147_DMVR
  if (NULL != srcPadBuf)
  {
    refBuf.buf = srcPadBuf;
    refBuf.stride = srcPadStride;
  }
  if (dmvrWidth)
  {
    width = dmvrWidth;
    height = dmvrHeight;
  }
#endif
  // backup data
  int backupWidth = width;
  int backupHeight = height;
  Pel *backupDstBufPtr = dstBuf.buf;
  int backupDstBufStride = dstBuf.stride;

  if (bioApplied && compID == COMPONENT_Y)
  {
    width = width + 2 * BIO_EXTEND_SIZE + 2;
    height = height + 2 * BIO_EXTEND_SIZE + 2;

    // change MC output
    dstBuf.stride = width;
    dstBuf.buf = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + 2 * dstBuf.stride + 2;
  }

  if( yFrac == 0 )
  {
#if JVET_M0147_DMVR
    m_if.filterHor(compID, (Pel*)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, xFrac, rndRes, chFmt, clpRng, bilinearMC, bilinearMC);
#else
    m_if.filterHor(compID, (Pel*)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, xFrac, rndRes, chFmt, clpRng);
#endif
  }
  else if( xFrac == 0 )
  {
#if JVET_M0147_DMVR
    m_if.filterVer(compID, (Pel*)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, yFrac, true, rndRes, chFmt, clpRng, bilinearMC, bilinearMC);
#else
    m_if.filterVer(compID, (Pel*)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, yFrac, true, rndRes, chFmt, clpRng);
#endif
  }
  else
  {
#if JVET_M0147_DMVR
    PelBuf tmpBuf = dmvrWidth ? PelBuf(m_filteredBlockTmp[0][compID], Size(dmvrWidth, dmvrHeight)) : PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
    if (dmvrWidth == 0)
      tmpBuf.stride = dstBuf.stride;
#else
    PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
    tmpBuf.stride = dstBuf.stride;
#endif

    int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;
#if JVET_M0147_DMVR
    if (bilinearMC)
    {
      vFilterSize = NTAPS_BILINEAR;
    }
#endif
#if JVET_M0147_DMVR
    m_if.filterHor(compID, (Pel*)refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, backupWidth, backupHeight + vFilterSize - 1, xFrac, false, chFmt, clpRng, bilinearMC, bilinearMC);
#else
    m_if.filterHor(compID, (Pel*)refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, backupWidth, backupHeight + vFilterSize - 1, xFrac, false, chFmt, clpRng);
#endif
    JVET_J0090_SET_CACHE_ENABLE( false );
#if JVET_M0147_DMVR
    m_if.filterVer(compID, (Pel*)tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, yFrac, false, rndRes, chFmt, clpRng, bilinearMC, bilinearMC);
#else
    m_if.filterVer(compID, (Pel*)tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, yFrac, false, rndRes, chFmt, clpRng);
#endif
  }
  JVET_J0090_SET_CACHE_ENABLE( true );
  if (bioApplied && compID == COMPONENT_Y)
  {
#if JVET_M0487_INT_EXTEND
    const int shift = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
    const Pel* refPel = refBuf.buf - refBuf.stride - 1;
    Pel* dstPel = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + dstBuf.stride + 1;
    for (int w = 0; w < (width - 2 * BIO_EXTEND_SIZE); w++)
    {
      Pel val = leftShift_round(refPel[w], shift);
      dstPel[w] = val - (Pel)IF_INTERNAL_OFFS;
    }

    refPel = refBuf.buf - 1;
    dstPel = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + 2 * dstBuf.stride + 1;
    for (int h = 0; h < (height - 2 * BIO_EXTEND_SIZE - 2); h++)
    {
      Pel val = leftShift_round(refPel[0], shift);
      dstPel[0] = val - (Pel)IF_INTERNAL_OFFS;

      val = leftShift_round(refPel[width - 3], shift);
      dstPel[width - 3] = val - (Pel)IF_INTERNAL_OFFS;

      refPel += refBuf.stride;
      dstPel += dstBuf.stride;
    }

    refPel = refBuf.buf + (height - 2 * BIO_EXTEND_SIZE - 2)*refBuf.stride - 1;
    dstPel = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + (height - 2 * BIO_EXTEND_SIZE)*dstBuf.stride + 1;
    for (int w = 0; w < (width - 2 * BIO_EXTEND_SIZE); w++)
    {
      Pel val = leftShift_round(refPel[w], shift);
      dstPel[w] = val - (Pel)IF_INTERNAL_OFFS;
    }
#else
    refBuf.buf = refBuf.buf - refBuf.stride - 1;
#if JVET_M0147_DMVR
    if (srcPadBuf)
    {
      refBuf.buf = srcPadBuf - srcPadStride - 1;
      refBuf.stride = srcPadStride;
    }
#endif
    dstBuf.buf = m_filteredBlockTmp[2 + m_iRefListIdx][compID] + dstBuf.stride + 1;
    bioSampleExtendBilinearFilter(refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, width - 2, height - 2, 1, xFrac, yFrac, rndRes, chFmt, clpRng);
#endif

    // restore data
    width = backupWidth;
    height = backupHeight;
    dstBuf.buf = backupDstBufPtr;
    dstBuf.stride = backupDstBufStride;
  }
}

void InterPrediction::xPredAffineBlk( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool& bi, const ClpRng& clpRng )
{
  if ( (pu.cu->affineType == AFFINEMODEL_6PARAM && _mv[0] == _mv[1] && _mv[0] == _mv[2])
    || (pu.cu->affineType == AFFINEMODEL_4PARAM && _mv[0] == _mv[1])
    )
  {
    Mv mvTemp = _mv[0];
    clipMv( mvTemp, pu.cu->lumaPos(),
            pu.cu->lumaSize(),
            *pu.cs->sps );
    xPredInterBlk( compID, pu, refPic, mvTemp, dstPic, bi, clpRng
                  , false
                  , false
                  );
    return;
  }

  JVET_J0090_SET_REF_PICTURE( refPic, compID );
  const ChromaFormat chFmt = pu.chromaFormat;
  int iScaleX = ::getComponentScaleX( compID, chFmt );
  int iScaleY = ::getComponentScaleY( compID, chFmt );

  Mv mvLT =_mv[0];
  Mv mvRT =_mv[1];
  Mv mvLB =_mv[2];


  // get affine sub-block width and height
  const int width  = pu.Y().width;
  const int height = pu.Y().height;
  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;

  blockWidth  >>= iScaleX;
  blockHeight >>= iScaleY;

  blockWidth =  std::max(blockWidth, AFFINE_MIN_BLOCK_SIZE);
  blockHeight = std::max(blockHeight, AFFINE_MIN_BLOCK_SIZE);

  VTMCHECK(blockWidth  > (width >> iScaleX ), "Sub Block width  > Block width");
  VTMCHECK(blockHeight > (height >> iScaleX), "Sub Block height > Block height");
  const int MVBUFFER_SIZE = MAX_CU_SIZE / MIN_PU_SIZE;

  const int cxWidth  = width  >> iScaleX;
  const int cxHeight = height >> iScaleY;
  const int iHalfBW  = blockWidth  >> 1;
  const int iHalfBH  = blockHeight >> 1;

  const int iBit = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
  iDMvHorX = (mvRT - mvLT).getHor() << (iBit - g_aucLog2[cxWidth]);
  iDMvHorY = (mvRT - mvLT).getVer() << (iBit - g_aucLog2[cxWidth]);
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (iBit - g_aucLog2[cxHeight]);
    iDMvVerY = (mvLB - mvLT).getVer() << (iBit - g_aucLog2[cxHeight]);
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << iBit;
  int iMvScaleVer = mvLT.getVer() << iBit;
  const SPS &sps    = *pu.cs->sps;
  const int iMvShift = 4;
  const int iOffset  = 8;
  const int iHorMax = ( sps.getPicWidthInLumaSamples()     + iOffset -      pu.Y().x - 1 ) << iMvShift;
  const int iHorMin = (      -(int)pu.cs->pcv->maxCUWidth  - iOffset - (int)pu.Y().x + 1 ) << iMvShift;
  const int iVerMax = ( sps.getPicHeightInLumaSamples()    + iOffset -      pu.Y().y - 1 ) << iMvShift;
  const int iVerMin = (      -(int)pu.cs->pcv->maxCUHeight - iOffset - (int)pu.Y().y + 1 ) << iMvShift;

  PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][compID], pu.blocks[compID]);
  const int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;

  const int shift = iBit - 4 + MV_FRACTIONAL_BITS_INTERNAL;

  // get prediction block by block
  for ( int h = 0; h < cxHeight; h += blockHeight )
  {
    for ( int w = 0; w < cxWidth; w += blockWidth )
    {

      int iMvScaleTmpHor, iMvScaleTmpVer;
      if(compID == COMPONENT_Y)
      {
        iMvScaleTmpHor = iMvScaleHor + iDMvHorX * (iHalfBW + w) + iDMvVerX * (iHalfBH + h);
        iMvScaleTmpVer = iMvScaleVer + iDMvHorY * (iHalfBW + w) + iDMvVerY * (iHalfBH + h);
        roundAffineMv(iMvScaleTmpHor, iMvScaleTmpVer, shift);
#if JVET_M0145_AFFINE_MV_CLIP
        Mv tmpMv(iMvScaleTmpHor, iMvScaleTmpVer);
        tmpMv.clipToStorageBitDepth();
        iMvScaleTmpHor = tmpMv.getHor();
        iMvScaleTmpVer = tmpMv.getVer();
#endif

        // clip and scale
        if (sps.getWrapAroundEnabledFlag())
        {
          m_storedMv[h / AFFINE_MIN_BLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_MIN_BLOCK_SIZE].set(iMvScaleTmpHor, iMvScaleTmpVer);
          Mv tmpMv(iMvScaleTmpHor, iMvScaleTmpVer);
          clipMv(tmpMv, Position(pu.Y().x + w, pu.Y().y + h), Size(blockWidth, blockHeight), sps);
          iMvScaleTmpHor = tmpMv.getHor();
          iMvScaleTmpVer = tmpMv.getVer();
        }
        else
        {
#if JVET_M0265_MV_ROUNDING_CLEANUP
          m_storedMv[h / AFFINE_MIN_BLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_MIN_BLOCK_SIZE].set(iMvScaleTmpHor, iMvScaleTmpVer);
#endif
          iMvScaleTmpHor = std::min<int>(iHorMax, std::max<int>(iHorMin, iMvScaleTmpHor));
          iMvScaleTmpVer = std::min<int>(iVerMax, std::max<int>(iVerMin, iMvScaleTmpVer));
#if !JVET_M0265_MV_ROUNDING_CLEANUP
          m_storedMv[h / AFFINE_MIN_BLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_MIN_BLOCK_SIZE].set(iMvScaleTmpHor, iMvScaleTmpVer);
#endif
        }
      }
      else
      {
#if JVET_M0265_MV_ROUNDING_CLEANUP
#if JVET_M0192_AFF_CHROMA_SIMPL
        Mv curMv = m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE) * MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE + 1)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE + 1)];
        roundAffineMv(curMv.hor, curMv.ver, 1);
#else
        Mv curMv = m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE) * MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE + 1)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE + 1)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE + 1)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE + 1)];
        roundAffineMv(curMv.hor, curMv.ver, 2);
#endif
#else
#if JVET_M0192_AFF_CHROMA_SIMPL
        Mv curMv = m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE) * MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE + 1)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE + 1)];
        roundAffineMv(curMv.hor, curMv.ver, 1);
#else
        Mv curMv = (m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE) * MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE + 1)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE + 1)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE + 1)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE + 1)] +
          Mv(2, 2));
        curMv.set(curMv.getHor() >> 2, curMv.getVer() >> 2);
#endif
#endif
        if (sps.getWrapAroundEnabledFlag())
        {
          clipMv(curMv, Position(pu.Y().x + (w << iScaleX), pu.Y().y + (h << iScaleY)), Size(blockWidth << iScaleX, blockHeight << iScaleY), sps);
        }
#if JVET_M0265_MV_ROUNDING_CLEANUP
        else
        {
          curMv.hor = std::min<int>(iHorMax, std::max<int>(iHorMin, curMv.hor));
          curMv.ver = std::min<int>(iVerMax, std::max<int>(iVerMin, curMv.ver));
        }
#endif
        iMvScaleTmpHor = curMv.hor;
        iMvScaleTmpVer = curMv.ver;
      }
      // get the MV in high precision
      int xFrac, yFrac, xInt, yInt;

      if (!iScaleX)
      {
        xInt  = iMvScaleTmpHor >> 4;
        xFrac = iMvScaleTmpHor & 15;
      }
      else
      {
        xInt  = iMvScaleTmpHor >> 5;
        xFrac = iMvScaleTmpHor & 31;
      }
      if (!iScaleY)
      {
        yInt  = iMvScaleTmpVer >> 4;
        yFrac = iMvScaleTmpVer & 15;
      }
      else
      {
        yInt  = iMvScaleTmpVer >> 5;
        yFrac = iMvScaleTmpVer & 31;
      }

      const CPelBuf refBuf = refPic->getRecoBuf( CompArea( compID, chFmt, pu.blocks[compID].offset(xInt + w, yInt + h), pu.blocks[compID] ) );
      PelBuf &dstBuf = dstPic.bufs[compID];

      if ( yFrac == 0 )
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, xFrac, !bi, chFmt, clpRng );
      }
      else if ( xFrac == 0 )
      {
        m_if.filterVer( compID, (Pel*) refBuf.buf, refBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, true, !bi, chFmt, clpRng );
      }
      else
      {
        m_if.filterHor( compID, (Pel*) refBuf.buf - ((vFilterSize>>1) -1)*refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, blockWidth, blockHeight+vFilterSize-1, xFrac, false,      chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( false );
        m_if.filterVer( compID, tmpBuf.buf + ((vFilterSize>>1) -1)*tmpBuf.stride, tmpBuf.stride, dstBuf.buf + w + h * dstBuf.stride, dstBuf.stride, blockWidth, blockHeight, yFrac, false, !bi, chFmt, clpRng);
        JVET_J0090_SET_CACHE_ENABLE( true );
      }
    }
  }
}

int getMSB( unsigned x )
{
  int msb = 0, bits = ( sizeof(int) << 3 ), y = 1;
  while( x > 1u )
  {
    bits >>= 1;
    y      = x >> bits;
    if( y )
    {
      x    = y;
      msb += bits;
    }
  }
  msb += y;
  return msb;
}

void InterPrediction::applyBiOptFlow(const PredictionUnit &pu, const CPelUnitBuf &yuvSrc0, const CPelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths)
{
  const int     height = yuvDst.Y().height;
  const int     width = yuvDst.Y().width;
  int           heightG = height + 2 * BIO_EXTEND_SIZE;
  int           widthG = width + 2 * BIO_EXTEND_SIZE;
  int           offsetPos = widthG*BIO_EXTEND_SIZE + BIO_EXTEND_SIZE;

  Pel*          gradX0 = m_gradX0;
  Pel*          gradX1 = m_gradX1;
  Pel*          gradY0 = m_gradY0;
  Pel*          gradY1 = m_gradY1;

  int           stridePredMC = widthG + 2;
  const Pel*    srcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + stridePredMC + 1;
  const Pel*    srcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + stridePredMC + 1;
  const int     src0Stride = stridePredMC;
  const int     src1Stride = stridePredMC;

  Pel*          dstY = yuvDst.Y().buf;
  const int     dstStride = yuvDst.Y().stride;
  const Pel*    srcY0Temp = srcY0;
  const Pel*    srcY1Temp = srcY1;

  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    Pel* dstTempPtr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + stridePredMC + 1;
    Pel* gradY = (refList == 0) ? m_gradY0 : m_gradY1;
    Pel* gradX = (refList == 0) ? m_gradX0 : m_gradX1;

#if JVET_M0063_BDOF_FIX
    xBioGradFilter(dstTempPtr, stridePredMC, widthG, heightG, widthG, gradX, gradY, clipBitDepths.recon[toChannelType(COMPONENT_Y)]);
#else
    xBioGradFilter(dstTempPtr, stridePredMC, widthG, heightG, widthG, gradX, gradY);
#endif
    Pel* padStr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + 2 * stridePredMC + 2;
    for (int y = 0; y< height; y++)
    {
      padStr[-1] = padStr[0];
      padStr[width] = padStr[width - 1];
      padStr += stridePredMC;
    }

    padStr = m_filteredBlockTmp[2 + refList][COMPONENT_Y] + 2 * stridePredMC + 1;
    ::memcpy(padStr - stridePredMC, padStr, sizeof(Pel)*(widthG));
    ::memcpy(padStr + height*stridePredMC, padStr + (height - 1)*stridePredMC, sizeof(Pel)*(widthG));
  }

  const ClpRng& clpRng = pu.cu->cs->slice->clpRng(COMPONENT_Y);
  const int   bitDepth = clipBitDepths.recon[toChannelType(COMPONENT_Y)];
  const int   shiftNum = IF_INTERNAL_PREC + 1 - bitDepth;
  const int   offset = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;
#if JVET_M0063_BDOF_FIX
  const int   limit = (bitDepth>12)? 2 : ((int)1 << (4 + IF_INTERNAL_PREC - bitDepth - 5));
#else
  const int   limit = ((int)1 << (4 + IF_INTERNAL_PREC - bitDepth - 5));
#endif

  int*     dotProductTemp1 = m_dotProduct1;
  int*     dotProductTemp2 = m_dotProduct2;
  int*     dotProductTemp3 = m_dotProduct3;
  int*     dotProductTemp5 = m_dotProduct5;
  int*     dotProductTemp6 = m_dotProduct6;

#if JVET_M0063_BDOF_FIX
  xCalcBIOPar(srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, src0Stride, src1Stride, widthG, widthG, heightG, bitDepth);
#else
  xCalcBIOPar(srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, src0Stride, src1Stride, widthG, widthG, heightG);
#endif

  int xUnit = (width >> 2);
  int yUnit = (height >> 2);

  Pel *dstY0 = dstY;
  gradX0 = m_gradX0; gradX1 = m_gradX1;
  gradY0 = m_gradY0; gradY1 = m_gradY1;

  for (int yu = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++)
    {
      if (m_bioPredSubBlkDist[yu*xUnit + xu] < m_bioSubBlkDistThres)
      {
        srcY0Temp = srcY0 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
        srcY1Temp = srcY1 + (stridePredMC + 1) + ((yu*src1Stride + xu) << 2);
        dstY0 = dstY + ((yu*dstStride + xu) << 2);
        PelBuf dstPelBuf(dstY0, dstStride, Size(4, 4));
        dstPelBuf.addAvg(CPelBuf(srcY0Temp, src0Stride, Size(4, 4)), CPelBuf(srcY1Temp, src1Stride, Size(4, 4)), clpRng);
        continue;
      }

      int     sGxdI = 0, sGydI = 0, sGxGy = 0, sGx2 = 0, sGy2 = 0;
      int     tmpx = 0, tmpy = 0;

      dotProductTemp1 = m_dotProduct1 + offsetPos + ((yu*widthG + xu) << 2);
      dotProductTemp2 = m_dotProduct2 + offsetPos + ((yu*widthG + xu) << 2);
      dotProductTemp3 = m_dotProduct3 + offsetPos + ((yu*widthG + xu) << 2);
      dotProductTemp5 = m_dotProduct5 + offsetPos + ((yu*widthG + xu) << 2);
      dotProductTemp6 = m_dotProduct6 + offsetPos + ((yu*widthG + xu) << 2);

      xCalcBlkGradient(xu << 2, yu << 2, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, sGx2, sGy2, sGxGy, sGxdI, sGydI, widthG, heightG, (1 << 2));

      if (sGx2 > 0)
      {
        tmpx = rightShiftMSB(sGxdI << 3, sGx2);
        tmpx = Clip3(-limit, limit, tmpx);
      }
      if (sGy2 > 0)
      {
        int     mainsGxGy = sGxGy >> 12;
        int     secsGxGy = sGxGy & ((1 << 12) - 1);
        int     tmpData = tmpx * mainsGxGy;
        tmpData = ((tmpData << 12) + tmpx*secsGxGy) >> 1;
        tmpy = rightShiftMSB(((sGydI << 3) - tmpData), sGy2);
        tmpy = Clip3(-limit, limit, tmpy);
      }

      srcY0Temp = srcY0 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      srcY1Temp = srcY1 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      gradX0 = m_gradX0 + offsetPos + ((yu*widthG + xu) << 2);
      gradX1 = m_gradX1 + offsetPos + ((yu*widthG + xu) << 2);
      gradY0 = m_gradY0 + offsetPos + ((yu*widthG + xu) << 2);
      gradY1 = m_gradY1 + offsetPos + ((yu*widthG + xu) << 2);

      dstY0 = dstY + ((yu*dstStride + xu) << 2);
      xAddBIOAvg4(srcY0Temp, src0Stride, srcY1Temp, src1Stride, dstY0, dstStride, gradX0, gradX1, gradY0, gradY1, widthG, (1 << 2), (1 << 2), (int)tmpx, (int)tmpy, shiftNum, offset, clpRng);
    }  // xu
  }  // yu
}

#if !JVET_M0487_INT_EXTEND
void InterPrediction::bioSampleExtendBilinearFilter(Pel const* src, int srcStride, Pel *dst, int dstStride, int width, int height, int dim, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng)
{
  Pel const* pSrc = NULL;
  Pel*       pDst = NULL;

  int vFilterSize = NTAPS_BILINEAR;
  int widthTmp = 0;
  int heightTmp = 0;

  for (int cand = 0; cand < 4; cand++)  // top, left, bottom and right
  {

    if (cand == 0)  // top
    {
      pSrc = src;
      pDst = dst;
      widthTmp = width;
      heightTmp = dim;
    }
    else if (cand == 1)  // left
    {
      pSrc = src + dim*srcStride;
      pDst = dst + dim*dstStride;
      widthTmp = dim;
      heightTmp = height - 2 * dim;
    }
    else if (cand == 2)  // bottom
    {
      pSrc = src + (height - dim)*srcStride;
      pDst = dst + (height - dim)*dstStride;
      widthTmp = width;
      heightTmp = dim;
    }
    else if (cand == 3)  // right
    {
      pSrc = src + dim*srcStride + width - dim;
      pDst = dst + dim*dstStride + width - dim;
      widthTmp = dim;
      heightTmp = height - 2 * dim;
    }

    if (fracY == 0)
    {
      m_if.filterHor(COMPONENT_Y, pSrc, srcStride, pDst, dstStride, widthTmp, heightTmp, fracX, isLast, fmt, clpRng, 1);
    }
    else if (fracX == 0)
    {
      m_if.filterVer(COMPONENT_Y, pSrc, srcStride, pDst, dstStride, widthTmp, heightTmp, fracY, true, isLast, fmt, clpRng, 1);
    }
    else
    {
      PelBuf tmpBuf = PelBuf(m_filteredBlockTmp[0][COMPONENT_Y], Size(width, height));
      tmpBuf.stride = width;

      m_if.filterHor(COMPONENT_Y, pSrc - ((vFilterSize >> 1) - 1) * srcStride, srcStride, tmpBuf.buf, tmpBuf.stride, widthTmp, heightTmp + vFilterSize - 1, fracX, false, fmt, clpRng, 1);
      JVET_J0090_SET_CACHE_ENABLE( false );
      m_if.filterVer(COMPONENT_Y, tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, pDst, dstStride, widthTmp, heightTmp, fracY, false, isLast, fmt, clpRng, 1);
      JVET_J0090_SET_CACHE_ENABLE( true );
    }
  }
}
#endif

bool InterPrediction::xCalcBiPredSubBlkDist(const PredictionUnit &pu, const Pel* pYuvSrc0, const int src0Stride, const Pel* pYuvSrc1, const int src1Stride, const BitDepths &clipBitDepths)
{
  const int     width = pu.lwidth();
  const int     height = pu.lheight();
  const int     clipbd = clipBitDepths.recon[toChannelType(COMPONENT_Y)];
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(clipbd);
  const int     shift = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
  const int     xUnit = (width >> 2);
  const int     yUnit = (height >> 2);

  m_bioDistThres = (shift <= 5) ? (((32 << (clipbd - 8))*width*height) >> (5 - shift)) : (((32 << (clipbd - 8))*width*height) << (shift - 5));
  m_bioSubBlkDistThres = (shift <= 5) ? (((64 << (clipbd - 8)) << 4) >> (5 - shift)) : (((64 << (clipbd - 8)) << 4) << (shift - 5));

  m_bioDistThres >>= distortionShift;
  m_bioSubBlkDistThres >>= distortionShift;

  DistParam cDistParam;
  Distortion dist = 0;
  for (int yu = 0, blkIdx = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++, blkIdx++)
    {
      const Pel* pPred0 = pYuvSrc0 + ((yu*src0Stride + xu) << 2);
      const Pel* pPred1 = pYuvSrc1 + ((yu*src1Stride + xu) << 2);

      m_pcRdCost->setDistParam(cDistParam, pPred0, pPred1, src0Stride, src1Stride, clipbd, COMPONENT_Y, (1 << 2), (1 << 2), 0, 1, false, true);
      m_bioPredSubBlkDist[blkIdx] = cDistParam.distFunc(cDistParam);
      dist += m_bioPredSubBlkDist[blkIdx];
    }
  }

  return (dist >= m_bioDistThres);
}

void InterPrediction::xAddBIOAvg4(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel *dst, int dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
{
  g_pelBufOP.addBIOAvg4(src0, src0Stride, src1, src1Stride, dst, dstStride, gradX0, gradX1, gradY0, gradY1, gradStride, width, height, tmpx, tmpy, shift, offset, clpRng);
}

#if JVET_M0063_BDOF_FIX
void InterPrediction::xBioGradFilter(Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, int bitDepth)
{
  g_pelBufOP.bioGradFilter(pSrc, srcStride, width, height, gradStride, gradX, gradY, bitDepth);
}

void InterPrediction::xCalcBIOPar(const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG, int bitDepth)
{
  g_pelBufOP.calcBIOPar(srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, src0Stride, src1Stride, gradStride, widthG, heightG, bitDepth);
}
#else
void InterPrediction::xBioGradFilter(Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY)
{
  g_pelBufOP.bioGradFilter(pSrc, srcStride, width, height, gradStride, gradX, gradY);
}

void InterPrediction::xCalcBIOPar(const Pel* srcY0Temp, const Pel* srcY1Temp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int* dotProductTemp1, int* dotProductTemp2, int* dotProductTemp3, int* dotProductTemp5, int* dotProductTemp6, const int src0Stride, const int src1Stride, const int gradStride, const int widthG, const int heightG)
{
  g_pelBufOP.calcBIOPar(srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, dotProductTemp1, dotProductTemp2, dotProductTemp3, dotProductTemp5, dotProductTemp6, src0Stride, src1Stride, gradStride, widthG, heightG);
}
#endif

void InterPrediction::xCalcBlkGradient(int sx, int sy, int    *arraysGx2, int     *arraysGxGy, int     *arraysGxdI, int     *arraysGy2, int     *arraysGydI, int     &sGx2, int     &sGy2, int     &sGxGy, int     &sGxdI, int     &sGydI, int width, int height, int unitSize)
{
  g_pelBufOP.calcBlkGradient(sx, sy, arraysGx2, arraysGxGy, arraysGxdI, arraysGy2, arraysGydI, sGx2, sGy2, sGxGy, sGxdI, sGydI, width, height, unitSize);
}

void InterPrediction::xWeightedAverage(const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied )
{
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];

  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    if( pu.cu->GBiIdx != GBI_DEFAULT )
    {
      VTMCHECK(bioApplied, "GBi is disallowed with BIO");
      pcYuvDst.addWeightedAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, pu.cu->GBiIdx);
      return;
    }
    if (bioApplied)
    {
      const int  src0Stride = pu.lwidth() + 2 * BIO_EXTEND_SIZE + 2;
      const int  src1Stride = pu.lwidth() + 2 * BIO_EXTEND_SIZE + 2;
      const Pel* pSrcY0 = m_filteredBlockTmp[2][COMPONENT_Y] + 2 * src0Stride + 2;
      const Pel* pSrcY1 = m_filteredBlockTmp[3][COMPONENT_Y] + 2 * src1Stride + 2;

      bool bioEnabled = xCalcBiPredSubBlkDist(pu, pSrcY0, src0Stride, pSrcY1, src1Stride, clipBitDepths);
      if (bioEnabled)
      {
        applyBiOptFlow(pu, pcYuvSrc0, pcYuvSrc1, iRefIdx0, iRefIdx1, pcYuvDst, clipBitDepths);
      }
      else
      {
        pcYuvDst.bufs[0].addAvg(CPelBuf(pSrcY0, src0Stride, pu.lumaSize()), CPelBuf(pSrcY1, src1Stride, pu.lumaSize()), clpRngs.comp[0]);
      }
    }
    pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, bioApplied);
  }
  else if( iRefIdx0 >= 0 && iRefIdx1 < 0 )
  {
    if( pu.cu->triangle )
    {
      pcYuvDst.copyFrom( pcYuvSrc0 );
    }
    else
    pcYuvDst.copyClip( pcYuvSrc0, clpRngs );
  }
  else if( iRefIdx0 < 0 && iRefIdx1 >= 0 )
  {
    if( pu.cu->triangle )
    {
      pcYuvDst.copyFrom( pcYuvSrc1 );
    }
    else
    pcYuvDst.copyClip( pcYuvSrc1, clpRngs );
  }
}

void InterPrediction::motionCompensation( PredictionUnit &pu, PelUnitBuf &predBuf, const RefPicList &eRefPicList
  , const bool luma, const bool chroma
)
{
  // dual tree handling for IBC as the only ref
  if ((!luma || !chroma) && eRefPicList == REF_PIC_LIST_0)
  {
    if (!luma && chroma)
    {
      xChromaMC(pu, predBuf);
      return;
    }
    else // (luma && !chroma)
    {
      xPredInterUni(pu, eRefPicList, predBuf, false
        , false
        , luma, chroma);
      return;
    }
  }
  // else, go with regular MC below
        CodingStructure &cs = *pu.cs;
  const PPS &pps            = *cs.pps;
  const SliceType sliceType =  cs.slice->getSliceType();

  if( eRefPicList != REF_PIC_LIST_X )
  {
    if( ( ( sliceType == P_SLICE && pps.getUseWP() ) || ( sliceType == B_SLICE && pps.getWPBiPred() ) ) )
    {
      xPredInterUni         ( pu,          eRefPicList, predBuf, true
        , false
        , true, true
      );
      xWeightedPredictionUni( pu, predBuf, eRefPicList, predBuf, -1, m_maxCompIDToPred );
    }
    else
    {
      xPredInterUni( pu, eRefPicList, predBuf, false
        , false
        , true, true
      );
    }
  }
  else
  {
    if (pu.mergeType != MRG_TYPE_DEFAULT_N && pu.mergeType != MRG_TYPE_IBC)
    {
      xSubPuMC( pu, predBuf, eRefPicList );
    }
    else if( xCheckIdenticalMotion( pu ) )
    {
      xPredInterUni( pu, REF_PIC_LIST_0, predBuf, false
        , false
        , true, true
      );
    }
    else
    {
      xPredInterBi( pu, predBuf );
    }
  }
  return;
}

void InterPrediction::motionCompensation( CodingUnit &cu, const RefPicList &eRefPicList
  , const bool luma, const bool chroma
)
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    PelUnitBuf predBuf = cu.cs->getPredBuf( pu );
#if JVET_M0147_DMVR
    pu.mvRefine = true;
#endif
    motionCompensation( pu, predBuf, eRefPicList
      , luma, chroma
    );
#if JVET_M0147_DMVR
    pu.mvRefine = false;
#endif
  }
}

void InterPrediction::motionCompensation( PredictionUnit &pu, const RefPicList &eRefPicList /*= REF_PIC_LIST_X*/
  , const bool luma, const bool chroma
)
{
  PelUnitBuf predBuf = pu.cs->getPredBuf( pu );
  motionCompensation( pu, predBuf, eRefPicList
    , luma, chroma
  );
}

int InterPrediction::rightShiftMSB(int numer, int denom)
{
  int     d;
  int msbIdx = 0;
  for (msbIdx = 0; msbIdx<32; msbIdx++)
  {
    if (denom < ((int)1 << msbIdx))
    {
      break;
    }
  }

  int shiftIdx = msbIdx - 1;
  d = (numer >> shiftIdx);

  return d;
}

void InterPrediction::motionCompensation4Triangle( CodingUnit &cu, MergeCtx &triangleMrgCtx, const bool splitDir, const uint8_t candIdx0, const uint8_t candIdx1 )
{
  for( auto &pu : CU::traversePUs( cu ) )
  {
    const UnitArea localUnitArea( cu.cs->area.chromaFormat, Area( 0, 0, pu.lwidth(), pu.lheight() ) );
    PelUnitBuf tmpTriangleBuf = m_triangleBuf.getBuf( localUnitArea );
    PelUnitBuf predBuf        = cu.cs->getPredBuf( pu );

    triangleMrgCtx.setMergeInfo( pu, candIdx0 );
    PU::spanMotionInfo( pu );
    motionCompensation( pu, tmpTriangleBuf );

    triangleMrgCtx.setMergeInfo( pu, candIdx1 );
    PU::spanMotionInfo( pu );
    motionCompensation( pu, predBuf );

#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
    weightedTriangleBlk( pu, splitDir, MAX_NUM_CHANNEL_TYPE, predBuf, tmpTriangleBuf, predBuf );
#else
    weightedTriangleBlk( pu, PU::getTriangleWeights(pu, triangleMrgCtx, candIdx0, candIdx1), splitDir, MAX_NUM_CHANNEL_TYPE, predBuf, tmpTriangleBuf, predBuf );
#endif
  }
}

#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
void InterPrediction::weightedTriangleBlk( PredictionUnit &pu, const bool splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 )
#else
void InterPrediction::weightedTriangleBlk( PredictionUnit &pu, bool weights, const bool splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 )
#endif
{
#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
  if( channel == CHANNEL_TYPE_LUMA )
  {
    xWeightedTriangleBlk( pu, pu.lumaSize().width, pu.lumaSize().height, COMPONENT_Y, splitDir, predDst, predSrc0, predSrc1 );
  }
  else if( channel == CHANNEL_TYPE_CHROMA )
  {
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, predDst, predSrc0, predSrc1 );
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, predDst, predSrc0, predSrc1 );
  }
  else
  {
    xWeightedTriangleBlk( pu, pu.lumaSize().width,   pu.lumaSize().height,   COMPONENT_Y,  splitDir, predDst, predSrc0, predSrc1 );
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, predDst, predSrc0, predSrc1 );
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, predDst, predSrc0, predSrc1 );
  }
#else
  if( channel == CHANNEL_TYPE_LUMA )
  {
    xWeightedTriangleBlk( pu, pu.lumaSize().width, pu.lumaSize().height, COMPONENT_Y, splitDir, weights, predDst, predSrc0, predSrc1 );
  }
  else if( channel == CHANNEL_TYPE_CHROMA )
  {
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, weights, predDst, predSrc0, predSrc1 );
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, weights, predDst, predSrc0, predSrc1 );
  }
  else
  {
    xWeightedTriangleBlk( pu, pu.lumaSize().width,   pu.lumaSize().height,   COMPONENT_Y,  splitDir, weights, predDst, predSrc0, predSrc1 );
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, weights, predDst, predSrc0, predSrc1 );
    xWeightedTriangleBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, weights, predDst, predSrc0, predSrc1 );
  }
#endif
}

#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
void InterPrediction::xWeightedTriangleBlk( const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const bool splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 )
#else
void InterPrediction::xWeightedTriangleBlk( const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const bool splitDir, const bool weights, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 )
#endif
{
  Pel*    dst        = predDst .get(compIdx).buf;
  Pel*    src0       = predSrc0.get(compIdx).buf;
  Pel*    src1       = predSrc1.get(compIdx).buf;
  int32_t strideDst  = predDst .get(compIdx).stride  - width;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride  - width;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride  - width;

  const char    log2WeightBase    = 3;
  const ClpRng  clipRng           = pu.cu->slice->clpRngs().comp[compIdx];
  const int32_t clipbd            = clipRng.bd;
  const int32_t shiftDefault      = std::max<int>(2, (IF_INTERNAL_PREC - clipbd));
  const int32_t offsetDefault     = (1<<(shiftDefault-1)) + IF_INTERNAL_OFFS;
  const int32_t shiftWeighted     = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + log2WeightBase;
  const int32_t offsetWeighted    = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);

  const int32_t ratioWH           = (width > height) ? (width / height) : 1;
  const int32_t ratioHW           = (width > height) ? 1 : (height / width);
#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
  const bool    longWeight        = (compIdx == COMPONENT_Y) || ( predDst.chromaFormat == CHROMA_444 );
  const int32_t weightedLength    = longWeight ? 7 : 3;
#else
  const Pel*    pelWeighted       = (compIdx == COMPONENT_Y) ? g_trianglePelWeightedLuma[splitDir][weights] : g_trianglePelWeightedChroma[predDst.chromaFormat == CHROMA_444 ? 0 : 1][splitDir][weights];
  const int32_t weightedLength    = (compIdx == COMPONENT_Y) ? g_triangleWeightLengthLuma[weights] : g_triangleWeightLengthChroma[predDst.chromaFormat == CHROMA_444 ? 0 : 1][weights];
#endif
        int32_t weightedStartPos  = ( splitDir == 0 ) ? ( 0 - (weightedLength >> 1) * ratioWH ) : ( width - ((weightedLength + 1) >> 1) * ratioWH );
        int32_t weightedEndPos    = weightedStartPos + weightedLength * ratioWH - 1;
        int32_t weightedPosoffset =( splitDir == 0 ) ? ratioWH : -ratioWH;

#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
        Pel     tmpPelWeighted;
        int32_t weightIdx;
#else
  const Pel*    tmpPelWeighted;
#endif
        int32_t x, y, tmpX, tmpY, tmpWeightedStart, tmpWeightedEnd;

  for( y = 0; y < height; y+= ratioHW )
  {
    for( tmpY = ratioHW; tmpY > 0; tmpY-- )
    {
      for( x = 0; x < weightedStartPos; x++ )
      {
        *dst++ = ClipPel( rightShift( (splitDir == 0 ? *src1 : *src0) + offsetDefault, shiftDefault), clipRng );
        src0++;
        src1++;
      }

      tmpWeightedStart = std::max((int32_t)0, weightedStartPos);
      tmpWeightedEnd   = std::min(weightedEndPos, (int32_t)(width - 1));
#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
      weightIdx        = 1;
#else
      tmpPelWeighted   = pelWeighted;
#endif
      if( weightedStartPos < 0 )
      {
#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
        weightIdx     += abs(weightedStartPos) / ratioWH;
#else
        tmpPelWeighted += abs(weightedStartPos) / ratioWH;
#endif
      }
      for( x = tmpWeightedStart; x <= tmpWeightedEnd; x+= ratioWH )
      {
        for( tmpX = ratioWH; tmpX > 0; tmpX-- )
        {
#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
          tmpPelWeighted = Clip3( 1, 7, longWeight ? weightIdx : (weightIdx * 2));
          tmpPelWeighted = splitDir ? ( 8 - tmpPelWeighted ) : tmpPelWeighted;
          *dst++         = ClipPel( rightShift( (tmpPelWeighted*(*src0++) + ((8 - tmpPelWeighted) * (*src1++)) + offsetWeighted), shiftWeighted ), clipRng );
#else
          *dst++ = ClipPel( rightShift( ((*tmpPelWeighted)*(*src0++) + ((8 - (*tmpPelWeighted)) * (*src1++)) + offsetWeighted), shiftWeighted ), clipRng );
#endif
        }
#if JVET_M0328_KEEP_ONE_WEIGHT_GROUP
        weightIdx ++;
#else
        tmpPelWeighted++;
#endif
      }

      for( x = weightedEndPos + 1; x < width; x++ )
      {
        *dst++ = ClipPel( rightShift( (splitDir == 0 ? *src0 : *src1) + offsetDefault, shiftDefault ), clipRng );
        src0++;
        src1++;
      }

      dst  += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
    }
    weightedStartPos += weightedPosoffset;
    weightedEndPos   += weightedPosoffset;
  }
}

#if JVET_M0147_DMVR
void InterPrediction::xPrefetchPad(PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId)
{
  int offset, width, height;
  int padsize;
  Mv cMv;
  const Picture* refPic = pu.cu->slice->getRefPic(refId, pu.refIdx[refId]);
  int mvShift = (MV_FRACTIONAL_BITS_INTERNAL);
  for (int compID = 0; compID < MAX_NUM_COMPONENT; compID++)
  {
    cMv = Mv(pu.mv[refId].getHor(), pu.mv[refId].getVer());
    pcPad.bufs[compID].stride = (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA);
    int filtersize = (compID == (COMPONENT_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
    width = pcPad.bufs[compID].width;
    height = pcPad.bufs[compID].height;
    offset = (DMVR_NUM_ITERATION) * (pcPad.bufs[compID].stride + 1);
    padsize = (DMVR_NUM_ITERATION) >> getComponentScaleX((ComponentID)compID, pu.chromaFormat);
    int mvshiftTemp = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
    width += (filtersize - 1);
    height += (filtersize - 1);
    cMv += Mv(-(((filtersize >> 1) - 1) << mvshiftTemp),
      -(((filtersize >> 1) - 1) << mvshiftTemp));
    clipMv(cMv, pu.lumaPos(), pu.lumaSize(),*pu.cs->sps);
    /* Pre-fetch similar to HEVC*/
    {
      CPelBuf refBuf;
      Position Rec_offset = pu.blocks[compID].pos().offset(cMv.getHor() >> mvshiftTemp, cMv.getVer() >> mvshiftTemp);
      refBuf = refPic->getRecoBuf(CompArea((ComponentID)compID, pu.chromaFormat, Rec_offset, pu.blocks[compID].size()));
      PelBuf &dstBuf = pcPad.bufs[compID];
      g_pelBufOP.copyBuffer((Pel *)refBuf.buf, refBuf.stride, ((Pel *)dstBuf.buf) + offset, dstBuf.stride, width, height);
    }
    /*padding on all side of size DMVR_PAD_LENGTH*/
    {
      g_pelBufOP.padding(pcPad.bufs[compID].buf + offset, pcPad.bufs[compID].stride, width, height, padsize);
    }
  }
}
inline int32_t div_for_maxq7(int64_t N, int64_t D)
{
  int32_t sign, q;
  sign = 0;
  if (N < 0)
  {
    sign = 1;
    N = -N;
  }

  q = 0;
  D = (D << 3);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  D = (D >> 1);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  if (N >= (D >> 1))
    q++;

  if (sign)
    return (-q);
  return(q);
}

void xSubPelErrorSrfc(uint64_t *sadBuffer, int32_t *deltaMv)
{
  int64_t numerator, denominator;
  int32_t mvDeltaSubPel;
  int32_t mvSubPelLvl = 4;/*1: half pel, 2: Qpel, 3:1/8, 4: 1/16*/
                                                        /*horizontal*/
    numerator = (int64_t)((sadBuffer[1] - sadBuffer[3]) << mvSubPelLvl);
    denominator = (int64_t)((sadBuffer[1] + sadBuffer[3] - (sadBuffer[0] << 1)));

    if (0 != denominator)
    {
      if ((sadBuffer[1] != sadBuffer[0]) && (sadBuffer[3] != sadBuffer[0]))
      {
        mvDeltaSubPel = div_for_maxq7(numerator, denominator);
        deltaMv[0] = (mvDeltaSubPel);
      }
      else
      {
        if (sadBuffer[1] == sadBuffer[0])
        {
          deltaMv[0] = -8;// half pel
        }
        else
        {
          deltaMv[0] = 8;// half pel
        }
      }
    }

    /*vertical*/
    numerator = (int64_t)((sadBuffer[2] - sadBuffer[4]) << mvSubPelLvl);
    denominator = (int64_t)((sadBuffer[2] + sadBuffer[4] - (sadBuffer[0] << 1)));
    if (0 != denominator)
    {
      if ((sadBuffer[2] != sadBuffer[0]) && (sadBuffer[4] != sadBuffer[0]))
      {
        mvDeltaSubPel = div_for_maxq7(numerator, denominator);
        deltaMv[1] = (mvDeltaSubPel);
      }
      else
      {
        if (sadBuffer[2] == sadBuffer[0])
        {
          deltaMv[1] = -8;// half pel
        }
        else
        {
          deltaMv[1] = 8;// half pel
        }
      }
    }
  return;
}

void InterPrediction::xBIPMVRefine(int bd, Pel *pRefL0, Pel *pRefL1, uint64_t& minCost, int16_t *deltaMV, uint64_t *pSADsArray, int width, int height)
{
  const int32_t refStrideL0 = m_biLinearBufStride;
  const int32_t refStrideL1 = m_biLinearBufStride;
  Pel *pRefL0Orig = pRefL0;
  Pel *pRefL1Orig = pRefL1;
  for (int nIdx = SAD_BOTTOM; nIdx <= SAD_TOP_LEFT; ++nIdx)
  {
    int32_t sadOffset = ((m_pSearchOffset[nIdx].getVer() * ((2 * DMVR_NUM_ITERATION) + 1)) + m_pSearchOffset[nIdx].getHor());
    pRefL0 = pRefL0Orig + m_pSearchOffset[nIdx].hor + (m_pSearchOffset[nIdx].ver * refStrideL0);
    pRefL1 = pRefL1Orig - m_pSearchOffset[nIdx].hor - (m_pSearchOffset[nIdx].ver * refStrideL1);
    if (*(pSADsArray + sadOffset) == MAX_UINT64)
    {
      const uint64_t cost = xDMVRCost(bd, pRefL0, refStrideL0, pRefL1, refStrideL1, width, height);
      *(pSADsArray + sadOffset) = cost;
    }
    if (nIdx == SAD_LEFT)
    {
      int32_t down = -1, right = -1;
      if (pSADsArray[(((2 * DMVR_NUM_ITERATION) + 1))] < pSADsArray[-(((2 * DMVR_NUM_ITERATION) + 1))])
      {
        down = 1;
      }
      if (pSADsArray[1] < pSADsArray[-1])
      {
        right = 1;
      }
      m_pSearchOffset[SAD_TOP_LEFT].set(right, down);
    }
    if (*(pSADsArray + sadOffset) < minCost)
    {
      minCost = *(pSADsArray + sadOffset);
      deltaMV[0] = m_pSearchOffset[nIdx].getHor();
      deltaMV[1] = m_pSearchOffset[nIdx].getVer();
    }
  }
}

void InterPrediction::xFinalPaddedMCForDMVR(PredictionUnit& pu, PelUnitBuf &pcYuvSrc0, PelUnitBuf &pcYuvSrc1, PelUnitBuf &pcPad0, PelUnitBuf &pcPad1, const bool bioApplied
  , const Mv mergeMV[NUM_REF_PIC_LIST_01]
)
{
  int offset, deltaIntMvX, deltaIntMvY;

  PelUnitBuf pcYUVTemp = pcYuvSrc0;
  PelUnitBuf pcPadTemp = pcPad0;
  /*always high precision MVs are used*/
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    RefPicList refId = (RefPicList)k;
    Mv cMv = pu.mv[refId];
    m_iRefListIdx = refId;
    const Picture* refPic = pu.cu->slice->getRefPic(refId, pu.refIdx[refId]);
    clipMv(cMv, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps);

    Mv startMv = mergeMV[refId];
    clipMv(startMv, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps);

    for (int compID = 0; compID < MAX_NUM_COMPONENT; compID++)
    {
      int mvshiftTemp = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
      int leftPixelExtra;
      if (compID == COMPONENT_Y)
      {
        leftPixelExtra = (NTAPS_LUMA >> 1) - 1;
      }
      else
      {
        leftPixelExtra = (NTAPS_CHROMA >> 1) - 1;
      }

      deltaIntMvX = (cMv.getHor() >> mvshiftTemp) -
        (startMv.getHor() >> mvshiftTemp);
      deltaIntMvY = (cMv.getVer() >> mvshiftTemp) -
        (startMv.getVer() >> mvshiftTemp);

      VTMCHECK((abs(deltaIntMvX) > DMVR_NUM_ITERATION) || (abs(deltaIntMvY) > DMVR_NUM_ITERATION), "not expected DMVR movement");

      offset = (DMVR_NUM_ITERATION + leftPixelExtra) * (pcPadTemp.bufs[compID].stride + 1);
      offset += (deltaIntMvY)* pcPadTemp.bufs[compID].stride;
      offset += (deltaIntMvX);
      PelBuf &srcBuf = pcPadTemp.bufs[compID];
      xPredInterBlk((ComponentID)compID, pu, refPic, cMv, pcYUVTemp, true, pu.cs->slice->getClpRngs().comp[compID],
        bioApplied, false, 0, 0, 0, (srcBuf.buf + offset), pcPadTemp.bufs[compID].stride);
    }
    pcYUVTemp = pcYuvSrc1;
    pcPadTemp = pcPad1;
  }
}

uint64_t InterPrediction::xDMVRCost(int bitDepth, Pel* pOrg, uint32_t refStride, const Pel* pRef, uint32_t orgStride, int width, int height)
{
  DistParam cDistParam;
  cDistParam.applyWeight = false;
  cDistParam.useMR = false;
  m_pcRdCost->setDistParam(cDistParam, pOrg, pRef, orgStride, refStride, bitDepth, COMPONENT_Y, width, height, 1);
  uint64_t uiCost = cDistParam.distFunc(cDistParam);
  return uiCost;
}

void xDMVRSubPixelErrorSurface(bool notZeroCost, int16_t *totalDeltaMV, int16_t *deltaMV, uint64_t *pSADsArray)
{

  int sadStride = (((2 * DMVR_NUM_ITERATION) + 1));
  uint64_t sadbuffer[5];
  if (notZeroCost && deltaMV[0] == 0 && deltaMV[1] == 0)
  {
    int32_t tempDeltaMv[2] = { 0,0 };
    sadbuffer[0] = pSADsArray[0];
    sadbuffer[1] = pSADsArray[-1];
    sadbuffer[2] = pSADsArray[-sadStride];
    sadbuffer[3] = pSADsArray[1];
    sadbuffer[4] = pSADsArray[sadStride];
    xSubPelErrorSrfc(sadbuffer, tempDeltaMv);
    totalDeltaMV[0] += tempDeltaMv[0];
    totalDeltaMV[1] += tempDeltaMv[1];
  }
}

void InterPrediction::xinitMC(PredictionUnit& pu, const ClpRngs &clpRngs)
{
  const int refIdx0 = pu.refIdx[0];
  const int refIdx1 = pu.refIdx[1];
  /*use merge MV as starting MV*/
  Mv mergeMVL0(pu.mv[REF_PIC_LIST_0]);
  Mv mergeMVL1(pu.mv[REF_PIC_LIST_1]);

  /*Clip the starting MVs*/
  clipMv(mergeMVL0, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps);
  clipMv(mergeMVL1, pu.lumaPos(), pu.lumaSize(), *pu.cs->sps);

  /*L0 MC for refinement*/
  {
    int offset;
    int leftPixelExtra = (NTAPS_LUMA >> 1) - 1;
    offset = (DMVR_NUM_ITERATION + leftPixelExtra) * (m_cYuvRefBuffDMVRL0.bufs[COMPONENT_Y].stride + 1);
    offset += (-(int)DMVR_NUM_ITERATION)* (int)m_cYuvRefBuffDMVRL0.bufs[COMPONENT_Y].stride;
    offset += (-(int)DMVR_NUM_ITERATION);
    PelBuf srcBuf = m_cYuvRefBuffDMVRL0.bufs[COMPONENT_Y];
    PelUnitBuf yuvPredTempL0 = PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVRL0,
      (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)), pu.lwidth() + (2 * DMVR_NUM_ITERATION), pu.lheight() + (2 * DMVR_NUM_ITERATION)));

    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdx0), mergeMVL0, yuvPredTempL0, true, clpRngs.comp[COMPONENT_Y],
      false, false, pu.lwidth() + (2 * DMVR_NUM_ITERATION), pu.lheight() + (2 * DMVR_NUM_ITERATION), true, ((Pel *)srcBuf.buf) + offset, srcBuf.stride
    );
  }

  /*L1 MC for refinement*/
  {
    int offset;
    int leftPixelExtra = (NTAPS_LUMA >> 1) - 1;
    offset = (DMVR_NUM_ITERATION + leftPixelExtra) * (m_cYuvRefBuffDMVRL1.bufs[COMPONENT_Y].stride + 1);
    offset += (-(int)DMVR_NUM_ITERATION)* (int)m_cYuvRefBuffDMVRL1.bufs[COMPONENT_Y].stride;
    offset += (-(int)DMVR_NUM_ITERATION);
    PelBuf srcBuf = m_cYuvRefBuffDMVRL1.bufs[COMPONENT_Y];
    PelUnitBuf yuvPredTempL1 = PelUnitBuf(pu.chromaFormat, PelBuf(m_cYuvPredTempDMVRL1,
      (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)), pu.lwidth() + (2 * DMVR_NUM_ITERATION), pu.lheight() + (2 * DMVR_NUM_ITERATION)));

    xPredInterBlk(COMPONENT_Y, pu, pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdx1), mergeMVL1, yuvPredTempL1, true, clpRngs.comp[COMPONENT_Y],
      false, false, pu.lwidth() + (2 * DMVR_NUM_ITERATION), pu.lheight() + (2 * DMVR_NUM_ITERATION), true, ((Pel *)srcBuf.buf) + offset, srcBuf.stride
    );
  }
}

void InterPrediction::xProcessDMVR(PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied)
{
  int iterationCount = DMVR_NUM_ITERATION;
  /*Always High Precision*/
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  /*use merge MV as starting MV*/
  Mv mergeMv[] = { pu.mv[REF_PIC_LIST_0] , pu.mv[REF_PIC_LIST_1] };

  m_biLinearBufStride = (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION));

  int dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
  int dx = std::min<int>(pu.lumaSize().width,  DMVR_SUBCU_WIDTH);
  /*L0 Padding*/
  m_cYuvRefBuffDMVRL0 = (pu.chromaFormat == CHROMA_400 ?
    PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL0[0], pcYuvDst.Y())) :
    PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL0[0], pcYuvDst.Y()),
      PelBuf(m_cRefSamplesDMVRL0[1], pcYuvDst.Cb()), PelBuf(m_cRefSamplesDMVRL0[2], pcYuvDst.Cr())));

  xPrefetchPad(pu, m_cYuvRefBuffDMVRL0, REF_PIC_LIST_0);

  /*L1 Padding*/
  m_cYuvRefBuffDMVRL1 = (pu.chromaFormat == CHROMA_400 ?
    PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL1[0], pcYuvDst.Y())) :
    PelUnitBuf(pu.chromaFormat, PelBuf(m_cRefSamplesDMVRL1[0], pcYuvDst.Y()), PelBuf(m_cRefSamplesDMVRL1[1], pcYuvDst.Cb()),
      PelBuf(m_cRefSamplesDMVRL1[2], pcYuvDst.Cr())));

  xPrefetchPad(pu, m_cYuvRefBuffDMVRL1, REF_PIC_LIST_1);

  xinitMC(pu, clpRngs);

  // point mc buffer to cetre point to avoid multiplication to reach each iteration to the begining
  Pel *biLinearPredL0 = m_cYuvPredTempDMVRL0 + (iterationCount * m_biLinearBufStride) + iterationCount;
  Pel *biLinearPredL1 = m_cYuvPredTempDMVRL1 + (iterationCount * m_biLinearBufStride) + iterationCount;

  Position puPos = pu.lumaPos();

  int bd = pu.cs->slice->getClpRngs().comp[COMPONENT_Y].bd;

  {
    int num = 0;

    int yStart = 0;
    for (int y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy, yStart = yStart + dy)
    {
      for (int x = puPos.x, xStart = 0; x < (puPos.x + pu.lumaSize().width); x = x + dx, xStart = xStart + dx)
      {
        uint64_t minCost = MAX_UINT64;
        bool notZeroCost = true;
        int16_t totalDeltaMV[2] = { 0,0 };
        int16_t deltaMV[2] = { 0, 0 };
        uint64_t  *pSADsArray;
        for (int i = 0; i < (((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)); i++)
        {
          m_SADsArray[i] = MAX_UINT64;
        }
        pSADsArray = &m_SADsArray[(((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)) >> 1];

        Pel *addrL0Centre = biLinearPredL0 + yStart * m_biLinearBufStride + xStart;
        Pel *addrL1Centre = biLinearPredL1 + yStart * m_biLinearBufStride + xStart;
        for (int i = 0; i < iterationCount; i++)
        {
          deltaMV[0] = 0;
          deltaMV[1] = 0;
          Pel *addrL0 = addrL0Centre + totalDeltaMV[0] + (totalDeltaMV[1] * m_biLinearBufStride);
          Pel *addrL1 = addrL1Centre - totalDeltaMV[0] - (totalDeltaMV[1] * m_biLinearBufStride);
          if (i == 0)
          {
            minCost = xDMVRCost(clpRngs.comp[COMPONENT_Y].bd, addrL0, m_biLinearBufStride, addrL1, m_biLinearBufStride, dx, dy);
            if (minCost < ((4 * dx * (dy >> 1/*for alternate line*/))))
            {
              notZeroCost = false;
              break;
            }
            pSADsArray[0] = minCost;
          }
          if (!minCost)
          {
            notZeroCost = false;
            break;
          }

          xBIPMVRefine(bd, addrL0, addrL1, minCost, deltaMV, pSADsArray, dx, dy);

          if (deltaMV[0] == 0 && deltaMV[1] == 0)
          {
            break;
          }
          totalDeltaMV[0] += deltaMV[0];
          totalDeltaMV[1] += deltaMV[1];
          pSADsArray += ((deltaMV[1] * (((2 * DMVR_NUM_ITERATION) + 1))) + deltaMV[0]);
        }

        totalDeltaMV[0] = (totalDeltaMV[0] << mvShift);
        totalDeltaMV[1] = (totalDeltaMV[1] << mvShift);
        xDMVRSubPixelErrorSurface(notZeroCost, totalDeltaMV, deltaMV, pSADsArray);

        pu.mvdL0SubPu[num] = Mv(totalDeltaMV[0], totalDeltaMV[1]);

        num++;
      }
    }
  }

  {
    PredictionUnit subPu = pu;
    subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(puPos.x, puPos.y, dx, dy)));
    PelUnitBuf           m_cYuvRefBuffSubCuDMVRL0;
    PelUnitBuf           m_cYuvRefBuffSubCuDMVRL1;
    PelUnitBuf srcPred0 = (pu.chromaFormat == CHROMA_400 ?
      PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y())) :
      PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[0][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[0][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[0][2], pcYuvDst.Cr())));
    PelUnitBuf srcPred1 = (pu.chromaFormat == CHROMA_400 ?
      PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y())) :
      PelUnitBuf(pu.chromaFormat, PelBuf(m_acYuvPred[1][0], pcYuvDst.Y()), PelBuf(m_acYuvPred[1][1], pcYuvDst.Cb()), PelBuf(m_acYuvPred[1][2], pcYuvDst.Cr())));

    srcPred0 = srcPred0.subBuf(UnitAreaRelative(pu, subPu));
    srcPred1 = srcPred1.subBuf(UnitAreaRelative(pu, subPu));
    PelUnitBuf subPredBuf = pcYuvDst.subBuf(UnitAreaRelative(pu, subPu));

    int x = 0, y = 0;
    int xStart = 0, yStart = 0;
    int num = 0;

    int dstStride[MAX_NUM_COMPONENT] = { pcYuvDst.bufs[COMPONENT_Y].stride, pcYuvDst.bufs[COMPONENT_Cb].stride, pcYuvDst.bufs[COMPONENT_Cr].stride };
    for (y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy, yStart = yStart + dy)
    {
      for (x = puPos.x, xStart = 0; x < (puPos.x + pu.lumaSize().width); x = x + dx, xStart = xStart + dx)
      {
        subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));

        subPu.mv[0] = mergeMv[REF_PIC_LIST_0] + pu.mvdL0SubPu[num];
        subPu.mv[1] = mergeMv[REF_PIC_LIST_1] - pu.mvdL0SubPu[num];
        m_cYuvRefBuffSubCuDMVRL0 = m_cYuvRefBuffDMVRL0.subBuf(UnitAreaRelative(pu, subPu));
        m_cYuvRefBuffSubCuDMVRL1 = m_cYuvRefBuffDMVRL1.subBuf(UnitAreaRelative(pu, subPu));
        xFinalPaddedMCForDMVR(subPu, srcPred0, srcPred1, m_cYuvRefBuffSubCuDMVRL0, m_cYuvRefBuffSubCuDMVRL1, bioApplied, mergeMv);

        subPredBuf.bufs[COMPONENT_Y].buf  = pcYuvDst.bufs[COMPONENT_Y].buf + xStart + yStart * dstStride[COMPONENT_Y];
        subPredBuf.bufs[COMPONENT_Cb].buf = pcYuvDst.bufs[COMPONENT_Cb].buf + (xStart >> 1) + ((yStart >> 1) * dstStride[COMPONENT_Cb]);
        subPredBuf.bufs[COMPONENT_Cr].buf = pcYuvDst.bufs[COMPONENT_Cr].buf + (xStart >> 1) + ((yStart >> 1) * dstStride[COMPONENT_Cr]);
        xWeightedAverage(subPu, srcPred0, srcPred1, subPredBuf, subPu.cu->slice->getSPS()->getBitDepths(), subPu.cu->slice->clpRngs(), bioApplied);
        num++;
      }
    }
  }
}
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
void InterPrediction::cacheAssign( CacheModel *cache )
{
  m_cacheModel = cache;
  m_if.cacheAssign( cache );
  m_if.initInterpolationFilter( !cache->isCacheEnable() );
}
#endif

//! \}
