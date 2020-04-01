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

/** \file     CABACWriter.cpp
 *  \brief    Writer for low level syntax
 */

#include "CommonLib/Contexts.h"
#include "CABACWriter.h"

#include "EncLib.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/BinaryDecisionTree.h"

#include <map>
#include <algorithm>
#include <limits>


//! \ingroup EncoderLib
//! \{

void CABACWriter::initCtxModels( const Slice& slice )
{
  int       qp                = slice.getSliceQp();
  SliceType sliceType         = slice.getSliceType();
  SliceType encCABACTableIdx  = slice.getEncCABACTableIdx();
  if( !slice.isIntra() && (encCABACTableIdx==B_SLICE || encCABACTableIdx==P_SLICE) && slice.getPPS()->getCabacInitPresentFlag() )
  {
    sliceType = encCABACTableIdx;
  }
  m_BinEncoder.reset( qp, (int)sliceType );
}



template <class BinProbModel>
SliceType xGetCtxInitId( const Slice& slice, const BinEncIf& binEncoder, Ctx& ctxTest )
{
  const CtxStore<BinProbModel>& ctxStoreTest = static_cast<const CtxStore<BinProbModel>&>( ctxTest );
  const CtxStore<BinProbModel>& ctxStoreRef  = static_cast<const CtxStore<BinProbModel>&>( binEncoder.getCtx() );
  int qp = slice.getSliceQp();
  if( !slice.isIntra() )
  {
    SliceType aSliceTypeChoices[] = { B_SLICE, P_SLICE };
    uint64_t  bestCost            = std::numeric_limits<uint64_t>::max();
    SliceType bestSliceType       = aSliceTypeChoices[0];
    for (uint32_t idx=0; idx<2; idx++)
    {
      uint64_t  curCost           = 0;
      SliceType curSliceType      = aSliceTypeChoices[idx];
      ctxTest.init( qp, (int)curSliceType );
      for( int k = 0; k < Ctx::NumberOfContexts; k++ )
      {
        if( binEncoder.getNumBins(k) > 0 )
        {
          curCost += uint64_t( binEncoder.getNumBins(k) ) * ctxStoreRef[k].estFracExcessBits( ctxStoreTest[k] );
        }
      }
      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}


SliceType CABACWriter::getCtxInitId( const Slice& slice )
{
  switch( m_TestCtx.getBPMType() )
  {
  case BPM_Std:   return  xGetCtxInitId<BinProbModel_Std>   ( slice, m_BinEncoder, m_TestCtx );
  default:        return  NUMBER_OF_SLICE_TYPES;
  }
}



unsigned estBits( BinEncIf& binEnc, const std::vector<bool>& bins, const Ctx& ctx, const int ctxId, const uint8_t winSize )
{
  binEnc.initCtxAndWinSize( ctxId, ctx, winSize );
  binEnc.start();
  const std::size_t numBins   = bins.size();
  unsigned          startBits = binEnc.getNumWrittenBits();
  for( std::size_t binId = 0; binId < numBins; binId++ )
  {
    unsigned  bin = ( bins[binId] ? 1 : 0 );
    binEnc.encodeBin( bin, ctxId );
  }
  unsigned endBits    = binEnc.getNumWrittenBits();
  unsigned codedBits  = endBits - startBits;
  return   codedBits;
}





//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    void  end_of_slice()
//================================================================================

void CABACWriter::end_of_slice()
{
  m_BinEncoder.encodeBinTrm ( 1 );
  m_BinEncoder.finish       ();
}




//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qp, ctuRsAddr, skipSao )
//================================================================================

void CABACWriter::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr, bool skipSao /* = false */ )
{
  CUCtx cuCtx( qps[CH_L] );
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );

  partitioner->initCtu( area, CH_L, *cs.slice );

  if( !skipSao )
  {
    sao( *cs.slice, ctuRsAddr );
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    codeAlfCtuEnableFlag( cs, ctuRsAddr, compIdx );
  }

#if ADCNN
  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
	  codeCnnlfCtuEnableFlag(cs, ctuRsAddr, compIdx);
  }
#endif

  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > 64 )
  {
    CUCtx chromaCuCtx(qps[CH_C]);
    Partitioner *chromaPartitioner = PartitionerFactory::get(*cs.slice);
    chromaPartitioner->initCtu(area, CH_C, *cs.slice);
    coding_tree(cs, *partitioner, cuCtx, chromaPartitioner, &chromaCuCtx);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = chromaCuCtx.qp;

    delete chromaPartitioner;
  }
  else
  {
    coding_tree( cs, *partitioner, cuCtx );
    qps[CH_L] = cuCtx.qp;
    if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner->initCtu( area, CH_C, *cs.slice );
      coding_tree( cs, *partitioner, cuCtxChroma );
      qps[CH_C] = cuCtxChroma.qp;
    }
  }

  delete partitioner;
}





//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao             ( slice, ctuRsAddr )
//    void  sao_block_pars  ( saoPars, bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, onlyEstMergeInfo )
//    void  sao_offset_pars ( ctbPars, compID, sliceEnabled, bitDepth )
//================================================================================

void CABACWriter::sao( const Slice& slice, unsigned ctuRsAddr )
{
  const SPS& sps = *slice.getSPS();
  if( !sps.getSAOEnabledFlag() )
  {
    return;
  }

  CodingStructure&     cs                     = *slice.getPic()->cs;
  const PreCalcValues& pcv                    = *cs.pcv;
  const SAOBlkParam&  sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool                slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool                slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  bool                sliceEnabled[3]         = { slice_sao_luma_flag, slice_sao_chroma_flag, slice_sao_chroma_flag };
  int                 frame_width_in_ctus     = pcv.widthInCtus;
  int                 ry                      = ctuRsAddr      / frame_width_in_ctus;
  int                 rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  const Position      pos                     ( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned      curSliceIdx             = slice.getIndependentSliceIdx();
#if HEVC_TILES_WPP
  const unsigned      curTileIdx              = cs.picture->tileMap->getTileIdxMap( pos );
  bool                leftMergeAvail          = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0  ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool                aboveMergeAvail         = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
#else
  bool                leftMergeAvail          = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0  ), curSliceIdx, CH_L ) ? true : false;
  bool                aboveMergeAvail         = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), curSliceIdx, CH_L ) ? true : false;
#endif
  sao_block_pars( sao_ctu_pars, sps.getBitDepths(), sliceEnabled, leftMergeAvail, aboveMergeAvail, false );
}


void CABACWriter::sao_block_pars( const SAOBlkParam& saoPars, const BitDepths& bitDepths, bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo )
{
  bool isLeftMerge  = false;
  bool isAboveMerge = false;
  if( leftMergeAvail )
  {
    // sao_merge_left_flag
    isLeftMerge   = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_LEFT );
    m_BinEncoder.encodeBin( (isLeftMerge), Ctx::SaoMergeFlag() );
  }
  if( aboveMergeAvail && !isLeftMerge )
  {
    // sao_merge_above_flag
    isAboveMerge  = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_ABOVE );
    m_BinEncoder.encodeBin( (isAboveMerge), Ctx::SaoMergeFlag() );
  }
  if( onlyEstMergeInfo )
  {
    return; //only for RDO
  }
  if( !isLeftMerge && !isAboveMerge )
  {
    // explicit parameters
    for( int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      sao_offset_pars( saoPars[compIdx], ComponentID(compIdx), sliceEnabled[compIdx], bitDepths.recon[ toChannelType(ComponentID(compIdx)) ] );
    }
  }
}


void CABACWriter::sao_offset_pars( const SAOOffset& ctbPars, ComponentID compID, bool sliceEnabled, int bitDepth )
{
  if( !sliceEnabled )
  {
    VTMCHECK( ctbPars.modeIdc != SAO_MODE_OFF, "Sao must be off, if it is disabled on slice level" );
    return;
  }
  const bool isFirstCompOfChType = ( getFirstComponentOfChannel( toChannelType(compID) ) == compID );

  if( isFirstCompOfChType )
  {
    // sao_type_idx_luma / sao_type_idx_chroma
    if( ctbPars.modeIdc == SAO_MODE_OFF )
    {
      m_BinEncoder.encodeBin  ( 0, Ctx::SaoTypeIdx() );
    }
    else if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 0 );
    }
    else
    {
      VTMCHECK(!( ctbPars.typeIdc < SAO_TYPE_START_BO ), "Unspecified error");
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 1 );
    }
  }

  if( ctbPars.modeIdc == SAO_MODE_NEW )
  {
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( bitDepth );
    int       numClasses    = ( ctbPars.typeIdc == SAO_TYPE_BO ? 4 : NUM_SAO_EO_CLASSES );
    int       k             = 0;
    int       offset[4];
    for( int i = 0; i < numClasses; i++ )
    {
      if( ctbPars.typeIdc != SAO_TYPE_BO && i == SAO_CLASS_EO_PLAIN )
      {
        continue;
      }
      int classIdx = ( ctbPars.typeIdc == SAO_TYPE_BO ? ( ctbPars.typeAuxInfo + i ) % NUM_SAO_BO_CLASSES : i );
      offset[k++]  = ctbPars.offset[classIdx];
    }

    // sao_offset_abs
    for( int i = 0; i < 4; i++ )
    {
      unsigned absOffset = ( offset[i] < 0 ? -offset[i] : offset[i] );
      unary_max_eqprob( absOffset, maxOffsetQVal );
    }

    // band offset mode
    if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      // sao_offset_sign
      for( int i = 0; i < 4; i++ )
      {
        if( offset[i] )
        {
          m_BinEncoder.encodeBinEP( (offset[i] < 0) );
        }
      }
      // sao_band_position
      m_BinEncoder.encodeBinsEP( ctbPars.typeAuxInfo, NUM_SAO_BO_CLASSES_LOG2 );
    }
    // edge offset mode
    else
    {
      if( isFirstCompOfChType )
      {
        // sao_eo_class_luma / sao_eo_class_chroma
        VTMCHECK( ctbPars.typeIdc - SAO_TYPE_START_EO < 0, "sao edge offset class is outside valid range" );
        m_BinEncoder.encodeBinsEP( ctbPars.typeIdc - SAO_TYPE_START_EO, NUM_SAO_EO_TYPES_LOG2 );
      }
    }
  }
}



//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    void  split_cu_flag     ( split, cs, partitioner )
//    void  split_cu_mode_mt  ( split, cs, partitioner )
//================================================================================

void CABACWriter::coding_tree(const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();
  const CodingUnit &cu        = *cs.getCU( currArea.blocks[partitioner.chType], partitioner.chType );

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if( pps.getUseDQP() && partitioner.currDepth <= pps.getMaxCuDQPDepth() )
  {
    cuCtx.isDQPCoded          = false;
  }
  if( cs.slice->getUseChromaQpAdj() && partitioner.currDepth <= pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
  }
  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if (CS::isDualITree(cs) && pPartitionerChroma != nullptr)
  {
    if (pps.getUseDQP() && pPartitionerChroma->currDepth <= pps.getMaxCuDQPDepth())
    {
      pCuCtxChroma->isDQPCoded = false;
    }
    if (cs.slice->getUseChromaQpAdj() && pPartitionerChroma->currDepth <= pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth())
    {
      pCuCtxChroma->isChromaQpAdjCoded = false;
    }
  }

#if JVET_M0421_SPLIT_SIG
  const PartSplit splitMode = CU::getSplitAtDepth( cu, partitioner.currDepth );

  split_cu_mode( splitMode, cs, partitioner );

  VTMCHECK( !partitioner.canSplit( splitMode, cs ), "The chosen split mode is invalid!" );

  if( splitMode != CU_DONT_SPLIT )
  {
#else
  const PartSplit implicitSplit = partitioner.getImplicitSplit( cs );

  // QT
  bool canQtSplit = partitioner.canSplit( CU_QUAD_SPLIT, cs );

  if( canQtSplit )
  {
    // split_cu_flag
    bool qtSplit = implicitSplit == CU_QUAD_SPLIT;

    if( !qtSplit && implicitSplit != CU_QUAD_SPLIT )
    {
      qtSplit = ( cu.qtDepth > partitioner.currQtDepth );
      split_cu_flag( qtSplit, cs, partitioner );
    }

    // quad-tree split
    if( qtSplit )
    {
#endif
      if (CS::isDualITree(cs) && pPartitionerChroma != nullptr && (partitioner.currArea().lwidth() >= 64 || partitioner.currArea().lheight() >= 64))
      {
        partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
        pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
        bool beContinue = true;
        bool lumaContinue = true;
        bool chromaContinue = true;

        while (beContinue)
        {
          if (partitioner.currArea().lwidth() > 64 || partitioner.currArea().lheight() > 64)
          {
            if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
            }
            lumaContinue = partitioner.nextPart(cs);
            chromaContinue = pPartitionerChroma->nextPart(cs);
            VTMCHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
          else
          {
            //dual tree coding under 64x64 block
            if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx);
            }
            lumaContinue = partitioner.nextPart(cs);
            if (cs.picture->blocks[pPartitionerChroma->chType].contains(pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
            {
              coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
            }
            chromaContinue = pPartitionerChroma->nextPart(cs);
            VTMCHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
        }
        partitioner.exitCurrSplit();
        pPartitionerChroma->exitCurrSplit();

      }
      else
      {
#if JVET_M0421_SPLIT_SIG
      partitioner.splitCurrArea( splitMode, cs );
#else
      partitioner.splitCurrArea( CU_QUAD_SPLIT, cs );
#endif

      do
      {
        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
      }
      return;
#if !JVET_M0421_SPLIT_SIG
    }
#endif
  }

#if !JVET_M0421_SPLIT_SIG
  {
    bool mtSplit = partitioner.canSplit( CU_MT_SPLIT, cs );

    if( mtSplit )
    {
      const PartSplit splitMode = CU::getSplitAtDepth( cu, partitioner.currDepth );

      split_cu_mode_mt( splitMode, cs, partitioner );

      if( splitMode != CU_DONT_SPLIT )
      {
        partitioner.splitCurrArea( splitMode, cs );
        do
        {
          if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
          {
            coding_tree( cs, partitioner, cuCtx );
          }
        } while( partitioner.nextPart( cs ) );

        partitioner.exitCurrSplit();
        return;
      }
    }
  }

#endif
  // Predict QP on start of quantization group
  if( pps.getUseDQP() && !cuCtx.isDQPCoded && CU::isQGStart( cu, partitioner ) )
  {
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }


  // coding unit
  coding_unit( cu, partitioner, cuCtx );

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
  DTRACE_BLOCK_REC_COND( ( !isEncoding() ), cs.picture->getRecoBuf( cu ), cu, cu.predMode );
}

#if JVET_M0421_SPLIT_SIG
void CABACWriter::split_cu_mode( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner )
{
  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl );

  const bool canSplit = canBh || canBv || canTh || canTv || canQt;
  const bool isNo     = split == CU_DONT_SPLIT;

  if( canNo && canSplit )
  {
    m_BinEncoder.encodeBin( !isNo, Ctx::SplitFlag( ctxSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, !isNo );

  if( isNo )
  {
    return;
  }

  const bool canBtt = canBh || canBv || canTh || canTv;
  const bool isQt   = split == CU_QUAD_SPLIT;

  if( canQt && canBtt )
  {
    m_BinEncoder.encodeBin( isQt, Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt );

  if( isQt )
  {
    return;
  }

  const bool canHor = canBh || canTh;
  const bool canVer = canBv || canTv;
  const bool  isVer = split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT;

  if( canVer && canHor )
  {
    m_BinEncoder.encodeBin( isVer, Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  const bool can12 = isVer ? canBv : canBh;
  const bool  is12 = isVer ? ( split == CU_VERT_SPLIT ) : ( split == CU_HORZ_SPLIT );

  if( can12 && can14 )
  {
    m_BinEncoder.encodeBin( is12, Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, split );
}
#else
void CABACWriter::split_cu_flag( bool split, const CodingStructure& cs, Partitioner& partitioner )
{
  unsigned maxQTDepth = ( g_aucLog2[cs.sps->getCTUSize()] - g_aucLog2[cs.sps->getMinQTSize(cs.slice->getSliceType(), partitioner.chType)] );
  if( partitioner.currDepth == maxQTDepth )
  {
    return;
  }
  unsigned  ctxId = DeriveCtx::CtxCUsplit( cs, partitioner );
  m_BinEncoder.encodeBin( (split), Ctx::SplitFlag(ctxId) );

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_flag() ctx=%d split=%d\n", ctxId, split ? 1 : 0 );
}

void CABACWriter::split_cu_mode_mt(const PartSplit split, const CodingStructure& cs, Partitioner& partitioner)
{
  unsigned ctxIdBT = DeriveCtx::CtxBTsplit( cs, partitioner );

  unsigned width   = partitioner.currArea().lumaSize().width;
  unsigned height  = partitioner.currArea().lumaSize().height;

#if REMOVE_BIN_DECISION_TREE
  unsigned btSCtxId = width == height ? 0 : ( width > height ? 1 : 2 );

  const bool canNo  = partitioner.canSplit( CU_DONT_SPLIT, cs );
  const bool canBh  = partitioner.canSplit( CU_HORZ_SPLIT, cs );
  const bool canBv  = partitioner.canSplit( CU_VERT_SPLIT, cs );
  const bool canTh  = partitioner.canSplit( CU_TRIH_SPLIT, cs );
  const bool canTv  = partitioner.canSplit( CU_TRIV_SPLIT, cs );

  const bool canSplit = canBh || canBv || canTh || canTv;
  const bool isNo     = split == CU_DONT_SPLIT;

  if( canNo && canSplit )
  {
    m_BinEncoder.encodeBin( !isNo, Ctx::BTSplitFlag( ctxIdBT ) );
  }

  if( isNo )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode_mt() ctx=%d split=%d\n", ctxIdBT, split );

    return;
  }

  const bool canHor = canBh || canTh;
  const bool canVer = canBv || canTv;
  const bool  isVer = split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT;

  if( canVer && canHor )
  {
    m_BinEncoder.encodeBin( isVer, Ctx::BTSplitFlag( 12 + btSCtxId ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  const bool can12 = isVer ? canBv : canBh;
  const bool  is12 = isVer ? ( split == CU_VERT_SPLIT ) : ( split == CU_HORZ_SPLIT );


  if( can12 && can14 )
  {
    m_BinEncoder.encodeBin( is12, Ctx::BTSplitFlag( 15 ) );
  }
#else
  DecisionTree dt( g_mtSplitDTT );

  dt.setAvail( DTT_SPLIT_BT_HORZ,  partitioner.canSplit( CU_HORZ_SPLIT, cs ) );
  dt.setAvail( DTT_SPLIT_BT_VERT,  partitioner.canSplit( CU_VERT_SPLIT, cs ) );

  dt.setAvail( DTT_SPLIT_TT_HORZ,  partitioner.canSplit( CU_TRIH_SPLIT, cs ) );
  dt.setAvail( DTT_SPLIT_TT_VERT,  partitioner.canSplit( CU_TRIV_SPLIT, cs ) );
  dt.setAvail( DTT_SPLIT_NO_SPLIT, partitioner.canSplit( CU_DONT_SPLIT, cs ) );

  unsigned btSCtxId = width == height ? 0 : ( width > height ? 1 : 2 );
  dt.setCtxId( DTT_SPLIT_DO_SPLIT_DECISION,   Ctx::BTSplitFlag( ctxIdBT ) );
  dt.setCtxId( DTT_SPLIT_HV_DECISION,         Ctx::BTSplitFlag( 12 + btSCtxId ) );

  dt.setCtxId( DTT_SPLIT_H_IS_BT_12_DECISION, Ctx::BTSplitFlag( 15 ) );
  dt.setCtxId( DTT_SPLIT_V_IS_BT_12_DECISION, Ctx::BTSplitFlag( 15 ) );


  encode_sparse_dt( dt, split == CU_DONT_SPLIT ? ( unsigned ) DTT_SPLIT_NO_SPLIT : ( unsigned ) split );
#endif

  DTRACE(g_trace_ctx, D_SYNTAX, "split_cu_mode_mt() ctx=%d split=%d\n", ctxIdBT, split);
}
#endif

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_transquant_bypass_flag ( cu )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  pcm_flag                  ( cu )
//    void  pcm_samples               ( tu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================

void CABACWriter::coding_unit( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  CodingStructure& cs = *cu.cs;
  cs.chType = partitioner.chType;
  // transquant bypass flag
  if( cs.pps->getTransquantBypassEnabledFlag() )
  {
    cu_transquant_bypass_flag( cu );
  }

  // skip flag
#if JVET_M0483_IBC
  if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag()) && cu.Y().valid())
#else
  if (!cs.slice->isIntra() && cu.Y().valid())
#endif
  {
    cu_skip_flag( cu );
  }


  // skip data
  if( cu.skip )
  {
    VTMCHECK( !cu.firstPU->mergeFlag, "Merge flag has to be on!" );
    PredictionUnit&   pu = *cu.firstPU;
    prediction_unit ( pu );
    end_of_ctu      ( cu, cuCtx );
    return;
  }

  // pcm samples
  if( CU::isIntra(cu) )
  {
    pcm_data( cu, partitioner );
    if( cu.ipcm )
    {
      end_of_ctu( cu, cuCtx );
      return;
    }
  }

  // prediction mode and partitioning data
  pred_mode ( cu );

  extend_ref_line(cu);

#if JVET_M0102_INTRA_SUBPARTITIONS
  isp_mode( cu );
#endif

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // end of cu
  end_of_ctu( cu, cuCtx );
}


void CABACWriter::cu_transquant_bypass_flag( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( (cu.transQuantBypass), Ctx::TransquantBypassFlag() );
}


void CABACWriter::cu_skip_flag( const CodingUnit& cu )
{
  unsigned ctxId = DeriveCtx::CtxSkipFlag( cu );

#if JVET_M0483_IBC
  if (cu.slice->isIntra() && cu.cs->slice->getSPS()->getIBCFlag())
  {
    m_BinEncoder.encodeBin((cu.skip), Ctx::SkipFlag(ctxId));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0);
    return;
  }
#endif

  m_BinEncoder.encodeBin( ( cu.skip ), Ctx::SkipFlag( ctxId ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0 );
#if JVET_M0483_IBC
  if (cu.skip && cu.cs->slice->getSPS()->getIBCFlag())
  {
    unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
    m_BinEncoder.encodeBin(CU::isIBC(cu) ? 1 : 0, Ctx::IBCFlag(ctxidx));
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);

    if (CU::isInter(cu))
    {
      m_BinEncoder.encodeBin(cu.mmvdSkip, Ctx::MmvdFlag(0));
      DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_cu_skip_flag() ctx=%d mmvd_skip=%d\n", 0, cu.mmvdSkip ? 1 : 0);
    }
  }
  if (cu.skip && !cu.cs->slice->getSPS()->getIBCFlag())
  {
    m_BinEncoder.encodeBin(cu.mmvdSkip, Ctx::MmvdFlag(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_cu_skip_flag() ctx=%d mmvd_skip=%d\n", 0, cu.mmvdSkip ? 1 : 0);
  }
#else
  if (cu.skip)
  {
    m_BinEncoder.encodeBin(cu.mmvdSkip, Ctx::MmvdFlag(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_cu_skip_flag() ctx=%d mmvd_skip=%d\n", 0, cu.mmvdSkip ? 1 : 0);
  }
#endif
}


void CABACWriter::pred_mode( const CodingUnit& cu )
{
#if JVET_M0483_IBC
  if (cu.cs->slice->getSPS()->getIBCFlag())
  {
#endif
#if JVET_M0483_IBC
    if (cu.cs->slice->isIntra())
    {
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
    }
    else
    {
#if JVET_M0502_PRED_MODE_CTX
      m_BinEncoder.encodeBin((CU::isIntra(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
#else
      m_BinEncoder.encodeBin((CU::isIntra(cu)), Ctx::PredMode());
#endif
      if (!CU::isIntra(cu))
      {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
      }
    }
  }
  else
  {
    if (cu.cs->slice->isIntra())
    {
      return;
    }
#if JVET_M0502_PRED_MODE_CTX
    m_BinEncoder.encodeBin((CU::isIntra(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
#else
    m_BinEncoder.encodeBin((CU::isIntra(cu)), Ctx::PredMode());
#endif
  }
#else
  if( cu.cs->slice->isIntra() )
  {
    return;
  }
#if JVET_M0502_PRED_MODE_CTX
  m_BinEncoder.encodeBin( ( CU::isIntra( cu ) ), Ctx::PredMode( DeriveCtx::CtxPredModeFlag( cu ) ) );
#else
  m_BinEncoder.encodeBin( ( CU::isIntra( cu ) ), Ctx::PredMode() );
#endif
#endif
}

void CABACWriter::pcm_data( const CodingUnit& cu, Partitioner& partitioner  )
{
  pcm_flag( cu, partitioner );
  if( cu.ipcm )
  {
    m_BinEncoder.pcmAlignBits();
    pcm_samples( *cu.firstTU );
  }
}

void CABACWriter::pcm_flag( const CodingUnit& cu, Partitioner& partitioner )
{
  const SPS& sps = *cu.cs->sps;
  if( !sps.getPCMEnabledFlag() || partitioner.currArea().lwidth() > (1 << sps.getPCMLog2MaxSize()) || partitioner.currArea().lwidth() < (1 << sps.getPCMLog2MinSize())
      || partitioner.currArea().lheight() > (1 << sps.getPCMLog2MaxSize()) || partitioner.currArea().lheight() < (1 << sps.getPCMLog2MinSize()) )
  {
    return;
  }
  m_BinEncoder.encodeBinTrm( cu.ipcm );
}


void CABACWriter::cu_pred_data( const CodingUnit& cu )
{
  if( CU::isIntra( cu ) )
  {
    intra_luma_pred_modes  ( cu );
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    prediction_unit( pu );
  }

  imv_mode   ( cu );
#if JVET_M0246_AFFINE_AMVR
  affine_amvr_mode( cu );
#endif

  cu_gbi_flag( cu );

}

void CABACWriter::cu_gbi_flag(const CodingUnit& cu)
{
  if(!CU::isGBiIdxCoded(cu))
  {
    return;
  }

  VTMCHECK(!(GBI_NUM > 1 && (GBI_NUM == 2 || (GBI_NUM & 0x01) == 1)), " !( GBI_NUM > 1 && ( GBI_NUM == 2 || ( GBI_NUM & 0x01 ) == 1 ) ) ");
  const uint8_t gbiCodingIdx = (uint8_t)g_GbiCodingOrder[CU::getValidGbiIdx(cu)];

  int ctxId = 0;

  int32_t numGBi = (cu.slice->getCheckLDC()) ? 5 : 3;

  m_BinEncoder.encodeBin((gbiCodingIdx == 0 ? 1 : 0), Ctx::GBiIdx(ctxId));

  if(numGBi > 2 && gbiCodingIdx != 0)
  {
    uint32_t prefixNumBits = numGBi - 2;
    uint32_t step = 1;
    uint8_t prefixSymbol = gbiCodingIdx;

    int ctxIdGBi = 4;
    uint8_t idx = 1;
    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
      if (prefixSymbol == idx)
      {
        m_BinEncoder.encodeBin(1, Ctx::GBiIdx(ctxIdGBi));
        break;
      }
      else
      {
        m_BinEncoder.encodeBin(0, Ctx::GBiIdx(ctxIdGBi));
        ctxIdGBi += step;
        idx += step;
      }
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_gbi_flag() gbi_idx=%d\n", cu.GBiIdx ? 1 : 0);
}

void CABACWriter::xWriteTruncBinCode(uint32_t symbol, uint32_t maxSymbol)
{
  int thresh;
  if (maxSymbol > 256)
  {
    int threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }

  int val = 1 << thresh;
  assert(val <= maxSymbol);
  assert((val << 1) > maxSymbol);
  assert(symbol < maxSymbol);
  int b = maxSymbol - val;
  assert(b < val);
  if (symbol < val - b)
  {
    m_BinEncoder.encodeBinsEP(symbol, thresh);
  }
  else
  {
    symbol += val - b;
    assert(symbol < (val << 1));
    assert((symbol >> 1) >= val - b);
    m_BinEncoder.encodeBinsEP(symbol, thresh + 1);
  }
}

void CABACWriter::extend_ref_line(const PredictionUnit& pu)
{
#if !ENABLE_JVET_L0283_MRL
  return;
#endif

  const CodingUnit& cu = *pu.cu;
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    return;
  }
  bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
  if (isFirstLineOfCtu)
  {
    return;
  }
  int multiRefIdx = pu.multiRefIdx;
  if (MRL_NUM_REF_LINES > 1)
  {
    m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
    if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
    {
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
      if (MRL_NUM_REF_LINES > 3 && multiRefIdx != MULTI_REF_LINE_IDX[1])
      {
        m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[2], Ctx::MultiRefLineIdx(2));
      }
    }
  }
}

void CABACWriter::extend_ref_line(const CodingUnit& cu)
{
#if !ENABLE_JVET_L0283_MRL
  return;
#endif

  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    return;
  }

  const int numBlocks = CU::getNumPUs(cu);
  const PredictionUnit* pu = cu.firstPU;

  for (int k = 0; k < numBlocks; k++)
  {
    bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
    if (isFirstLineOfCtu)
    {
      return;
    }
    int multiRefIdx = pu->multiRefIdx;
    if (MRL_NUM_REF_LINES > 1)
    {
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
        m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
        if (MRL_NUM_REF_LINES > 3 && multiRefIdx != MULTI_REF_LINE_IDX[1])
        {
          m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[2], Ctx::MultiRefLineIdx(2));
        }
      }

    }
    pu = pu->next;
  }
}

void CABACWriter::intra_luma_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  const int numMPMs   = NUM_MOST_PROBABLE_MODES;
  const int numBlocks = CU::getNumPUs( cu );
  unsigned  mpm_preds   [4][numMPMs];
  unsigned  mpm_idxs    [4];
  unsigned  ipred_modes [4];

  const PredictionUnit* pu = cu.firstPU;

  // prev_intra_luma_pred_flag
  for( int k = 0; k < numBlocks; k++ )
  {
    unsigned*  mpm_pred   = mpm_preds[k];
    unsigned&  mpm_idx    = mpm_idxs[k];
    unsigned&  ipred_mode = ipred_modes[k];

    PU::getIntraMPMs( *pu, mpm_pred );

    ipred_mode = pu->intraDir[0];
    mpm_idx    = numMPMs;
    for( unsigned idx = 0; idx < numMPMs; idx++ )
    {
      if( ipred_mode == mpm_pred[idx] )
      {
        mpm_idx = idx;
        break;
      }
    }
#if JVET_M0102_INTRA_SUBPARTITIONS
    if( pu->multiRefIdx || ( cu.ispMode && isLuma( cu.chType ) ) )
#else
    if (pu->multiRefIdx)
#endif
    {
      VTMCHECK(mpm_idx >= numMPMs, "use of non-MPM");
    }
    else
    {
      m_BinEncoder.encodeBin(mpm_idx < numMPMs, Ctx::IntraLumaMpmFlag());
    }

    pu = pu->next;
  }

  pu = cu.firstPU;

  // mpm_idx / rem_intra_luma_pred_mode
  for( int k = 0; k < numBlocks; k++ )
  {
    const unsigned& mpm_idx = mpm_idxs[k];
    if( mpm_idx < numMPMs )
    {
      {
        m_BinEncoder.encodeBinEP( mpm_idx > 0 );
        if( mpm_idx )
        {
          m_BinEncoder.encodeBinEP( mpm_idx > 1 );
        }
        if (mpm_idx > 1)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 2);
        }
        if (mpm_idx > 2)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 3);
        }
        if (mpm_idx > 3)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 4);
        }
      }
    }
    else
    {
      unsigned* mpm_pred   = mpm_preds[k];
      unsigned  ipred_mode = ipred_modes[k];

      // sorting of MPMs
      std::sort( mpm_pred, mpm_pred + numMPMs );

      {
        for (int idx = numMPMs - 1; idx >= 0; idx--)
        {
          if (ipred_mode > mpm_pred[idx])
          {
            ipred_mode--;
          }
        }
        VTMCHECK(ipred_mode >= 64, "Incorrect mode");
        xWriteTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
      }
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
    pu = pu->next;
  }
}


void CABACWriter::intra_luma_pred_mode( const PredictionUnit& pu )
{

  // prev_intra_luma_pred_flag
  const int numMPMs  = NUM_MOST_PROBABLE_MODES;
  unsigned  mpm_pred[numMPMs];

  PU::getIntraMPMs( pu, mpm_pred );

  unsigned ipred_mode = pu.intraDir[0];
  unsigned mpm_idx = numMPMs;

  for( int idx = 0; idx < numMPMs; idx++ )
  {
    if( ipred_mode == mpm_pred[idx] )
    {
      mpm_idx = idx;
      break;
    }
  }
#if JVET_M0102_INTRA_SUBPARTITIONS
  if( pu.multiRefIdx || ( pu.cu->ispMode && isLuma( pu.cu->chType ) ) )
#else
  if (pu.multiRefIdx)
#endif
  {
    VTMCHECK(mpm_idx >= numMPMs, "use of non-MPM");
  }
  else
  {
    m_BinEncoder.encodeBin(mpm_idx < numMPMs, Ctx::IntraLumaMpmFlag());
  }

  // mpm_idx / rem_intra_luma_pred_mode
  if( mpm_idx < numMPMs )
  {
    {
      m_BinEncoder.encodeBinEP( mpm_idx > 0 );
      if( mpm_idx )
      {
        m_BinEncoder.encodeBinEP( mpm_idx > 1 );
      }
      if (mpm_idx > 1)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 2);
      }
      if (mpm_idx > 2)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 3);
      }
      if (mpm_idx > 3)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 4);
      }
    }
  }
  else
  {
    std::sort( mpm_pred, mpm_pred + numMPMs );
    {
      for (int idx = numMPMs - 1; idx >= 0; idx--)
      {
        if (ipred_mode > mpm_pred[idx])
        {
          ipred_mode--;
        }
      }
      xWriteTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
    }
  }
}


void CABACWriter::intra_chroma_pred_modes( const CodingUnit& cu )
{
  if( cu.chromaFormat == CHROMA_400 || ( CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_LUMA ) )
  {
    return;
  }

  const PredictionUnit* pu = cu.firstPU;

  intra_chroma_pred_mode( *pu );
}

void CABACWriter::intra_chroma_lmc_mode( const PredictionUnit& pu )
{
  const unsigned intraDir = pu.intraDir[1];
    int lmModeList[10];
    int maxSymbol = PU::getLMSymbolList( pu, lmModeList );
    int symbol    = -1;
    for ( int k = 0; k < LM_SYMBOL_NUM; k++ )
    {
      if ( lmModeList[k] == intraDir || ( lmModeList[k] == -1 && intraDir < LM_CHROMA_IDX ) )
      {
        symbol = k;
        break;
      }
    }
    VTMCHECK( symbol < 0, "invalid symbol found" );

    unary_max_symbol(symbol, Ctx::IntraChromaPredMode(1), Ctx::IntraChromaPredMode(2), maxSymbol - 1);
}


void CABACWriter::intra_chroma_pred_mode( const PredictionUnit& pu )
{
  const unsigned intraDir = pu.intraDir[1];
  const bool     isDerivedMode = intraDir == DM_CHROMA_IDX;

  m_BinEncoder.encodeBin(isDerivedMode ? 0 : 1, Ctx::IntraChromaPredMode(0));

  if (isDerivedMode)
  {
    return;
  }

  // LM chroma mode
  if( pu.cs->sps->getUseLMChroma() )
  {
    intra_chroma_lmc_mode( pu );
    if ( PU::isLMCMode( intraDir ) )
    {
      return;
    }
  }

  // chroma candidate index
  unsigned chromaCandModes[ NUM_CHROMA_MODE ];
  PU::getIntraChromaCandModes( pu, chromaCandModes );

  int candId = 0;
  for ( ; candId < NUM_CHROMA_MODE; candId++ )
  {
    if( intraDir == chromaCandModes[ candId ] )
    {
      break;
    }
  }

  VTMCHECK( candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds" );
  VTMCHECK( chromaCandModes[ candId ] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path" );
  {
    m_BinEncoder.encodeBinsEP( candId, 2 );
  }
}


void CABACWriter::cu_residual( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
#if JVET_M0483_IBC
  if (!CU::isIntra(cu))
#else
  if( CU::isInter( cu ) )
#endif
  {
    PredictionUnit& pu = *cu.firstPU;
    if( !pu.mergeFlag )
    {
      rqt_root_cbf( cu );
    }
#if JVET_M0140_SBT
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }
#endif

    if( !cu.rootCbf )
    {
      return;
    }
  }


  ChromaCbfs chromaCbfs;
#if JVET_M0102_INTRA_SUBPARTITIONS
  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    TUIntraSubPartitioner subTuPartitioner( partitioner );
    transform_tree( *cu.cs, subTuPartitioner, cuCtx, chromaCbfs, CU::getISPType( cu, getFirstComponentOfChannel( partitioner.chType ) ), 0 );
  }
  else
  {
    transform_tree( *cu.cs, partitioner, cuCtx, chromaCbfs );
  }
#else
  transform_tree( *cu.cs, partitioner, cuCtx, chromaCbfs );
#endif
}

void CABACWriter::rqt_root_cbf( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( cu.rootCbf, Ctx::QtRootCbf() );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

#if JVET_M0140_SBT
void CABACWriter::sbt_mode( const CodingUnit& cu )
{
  uint8_t sbtAllowed = cu.checkAllowedSbt();
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();
  uint8_t sbtIdx = cu.getSbtIdx();
  uint8_t sbtPos = cu.getSbtPos();

  //bin - flag
  bool sbtFlag = cu.sbtInfo != 0;
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  m_BinEncoder.encodeBin( sbtFlag, Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  bool sbtQuadFlag = sbtIdx == SBT_HOR_QUAD || sbtIdx == SBT_VER_QUAD;
  bool sbtHorFlag = sbtIdx == SBT_HOR_HALF || sbtIdx == SBT_HOR_QUAD;
  bool sbtPosFlag = sbtPos == SBT_POS1;

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  //bin - type
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    m_BinEncoder.encodeBin( sbtQuadFlag, Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    assert( sbtQuadFlag == 0 );
  }

  //bin - dir
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    m_BinEncoder.encodeBin( sbtHorFlag, Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    assert( sbtHorFlag == ( ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow ) ) );
  }

  //bin - pos
  m_BinEncoder.encodeBin( sbtPosFlag, Ctx::SbtPosFlag( 0 ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );
}
#endif

void CABACWriter::end_of_ctu( const CodingUnit& cu, CUCtx& cuCtx )
{
  const Slice*  slice             = cu.cs->slice;
#if HEVC_TILES_WPP
  const TileMap& tileMap          = *cu.cs->picture->tileMap;
  const int     currentCTUTsAddr  = tileMap.getCtuRsToTsAddrMap( CU::getCtuAddr( cu ) );
#else
  const int     currentCTUTsAddr  = CU::getCtuAddr( cu );
#endif
  const bool    isLastSubCUOfCtu  = CU::isLastSubCUOfCtu( cu );

  if ( isLastSubCUOfCtu
    && ( !CS::isDualITree( *cu.cs ) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );

    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    // i.e. when the slice segment CurEnd CTU address is the current CTU address+1.
#if HEVC_DEPENDENT_SLICES
    if( slice->getSliceSegmentCurEndCtuTsAddr() != currentCTUTsAddr + 1 )
#else
    if(slice->getSliceCurEndCtuTsAddr() != currentCTUTsAddr + 1)
#endif
    {
      m_BinEncoder.encodeBinTrm( 0 );
    }
  }
}





//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu );
//    void  merge_flag      ( pu );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACWriter::prediction_unit( const PredictionUnit& pu )
{
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  VTMCHECK( pu.cacheUsed, "Processing a PU that should be in cache!" );
  VTMCHECK( pu.cu->cacheUsed, "Processing a CU that should be in cache!" );

#endif
  if( pu.cu->skip )
  {
    VTMCHECK( !pu.mergeFlag, "merge_flag must be true for skipped CUs" );
  }
  else
  {
    merge_flag( pu );
  }
  if( pu.mergeFlag )
  {
#if JVET_M0483_IBC
    if (CU::isIBC(*pu.cu))
    {
      merge_idx(pu);
      return;
    }
#endif
    subblock_merge_flag( *pu.cu );
    MHIntra_flag( pu );
    if ( pu.mhIntraFlag )
    {
      MHIntra_luma_pred_modes( *pu.cu );
    }
    triangle_mode( *pu.cu );
    if (pu.mmvdMergeFlag)
    {
      mmvd_merge_idx(pu);
    }
    else
    merge_idx    ( pu );
  }
#if JVET_M0483_IBC
  else if (CU::isIBC(*pu.cu))
  {
    ref_idx(pu, REF_PIC_LIST_0);
    mvd_coding(pu.mvd[REF_PIC_LIST_0], pu.cu->imv);
    mvp_flag(pu, REF_PIC_LIST_0);
  }
#endif
  else
  {
#if JVET_M0246_AFFINE_AMVR
    int8_t affineMvdShift = pu.cu->imv ? ( pu.cu->imv == 1 ? -1 : 1 ) : 0;
#endif
    inter_pred_idc( pu );
    affine_flag   ( *pu.cu );
#if JVET_M0444_SMVD
    smvd_mode( pu );
#endif
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      ref_idx     ( pu, REF_PIC_LIST_0 );
      if ( pu.cu->affine )
      {
#if JVET_M0246_AFFINE_AMVR
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_0][0], affineMvdShift );
        mvd_coding( pu.mvdAffi[REF_PIC_LIST_0][1], affineMvdShift );
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
          mvd_coding( pu.mvdAffi[REF_PIC_LIST_0][2], affineMvdShift );
        }
#else
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][0], 0);
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][1], 0);
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][2], 0);
        }
#endif
      }
      else
      {
        mvd_coding( pu.mvd[REF_PIC_LIST_0], pu.cu->imv );
      }
      mvp_flag    ( pu, REF_PIC_LIST_0 );
    }
    if( pu.interDir != 1 /* PRED_L0 */ )
    {
#if JVET_M0444_SMVD
      if ( pu.cu->smvdMode != 1 )
      {
#endif
      ref_idx     ( pu, REF_PIC_LIST_1 );
      if( !pu.cs->slice->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
      {
        if ( pu.cu->affine )
        {
#if JVET_M0246_AFFINE_AMVR
          mvd_coding( pu.mvdAffi[REF_PIC_LIST_1][0], affineMvdShift );
          mvd_coding( pu.mvdAffi[REF_PIC_LIST_1][1], affineMvdShift );
          if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
          {
            mvd_coding( pu.mvdAffi[REF_PIC_LIST_1][2], affineMvdShift );
          }
#else
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][0], 0);
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][1], 0);
          if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
          {
            mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][2], 0);
          }
#endif
        }
        else
        {
          mvd_coding( pu.mvd[REF_PIC_LIST_1], pu.cu->imv );
        }
      }
#if JVET_M0444_SMVD
      }
#endif
      mvp_flag    ( pu, REF_PIC_LIST_1 );
    }
  }
}

#if JVET_M0444_SMVD
void CABACWriter::smvd_mode( const PredictionUnit& pu )
{
  if ( pu.interDir != 3 || pu.cu->affine )
  {
    return;
  }

  if ( pu.cs->slice->getBiDirPred() == false )
  {
    return;
  }

  m_BinEncoder.encodeBin( pu.cu->smvdMode ? 1 : 0, Ctx::SmvdFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", pu.cu->smvdMode ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}
#endif

void CABACWriter::subblock_merge_flag( const CodingUnit& cu )
{
  if ( cu.firstPU->mergeFlag && (cu.firstPU->mmvdMergeFlag || cu.mmvdSkip) )
  {
    return;
  }

  if ( !cu.cs->slice->isIntra() && (cu.cs->sps->getUseAffine() || cu.cs->sps->getSBTMVPEnabledFlag()) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    m_BinEncoder.encodeBin( cu.affine, Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACWriter::affine_flag( const CodingUnit& cu )
{
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    m_BinEncoder.encodeBin( cu.affine, Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->getUseAffineType() )
    {
      unsigned ctxId = 0;
      m_BinEncoder.encodeBin( cu.affineType, Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
  }
}

void CABACWriter::merge_flag( const PredictionUnit& pu )
{
  m_BinEncoder.encodeBin( pu.mergeFlag, Ctx::MergeFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );

#if JVET_M0483_IBC
  if (pu.mergeFlag && CU::isIBC(*pu.cu))
  {
    return;
  }
#endif

  if (pu.mergeFlag)
  {
    m_BinEncoder.encodeBin(pu.mmvdMergeFlag, Ctx::MmvdFlag(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
  }
}

void CABACWriter::imv_mode( const CodingUnit& cu )
{
  const SPS *sps = cu.cs->sps;

  if( !sps->getAMVREnabledFlag() )
  {
    return;
  }
#if JVET_M0246_AFFINE_AMVR
  if ( cu.affine )
  {
    return;
  }
#endif

  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxIMVFlag( cu );
#if JVET_M0483_IBC
  if (CU::isIBC(cu) == false)
#else
  if (!(cu.firstPU->interDir == 1 && cu.cs->slice->getRefPic(REF_PIC_LIST_0, cu.firstPU->refIdx[REF_PIC_LIST_0])->getPOC() == cu.cs->slice->getPOC())) // the first bin of IMV flag does need to be signaled in IBC block
#endif
    m_BinEncoder.encodeBin( ( cu.imv > 0 ), Ctx::ImvFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 0), ctxId );

  if( sps->getAMVREnabledFlag() && cu.imv > 0 )
  {
    m_BinEncoder.encodeBin( ( cu.imv > 1 ), Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", ( cu.imv > 1 ), 3 );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}

#if JVET_M0246_AFFINE_AMVR
void CABACWriter::affine_amvr_mode( const CodingUnit& cu )
{
  const SPS* sps = cu.slice->getSPS();

  if( !sps->getAffineAmvrEnabledFlag() || !cu.affine )
  {
    return;
  }

  if ( !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  m_BinEncoder.encodeBin( ( cu.imv > 0 ), Ctx::ImvFlag( 4 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", ( cu.imv > 0 ), 4 );

  if( cu.imv > 0 )
  {
    m_BinEncoder.encodeBin( ( cu.imv > 1 ), Ctx::ImvFlag( 5 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", ( cu.imv > 1 ), 5 );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}
#endif

void CABACWriter::merge_idx( const PredictionUnit& pu )
{

  if ( pu.cu->affine )
  {
    int numCandminus1 = int( pu.cs->slice->getMaxNumAffineMergeCand() ) - 1;
    if ( numCandminus1 > 0 )
    {
      if ( pu.mergeIdx == 0 )
      {
        m_BinEncoder.encodeBin( 0, Ctx::AffMergeIdx() );
        DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
        return;
      }
      else
      {
#if !JVET_M0381_ONE_CTX_FOR_SUBBLOCK_MRG_IDX
        bool useExtCtx = pu.cs->sps->getSBTMVPEnabledFlag();
#endif
        m_BinEncoder.encodeBin( 1, Ctx::AffMergeIdx() );
        for ( unsigned idx = 1; idx < numCandminus1; idx++ )
        {
#if !JVET_M0381_ONE_CTX_FOR_SUBBLOCK_MRG_IDX
          if ( useExtCtx )
          {
            m_BinEncoder.encodeBin( pu.mergeIdx == idx ? 0 : 1, Ctx::AffMergeIdx( std::min<int>( idx, NUM_MERGE_IDX_EXT_CTX - 1 ) ) );
          }
          else
          {
#endif
            m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
#if !JVET_M0381_ONE_CTX_FOR_SUBBLOCK_MRG_IDX
          }
#endif
          if ( pu.mergeIdx == idx )
          {
            break;
          }
        }
      }
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
  }
  else
  {
    if( pu.cu->triangle )
    {
#if JVET_M0883_TRIANGLE_SIGNALING
      bool    splitDir = pu.triangleSplitDir;
      uint8_t candIdx0 = pu.triangleMergeIdx0;
      uint8_t candIdx1 = pu.triangleMergeIdx1;
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_split_dir=%d\n", splitDir );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_idx0=%d\n", candIdx0 );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_idx1=%d\n", candIdx1 );
      candIdx1 -= candIdx1 < candIdx0 ? 0 : 1;
      auto encodeOneIdx = [this](uint8_t mrgIdx, int numCandminus1)
      {
        if(mrgIdx == 0)
        {
          this->m_BinEncoder.encodeBin( 0, Ctx::MergeIdx() );
          return;
        }
        else
        {
          this->m_BinEncoder.encodeBin( 1, Ctx::MergeIdx() );
          for( unsigned idx = 1; idx < numCandminus1; idx++ )
          {
            this->m_BinEncoder.encodeBinEP( mrgIdx == idx ? 0 : 1 );
            if( mrgIdx == idx )
            {
              break;
            }
          }
        }
      };
      m_BinEncoder.encodeBinEP(splitDir);
      encodeOneIdx(candIdx0, TRIANGLE_MAX_NUM_UNI_CANDS - 1);
      encodeOneIdx(candIdx1, TRIANGLE_MAX_NUM_UNI_CANDS - 2);
#else
      if( pu.mergeIdx < 2 )
      {
        m_BinEncoder.encodeBin( 0, Ctx::TriangleIdx() );
        m_BinEncoder.encodeBinEP( pu.mergeIdx );
      }
      else
      {
        m_BinEncoder.encodeBin( 1, Ctx::TriangleIdx() );
        exp_golomb_eqprob( pu.mergeIdx - 2, 2 );
      }

      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() triangle_idx=%d\n", pu.mergeIdx );
#endif
      return;
    }
  int numCandminus1 = int( pu.cs->slice->getMaxNumMergeCand() ) - 1;
  if( numCandminus1 > 0 )
  {
    if( pu.mergeIdx == 0 )
    {
      m_BinEncoder.encodeBin( 0, Ctx::MergeIdx() );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 1, Ctx::MergeIdx() );
      for( unsigned idx = 1; idx < numCandminus1; idx++ )
      {
          m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
        if( pu.mergeIdx == idx )
        {
          break;
        }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
  }
}
void CABACWriter::mmvd_merge_idx(const PredictionUnit& pu)
{
  int var0, var1, var2;
  int mvpIdx = pu.mmvdMergeIdx;
  var0 = mvpIdx / MMVD_MAX_REFINE_NUM;
  var1 = (mvpIdx - (var0 * MMVD_MAX_REFINE_NUM)) / 4;
  var2 = mvpIdx - (var0 * MMVD_MAX_REFINE_NUM) - var1 * 4;

  int numCandminus1_base = MMVD_BASE_MV_NUM - 1;
  if (numCandminus1_base > 0)
  {
    if (var0 == 0)
    {
      m_BinEncoder.encodeBin(0, Ctx::MmvdMergeIdx());
    }
    else
    {
      m_BinEncoder.encodeBin(1, Ctx::MmvdMergeIdx());
      for (unsigned idx = 1; idx < numCandminus1_base; idx++)
      {
        m_BinEncoder.encodeBinEP(var0 == idx ? 0 : 1);
        if (var0 == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);

  int numCandminus1_step = MMVD_REFINE_STEP - 1;
  if (numCandminus1_step > 0)
  {
    if (var1 == 0)
    {
      m_BinEncoder.encodeBin(0, Ctx::MmvdStepMvpIdx());
    }
    else
    {
      m_BinEncoder.encodeBin(1, Ctx::MmvdStepMvpIdx());
      for (unsigned idx = 1; idx < numCandminus1_step; idx++)
      {
        m_BinEncoder.encodeBinEP(var1 == idx ? 0 : 1);
        if (var1 == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", var1);

  m_BinEncoder.encodeBinsEP(var2, 2);

  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
}
void CABACWriter::inter_pred_idc( const PredictionUnit& pu )
{
  if( !pu.cs->slice->isInterB() )
  {
    return;
  }
  if( !(PU::isBipredRestriction(pu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( pu.interDir == 3 )
    {
      m_BinEncoder.encodeBin( 1, Ctx::InterDir(ctxId) );
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 0, Ctx::InterDir(ctxId) );
    }
  }
  m_BinEncoder.encodeBin( ( pu.interDir == 2 ), Ctx::InterDir( 4 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=4 value=%d pos=(%d,%d)\n", pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
}


void CABACWriter::ref_idx( const PredictionUnit& pu, RefPicList eRefList )
{
#if JVET_M0444_SMVD
  if ( pu.cu->smvdMode )
  {
    VTMCHECK( pu.refIdx[eRefList] != pu.cs->slice->getSymRefIdx( eRefList ), "Invalid reference index!\n" );
    return;
  }
#endif

  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);

#if JVET_M0483_IBC
  if (eRefList == REF_PIC_LIST_0 && pu.cs->sps->getIBCFlag())
  {
    if (CU::isIBC(*pu.cu))
      return;
  }
#endif

  if( numRef <= 1 )
  {
    return;
  }
  int refIdx  = pu.refIdx[eRefList];
  m_BinEncoder.encodeBin( (refIdx > 0), Ctx::RefPic() );
  if( numRef <= 2 || refIdx == 0 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  m_BinEncoder.encodeBin( (refIdx > 1), Ctx::RefPic(1) );
  if( numRef <= 3 || refIdx == 1 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  for( int idx = 3; idx < numRef; idx++ )
  {
    if( refIdx > idx - 1 )
    {
      m_BinEncoder.encodeBinEP( 1 );
    }
    else
    {
      m_BinEncoder.encodeBinEP( 0 );
      break;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
}

void CABACWriter::mvp_flag( const PredictionUnit& pu, RefPicList eRefList )
{
  m_BinEncoder.encodeBin( pu.mvpIdx[eRefList], Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", pu.mvpIdx[eRefList], pu.lumaPos().x, pu.lumaPos().y );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, pu.mvpIdx[eRefList] );
}

void CABACWriter::MHIntra_flag(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseMHIntra())
  {
    VTMCHECK(pu.mhIntraFlag == true, "invalid MHIntra SPS");
    return;
  }
  if (pu.cu->skip)
  {
    VTMCHECK(pu.mhIntraFlag == true, "invalid MHIntra and skip");
    return;
  }
  if (pu.mmvdMergeFlag)
  {
    VTMCHECK(pu.mhIntraFlag == true, "invalid MHIntra and mmvd");
    return;
  }
  if (pu.cu->affine)
  {
    VTMCHECK(pu.mhIntraFlag == true, "invalid MHIntra and affine");
    return;
  }
  if (pu.cu->lwidth() * pu.cu->lheight() < 64 || pu.cu->lwidth() >= MAX_CU_SIZE || pu.cu->lheight() >= MAX_CU_SIZE)
  {
    VTMCHECK(pu.mhIntraFlag == true, "invalid MHIntra and blk");
    return;
  }
  m_BinEncoder.encodeBin(pu.mhIntraFlag, Ctx::MHIntraFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "MHIntra_flag() MHIntra=%d pos=(%d,%d) size=%dx%d\n", pu.mhIntraFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
}

void CABACWriter::MHIntra_luma_pred_modes(const CodingUnit& cu)
{
  if (!cu.Y().valid())
  {
    return;
  }

  const int numMPMs = 3;
  int      numBlocks = CU::getNumPUs(cu);
  unsigned mpm_idxs[4];
  unsigned pred_modes[4];

  const PredictionUnit* pu = cu.firstPU;

  unsigned mpm_pred[numMPMs];
  for (int k = 0; k < numBlocks; k++)
  {
    unsigned&  mpm_idx = mpm_idxs[k];
    unsigned&  pred_mode = pred_modes[k];

    PU::getMHIntraMPMs(*pu, mpm_pred);

    pred_mode = pu->intraDir[0];

    mpm_idx = numMPMs;

    for (int idx = 0; idx < numMPMs; idx++)
    {
      if (pred_mode == mpm_pred[idx])
      {
        mpm_idx = idx;
        break;
      }
    }
    if (PU::getNarrowShape(pu->lwidth(), pu->lheight()) == 0)
    {
      m_BinEncoder.encodeBin(mpm_idx < numMPMs, Ctx::MHIntraPredMode());
    }
    pu = pu->next;
  }

  pu = cu.firstPU;

  // mpm_idx / rem_intra_luma_pred_mode
  for (int k = 0; k < numBlocks; k++)
  {
    const unsigned& mpm_idx = mpm_idxs[k];
    if (mpm_idx < numMPMs)
    {
      m_BinEncoder.encodeBinEP(mpm_idx > 0);
      if (mpm_idx)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 1);
      }
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0]);
    pu = pu->next;
  }
}

void CABACWriter::triangle_mode( const CodingUnit& cu )
{
  if( !cu.cs->slice->getSPS()->getUseTriangle() || !cu.cs->slice->isInterB() || cu.lwidth() * cu.lheight() < TRIANGLE_MIN_SIZE || cu.affine )
  {
    return;
  }

#if JVET_M0118_M0185_TRIANGLE_FLAG_FIX
  if ( cu.firstPU->mmvdMergeFlag || cu.mmvdSkip )
  {
    return;
  }

  if ( cu.firstPU->mhIntraFlag )
  {
    return;
  }
#endif

  unsigned flag_idx     = DeriveCtx::CtxTriangleFlag( cu );

  m_BinEncoder.encodeBin( cu.triangle, Ctx::TriangleFlag(flag_idx) );

  DTRACE( g_trace_ctx, D_SYNTAX, "triangle_mode() triangle_mode=%d pos=(%d,%d) size: %dx%d\n", cu.triangle, cu.Y().x, cu.Y().y, cu.lumaSize().width, cu.lumaSize().height );
}

//================================================================================
//  clause 7.3.8.7
//--------------------------------------------------------------------------------
//    void  pcm_samples( tu )
//================================================================================

void CABACWriter::pcm_samples( const TransformUnit& tu )
{
  VTMCHECK( !tu.cu->ipcm, "pcm mode expected" );

  const SPS&        sps       = *tu.cu->cs->sps;

  const CodingStructure *cs = tu.cs;
  const ChannelType chType = tu.chType;

  ComponentID compStr = (CS::isDualITree(*cs) && !isLuma(chType)) ? COMPONENT_Cb: COMPONENT_Y;
  ComponentID compEnd = (CS::isDualITree(*cs) && isLuma(chType)) ? COMPONENT_Y : COMPONENT_Cr;
  for( ComponentID compID = compStr; compID <= compEnd; compID = ComponentID(compID+1) )
  {
    const CPelBuf   samples     = tu.getPcmbuf( compID );
    const unsigned  sampleBits  = sps.getPCMBitDepth( toChannelType(compID) );
    for( unsigned y = 0; y < samples.height; y++ )
    {
      for( unsigned x = 0; x < samples.width; x++ )
      {
        m_BinEncoder.encodeBinsPCM( samples.at(x, y), sampleBits );
      }
    }
  }
  m_BinEncoder.restart();
}



//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( split, depth )
//    bool  cbf_comp            ( cbf, area, depth )
//================================================================================

#if JVET_M0102_INTRA_SUBPARTITIONS
void CABACWriter::transform_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, ChromaCbfs& chromaCbfs, const PartSplit ispType, const int subTuIdx )
#else
void CABACWriter::transform_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
#endif
{
#if JVET_M0140_SBT
  ChromaCbfs chromaCbfsLastDepth;
  chromaCbfsLastDepth.Cb              = chromaCbfs.Cb;
  chromaCbfsLastDepth.Cr              = chromaCbfs.Cr;
#endif
  const UnitArea&       area          = partitioner.currArea();
#if JVET_M0102_INTRA_SUBPARTITIONS
        int             subTuCounter  = subTuIdx;
  const TransformUnit&  tu            = *cs.getTU( area.blocks[partitioner.chType].pos(), partitioner.chType, subTuIdx );
#else
  const TransformUnit&  tu            = *cs.getTU( area.blocks[partitioner.chType].pos(), partitioner.chType );
#endif
  const CodingUnit&     cu            = *tu.cu;
  const unsigned        trDepth       = partitioner.currTrDepth;
  const bool            split         = ( tu.depth > trDepth );
#if JVET_M0102_INTRA_SUBPARTITIONS
  const bool            chromaCbfISP  = area.blocks[COMPONENT_Cb].valid() && cu.ispMode && !split;
#endif

  // split_transform_flag
  if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
  {
    VTMCHECK( !split, "transform split implied" );
  }
#if JVET_M0140_SBT
  else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
  {
    VTMCHECK( !split, "transform split implied - sbt" );
  }
#endif
  else
#if JVET_M0102_INTRA_SUBPARTITIONS
  VTMCHECK( split && !cu.ispMode, "transform split not allowed with QTBT" );
#else
  VTMCHECK( split, "transform split not allowed with QTBT" );
#endif


  // cbf_cb & cbf_cr
#if JVET_M0102_INTRA_SUBPARTITIONS
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !CS::isDualITree( cs ) || partitioner.chType == CHANNEL_TYPE_CHROMA ) && ( !cu.ispMode || chromaCbfISP ) )
#else
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !CS::isDualITree( cs ) || partitioner.chType == CHANNEL_TYPE_CHROMA ) )
#endif
  {
    {
#if JVET_M0102_INTRA_SUBPARTITIONS
      unsigned cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
      if( trDepth == 0 || chromaCbfs.Cb || chromaCbfISP )
      {
        chromaCbfs.Cb = TU::getCbfAtDepth( tu, COMPONENT_Cb, trDepth );
#if JVET_M0140_SBT
        if( !( cu.sbtInfo && trDepth == 1 ) )
#endif
        cbf_comp( cs, chromaCbfs.Cb, area.blocks[COMPONENT_Cb], cbfDepth );
      }
      else
      {
        VTMCHECK( TU::getCbfAtDepth( tu, COMPONENT_Cb, cbfDepth ) != chromaCbfs.Cb, "incorrect Cb cbf" );
      }

      if( trDepth == 0 || chromaCbfs.Cr || chromaCbfISP )
      {
        chromaCbfs.Cr = TU::getCbfAtDepth( tu, COMPONENT_Cr, trDepth );
#if JVET_M0140_SBT
        if( !( cu.sbtInfo && trDepth == 1 ) )
#endif
        cbf_comp( cs, chromaCbfs.Cr, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb );
      }
      else
      {
        VTMCHECK( TU::getCbfAtDepth( tu, COMPONENT_Cr, cbfDepth ) != chromaCbfs.Cr, "incorrect Cr cbf" );
      }
#else
      if( trDepth == 0 || chromaCbfs.Cb )
      {
        chromaCbfs.Cb = TU::getCbfAtDepth( tu, COMPONENT_Cb, trDepth );
#if JVET_M0140_SBT
        if( !( cu.sbtInfo && trDepth == 1 ) )
#endif
        cbf_comp( cs, chromaCbfs.Cb, area.blocks[COMPONENT_Cb], trDepth );
      }
      else
      {
        VTMCHECK( TU::getCbfAtDepth( tu, COMPONENT_Cb, trDepth ) != chromaCbfs.Cb, "incorrect Cb cbf" );
      }

      if( trDepth == 0 || chromaCbfs.Cr )
      {
        chromaCbfs.Cr = TU::getCbfAtDepth( tu, COMPONENT_Cr,   trDepth );
#if JVET_M0140_SBT
        if( !( cu.sbtInfo && trDepth == 1 ) )
#endif
        cbf_comp( cs, chromaCbfs.Cr, area.blocks[COMPONENT_Cr], trDepth, chromaCbfs.Cb );
      }
      else
      {
        VTMCHECK( TU::getCbfAtDepth( tu, COMPONENT_Cr, trDepth ) != chromaCbfs.Cr, "incorrect Cr cbf" );
      }
#endif
    }
  }
  else if( CS::isDualITree( cs ) )
  {
    chromaCbfs = ChromaCbfs( false );
  }

  if( split )
  {
    if( area.chromaFormat != CHROMA_400 )
    {
      chromaCbfs.Cb        = TU::getCbfAtDepth( tu, COMPONENT_Cb,  trDepth );
      chromaCbfs.Cr        = TU::getCbfAtDepth( tu, COMPONENT_Cr,  trDepth );
    }
#if !JVET_M0464_UNI_MTS
#if JVET_M0102_INTRA_SUBPARTITIONS
    if ( trDepth == 0 && !cu.ispMode ) emt_cu_flag( cu );
#else
    if( trDepth == 0 ) emt_cu_flag( cu );
#endif
#endif

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
#if ENABLE_TRACING
      const CompArea &tuArea = partitioner.currArea().blocks[partitioner.chType];
      DTRACE( g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n", partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height );

#endif
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
#if JVET_M0102_INTRA_SUBPARTITIONS
    else if( cu.ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
#endif
#if JVET_M0140_SBT
    else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
    {
      partitioner.splitCurrArea( PartSplit( cu.getSbtTuSplit() ), cs );
    }
#endif
    else
      THROW( "Implicit TU split not available" );

    do
    {
      ChromaCbfs subChromaCbfs = chromaCbfs;
#if JVET_M0102_INTRA_SUBPARTITIONS
      transform_tree( cs, partitioner, cuCtx, subChromaCbfs, ispType, subTuCounter );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
#else
      transform_tree( cs, partitioner, cuCtx, subChromaCbfs );
#endif
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

    if( !isChroma( partitioner.chType ) )
    {
      if( !CU::isIntra( cu ) && trDepth == 0 && !chromaCbfs.sigChroma( area.chromaFormat ) )
      {
        VTMCHECK( !TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), "Luma cbf must be true for inter units with no chroma coeffs" );
      }
#if JVET_M0140_SBT
      else if( cu.sbtInfo && tu.noResidual )
      {
        VTMCHECK( TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), "Luma cbf must be false for inter sbt no-residual tu" );
      }
      else if( cu.sbtInfo && !chromaCbfsLastDepth.sigChroma( area.chromaFormat ) )
      {
        assert( !tu.noResidual );
        VTMCHECK( !TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), "Luma cbf must be true for inter sbt residual tu" );
      }
#endif
      else
      {
#if JVET_M0102_INTRA_SUBPARTITIONS
        bool previousCbf       = false;
        bool rootCbfSoFar      = false;
        bool lastCbfIsInferred = false;
        if( cu.ispMode )
        {
          uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> g_aucLog2[tu.lheight()] : cu.lwidth() >> g_aucLog2[tu.lwidth()];
          if( subTuCounter == nTus - 1 )
          {
            TransformUnit* tuPointer = cu.firstTU;
            for( int tuIdx = 0; tuIdx < subTuCounter; tuIdx++ )
            {
              rootCbfSoFar |= TU::getCbfAtDepth( *tuPointer, COMPONENT_Y, trDepth );
              tuPointer = tuPointer->next;
            }
            if( !rootCbfSoFar )
            {
              lastCbfIsInferred = true;
            }
          }
          if( !lastCbfIsInferred )
          {
            previousCbf = TU::getPrevTuCbfAtDepth( tu, COMPONENT_Y, partitioner.currTrDepth );
          }
        }
        if( !lastCbfIsInferred )
        {
          cbf_comp( cs, TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), tu.Y(), trDepth, previousCbf, cu.ispMode );
        }
#else
        cbf_comp( cs, TU::getCbfAtDepth( tu, COMPONENT_Y, trDepth ), tu.Y(), trDepth );
#endif
      }
    }

#if !JVET_M0464_UNI_MTS
    if( trDepth == 0 && TU::getCbfAtDepth( tu, COMPONENT_Y, 0 ) ) emt_cu_flag( cu );
#endif

    transform_unit( tu, cuCtx, chromaCbfs );
  }
}

#if JVET_M0102_INTRA_SUBPARTITIONS
void CABACWriter::cbf_comp( const CodingStructure& cs, bool cbf, const CompArea& area, unsigned depth, const bool prevCbCbf, const bool useISP )
{
  const unsigned  ctxId   = DeriveCtx::CtxQtCbf( area.compID, depth, prevCbCbf, useISP && isLuma(area.compID) );
#else
void CABACWriter::cbf_comp( const CodingStructure& cs, bool cbf, const CompArea& area, unsigned depth, const bool prevCbCbf )
{
  const unsigned  ctxId   = DeriveCtx::CtxQtCbf( area.compID, depth, prevCbCbf );
#endif
  const CtxSet&   ctxSet  = Ctx::QtCbf[ area.compID ];

  m_BinEncoder.encodeBin( cbf, ctxSet( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
}





//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================
#if JVET_M0246_AFFINE_AMVR
void CABACWriter::mvd_coding( const Mv &rMvd, int8_t imv )
#else
void CABACWriter::mvd_coding( const Mv &rMvd, uint8_t imv )
#endif
{
  int       horMvd = rMvd.getHor();
  int       verMvd = rMvd.getVer();
#if JVET_M0246_AFFINE_AMVR
  if ( imv > 0 )
#else
  if( imv )
#endif
  {
    VTMCHECK( (horMvd % 4) != 0 && (verMvd % 4) != 0, "IMV: MVD is not a multiple of 4" );
    horMvd >>= 2;
    verMvd >>= 2;
    if( imv == 2 )//IMV_4PEL
    {
      VTMCHECK( (horMvd % 4) != 0 && (verMvd % 4) != 0, "IMV: MVD is not a multiple of 8" );
      horMvd >>= 2;
      verMvd >>= 2;
    }
  }
  unsigned  horAbs  = unsigned( horMvd < 0 ? -horMvd : horMvd );
  unsigned  verAbs  = unsigned( verMvd < 0 ? -verMvd : verMvd );


  // abs_mvd_greater0_flag[ 0 | 1 ]
  m_BinEncoder.encodeBin( (horAbs > 0), Ctx::Mvd() );
  m_BinEncoder.encodeBin( (verAbs > 0), Ctx::Mvd() );

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    m_BinEncoder.encodeBin( (horAbs > 1), Ctx::Mvd(1) );
  }
  if( verAbs > 0 )
  {
    m_BinEncoder.encodeBin( (verAbs > 1), Ctx::Mvd(1) );
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    if( horAbs > 1 )
    {
      exp_golomb_eqprob( horAbs - 2, 1 );
    }
    m_BinEncoder.encodeBinEP( (horMvd < 0) );
  }
  if( verAbs > 0 )
  {
    if( verAbs > 1 )
    {
      exp_golomb_eqprob( verAbs - 2, 1 );
    }
    m_BinEncoder.encodeBinEP( (verMvd < 0) );
  }
}




//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================

void CABACWriter::transform_unit( const TransformUnit& tu, CUCtx& cuCtx, ChromaCbfs& chromaCbfs )
{
  CodingUnit& cu        = *tu.cu;
  bool        lumaOnly  = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  bool        cbf[3]    = { TU::getCbf( tu, COMPONENT_Y ), chromaCbfs.Cb, chromaCbfs.Cr };
  bool        cbfLuma   = ( cbf[ COMPONENT_Y ] != 0 );
  bool        cbfChroma = false;

#if JVET_M0102_INTRA_SUBPARTITIONS
  if( !lumaOnly )
#else
  if( cu.chromaFormat != CHROMA_400 )
#endif
  {
    if( tu.blocks[COMPONENT_Cb].valid() )
    {
      cbf   [ COMPONENT_Cb  ] = TU::getCbf( tu, COMPONENT_Cb );
      cbf   [ COMPONENT_Cr  ] = TU::getCbf( tu, COMPONENT_Cr );
    }
    cbfChroma = ( cbf[ COMPONENT_Cb ] || cbf[ COMPONENT_Cr ] );
  }
  if( cbfLuma || cbfChroma )
  {
    if( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      if (!CS::isDualITree(*tu.cs) || isLuma(tu.chType))
      {
        cu_qp_delta(cu, cuCtx.qp, cu.qp);
        cuCtx.qp = cu.qp;
        cuCtx.isDQPCoded = true;
      }
    }
    if( cu.cs->slice->getUseChromaQpAdj() && cbfChroma && !cu.transQuantBypass && !cuCtx.isChromaQpAdjCoded )
    {
      cu_chroma_qp_offset( cu );
      cuCtx.isChromaQpAdjCoded = true;
    }
    if( cbfLuma )
    {
      residual_coding( tu, COMPONENT_Y );
    }
    if( !lumaOnly )
    {
      for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
      {
        if( TU::hasCrossCompPredInfo( tu, compID ) )
        {
          cross_comp_pred( tu, compID );
        }
        if( cbf[ compID ] )
        {
          residual_coding( tu, compID );
        }
      }
    }
  }
}

void CABACWriter::cu_qp_delta( const CodingUnit& cu, int predQP, const int8_t qp )
{
  VTMCHECK(!( predQP != std::numeric_limits<int>::max()), "Unspecified error");
  int       DQp         = qp - predQP;
  int       qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
  DQp                   = ( DQp + (MAX_QP + 1) + (MAX_QP + 1) / 2 + qpBdOffsetY + (qpBdOffsetY / 2)) % ((MAX_QP + 1) + qpBdOffsetY) - (MAX_QP + 1) / 2 - (qpBdOffsetY / 2);
  unsigned  absDQP      = unsigned( DQp < 0 ? -DQp : DQp );
  unsigned  unaryDQP    = std::min<unsigned>( absDQP, CU_DQP_TU_CMAX );

  unary_max_symbol( unaryDQP, Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( absDQP >= CU_DQP_TU_CMAX )
  {
    exp_golomb_eqprob( absDQP - CU_DQP_TU_CMAX, CU_DQP_EG_k );
  }
  if( absDQP > 0 )
  {
    m_BinEncoder.encodeBinEP( DQp < 0 );
  }

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACWriter::cu_chroma_qp_offset( const CodingUnit& cu )
{
  // cu_chroma_qp_offset_flag
  unsigned qpAdj = cu.chromaQpAdj;
  if( qpAdj == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ChromaQpAdjFlag() );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ChromaQpAdjFlag() );
    int length = cu.cs->pps->getPpsRangeExtension().getChromaQpOffsetListLen();
    if( length > 1 )
    {
      unary_max_symbol( qpAdj-1, Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
    }
  }
}





//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    void        transform_skip_flag     ( tu, compID )
//    void        explicit_rdpcm_mode     ( tu, compID )
//    void        last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACWriter::residual_coding( const TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu = *tu.cu;
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );

  // code transform skip and explicit rdpcm mode
#if JVET_M0464_UNI_MTS
  mts_coding         ( tu, compID );
#else
  transform_skip_flag( tu, compID );
#endif
  explicit_rdpcm_mode( tu, compID );

#if HEVC_USE_SIGN_HIDING
  // determine sign hiding
  bool signHiding  = ( cu.cs->slice->getSignDataHidingEnabledFlag() && !cu.transQuantBypass && tu.rdpcm[compID] == RDPCM_OFF );
#if JVET_M0464_UNI_MTS
  if(  signHiding && CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && tu.mtsIdx==1 )
#else
  if(  signHiding && CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && tu.transformSkip[compID] )
#endif
  {
    const ChannelType chType    = toChannelType( compID );
    const unsigned    intraMode = PU::getFinalIntraMode( *cu.cs->getPU( tu.blocks[compID].pos(), chType ), chType );
    if( intraMode == HOR_IDX || intraMode == VER_IDX )
    {
      signHiding = false;
    }
  }
#endif

  // init coeff coding context
#if HEVC_USE_SIGN_HIDING
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
#else
  CoeffCodingContext  cctx    ( tu, compID );
#endif
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;
#if !JVET_M0464_UNI_MTS
  unsigned            numSig  = 0;
#endif

  // determine and set last coeff position and sig group flags
  int                      scanPosLast = -1;
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      scanPosLast = scanPos;
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }
  VTMCHECK( scanPosLast < 0, "Coefficient coding called for empty TU" );
  cctx.setScanPosLast(scanPosLast);

  // code last coeff position
#if JVET_M0297_32PT_MTS_ZERO_OUT
  last_sig_coeff( cctx, tu, compID );
#else
  last_sig_coeff( cctx );
#endif

  // code subblocks
  const int stateTab  = ( tu.cs->slice->getDepQuantEnabledFlag() ? 32040 : 0 );
  int       state     = 0;
#if !JVET_M0464_UNI_MTS
  bool useEmt = ( cu.cs->sps->getUseIntraEMT() && cu.predMode == MODE_INTRA ) || ( cu.cs->sps->getUseInterEMT() && cu.predMode != MODE_INTRA );
  useEmt = useEmt && isLuma(compID);
#if JVET_M0102_INTRA_SUBPARTITIONS
  useEmt = useEmt && !cu.ispMode;
#endif
#endif

  for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock       ( subSetId, sigGroupFlags[subSetId] );
#if JVET_M0297_32PT_MTS_ZERO_OUT
#if JVET_M0140_SBT
#if JVET_M0464_UNI_MTS
    if( ( tu.mtsIdx > 1 || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
    if( ( ( tu.cu->emtFlag && !tu.transformSkip[ compID ] ) || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
#else
#if JVET_M0464_UNI_MTS
    if( tu.mtsIdx > 1 && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
    if( tu.cu->emtFlag && !tu.transformSkip[ compID ] && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
#endif
    {
      if( ( tu.blocks[ compID ].height == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) )
       || ( tu.blocks[ compID ].width  == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth()  ) ) )
      {
        continue;
      }
    }
#endif
    residual_coding_subblock( cctx, coeff, stateTab, state );

#if !JVET_M0464_UNI_MTS
    if (useEmt)
    {
      numSig += cctx.emtNumSigCoeff();
      cctx.setEmtNumSigCoeff( 0 );
    }
#endif
  }


#if !JVET_M0464_UNI_MTS
  if( useEmt && !tu.transformSkip[compID] && compID == COMPONENT_Y && tu.cu->emtFlag )
  {
    if( CU::isIntra( *tu.cu ) )
    {
      emt_tu_index(tu);
    }
    else
    {
      emt_tu_index( tu );
    }
  }
#endif
}

#if JVET_M0464_UNI_MTS
void CABACWriter::mts_coding( const TransformUnit& tu, ComponentID compID )
{
  const CodingUnit  &cu = *tu.cu;
  const bool  tsAllowed = TU::isTSAllowed ( tu, compID );
  const bool mtsAllowed = TU::isMTSAllowed( tu, compID );

  if( !mtsAllowed && !tsAllowed ) return;

  int symbol  = 0;
  int ctxIdx  = 0;

  if( tsAllowed )
  {
    symbol = 1 - ( tu.mtsIdx == 1 ? 1 : 0 );
    ctxIdx = 6;
    m_BinEncoder.encodeBin( symbol, Ctx::MTSIndex( ctxIdx ) );
  }

  if( tu.mtsIdx != 1 )
  {
    if( mtsAllowed )
    {
      symbol = tu.mtsIdx != 0 ? 1 : 0;
      ctxIdx = std::min( (int)cu.qtDepth, 5 );
      m_BinEncoder.encodeBin( symbol, Ctx::MTSIndex( ctxIdx ) );

      if( symbol )
      {
        ctxIdx = 7;
        for( int i = 0; i < 3; i++, ctxIdx++ )
        {
          symbol = tu.mtsIdx > i + 2 ? 1 : 0;
          m_BinEncoder.encodeBin( symbol, Ctx::MTSIndex( ctxIdx ) );

          if( !symbol )
          {
            break;
          }
        }
      }
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "mts_coding() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), tu.mtsIdx );
}
#else
void CABACWriter::transform_skip_flag( const TransformUnit& tu, ComponentID compID )
{
#if JVET_M0102_INTRA_SUBPARTITIONS
  if (!tu.cu->cs->pps->getUseTransformSkip() || tu.cu->transQuantBypass || !TU::hasTransformSkipFlag(*tu.cs, tu.blocks[compID]) || (isLuma(compID) && tu.cu->emtFlag) || (tu.cu->ispMode && isLuma(compID)))
#else
  if( !tu.cu->cs->pps->getUseTransformSkip() || tu.cu->transQuantBypass || !TU::hasTransformSkipFlag( *tu.cs, tu.blocks[compID] ) || ( isLuma( compID ) && tu.cu->emtFlag ) )
#endif
  {
    return;
  }
  m_BinEncoder.encodeBin( tu.transformSkip[compID], Ctx::TransformSkipFlag(toChannelType(compID)) );

  DTRACE( g_trace_ctx, D_SYNTAX, "transform_skip_flag() etype=%d pos=(%d,%d) trSkip=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, (int)tu.transformSkip[compID] );
}

void CABACWriter::emt_tu_index( const TransformUnit& tu )
{
  int maxSizeEmtIntra = EMT_INTRA_MAX_CU_WITH_QTBT;
  int maxSizeEmtInter = EMT_INTER_MAX_CU_WITH_QTBT;

  if( CU::isIntra( *tu.cu ) && ( tu.cu->Y().width <= maxSizeEmtIntra ) && ( tu.cu->Y().height <= maxSizeEmtIntra ) )
  {
    uint8_t trIdx = tu.emtIdx;
    m_BinEncoder.encodeBin( ( trIdx & 1 ) ? 1 : 0, Ctx::EMTTuIndex( 0 ) );
    m_BinEncoder.encodeBin( ( trIdx / 2 ) ? 1 : 0, Ctx::EMTTuIndex( 1 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_tu_index() etype=%d pos=(%d,%d) emtTrIdx=%d\n", COMPONENT_Y, tu.blocks[COMPONENT_Y].x, tu.blocks[COMPONENT_Y].y, ( int ) tu.emtIdx );
  }
  if( !CU::isIntra( *tu.cu ) && ( tu.cu->Y().width <= maxSizeEmtInter ) && ( tu.cu->Y().height <= maxSizeEmtInter ) )
  {
    uint8_t trIdx = tu.emtIdx;
    m_BinEncoder.encodeBin( ( trIdx & 1 ) ? 1 : 0, Ctx::EMTTuIndex( 2 ) );
    m_BinEncoder.encodeBin( ( trIdx / 2 ) ? 1 : 0, Ctx::EMTTuIndex( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_tu_index() etype=%d pos=(%d,%d) emtTrIdx=%d\n", COMPONENT_Y, tu.blocks[COMPONENT_Y].x, tu.blocks[COMPONENT_Y].y, ( int ) tu.emtIdx );
  }
}

//void CABACWriter::emt_cu_flag(const CodingUnit& cu, uint32_t depth, bool codeCuFlag, const int tuWidth,const int tuHeight)
void CABACWriter::emt_cu_flag( const CodingUnit& cu )
{
#if JVET_M0102_INTRA_SUBPARTITIONS
  if ( CU::isIntra( cu ) && cu.ispMode )
  {
    return;
  }
#endif
  const CodingStructure& cs = *cu.cs;

#if JVET_M0483_IBC
  if (!((cs.sps->getUseIntraEMT() && CU::isIntra(cu)) || (cs.sps->getUseInterEMT() && !CU::isIntra(cu))) || isChroma(cu.chType))
#else
  if( !( ( cs.sps->getUseIntraEMT() && CU::isIntra( cu ) ) || ( cs.sps->getUseInterEMT() && CU::isInter( cu ) ) ) || isChroma( cu.chType ) )
#endif
  {
    return;
  }

  unsigned depth          = cu.qtDepth;
  const unsigned cuWidth  = cu.lwidth();
  const unsigned cuHeight = cu.lheight();

  if( depth >= NUM_EMT_CU_FLAG_CTX )
  {
    depth = NUM_EMT_CU_FLAG_CTX - 1;
  }
  int maxSizeEmtIntra = EMT_INTRA_MAX_CU_WITH_QTBT;
  int maxSizeEmtInter = EMT_INTER_MAX_CU_WITH_QTBT;

  const unsigned maxSizeEmt = CU::isIntra( cu ) ? maxSizeEmtIntra : maxSizeEmtInter;

  if( cuWidth <= maxSizeEmt && cuHeight <= maxSizeEmt )
  {
    m_BinEncoder.encodeBin( cu.emtFlag, Ctx::EMTCuFlag( depth ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "emt_cu_flag() etype=%d pos=(%d,%d) emtCuFlag=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.emtFlag );
  }
}
#endif

#if JVET_M0102_INTRA_SUBPARTITIONS
void CABACWriter::isp_mode( const CodingUnit& cu )
{
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx )
  {
    VTMCHECK( cu.ispMode != NOT_INTRA_SUBPARTITIONS, "error: cu.intraSubPartitions != 0" );
    return;
  }
  const ISPType allowedSplits = CU::canUseISPSplit( cu, getFirstComponentOfChannel( cu.chType ) );
  if( allowedSplits == NOT_INTRA_SUBPARTITIONS ) return;

  if( cu.ispMode == NOT_INTRA_SUBPARTITIONS )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ISPMode( 0 ) );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ISPMode( 0 ) );

    if( allowedSplits == CAN_USE_VER_AND_HORL_SPLITS )
    {
      m_BinEncoder.encodeBin( cu.ispMode - 1, Ctx::ISPMode( 1 ) );
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subPartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}
#endif

void CABACWriter::explicit_rdpcm_mode( const TransformUnit& tu, ComponentID compID )
{
  const CodingUnit& cu = *tu.cu;
#if JVET_M0464_UNI_MTS
  if( !CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && ( tu.mtsIdx==1 || cu.transQuantBypass ) )
#else
  if( !CU::isIntra(cu) && CU::isRDPCMEnabled(cu) && ( tu.transformSkip[compID] || cu.transQuantBypass ) )
#endif
  {
    ChannelType chType = toChannelType( compID );
    switch( tu.rdpcm[compID] )
    {
    case RDPCM_VER:
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmFlag(chType) );
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmDir (chType) );
      break;
    case RDPCM_HOR:
      m_BinEncoder.encodeBin( 1, Ctx::RdpcmFlag(chType) );
      m_BinEncoder.encodeBin( 0, Ctx::RdpcmDir (chType) );
      break;
    default: // RDPCM_OFF
      m_BinEncoder.encodeBin( 0, Ctx::RdpcmFlag(chType) );
    }
  }
}


#if JVET_M0297_32PT_MTS_ZERO_OUT
void CABACWriter::last_sig_coeff( CoeffCodingContext& cctx, const TransformUnit& tu, ComponentID compID )
#else
void CABACWriter::last_sig_coeff( CoeffCodingContext& cctx )
#endif
{
  unsigned blkPos = cctx.blockPos( cctx.scanPosLast() );
  unsigned posX, posY;
#if HEVC_USE_MDCS
  if( cctx.scanType() == SCAN_VER )
  {
    posX  = blkPos / cctx.width();
    posY  = blkPos - ( posX * cctx.width() );
  }
  else
#endif
  {
    posY  = blkPos / cctx.width();
    posX  = blkPos - ( posY * cctx.width() );
  }

  unsigned CtxLast;
  unsigned GroupIdxX = g_uiGroupIdx[ posX ];
  unsigned GroupIdxY = g_uiGroupIdx[ posY ];

#if JVET_M0297_32PT_MTS_ZERO_OUT
  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

#if JVET_M0140_SBT
#if JVET_M0464_UNI_MTS
  if( ( tu.mtsIdx > 1 || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
  if( ( ( tu.cu->emtFlag && !tu.transformSkip[ compID ] ) || ( tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
#else
#if JVET_M0464_UNI_MTS
  if( tu.mtsIdx > 1 && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
  if( tu.cu->emtFlag && !tu.transformSkip[ compID ] && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
#endif
  {
    maxLastPosX = ( tu.blocks[compID].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[compID].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }
#endif

  for( CtxLast = 0; CtxLast < GroupIdxX; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastXCtxId( CtxLast ) );
  }
#if JVET_M0297_32PT_MTS_ZERO_OUT
  if( GroupIdxX < maxLastPosX )
#else
  if( GroupIdxX < cctx.maxLastPosX() )
#endif
  {
    m_BinEncoder.encodeBin( 0, cctx.lastXCtxId( CtxLast ) );
  }
  for( CtxLast = 0; CtxLast < GroupIdxY; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastYCtxId( CtxLast ) );
  }
#if JVET_M0297_32PT_MTS_ZERO_OUT
  if( GroupIdxY < maxLastPosY )
#else
  if( GroupIdxY < cctx.maxLastPosY() )
#endif
  {
    m_BinEncoder.encodeBin( 0, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxX > 3 )
  {
    posX -= g_uiMinInGroup[ GroupIdxX ];
    for (int i = ( ( GroupIdxX - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posX >> i ) & 1 );
    }
  }
  if( GroupIdxY > 3 )
  {
    posY -= g_uiMinInGroup[ GroupIdxY ];
    for ( int i = ( ( GroupIdxY - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posY >> i ) & 1 );
    }
  }
}



void CABACWriter::residual_coding_subblock( CoeffCodingContext& cctx, const TCoeff* coeff, const int stateTransTable, int& state )
{
  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if( !isLast && cctx.isNotFirst() )
  {
    if( cctx.isSigGroup() )
    {
      m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId() );
    }
    else
    {
      m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  uint8_t   ctxOffset[16];

  //===== encode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
#if HEVC_USE_SIGN_HIDING
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
#endif
  int       remAbsLevel   = -1;
  int       numNonZero    =  0;
  unsigned  signPattern   =  0;
  bool      is2x2subblock = ( cctx.log2CGSize() == 2 );
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  int       remRegBins    = ( is2x2subblock ? MAX_NUM_REG_BINS_2x2SUBBLOCK : MAX_NUM_REG_BINS_4x4SUBBLOCK );
#else
  int       remGt2Bins    = ( is2x2subblock ? MAX_NUM_GT2_BINS_2x2SUBBLOCK : MAX_NUM_GT2_BINS_4x4SUBBLOCK );
  int       remRegBins    = ( is2x2subblock ? MAX_NUM_REG_BINS_2x2SUBBLOCK : MAX_NUM_REG_BINS_4x4SUBBLOCK ) - remGt2Bins;
#endif
  int       firstPosMode2 = minSubPos - 1;
#if !JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  int       firstPosMode1 = minSubPos - 1;
#endif

#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
#else
  for( ; nextSigPos >= minSubPos && remRegBins >= 3; nextSigPos-- )
#endif
  {
    TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned  sigFlag    = ( Coeff != 0 );
    if( numNonZero || nextSigPos != inferSigPos )
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbs( nextSigPos, coeff, state );
      m_BinEncoder.encodeBin( sigFlag, sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      remRegBins--;
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff  = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff            = cctx.ctxOffsetAbs();
      numNonZero++;
#if HEVC_USE_SIGN_HIDING
      firstNZPos  = nextSigPos;
      lastNZPos   = std::max<int>( lastNZPos, nextSigPos );
#endif
      remAbsLevel = abs( Coeff ) - 1;

      if( nextSigPos != cctx.scanPosLast() ) signPattern <<= 1;
      if( Coeff < 0 )                        signPattern++;

      unsigned gt1 = !!remAbsLevel;
      m_BinEncoder.encodeBin( gt1, cctx.greater1CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1, cctx.greater1CtxIdAbs(ctxOff) );
      remRegBins--;

      if( gt1 )
      {
        remAbsLevel  -= 1;
        m_BinEncoder.encodeBin( remAbsLevel&1, cctx.parityCtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", remAbsLevel&1, cctx.parityCtxIdAbs( ctxOff ) );
        remAbsLevel >>= 1;

        remRegBins--;
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
        unsigned gt2 = !!remAbsLevel;
        m_BinEncoder.encodeBin(gt2, cctx.greater2CtxIdAbs(ctxOff));
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2, cctx.greater2CtxIdAbs(ctxOff));
        remRegBins--;
#else
        if( remGt2Bins && !--remGt2Bins )
        {
          firstPosMode1 = nextSigPos - 1;
        }
#endif
      }
    }

    state = ( stateTransTable >> ((state<<2)+((Coeff&1)<<1)) ) & 3;
  }
  firstPosMode2 = nextSigPos;
#if !JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  firstPosMode1 = ( firstPosMode1 > firstPosMode2 ? firstPosMode1 : firstPosMode2 );
#endif


#if !JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  //===== 2nd PASS: gt2 =====
  for( int scanPos = firstSigPos; scanPos > firstPosMode1; scanPos-- )
  {
    unsigned absLevel = abs( coeff[ cctx.blockPos( scanPos ) ] );
    if( absLevel >= 2 )
    {
      uint8_t& ctxOff = ctxOffset[ scanPos - minSubPos ];
      unsigned gt2    = ( absLevel >= 4 );
      m_BinEncoder.encodeBin( gt2, cctx.greater2CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2, cctx.greater2CtxIdAbs(ctxOff) );
    }
  }
#endif

#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  //===== 2nd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
#else
  //===== 3rd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode1; scanPos-- )
#endif
  {
    unsigned absLevel = abs( coeff[ cctx.blockPos( scanPos ) ] );
    if( absLevel >= 4 )
    {
      unsigned rem      = ( absLevel - 4 ) >> 1;
      m_BinEncoder.encodeRemAbsEP( rem, ricePar, cctx.extPrec(), cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
      if( ricePar < 3 && rem > (3<<ricePar)-1 )
      {
        ricePar++;
      }
    }
  }
#if !JVET_M0173_MOVE_GT2_TO_FIRST_PASS
  for( int scanPos = firstPosMode1; scanPos > firstPosMode2; scanPos-- )
  {
    unsigned absLevel = abs( coeff[ cctx.blockPos( scanPos ) ] );
    if( absLevel >= 2 )
    {
      unsigned rem      = ( absLevel - 2 ) >> 1;
      m_BinEncoder.encodeRemAbsEP( rem, ricePar, cctx.extPrec(), cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
      if( ricePar < 3 && rem > (3<<ricePar)-1 )
      {
        ricePar++;
      }
    }
  }
#endif

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
    TCoeff    Coeff     = coeff[ cctx.blockPos( scanPos ) ];
    unsigned  absLevel  = abs( Coeff );
    int       sumAll    = cctx.templateAbsSum(scanPos, coeff);
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0[std::max(0, state - 1)][sumAll];
    unsigned  rem       = ( absLevel == 0 ? pos0 : absLevel <= pos0 ? absLevel-1 : absLevel );
    m_BinEncoder.encodeRemAbsEP( rem, rice, cctx.extPrec(), cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    state = ( stateTransTable >> ((state<<2)+((absLevel&1)<<1)) ) & 3;
    if( absLevel )
    {
      numNonZero++;
#if HEVC_USE_SIGN_HIDING
      lastNZPos   = std::max<int>( lastNZPos, scanPos );
#endif
      signPattern <<= 1;
      if( Coeff < 0 ) signPattern++;
    }
  }

  //===== encode sign's =====
#if HEVC_USE_SIGN_HIDING
  unsigned numSigns = numNonZero;
  if( cctx.hideSign( firstNZPos, lastNZPos ) )
  {
    numSigns    --;
    signPattern >>= 1;
  }
  m_BinEncoder.encodeBinsEP( signPattern, numSigns );
#else
  m_BinEncoder.encodeBinsEP( signPattern, numNonZero );
#endif
#if !JVET_M0464_UNI_MTS
  cctx.setEmtNumSigCoeff(numNonZero);
#endif
}






//================================================================================
//  clause 7.3.8.12
//--------------------------------------------------------------------------------
//    void  cross_comp_pred( tu, compID )
//================================================================================

void CABACWriter::cross_comp_pred( const TransformUnit& tu, ComponentID compID )
{
  VTMCHECK(!( !isLuma( compID ) ), "Unspecified error");
  signed char alpha   = tu.compAlpha[compID];
  unsigned    ctxBase = ( compID == COMPONENT_Cr ? 5 : 0 );
  if( alpha == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::CrossCompPred( ctxBase ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
    return;
  }

  static const unsigned log2AbsAlphaMinus1Table[8] = { 0, 1, 1, 2, 2, 2, 3, 3 };
  unsigned sign = ( alpha < 0 );
  if( sign )
  {
    alpha = -alpha;
  }
  VTMCHECK(!( alpha <= 8 ), "Unspecified error");
  m_BinEncoder.encodeBin( 1, Ctx::CrossCompPred(ctxBase) );
  if( alpha > 1)
  {
     m_BinEncoder.encodeBin( 1, Ctx::CrossCompPred(ctxBase+1) );
     unary_max_symbol( log2AbsAlphaMinus1Table[alpha-1]-1, Ctx::CrossCompPred(ctxBase+2), Ctx::CrossCompPred(ctxBase+3), 2 );
  }
  else
  {
     m_BinEncoder.encodeBin( 0, Ctx::CrossCompPred(ctxBase+1) );
  }
  m_BinEncoder.encodeBin( sign, Ctx::CrossCompPred(ctxBase+4) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cross_comp_pred() etype=%d pos=(%d,%d) alpha=%d\n", compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.compAlpha[compID] );
}




//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    void  unary_max_symbol  ( symbol, ctxId0, ctxIdN, maxSymbol )
//    void  unary_max_eqprob  ( symbol,                 maxSymbol )
//    void  exp_golomb_eqprob ( symbol, count )
//================================================================================

void CABACWriter::unary_max_symbol( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol )
{
  VTMCHECK( symbol > maxSymbol, "symbol > maxSymbol" );
  const unsigned totalBinsToWrite = std::min( symbol + 1, maxSymbol );
  for( unsigned binsWritten = 0; binsWritten < totalBinsToWrite; ++binsWritten )
  {
    const unsigned nextBin = symbol > binsWritten;
    m_BinEncoder.encodeBin( nextBin, binsWritten == 0 ? ctxId0 : ctxIdN );
  }
}


void CABACWriter::unary_max_eqprob( unsigned symbol, unsigned maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return;
  }
  bool     codeLast = ( maxSymbol > symbol );
  unsigned bins     = 0;
  unsigned numBins  = 0;
  while( symbol-- )
  {
    bins   <<= 1;
    bins   ++;
    numBins++;
  }
  if( codeLast )
  {
    bins  <<= 1;
    numBins++;
  }
  VTMCHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}


void CABACWriter::exp_golomb_eqprob( unsigned symbol, unsigned count )
{
  unsigned bins    = 0;
  unsigned numBins = 0;
  while( symbol >= (unsigned)(1<<count) )
  {
    bins <<= 1;
    bins++;
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  bins <<= 1;
  numBins++;
  bins = (bins << count) | symbol;
  numBins += count;
  VTMCHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}

#if !REMOVE_BIN_DECISION_TREE
void CABACWriter::encode_sparse_dt( DecisionTree& dt, unsigned toCodeId )
{
  // propagate the sparsity information from end-nodes to intermediate nodes
  dt.reduce();

  unsigned depth  = dt.dtt.depth;
  unsigned offset = 0;

  const unsigned encElPos = dt.dtt.mapping[toCodeId];

  while( dt.dtt.hasSub[offset] )
  {
    CHECKD( depth == 0, "Depth is '0' for a decision node in a decision tree" );

    const unsigned posRight = offset + 1;
    const unsigned posLeft  = offset + ( 1u << depth );

    const bool isLeft = encElPos >= posLeft;

    if( dt.isAvail[posRight] && dt.isAvail[posLeft] )
    {
      // encode the decision as both sub-paths are available
      const unsigned ctxId = dt.ctxId[offset];

      if( ctxId > 0 )
      {
        DTRACE( g_trace_ctx, D_DECISIONTREE, "Decision coding using context %d\n", ctxId - 1 );
        m_BinEncoder.encodeBin( isLeft ? 0 : 1, ctxId - 1 );
      }
      else
      {
        DTRACE( g_trace_ctx, D_DECISIONTREE, "Decision coding as an EP bin\n" );
        m_BinEncoder.encodeBinEP( isLeft ? 0 : 1 );
      }
    }

    DTRACE( g_trace_ctx, D_DECISIONTREE, "Following the tree to the %s sub-node\n", isLeft ? "left" : "right" );

    offset = isLeft ? posLeft : posRight;
    depth--;
  }

  CHECKD( offset != encElPos,             "Encoded a different element than assigned" );
  CHECKD( dt.dtt.ids[offset] != toCodeId, "Encoded a different element than assigned" );
  CHECKD( dt.isAvail[offset] == false,    "The encoded element is not available" );
  DTRACE( g_trace_ctx, D_DECISIONTREE,    "Found an end-node of the tree\n" );
  return;
}

#endif
void CABACWriter::codeAlfCtuEnableFlags( CodingStructure& cs, ChannelType channel, AlfSliceParam* alfParam)
{
  if( isLuma( channel ) )
  {
    if (alfParam->enabledFlag[COMPONENT_Y])
      codeAlfCtuEnableFlags( cs, COMPONENT_Y, alfParam );
  }
  else
  {
    if (alfParam->enabledFlag[COMPONENT_Cb])
      codeAlfCtuEnableFlags( cs, COMPONENT_Cb, alfParam );
    if (alfParam->enabledFlag[COMPONENT_Cr])
      codeAlfCtuEnableFlags( cs, COMPONENT_Cr, alfParam );
  }
}
void CABACWriter::codeAlfCtuEnableFlags( CodingStructure& cs, ComponentID compID, AlfSliceParam* alfParam)
{
  uint32_t numCTUs = cs.pcv->sizeInCtus;

  for( int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++ )
  {
    codeAlfCtuEnableFlag( cs, ctuIdx, compID, alfParam );
  }
}

void CABACWriter::codeAlfCtuEnableFlag( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, AlfSliceParam* alfParam)
{
  AlfSliceParam& alfSliceParam = alfParam ? (*alfParam) : cs.slice->getAlfSliceParam();

  if( cs.sps->getALFEnabledFlag() && alfSliceParam.enabledFlag[compIdx] )
  {
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
    const uint32_t          curSliceIdx = cs.slice->getIndependentSliceIdx();
#if HEVC_TILES_WPP
    const uint32_t          curTileIdx = cs.picture->tileMap->getTileIdxMap( pos );
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), curSliceIdx, curTileIdx, CH_L ) ? true : false;
#else
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), curSliceIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), curSliceIdx, CH_L ) ? true : false;
#endif

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    if( alfSliceParam.enabledFlag[compIdx] )
    {
      uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );
      int ctx = 0;
      ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
      ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;
      m_BinEncoder.encodeBin( ctbAlfFlag[ctuRsAddr], Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );
    }
  }
}

#if ADCNN
void CABACWriter::codeCnnlfCtuEnableFlags(CodingStructure& cs, ChannelType channel, CnnlfSliceParam* cnnlfParam)
{
	if (isLuma(channel))
	{
		if (cnnlfParam->enabledFlag[COMPONENT_Y])
			codeCnnlfCtuEnableFlags(cs, COMPONENT_Y, cnnlfParam);
	}
	else
	{
		if (cnnlfParam->enabledFlag[COMPONENT_Cb])
			codeCnnlfCtuEnableFlags(cs, COMPONENT_Cb, cnnlfParam);
		if (cnnlfParam->enabledFlag[COMPONENT_Cr])
			codeCnnlfCtuEnableFlags(cs, COMPONENT_Cr, cnnlfParam);
	}
}
void CABACWriter::codeCnnlfCtuEnableFlags(CodingStructure& cs, ComponentID compID, CnnlfSliceParam* cnnlfParam)
{
	uint32_t numCTUs = cs.pcv->sizeInCtus;

	for (int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++)
	{
		codeCnnlfCtuEnableFlag(cs, ctuIdx, compID, cnnlfParam);
	}
}

void CABACWriter::codeCnnlfCtuEnableFlag(CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, CnnlfSliceParam* cnnlfParam)
{
	CnnlfSliceParam& cnnlfSliceParam = cnnlfParam ? (*cnnlfParam) : cs.slice->getCnnlfSliceParam();
	if (cs.sps->getCNNLFEnabledFlag() && cnnlfSliceParam.enabledFlag[compIdx])
	{
		const PreCalcValues& pcv = *cs.pcv;
		int                 frame_width_in_ctus = pcv.widthInCtus;
		int                 ry = ctuRsAddr / frame_width_in_ctus;
		int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
		const Position      pos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);
		const uint32_t          curSliceIdx = cs.slice->getIndependentSliceIdx();
#if HEVC_TILES_WPP
		const uint32_t          curTileIdx = cs.picture->tileMap->getTileIdxMap(pos);
		bool                leftAvail = cs.getCURestricted(pos.offset(-(int)pcv.maxCUWidth, 0), curSliceIdx, curTileIdx, CH_L) ? true : false;
		bool                aboveAvail = cs.getCURestricted(pos.offset(0, -(int)pcv.maxCUHeight), curSliceIdx, curTileIdx, CH_L) ? true : false;
#else
		bool                leftAvail = cs.getCURestricted(pos.offset(-(int)pcv.maxCUWidth, 0), curSliceIdx, CH_L) ? true : false;
		bool                aboveAvail = cs.getCURestricted(pos.offset(0, -(int)pcv.maxCUHeight), curSliceIdx, CH_L) ? true : false;
#endif
		int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
		int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;
		if (cnnlfSliceParam.enabledFlag[compIdx])
		{
			uint8_t* ctbCnnlfFlag = cs.slice->getPic()->getCnnlfCtuEnableFlag(compIdx);
			int ctx = 0;
			ctx += leftCTUAddr > -1 ? (ctbCnnlfFlag[leftCTUAddr] ? 1 : 0) : 0;
			ctx += aboveCTUAddr > -1 ? (ctbCnnlfFlag[aboveCTUAddr] ? 1 : 0) : 0;
			m_BinEncoder.encodeBin(ctbCnnlfFlag[ctuRsAddr], Ctx::ctbCnnlfFlag(compIdx * 3 + ctx));
		}
	}
}
#endif
//! \}
