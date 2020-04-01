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

/** \file     LoopFilter.cpp
    \brief    deblocking filter
*/

#include "LoopFilter.h"
#include "Slice.h"
#include "Mv.h"
#include "Unit.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

//#define   EDGE_VER    0
//#define   EDGE_HOR    1

#define DEBLOCK_SMALLEST_BLOCK  8


#define DEFAULT_INTRA_TC_OFFSET 2 ///< Default intra TC offset

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint8_t LoopFilter::sm_tcTable[MAX_QP + 1 + DEFAULT_INTRA_TC_OFFSET] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,6,7,8,9,10,11,13,14,16,18,20,22,25
  , 28, 31, 35, 39, 44, 50, 56, 63, 70, 79, 88, 99
};

const uint8_t LoopFilter::sm_betaTable[MAX_QP + 1] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64
  , 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88
};

inline static uint32_t getRasterIdx(const Position& pos, const PreCalcValues& pcv)
{
  return ( ( pos.x & pcv.maxCUWidthMask ) >> pcv.minCUWidthLog2 ) + ( ( pos.y & pcv.maxCUHeightMask ) >> pcv.minCUHeightLog2 ) * pcv.partsInCtuWidth;
}

// ====================================================================================================================
// utility functions
// ====================================================================================================================

#if HEVC_TILES_WPP
static bool isAvailableLeft( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction, const bool bEnforceTileRestriction )
{
  return ( ( !bEnforceSliceRestriction || CU::isSameSlice( cu, cu2 ) ) && ( !bEnforceTileRestriction || CU::isSameTile( cu, cu2 ) ) );
}

static bool isAvailableAbove( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction, const bool bEnforceTileRestriction )
{
  return ( !bEnforceSliceRestriction || CU::isSameSlice( cu, cu2 ) ) && ( !bEnforceTileRestriction || CU::isSameTile( cu, cu2 ) );
}
#else
static bool isAvailable( const CodingUnit& cu, const CodingUnit& cu2, const bool bEnforceSliceRestriction )
{
  return ( !bEnforceSliceRestriction || CU::isSameSlice( cu, cu2 ) );
}
#endif


// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

LoopFilter::LoopFilter()
{
}

LoopFilter::~LoopFilter()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
void LoopFilter::create( const unsigned uiMaxCUDepth )
{
  destroy();
  const unsigned numPartitions = 1 << ( uiMaxCUDepth << 1 );
  for( int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_aapucBS       [edgeDir].resize( numPartitions );
    m_aapbEdgeFilter[edgeDir].resize( numPartitions );
  }
}

void LoopFilter::destroy()
{
  for( int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_aapucBS       [edgeDir].clear();
    m_aapbEdgeFilter[edgeDir].clear();
  }
}

/**
 - call deblocking function for every CU
 .
 \param  pcPic   picture class (Pic) pointer
 */
void LoopFilter::loopFilterPic( CodingStructure& cs
                                )
{
  const PreCalcValues& pcv = *cs.pcv;

  DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", cs.slice->getPOC() ) ) );
#if ENABLE_TRACING
  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );
      DTRACE    ( g_trace_ctx, D_CRC, "CTU %d %d", ctuArea.Y().x, ctuArea.Y().y );
      DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );
    }
  }
#endif

  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      memset( m_aapucBS       [EDGE_VER].data(), 0,     m_aapucBS       [EDGE_VER].byte_size() );
      memset( m_aapbEdgeFilter[EDGE_VER].data(), false, m_aapbEdgeFilter[EDGE_VER].byte_size() );

      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );

      // CU-based deblocking
      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L ), CH_L ) )
      {
        xDeblockCU( currCU, EDGE_VER );
      }

      if( CS::isDualITree( cs ) )
      {
        memset( m_aapucBS       [EDGE_VER].data(), 0,     m_aapucBS       [EDGE_VER].byte_size() );
        memset( m_aapbEdgeFilter[EDGE_VER].data(), false, m_aapbEdgeFilter[EDGE_VER].byte_size() );

        for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_C ), CH_C ) )
        {
          xDeblockCU( currCU, EDGE_VER );
        }
      }
    }
  }

  // Vertical filtering
  for( int y = 0; y < pcv.heightInCtus; y++ )
  {
    for( int x = 0; x < pcv.widthInCtus; x++ )
    {
      memset( m_aapucBS       [EDGE_HOR].data(), 0,     m_aapucBS       [EDGE_HOR].byte_size() );
      memset( m_aapbEdgeFilter[EDGE_HOR].data(), false, m_aapbEdgeFilter[EDGE_HOR].byte_size() );

      const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth ) );

      // CU-based deblocking
      for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L ), CH_L ) )
      {
        xDeblockCU( currCU, EDGE_HOR );
      }

      if( CS::isDualITree( cs ) )
      {
        memset( m_aapucBS       [EDGE_HOR].data(), 0,     m_aapucBS       [EDGE_HOR].byte_size() );
        memset( m_aapbEdgeFilter[EDGE_HOR].data(), false, m_aapbEdgeFilter[EDGE_HOR].byte_size() );

        for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_C ), CH_C ) )
        {
          xDeblockCU( currCU, EDGE_HOR );
        }
      }
    }
  }

  DTRACE_PIC_COMP(D_REC_CB_LUMA_LF,   cs, cs.getRecoBuf(), COMPONENT_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMPONENT_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMPONENT_Cr);

  DTRACE    ( g_trace_ctx, D_CRC, "LoopFilter" );
  DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
}


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 Deblocking filter process in CU-based (the same function as conventional's)

 \param cu               the CU to be deblocked
 \param edgeDir          the direction of the edge in block boundary (horizontal/vertical), which is added newly
*/
void LoopFilter::xDeblockCU( CodingUnit& cu, const DeblockEdgeDir edgeDir )
{
  const PreCalcValues& pcv = *cu.cs->pcv;
  const Area area          = cu.Y().valid() ? cu.Y() : Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) );

  xSetLoopfilterParam( cu );
#if  JVET_M0471_LONG_DEBLOCKING_FILTERS
  bool implicitTU = false;
#endif
  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    const Area& areaTu    = cu.Y().valid() ? currTU.block( COMPONENT_Y ) : area;
#if  JVET_M0471_LONG_DEBLOCKING_FILTERS
    const bool xOff = currTU.blocks[cu.chType].x != cu.blocks[cu.chType].x;
    const bool yOff = currTU.blocks[cu.chType].y != cu.blocks[cu.chType].y;
    if ((yOff != 0) && (edgeDir == EDGE_HOR))
    {
      implicitTU = true;
    }
    if ((xOff != 0) && (edgeDir == EDGE_VER))
    {
      implicitTU = true;
    }
#endif
    xSetEdgefilterMultiple( cu, EDGE_VER, areaTu, m_stLFCUParam.internalEdge );
    xSetEdgefilterMultiple( cu, EDGE_HOR, areaTu, m_stLFCUParam.internalEdge );
  }

  bool mvSubBlocks = false;
  int subBlockSize = 8;
  for( auto &currPU : CU::traversePUs( cu ) )
  {
    const Area& areaPu = cu.Y().valid() ? currPU.block( COMPONENT_Y ) : area;
    const bool xOff    = currPU.blocks[cu.chType].x != cu.blocks[cu.chType].x;
    const bool yOff    = currPU.blocks[cu.chType].y != cu.blocks[cu.chType].y;

    xSetEdgefilterMultiple( cu, EDGE_VER, areaPu, (xOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.leftEdge), xOff );
    xSetEdgefilterMultiple( cu, EDGE_HOR, areaPu, (yOff ? m_stLFCUParam.internalEdge : m_stLFCUParam.topEdge),  yOff );

    if ((currPU.mergeFlag && (currPU.mergeType == MRG_TYPE_SUBPU_ATMVP)) || cu.affine)
    {
      mvSubBlocks = true;
      if (edgeDir == EDGE_HOR)
      {
        for (uint32_t off = subBlockSize; off < areaPu.height; off += subBlockSize)
        {
          const Area mvBlockH(cu.Y().x, cu.Y().y + off, cu.Y().width, pcv.minCUHeight);
          xSetEdgefilterMultiple(cu, EDGE_HOR, mvBlockH, m_stLFCUParam.internalEdge, 1);
        }
      }
      else
      {
        for (uint32_t off = subBlockSize; off < areaPu.width; off += subBlockSize)
        {
          const Area mvBlockV(cu.Y().x + off, cu.Y().y, pcv.minCUWidth, cu.Y().height);
          xSetEdgefilterMultiple(cu, EDGE_VER, mvBlockV, m_stLFCUParam.internalEdge, 1);
        }
      }
    }
  }
#if JVET_M0908_CIIP_DB
  if (cu.firstPU->mhIntraFlag)
  {
    const uint32_t dirMode = PU::getFinalIntraMode(*(cu.firstPU), cu.chType);
    if (edgeDir == EDGE_VER && dirMode == HOR_IDX)
    {
      mvSubBlocks = true;
      subBlockSize = std::max(8u, (area.width >> 2));
      for (uint32_t off = subBlockSize; off < area.width; off += subBlockSize)
      {
        const Area mvBlockV(cu.Y().x + off, cu.Y().y, pcv.minCUWidth, cu.Y().height);
        xSetEdgefilterMultiple(cu, EDGE_VER, mvBlockV, m_stLFCUParam.internalEdge, 1);
      }
    }
    else if (edgeDir == EDGE_HOR && dirMode == VER_IDX)
    {
      mvSubBlocks = true;
      subBlockSize = std::max(8u, (area.height >> 2));
      for (uint32_t off = subBlockSize; off < area.height; off += subBlockSize)
      {
        const Area mvBlockH(cu.Y().x, cu.Y().y + off, cu.Y().width, pcv.minCUHeight);
        xSetEdgefilterMultiple(cu, EDGE_HOR, mvBlockH, m_stLFCUParam.internalEdge, 1);
      }
    }
  }
#endif

  const unsigned uiPelsInPart = pcv.minCUWidth;

  for( int y = 0; y < area.height; y += uiPelsInPart )
  {
    for( int x = 0; x < area.width; x += uiPelsInPart )
    {
      unsigned uiBSCheck = 1;
      const Position localPos  { area.x + x, area.y + y };
      const unsigned rasterIdx = getRasterIdx( localPos, pcv );

      if( m_aapbEdgeFilter[edgeDir][rasterIdx] && uiBSCheck )
      {
        m_aapucBS[edgeDir][rasterIdx] = xGetBoundaryStrengthSingle( cu, edgeDir, localPos );
      }
    }
  }

  if (edgeDir == EDGE_HOR)
  {
    if (!((cu.block(COMPONENT_Y).y % 8) == 0))
      return;
  }
  else
  {
    if (!((cu.block(COMPONENT_Y).x % 8) == 0))
      return;
  }

  unsigned int orthogonalLength = 1;
  unsigned int orthogonalIncrement = 1;
#if FIX_DB_MAX_TRANSFORM_SIZE
  int maxTsize = 64;
  maxTsize = 1 << cu.slice->getSPS()->getQuadtreeTULog2MaxSize();
#endif
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
#if FIX_DB_MAX_TRANSFORM_SIZE
  int maxFilterLengthQ = 7;
  int maxFilterLengthP = 7;
  if (implicitTU && maxTsize < 32)
  {
    maxFilterLengthQ = 3;
    maxFilterLengthP = 3;
  }
#else
  int maxFilterLength = 7;
#endif
#endif
  if (cu.blocks[COMPONENT_Y].valid())
  {
    if (mvSubBlocks)
    {
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
#if FIX_DB_MAX_TRANSFORM_SIZE
      maxFilterLengthQ = std::min(maxFilterLengthQ, 5);
#else
      maxFilterLength = 5;
#endif
#endif
      orthogonalIncrement = subBlockSize / 4;
      orthogonalLength = (edgeDir == EDGE_HOR) ? cu.blocks[COMPONENT_Y].height / 4 : cu.blocks[COMPONENT_Y].width / 4;
    }
#if FIX_DB_MAX_TRANSFORM_SIZE
    if ((cu.blocks[COMPONENT_Y].height > maxTsize) && (edgeDir == EDGE_HOR) && !mvSubBlocks)
    {
      orthogonalIncrement = maxTsize / 4;
      orthogonalLength = cu.blocks[COMPONENT_Y].height / 4;
    }
    if ((cu.blocks[COMPONENT_Y].width > maxTsize) && (edgeDir == EDGE_VER) && !mvSubBlocks)
    {
      orthogonalIncrement = maxTsize / 4;
      orthogonalLength = cu.blocks[COMPONENT_Y].width / 4;

    }
#else
    if ((cu.blocks[COMPONENT_Y].height > 64) && (edgeDir == EDGE_HOR) && !mvSubBlocks)
    {
      orthogonalIncrement = 64 / 4;
      orthogonalLength = cu.blocks[COMPONENT_Y].height / 4;
    }
    if ((cu.blocks[COMPONENT_Y].width > 64) && (edgeDir == EDGE_VER) && !mvSubBlocks)
    {
      orthogonalIncrement = 64 / 4;
      orthogonalLength = cu.blocks[COMPONENT_Y].width / 4;

    }
#endif
  }

  for (int edge = 0; edge < orthogonalLength; edge += orthogonalIncrement)
  {
    if (cu.blocks[COMPONENT_Y].valid())
    {
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
      if (edge == 0)
      {
#if FIX_DB_MAX_TRANSFORM_SIZE
        xEdgeFilterLuma(cu, edgeDir, edge, maxFilterLengthP, maxFilterLengthQ);
#else
        xEdgeFilterLuma(cu, edgeDir, edge, 7, maxFilterLength);
#endif
      }
      else
      {
#if FIX_DB_MAX_TRANSFORM_SIZE
        if (implicitTU && ((edge % (maxTsize / 4)) == 0))
#else
        if ( implicitTU && (edge == (64 / 4)) )
#endif
        {
#if FIX_DB_MAX_TRANSFORM_SIZE
          xEdgeFilterLuma(cu, edgeDir, edge, maxFilterLengthQ, maxFilterLengthQ);
#else
          xEdgeFilterLuma(cu, edgeDir, edge, maxFilterLength, maxFilterLength);
#endif
        }
#if FIX_DB_MAX_TRANSFORM_SIZE
        else if ((edge == 2 || edge == (orthogonalLength - 2)) || (implicitTU && (((edge - 2) % ((maxTsize) / 4) == 0) || ((edge + 2) % ((maxTsize) / 4) == 0))))
#else
        else if ( (edge == 2 || edge == (orthogonalLength - 2)) || (implicitTU && (edge == (56 / 4) || edge == (72 / 4))) )
#endif
        {
          xEdgeFilterLuma(cu, edgeDir, edge, 2, 2);
        }
        else
        {
          xEdgeFilterLuma(cu, edgeDir, edge, 3, 3);
        }
      }
#else
      xEdgeFilterLuma(cu, edgeDir, edge);
#endif
    }
    if (cu.blocks[COMPONENT_Cb].valid() && pcv.chrFormat != CHROMA_400)
    {
      xEdgeFilterChroma(cu, edgeDir, edge);
    }
  }
}


void LoopFilter::xSetEdgefilterMultiple( const CodingUnit&    cu,
                                         const DeblockEdgeDir edgeDir,
                                         const Area&          area,
                                         const bool           bValue,
                                         const bool           EdgeIdx )
{
  const PreCalcValues& pcv = *cu.cs->pcv;

  const unsigned uiAdd     = ( edgeDir == EDGE_VER ) ? pcv.partsInCtuWidth : 1;
  const unsigned uiNumElem = ( edgeDir == EDGE_VER ) ? ( area.height / pcv.minCUHeight ) : ( area.width / pcv.minCUWidth );
  unsigned uiBsIdx         = getRasterIdx( area, pcv );

  for( int ui = 0; ui < uiNumElem; ui++ )
  {
    m_aapbEdgeFilter[edgeDir][uiBsIdx] = bValue;
    if( ! EdgeIdx )
    {
      m_aapucBS[edgeDir][uiBsIdx] = bValue;
    }
    uiBsIdx += uiAdd;
  }
}
void LoopFilter::xSetLoopfilterParam( const CodingUnit& cu )
{
  const Slice& slice = *cu.slice;
#if HEVC_TILES_WPP
  const PPS&   pps   = *cu.cs->pps;
#endif

  if( slice.getDeblockingFilterDisable() )
  {
    m_stLFCUParam.leftEdge = m_stLFCUParam.topEdge = m_stLFCUParam.internalEdge = false;
    return;
  }

  const Position& pos = cu.blocks[cu.chType].pos();

  m_stLFCUParam.internalEdge = true;
#if HEVC_TILES_WPP
  m_stLFCUParam.leftEdge     = ( 0 < pos.x ) && isAvailableLeft ( cu, *cu.cs->getCU( pos.offset( -1,  0 ), cu.chType ), !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag() );
  m_stLFCUParam.topEdge      = ( 0 < pos.y ) && isAvailableAbove( cu, *cu.cs->getCU( pos.offset(  0, -1 ), cu.chType ), !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag() );
#else
  m_stLFCUParam.leftEdge     = ( 0 < pos.x ) && isAvailable ( cu, *cu.cs->getCU( pos.offset( -1,  0 ), cu.chType ), !slice.getLFCrossSliceBoundaryFlag());
  m_stLFCUParam.topEdge      = ( 0 < pos.y ) && isAvailable ( cu, *cu.cs->getCU( pos.offset(  0, -1 ), cu.chType ), !slice.getLFCrossSliceBoundaryFlag());
#endif
#if JVET_M0102_INTRA_SUBPARTITIONS
  m_stLFCUParam.internalEdge &= !cu.ispMode;
#endif
}

unsigned LoopFilter::xGetBoundaryStrengthSingle ( const CodingUnit& cu, const DeblockEdgeDir edgeDir, const Position& localPos ) const
{
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  // The boundary strength that is output by the function xGetBoundaryStrengthSingle is a multi component boundary strength that contains boundary strength for luma (bits 0 to 1), cb (bits 2 to 3) and cr (bits 4 to 5).
#endif

  const Slice& sliceQ = *cu.slice;

  int shiftHor = cu.Y().valid() ? 0 : ::getComponentScaleX(COMPONENT_Cb, cu.firstPU->chromaFormat);
  int shiftVer = cu.Y().valid() ? 0 : ::getComponentScaleY(COMPONENT_Cb, cu.firstPU->chromaFormat);
  const Position& posQ = Position{ localPos.x >> shiftHor,  localPos.y >> shiftVer };
  const Position  posP  = ( edgeDir == EDGE_VER ) ? posQ.offset( -1, 0 ) : posQ.offset( 0, -1 );

  const CodingUnit& cuQ = cu;
  const CodingUnit& cuP = *cu.cs->getCU( posP, cu.chType );

  //-- Set BS for Intra MB : BS = 4 or 3
  if( ( MODE_INTRA == cuP.predMode ) || ( MODE_INTRA == cuQ.predMode ) )
  {
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    return (BsSet(2, COMPONENT_Y) + BsSet(2, COMPONENT_Cb) + BsSet(2, COMPONENT_Cr));
#else
    return 2;
#endif
  }

  const TransformUnit& tuQ = *cuQ.cs->getTU(posQ, cuQ.chType);
  const TransformUnit& tuP = *cuP.cs->getTU(posP, cuP.chType);
  const PreCalcValues& pcv = *cu.cs->pcv;
  const unsigned rasterIdx = getRasterIdx( posQ, pcv );
#if JVET_M0908_CIIP_DB
  if (m_aapucBS[edgeDir][rasterIdx] && (cuP.firstPU->mhIntraFlag || cuQ.firstPU->mhIntraFlag))
  {
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
     return (BsSet(2, COMPONENT_Y) + BsSet(2, COMPONENT_Cb) + BsSet(2, COMPONENT_Cr));
#else
     return 2;
#endif
  }
#endif

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  unsigned tmpBs = 0;
  //-- Set BS for not Intra MB : BS = 2 or 1 or 0
  // Y
  if (m_aapucBS[edgeDir][rasterIdx] && (TU::getCbf(tuQ, COMPONENT_Y) || TU::getCbf(tuP, COMPONENT_Y)))
  {
    tmpBs += BsSet(1, COMPONENT_Y);
  }
  // U
  if (m_aapucBS[edgeDir][rasterIdx] && (TU::getCbf(tuQ, COMPONENT_Cb) || TU::getCbf(tuP, COMPONENT_Cb)))
  {
    tmpBs += BsSet(1, COMPONENT_Cb);
  }
  // V
  if (m_aapucBS[edgeDir][rasterIdx] && (TU::getCbf(tuQ, COMPONENT_Cr) || TU::getCbf(tuP, COMPONENT_Cr)))
  {
    tmpBs += BsSet(1, COMPONENT_Cr);
  }
  if (BsGet(tmpBs, COMPONENT_Y) == 1)
  {
    return tmpBs;
  }
#else
  //-- Set BS for not Intra MB : BS = 2 or 1 or 0
  if (m_aapucBS[edgeDir][rasterIdx] && (TU::getCbf(tuQ, COMPONENT_Y) || TU::getCbf(tuP, COMPONENT_Y)))
  {
    return 1;
  }
#endif
#if JVET_M0908_CIIP_DB
  if ((cuP.firstPU->mhIntraFlag || cuQ.firstPU->mhIntraFlag))
  {
    return 1;
  }
#endif

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  if ( !cu.Y().valid() )
  {
    return tmpBs;
  }
#endif

  // and now the pred
  const Position& lumaPosQ  = Position{ localPos.x,  localPos.y };
  const Position  lumaPosP  = ( edgeDir == EDGE_VER ) ? lumaPosQ.offset( -1, 0 ) : lumaPosQ.offset( 0, -1 );
  const MotionInfo&     miQ = cuQ.cs->getMotionInfo( lumaPosQ );
  const MotionInfo&     miP = cuP.cs->getMotionInfo( lumaPosP );
  const Slice&       sliceP = *cuP.slice;

  if (sliceQ.isInterB() || sliceP.isInterB())
  {
#if JVET_M0483_IBC
    const Picture *piRefP0 = (CU::isIBC(cuP) ? sliceP.getPic() : ((0 > miP.refIdx[0]) ? NULL : sliceP.getRefPic(REF_PIC_LIST_0, miP.refIdx[0])));
    const Picture *piRefP1 = (CU::isIBC(cuP) ? NULL            : ((0 > miP.refIdx[1]) ? NULL : sliceP.getRefPic(REF_PIC_LIST_1, miP.refIdx[1])));
    const Picture *piRefQ0 = (CU::isIBC(cuQ) ? sliceQ.getPic() : ((0 > miQ.refIdx[0]) ? NULL : sliceQ.getRefPic(REF_PIC_LIST_0, miQ.refIdx[0])));
    const Picture *piRefQ1 = (CU::isIBC(cuQ) ? NULL            : ((0 > miQ.refIdx[1]) ? NULL : sliceQ.getRefPic(REF_PIC_LIST_1, miQ.refIdx[1])));
#else
    const Picture *piRefP0 = ( 0 > miP.refIdx[0] ) ? NULL : sliceP.getRefPic( REF_PIC_LIST_0, miP.refIdx[0] );
    const Picture *piRefP1 = ( 0 > miP.refIdx[1] ) ? NULL : sliceP.getRefPic( REF_PIC_LIST_1, miP.refIdx[1] );
    const Picture *piRefQ0 = ( 0 > miQ.refIdx[0] ) ? NULL : sliceQ.getRefPic( REF_PIC_LIST_0, miQ.refIdx[0] );
    const Picture *piRefQ1 = ( 0 > miQ.refIdx[1] ) ? NULL : sliceQ.getRefPic( REF_PIC_LIST_1, miQ.refIdx[1] );
#endif
    Mv mvP0, mvP1, mvQ0, mvQ1;

    if( 0 <= miP.refIdx[0] ) { mvP0 = miP.mv[0]; }
    if( 0 <= miP.refIdx[1] ) { mvP1 = miP.mv[1]; }
    if( 0 <= miQ.refIdx[0] ) { mvQ0 = miQ.mv[0]; }
    if( 0 <= miQ.refIdx[1] ) { mvQ1 = miQ.mv[1]; }

    int nThreshold = 1 << MV_FRACTIONAL_BITS_INTERNAL;
    unsigned uiBs = 0;

    //th can be optimized
    if ( ((piRefP0==piRefQ0)&&(piRefP1==piRefQ1)) || ((piRefP0==piRefQ1)&&(piRefP1==piRefQ0)) )
    {
      if ( piRefP0 != piRefP1 )   // Different L0 & L1
      {
        if ( piRefP0 == piRefQ0 )
        {
          uiBs  = ((abs(mvQ0.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP0.getVer()) >= nThreshold) ||
                   (abs(mvQ1.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP1.getVer()) >= nThreshold))
                  ? 1 : 0;
        }
        else
        {
          uiBs  = ((abs(mvQ1.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP0.getVer()) >= nThreshold) ||
                   (abs(mvQ0.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP1.getVer()) >= nThreshold))
                  ? 1 : 0;
        }
      }
      else    // Same L0 & L1
      {
        uiBs  = ((abs(mvQ0.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP0.getVer()) >= nThreshold) ||
                 (abs(mvQ1.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP1.getVer()) >= nThreshold))
              &&
                ((abs(mvQ1.getHor() - mvP0.getHor()) >= nThreshold) || (abs(mvQ1.getVer() - mvP0.getVer()) >= nThreshold) ||
                 (abs(mvQ0.getHor() - mvP1.getHor()) >= nThreshold) || (abs(mvQ0.getVer() - mvP1.getVer()) >= nThreshold))
              ? 1 : 0;
      }
    }
    else // for all different Ref_Idx
    {
      uiBs = 1;
    }
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    return uiBs + tmpBs;
#else
    return uiBs;
#endif
  }


  // pcSlice->isInterP()
#if JVET_M0483_IBC
  VTMCHECK(CU::isInter(cuP) && 0 > miP.refIdx[0], "Invalid reference picture list index");
  VTMCHECK(CU::isInter(cuP) && 0 > miQ.refIdx[0], "Invalid reference picture list index");
  const Picture *piRefP0 = (CU::isIBC(cuP) ? sliceP.getPic() : sliceP.getRefPic(REF_PIC_LIST_0, miP.refIdx[0]));
  const Picture *piRefQ0 = (CU::isIBC(cuQ) ? sliceQ.getPic() : sliceQ.getRefPic(REF_PIC_LIST_0, miQ.refIdx[0]));
#else
  VTMCHECK(0 > miP.refIdx[0], "Invalid reference picture list index");
  VTMCHECK(0 > miQ.refIdx[0], "Invalid reference picture list index");
  const Picture *piRefP0 = sliceP.getRefPic(REF_PIC_LIST_0, miP.refIdx[0]);
  const Picture *piRefQ0 = sliceQ.getRefPic(REF_PIC_LIST_0, miQ.refIdx[0]);
#endif
  if (piRefP0 != piRefQ0)
  {
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    return tmpBs + 1;
#else
    return 1;
#endif
  }

  Mv mvP0 = miP.mv[0];
  Mv mvQ0 = miQ.mv[0];

  int nThreshold = 1 << MV_FRACTIONAL_BITS_INTERNAL;
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  return ( ( abs( mvQ0.getHor() - mvP0.getHor() ) >= nThreshold ) || ( abs( mvQ0.getVer() - mvP0.getVer() ) >= nThreshold ) ) ? (tmpBs + 1) : tmpBs;
#else
  return ( ( abs( mvQ0.getHor() - mvP0.getHor() ) >= nThreshold ) || ( abs( mvQ0.getVer() - mvP0.getVer() ) >= nThreshold ) ) ? 1 : 0;
#endif
}

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
void LoopFilter::deriveLADFShift( const Pel* src, const int stride, int& shift, const DeblockEdgeDir edgeDir, const SPS sps )
{
  uint32_t lumaLevel = 0;
  shift = sps.getLadfQpOffset(0);

  if (edgeDir == EDGE_VER)
  {
    lumaLevel = (src[0] + src[3*stride] + src[-1] + src[3*stride - 1]) >> 2;
  }
  else // (edgeDir == EDGE_HOR)
  {
    lumaLevel = (src[0] + src[3] + src[-stride] + src[-stride + 3]) >> 2;
  }

  for ( int k = 1; k < sps.getLadfNumIntervals(); k++ )
  {
    const int th = sps.getLadfIntervalLowerBound( k );
    if ( lumaLevel > th )
    {
      shift = sps.getLadfQpOffset( k );
    }
    else
    {
      break;
    }
  }
}
#endif

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
void LoopFilter::xEdgeFilterLuma(const CodingUnit& cu, const DeblockEdgeDir edgeDir, const int iEdge, const int initialMaxFilterLengthP, const int initialMaxFilterLengthQ)
#else
void LoopFilter::xEdgeFilterLuma(const CodingUnit& cu, const DeblockEdgeDir edgeDir, const int iEdge)
#endif
{
  const CompArea&  lumaArea = cu.block(COMPONENT_Y);
  const PreCalcValues& pcv = *cu.cs->pcv;

  PelBuf        picYuvRec = cu.cs->getRecoBuf( lumaArea );
  Pel           *piSrc    = picYuvRec.buf;
  const int     iStride   = picYuvRec.stride;
  Pel           *piTmpSrc = piSrc;
  const PPS     &pps      = *(cu.cs->pps);
  const SPS     &sps      = *(cu.cs->sps);
  const Slice   &slice    = *(cu.slice);
  const bool    ppsTransquantBypassEnabledFlag = pps.getTransquantBypassEnabledFlag();
  const int     bitDepthLuma                   = sps.getBitDepth(CHANNEL_TYPE_LUMA);
  const ClpRng& clpRng( cu.cs->slice->clpRng(COMPONENT_Y) );

  int          iQP          = 0;
  unsigned     uiNumParts   = ( ( ( edgeDir == EDGE_VER ) ? lumaArea.height / pcv.minCUHeight : lumaArea.width / pcv.minCUWidth ) );
  int          pelsInPart   = pcv.minCUWidth;
  unsigned     uiBsAbsIdx   = 0, uiBs = 0;
  int          iOffset, iSrcStep;

  bool  bPCMFilter      = (sps.getPCMEnabledFlag() && sps.getPCMFilterDisableFlag()) ? true : false;
  bool  bPartPNoFilter  = false;
  bool  bPartQNoFilter  = false;
  int   betaOffsetDiv2  = slice.getDeblockingFilterBetaOffsetDiv2();
  int   tcOffsetDiv2    = slice.getDeblockingFilterTcOffsetDiv2();
  int   xoffset, yoffset;

  Position pos;

  if (edgeDir == EDGE_VER)
  {
    xoffset   = 0;
    yoffset   = pelsInPart;
    iOffset   = 1;
    iSrcStep  = iStride;
    piTmpSrc += iEdge * pelsInPart;
    pos       = Position{ lumaArea.x + iEdge * pelsInPart, lumaArea.y - yoffset };
  }
  else  // (edgeDir == EDGE_HOR)
  {
    xoffset   = pelsInPart;
    yoffset   = 0;
    iOffset   = iStride;
    iSrcStep  = 1;
    piTmpSrc += iEdge*pelsInPart*iStride;
    pos       = Position{ lumaArea.x - xoffset, lumaArea.y + iEdge * pelsInPart };
  }

  const int iBitdepthScale = 1 << (bitDepthLuma - 8);

  // dec pos since within the loop we first calc the pos
  for( int iIdx = 0; iIdx < uiNumParts; iIdx++ )
  {
    pos.x += xoffset;
    pos.y += yoffset;

    uiBsAbsIdx = getRasterIdx( pos, pcv );
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    uiBs = BsGet(m_aapucBS[edgeDir][uiBsAbsIdx], COMPONENT_Y);
#else
    uiBs       = m_aapucBS[edgeDir][uiBsAbsIdx];
#endif

    if( uiBs )
    {
      const CodingUnit& cuQ =  cu;
      const CodingUnit& cuP = *cu.cs->getCU(pos.offset(xoffset - pelsInPart, yoffset - pelsInPart), cu.chType);
      // Derive neighboring PU index
      if (edgeDir == EDGE_VER)
      {
#if HEVC_TILES_WPP
        VTMCHECK( !isAvailableLeft( cu, cuP, !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag() ), "Neighbour not available" );
#else
        VTMCHECK( !isAvailable( cu, cuP, !slice.getLFCrossSliceBoundaryFlag() ), "Neighbour not available" );
#endif
      }
      else  // (iDir == EDGE_HOR)
      {
#if HEVC_TILES_WPP
        VTMCHECK( !isAvailableAbove( cu, cuP, !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag() ), "Neighbour not available" );
#else
        VTMCHECK( !isAvailable( cu, cuP, !slice.getLFCrossSliceBoundaryFlag() ), "Neighbour not available" );
#endif
      }

      iQP = (cuP.qp + cuQ.qp + 1) >> 1;

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
      if ( sps.getLadfEnabled() )
      {
        int iShift = 0;
        deriveLADFShift( piTmpSrc + iSrcStep * (iIdx*pelsInPart), iStride, iShift, edgeDir, sps );
        iQP += iShift;
      }
#endif

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
      bool sidePisLarge   = false;
      bool sideQisLarge   = false;
      int maxFilterLengthP = initialMaxFilterLengthP;
      int maxFilterLengthQ = initialMaxFilterLengthQ;
      if (maxFilterLengthP > 3)
      {
        sidePisLarge = (edgeDir == EDGE_VER && cuP.block(COMPONENT_Y).width >= 32)
          || (edgeDir == EDGE_HOR && cuP.block(COMPONENT_Y).height >= 32);

        if (sidePisLarge && maxFilterLengthP > 5)
        {
          // restrict filter length if sub-blocks are used (e.g affine or ATMVP)
#if JVET_M0908_CIIP_DB
          bool ciipSubBlock = false;
          if (cuP.firstPU->mhIntraFlag)
          {
            const uint32_t dirMode = PU::getFinalIntraMode(*(cuP.firstPU), cuP.chType);
            ciipSubBlock = edgeDir == EDGE_HOR ? dirMode == VER_IDX : dirMode == HOR_IDX;
          }
          if (cuP.affine || ciipSubBlock)
#else
          if (cuP.affine)
#endif
          {
            maxFilterLengthP = std::min(maxFilterLengthP, 5);
          }
        }
      }
      if (maxFilterLengthQ > 3)
      {
        sideQisLarge = (edgeDir == EDGE_VER && cuQ.block(COMPONENT_Y).width >= 32)
          || (edgeDir == EDGE_HOR && cuQ.block(COMPONENT_Y).height >= 32);
      }

      if (edgeDir == EDGE_HOR && pos.y % slice.getSPS()->getCTUSize() == 0)
      {
        sidePisLarge = false;
      }
#endif
      const int iIndexTC  = Clip3(0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, int(iQP + DEFAULT_INTRA_TC_OFFSET*(uiBs - 1) + (tcOffsetDiv2 << 1)));
      const int iIndexB   = Clip3(0, MAX_QP, iQP + (betaOffsetDiv2 << 1));

      const int iTc       = sm_tcTable  [iIndexTC] * iBitdepthScale;
      const int iBeta     = sm_betaTable[iIndexB ] * iBitdepthScale;
      const int iSideThreshold = ( iBeta + ( iBeta >> 1 ) ) >> 3;
      const int iThrCut   = iTc * 10;

      const unsigned uiBlocksInPart = pelsInPart / 4 ? pelsInPart / 4 : 1;

      for( int iBlkIdx = 0; iBlkIdx < uiBlocksInPart; iBlkIdx++ )
      {
        const int dp0 = xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0), iOffset);
        const int dq0 = xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0), iOffset);
        const int dp3 = xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3), iOffset);
        const int dq3 = xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3), iOffset);
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
        int dp0L = dp0;
        int dq0L = dq0;
        int dp3L = dp3;
        int dq3L = dq3;

        if (sidePisLarge)
        {
          dp0L = (dp0L + xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0) - 3 * iOffset, iOffset) + 1) >> 1;
          dp3L = (dp3L + xCalcDP(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3) - 3 * iOffset, iOffset) + 1) >> 1;
        }
        if (sideQisLarge)
        {
          dq0L = (dq0L + xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 0) + 3 * iOffset, iOffset) + 1) >> 1;
          dq3L = (dq3L + xCalcDQ(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + 3) + 3 * iOffset, iOffset) + 1) >> 1;
        }
        bool useLongtapFilter = false;
        if (sidePisLarge || sideQisLarge)
        {
          int d0L = dp0L + dq0L;
          int d3L = dp3L + dq3L;

          int dpL = dp0L + dp3L;
          int dqL = dq0L + dq3L;

          int dL = d0L + d3L;

          bPartPNoFilter = bPartQNoFilter = false;
          if (bPCMFilter)
          {
            // Check if each of PUs is I_PCM with LF disabling
            bPartPNoFilter = cuP.ipcm;
            bPartQNoFilter = cuQ.ipcm;
          }
          if (ppsTransquantBypassEnabledFlag)
          {
            // check if each of PUs is lossless coded
            bPartPNoFilter = bPartPNoFilter || cuP.transQuantBypass;
            bPartQNoFilter = bPartQNoFilter || cuQ.transQuantBypass;
          }

          if (dL < iBeta)
          {
            const bool filterP = (dpL < iSideThreshold);
            const bool filterQ = (dqL < iSideThreshold);

            Pel* src0 = piTmpSrc + iSrcStep * (iIdx*pelsInPart + iBlkIdx * 4 + 0);
            Pel* src3 = piTmpSrc + iSrcStep * (iIdx*pelsInPart + iBlkIdx * 4 + 3);

            // adjust decision so that it is not read beyond p5 is maxFilterLengthP is 5 and q5 if maxFilterLengthQ is 5
            const bool swL = xUseStrongFiltering(src0, iOffset, 2 * d0L, iBeta, iTc, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ)
              && xUseStrongFiltering(src3, iOffset, 2 * d3L, iBeta, iTc, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ);
            if (swL)
            {
              useLongtapFilter = true;
              for (int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++)
              {
                xPelFilterLuma(piTmpSrc + iSrcStep*(iIdx*pelsInPart + iBlkIdx * 4 + i), iOffset, iTc, swL, bPartPNoFilter, bPartQNoFilter, iThrCut, filterP, filterQ, clpRng, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ);
              }
            }

          }
        }
        if (!useLongtapFilter)
        {
#endif
        const int d0 = dp0 + dq0;
        const int d3 = dp3 + dq3;

        const int dp = dp0 + dp3;
        const int dq = dq0 + dq3;
        const int d  = d0  + d3;

        bPartPNoFilter = bPartQNoFilter = false;
        if( bPCMFilter )
        {
          // Check if each of PUs is I_PCM with LF disabling
          bPartPNoFilter = cuP.ipcm;
          bPartQNoFilter = cuQ.ipcm;
        }
        if( ppsTransquantBypassEnabledFlag )
        {
          // check if each of PUs is lossless coded
          bPartPNoFilter = bPartPNoFilter || cuP.transQuantBypass;
          bPartQNoFilter = bPartQNoFilter || cuQ.transQuantBypass;
        }

        if( d < iBeta )
        {
          const bool bFilterP = (dp < iSideThreshold);
          const bool bFilterQ = (dq < iSideThreshold);
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
          bool sw = false;
          if (maxFilterLengthP > 2 && maxFilterLengthQ > 2)
          {
            sw = xUseStrongFiltering(piTmpSrc + iSrcStep * (iIdx*pelsInPart + iBlkIdx * 4 + 0), iOffset, 2 * d0, iBeta, iTc)
              && xUseStrongFiltering(piTmpSrc + iSrcStep * (iIdx*pelsInPart + iBlkIdx * 4 + 3), iOffset, 2 * d3, iBeta, iTc);
          }
#else
          const bool sw = xUseStrongFiltering( piTmpSrc + iSrcStep * ( iIdx*pelsInPart + iBlkIdx * 4 + 0 ), iOffset, 2 * d0, iBeta, iTc )
                       && xUseStrongFiltering( piTmpSrc + iSrcStep * ( iIdx*pelsInPart + iBlkIdx * 4 + 3 ), iOffset, 2 * d3, iBeta, iTc );
#endif
          for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
          {
            xPelFilterLuma( piTmpSrc + iSrcStep*( iIdx*pelsInPart + iBlkIdx * 4 + i ), iOffset, iTc, sw, bPartPNoFilter, bPartQNoFilter, iThrCut, bFilterP, bFilterQ, clpRng );
          }
        }
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
        }
#endif
      }
    }
  }
}


void LoopFilter::xEdgeFilterChroma(const CodingUnit& cu, const DeblockEdgeDir edgeDir, const int iEdge)
{
  const Position lumaPos   = cu.Y().valid() ? cu.Y().pos() : recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() );
  const Size     lumaSize  = cu.Y().valid() ? cu.Y().size() : recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() );

  const PreCalcValues& pcv = *cu.cs->pcv;
  unsigned  rasterIdx      = getRasterIdx( lumaPos, pcv );

  PelBuf     picYuvRecCb   = cu.cs->getRecoBuf( cu.block(COMPONENT_Cb) );
  PelBuf     picYuvRecCr   = cu.cs->getRecoBuf( cu.block(COMPONENT_Cr) );
  Pel       *piSrcCb       = picYuvRecCb.buf;
  Pel       *piSrcCr       = picYuvRecCr.buf;
  const int  iStride       = picYuvRecCb.stride;
  const SPS &sps           = *cu.cs->sps;
  const PPS &pps           = *cu.cs->pps;
  const Slice  &slice      = *cu.slice;
  const ChromaFormat nChromaFormat   = sps.getChromaFormatIdc();
  const unsigned uiPelsInPartChromaH = pcv.minCUWidth  >> ::getComponentScaleX(COMPONENT_Cb, nChromaFormat);
  const unsigned uiPelsInPartChromaV = pcv.minCUHeight >> ::getComponentScaleY(COMPONENT_Cb, nChromaFormat);

  int       iOffset, iSrcStep;
  unsigned  uiLoopLength;

  bool      bPCMFilter      = (sps.getPCMEnabledFlag() && sps.getPCMFilterDisableFlag()) ? true : false;
  bool      bPartPNoFilter  = false;
  bool      bPartQNoFilter  = false;
  const int tcOffsetDiv2    = slice.getDeblockingFilterTcOffsetDiv2();
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  const int betaOffsetDiv2  = slice.getDeblockingFilterBetaOffsetDiv2();
#endif

  // Vertical Position
  unsigned uiEdgeNumInCtuVert = rasterIdx % pcv.partsInCtuWidth + iEdge;
  unsigned uiEdgeNumInCtuHor  = rasterIdx / pcv.partsInCtuWidth + iEdge;

  if( ( uiPelsInPartChromaH < DEBLOCK_SMALLEST_BLOCK ) && ( uiPelsInPartChromaV < DEBLOCK_SMALLEST_BLOCK ) &&
      (
        ( ( uiEdgeNumInCtuVert % ( DEBLOCK_SMALLEST_BLOCK / uiPelsInPartChromaH ) ) && ( edgeDir == EDGE_VER ) ) ||
        ( ( uiEdgeNumInCtuHor  % ( DEBLOCK_SMALLEST_BLOCK / uiPelsInPartChromaV ) ) && ( edgeDir == EDGE_HOR ) )
      )
    )
  {
    return;
  }

  unsigned uiNumParts =  ( edgeDir == EDGE_VER ) ? lumaSize.height / pcv.minCUHeight : lumaSize.width / pcv.minCUWidth ;
  int   uiNumPelsLuma = pcv.minCUWidth;
  unsigned uiBsAbsIdx;
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  unsigned bS[2];
#else
  unsigned ucBs;
#endif

  Pel* piTmpSrcCb = piSrcCb;
  Pel* piTmpSrcCr = piSrcCr;
  int xoffset, yoffset;
  Position pos( lumaPos.x, lumaPos.y );

  if( edgeDir == EDGE_VER )
  {
    xoffset      = 0;
    yoffset      = uiNumPelsLuma;
    iOffset      = 1;
    iSrcStep     = iStride;
    piTmpSrcCb  += iEdge*uiPelsInPartChromaH;
    piTmpSrcCr  += iEdge*uiPelsInPartChromaH;
    uiLoopLength = uiPelsInPartChromaV;
    pos          = Position{ lumaPos.x + iEdge*uiNumPelsLuma, lumaPos.y - yoffset };
  }
  else  // (edgeDir == EDGE_HOR)
  {
    xoffset      = uiNumPelsLuma;
    yoffset      = 0;
    iOffset      = iStride;
    iSrcStep     = 1;
    piTmpSrcCb  += iEdge*iStride*uiPelsInPartChromaV;
    piTmpSrcCr  += iEdge*iStride*uiPelsInPartChromaV;
    uiLoopLength = uiPelsInPartChromaH;
    pos          = Position{ lumaPos.x - xoffset, lumaPos.y + iEdge*uiNumPelsLuma };
  }

  const int iBitdepthScale = 1 << (sps.getBitDepth(CHANNEL_TYPE_CHROMA) - 8);

  for( int iIdx = 0; iIdx < uiNumParts; iIdx++ )
  {
    pos.x += xoffset;
    pos.y += yoffset;

    uiBsAbsIdx = getRasterIdx( pos, pcv );
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    unsigned tmpBs = m_aapucBS[edgeDir][uiBsAbsIdx];

    tmpBs = m_aapucBS[edgeDir][uiBsAbsIdx];
    bS[0] = BsGet(tmpBs, COMPONENT_Cb);
    bS[1] = BsGet(tmpBs, COMPONENT_Cr);
#else
    ucBs       = m_aapucBS[edgeDir][uiBsAbsIdx];
#endif

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    if (bS[0] > 0 || bS[1] > 0)
#else
    if (ucBs > 1)
#endif
    {
      const CodingUnit& cuQ =  cu;
      const CodingUnit& cuP = *cu.cs->getCU( recalcPosition( cu.chromaFormat, CHANNEL_TYPE_LUMA, cu.chType, pos.offset( xoffset - uiNumPelsLuma, yoffset - uiNumPelsLuma ) ), cu.chType );

      if (edgeDir == EDGE_VER)
      {
#if HEVC_TILES_WPP
        VTMCHECK(!isAvailableLeft(cu, cuP, !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag()), "Neighbour not available");
#else
        VTMCHECK(!isAvailable(cu, cuP, !slice.getLFCrossSliceBoundaryFlag()), "Neighbour not available");
#endif
      }
      else  // (iDir == EDGE_HOR)
      {
#if HEVC_TILES_WPP
        VTMCHECK(!isAvailableAbove(cu, cuP, !slice.getLFCrossSliceBoundaryFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag()), "Neighbour not available");
#else
        VTMCHECK(!isAvailable(cu, cuP, !slice.getLFCrossSliceBoundaryFlag()), "Neighbour not available");
#endif
      }

      bPartPNoFilter = bPartQNoFilter = false;
      if (bPCMFilter)
      {
        // Check if each of PUs is I_PCM with LF disabling
        bPartPNoFilter = cuP.ipcm;
        bPartQNoFilter = cuQ.ipcm;
      }
      if( pps.getTransquantBypassEnabledFlag() )
      {
        // check if each of PUs is lossless coded
        bPartPNoFilter = bPartPNoFilter || cuP.transQuantBypass;
        bPartQNoFilter = bPartQNoFilter || cuQ.transQuantBypass;
      }

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
      const unsigned cuPWidth  = cuP.block(COMPONENT_Cb).width;
      const unsigned cuPHeight = cuP.block(COMPONENT_Cb).height;
      const unsigned cuQWidth  = cuQ.block(COMPONENT_Cb).width;
      const unsigned cuQHeight = cuQ.block(COMPONENT_Cb).height;

      bool largeBoundary = ((edgeDir == EDGE_VER && cuPWidth >= 8 && cuQWidth >= 8) || (edgeDir == EDGE_HOR && cuPHeight >= 8 && cuQHeight >= 8));

      if (edgeDir == EDGE_HOR && pos.y % cuP.slice->getSPS()->getCTUSize() == 0)
      {
        largeBoundary = false;
      }

#endif
      for( int chromaIdx = 0; chromaIdx < 2; chromaIdx++ )
      {
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
        if ((bS[chromaIdx] == 2) || (largeBoundary && (bS[chromaIdx] == 1)))
        {
#endif
        const ClpRng& clpRng( cu.cs->slice->clpRng( ComponentID( chromaIdx + 1 )) );
        const int chromaQPOffset = pps.getQpOffset( ComponentID( chromaIdx + 1 ) );
        Pel* piTmpSrcChroma = (chromaIdx == 0) ? piTmpSrcCb : piTmpSrcCr;

        int iQP = ( ( cuP.qp + cuQ.qp + 1 ) >> 1 ) + chromaQPOffset;
        if (iQP >= chromaQPMappingTableSize)
        {
          if( sps.getChromaFormatIdc() == CHROMA_420 )
          {
            iQP -= 6;
          }
          else if( iQP > MAX_QP )
          {
            iQP = MAX_QP;
          }
        }
        else if( iQP >= 0 )
        {
          iQP = getScaledChromaQP(iQP, sps.getChromaFormatIdc());
        }

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
        const int iIndexTC = Clip3<int>(0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, iQP + DEFAULT_INTRA_TC_OFFSET * (bS[chromaIdx] - 1) + (tcOffsetDiv2 << 1));
#else
        const int iIndexTC = Clip3<int>( 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, iQP + DEFAULT_INTRA_TC_OFFSET*( ucBs - 1 ) + ( tcOffsetDiv2 << 1 ) );
#endif
        const int iTc      = sm_tcTable[iIndexTC] * iBitdepthScale;

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
        bool useLongFilter = false;
        if (largeBoundary)
        {
        const int indexB = Clip3<int>(0, MAX_QP, iQP + (betaOffsetDiv2 << 1));
        const int beta = sm_betaTable[indexB] * iBitdepthScale;

        const int dp0 = xCalcDP(piTmpSrcChroma + iSrcStep*(iIdx*uiLoopLength + 0), iOffset);
        const int dq0 = xCalcDQ(piTmpSrcChroma + iSrcStep*(iIdx*uiLoopLength + 0), iOffset);
        const int dp1 = xCalcDP(piTmpSrcChroma + iSrcStep*(iIdx*uiLoopLength + 1), iOffset);
        const int dq1 = xCalcDQ(piTmpSrcChroma + iSrcStep*(iIdx*uiLoopLength + 1), iOffset);

        const int d0 = dp0 + dq0;
        const int d1 = dp1 + dq1;
        const int d = d0 + d1;

          if (d < beta)
          {
            useLongFilter = true;
            const bool sw = xUseStrongFiltering(piTmpSrcChroma + iSrcStep*(iIdx*uiLoopLength + 0), iOffset, 2 * d0, beta, iTc)
                && xUseStrongFiltering(piTmpSrcChroma + iSrcStep*(iIdx*uiLoopLength + 1), iOffset, 2 * d1, beta, iTc);

            for (unsigned step = 0; step < uiLoopLength; step++)
            {
              xPelFilterChroma(piTmpSrcChroma + iSrcStep*(step + iIdx*uiLoopLength), iOffset, iTc, sw, bPartPNoFilter, bPartQNoFilter, clpRng, largeBoundary);
            }
          }
        }
        if ( !useLongFilter )
        {
          for (unsigned step = 0; step < uiLoopLength; step++)
          {
            xPelFilterChroma(piTmpSrcChroma + iSrcStep*(step + iIdx*uiLoopLength), iOffset, iTc, false, bPartPNoFilter, bPartQNoFilter, clpRng, largeBoundary);
          }
        }
#else
        for( unsigned uiStep = 0; uiStep < uiLoopLength; uiStep++ )
        {
          xPelFilterChroma( piTmpSrcChroma + iSrcStep*( uiStep + iIdx*uiLoopLength ), iOffset, iTc, bPartPNoFilter, bPartQNoFilter, clpRng );
        }
#endif
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
        }
#endif
      }
    }
  }
}



/**
 - Deblocking for the luminance component with strong or weak filter
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param tc              tc value
 \param sw              decision strong/weak filter
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 \param iThrCut         threshold value for weak filter decision
 \param bFilterSecondP  decision weak filter/no filter for partP
 \param bFilterSecondQ  decision weak filter/no filter for partQ
 \param bitDepthLuma    luma bit depth
*/
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
inline void LoopFilter::xBilinearFilter(Pel* srcP, Pel* srcQ, int offset, int refMiddle, int refP, int refQ, int numberPSide, int numberQSide, const int* dbCoeffsP, const int* dbCoeffsQ, int tc) const
{
    int src;
    const char tc7[7] = { 6, 5, 4, 3, 2, 1, 1};
    const char tc3[3] = { 6, 4, 2 };
    const char *tcP  = (numberPSide == 3) ? tc3 : tc7;
    const char *tcQ  = (numberQSide == 3) ? tc3 : tc7;
    for (int pos = 0; pos < numberPSide; pos++)
    {
      src = srcP[-offset*pos];
      int cvalue = (tc * tcP[pos]) >>1;
      srcP[-offset * pos] = Clip3(src - cvalue, src + cvalue, ((refMiddle*dbCoeffsP[pos] + refP * (64 - dbCoeffsP[pos]) + 32) >> 6));
    }
    for (int pos = 0; pos < numberQSide; pos++)
    {
      src = srcQ[offset*pos];
      int cvalue = (tc * tcQ[pos]) >> 1;
      srcQ[offset*pos] = Clip3(src - cvalue, src + cvalue, ((refMiddle*dbCoeffsQ[pos] + refQ * (64 - dbCoeffsQ[pos]) + 32) >> 6));
    }
}

inline void LoopFilter::xFilteringPandQ(Pel* src, int offset, int numberPSide, int numberQSide, int tc) const
{
  VTMCHECK(numberPSide <= 3 && numberQSide <= 3, "Short filtering in long filtering function");
  Pel* srcP = src-offset;
  Pel* srcQ = src;

  int refP = 0;
  int refQ = 0;
  int refMiddle = 0;

  const int dbCoeffs7[7] = { 59, 50, 41,32,23,14,5 };
  const int dbCoeffs3[3] = { 53, 32, 11 };
  const int dbCoeffs5[5] = { 58, 45, 32,19,6};
  const int* dbCoeffsP   = numberPSide == 7 ? dbCoeffs7 : (numberPSide==5) ? dbCoeffs5 : dbCoeffs3;
  const int* dbCoeffsQ   = numberQSide == 7 ? dbCoeffs7 : (numberQSide==5) ? dbCoeffs5 : dbCoeffs3;

  switch (numberPSide)
  {
    case 7: refP = (srcP[-6*offset]   + srcP[-7 * offset] + 1) >> 1; break;
    case 3: refP = (srcP[-2 * offset] + srcP[-3 * offset] + 1) >> 1; break;
    case 5: refP = (srcP[-4 * offset] + srcP[-5 * offset] + 1) >> 1; break;
  }

  switch (numberQSide)
  {
    case 7: refQ = (srcQ[6 * offset] + srcQ[7 * offset] + 1) >> 1; break;
    case 3: refQ = (srcQ[2 * offset] + srcQ[3 * offset] + 1) >> 1; break;
    case 5: refQ = (srcQ[4 * offset] + srcQ[5 * offset] + 1) >> 1; break;
  }

  if (numberPSide == numberQSide)
  {
    if (numberPSide == 5)
    {
      refMiddle = (2 * (srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset]) + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + 8) >> 4;
    }
    else
    {
      refMiddle = (2 * (srcP[0] + srcQ[0]) + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + +srcP[-6 * offset] + srcQ[6 * offset] + 8) >> 4;
    }
  }
  else
  {
    Pel* srcPt = srcP;
    Pel* srcQt = srcQ;
    int offsetP = -offset;
    int offsetQ = offset;

    int newNumberQSide = numberQSide;
    int newNumberPSide = numberPSide;
    if (numberQSide > numberPSide)
    {
      std::swap(srcPt, srcQt);
      std::swap(offsetP, offsetQ);
      newNumberQSide = numberPSide;
      newNumberPSide = numberQSide;
    }

    if (newNumberPSide == 7 && newNumberQSide == 5)
    {
      refMiddle = (2 * (srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset]) + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + 8) >> 4;
    }
    else if (newNumberPSide == 7 && newNumberQSide == 3)
    {
      refMiddle = (2 * (srcPt[0] + srcQt[0]) + srcQt[0] + 2 * (srcQt[offsetQ] + srcQt[2 * offsetQ]) + srcPt[offsetP] + srcQt[offsetQ] + srcPt[2 * offsetP] + srcPt[3 * offsetP] + srcPt[4 * offsetP] + srcPt[5 * offsetP] + srcPt[6 * offsetP] + 8) >> 4;
    }
    else //if (newNumberPSide == 5 && newNumberQSide == 3)
    {
      refMiddle = (srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + 4) >> 3;
    }
  }
  xBilinearFilter(srcP,srcQ,offset,refMiddle,refP,refQ,numberPSide,numberQSide,dbCoeffsP,dbCoeffsQ,tc);
}

inline void LoopFilter::xPelFilterLuma(Pel* piSrc, const int iOffset, const int tc, const bool sw, const bool bPartPNoFilter, const bool bPartQNoFilter, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng, bool sidePisLarge, bool sideQisLarge, int maxFilterLengthP, int maxFilterLengthQ) const
#else
inline void LoopFilter::xPelFilterLuma( Pel* piSrc, const int iOffset, const int tc, const bool sw, const bool bPartPNoFilter, const bool bPartQNoFilter, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng ) const
#endif
{
  int delta;

  const Pel m4  = piSrc[ 0          ];
  const Pel m3  = piSrc[-iOffset    ];
  const Pel m5  = piSrc[ iOffset    ];
  const Pel m2  = piSrc[-iOffset * 2];
  const Pel m6  = piSrc[ iOffset * 2];
  const Pel m1  = piSrc[-iOffset * 3];
  const Pel m7  = piSrc[ iOffset * 3];
  const Pel m0  = piSrc[-iOffset * 4];

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  const Pel mP1 = piSrc[-iOffset * 5];
  const Pel mP2 = piSrc[-iOffset * 6];
  const Pel mP3 = piSrc[-iOffset * 7];
  const Pel m8  = piSrc[ iOffset * 4];
  const Pel m9  = piSrc[ iOffset * 5];
  const Pel m10 = piSrc[ iOffset * 6];
  const char tc3[3] = { 3, 2, 1};
#endif
  if (sw)
  {
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    if (sidePisLarge || sideQisLarge)
    {
      xFilteringPandQ(piSrc, iOffset, sidePisLarge ? maxFilterLengthP : 3, sideQisLarge ? maxFilterLengthQ : 3, tc);
    }
    else
    {
      piSrc[-iOffset]     = Clip3(m3 - tc3[0] * tc, m3 + tc3[0] * tc, ((m1 + 2 * m2 + 2 * m3 + 2 * m4 + m5 + 4) >> 3));
      piSrc[0]            = Clip3(m4 - tc3[0] * tc, m4 + tc3[0] * tc, ((m2 + 2 * m3 + 2 * m4 + 2 * m5 + m6 + 4) >> 3));
      piSrc[-iOffset * 2] = Clip3(m2 - tc3[1] * tc, m2 + tc3[1] * tc, ((m1 + m2 + m3 + m4 + 2) >> 2));
      piSrc[iOffset]      = Clip3(m5 - tc3[1] * tc, m5 + tc3[1] * tc, ((m3 + m4 + m5 + m6 + 2) >> 2));
      piSrc[-iOffset * 3] = Clip3(m1 - tc3[2] * tc, m1 + tc3[2] * tc, ((2 * m0 + 3 * m1 + m2 + m3 + m4 + 4) >> 3));
      piSrc[iOffset * 2]  = Clip3(m6 - tc3[2] * tc, m6 + tc3[2] * tc, ((m3 + m4 + m5 + 3 * m6 + 2 * m7 + 4) >> 3));
    }
#else
    piSrc[-iOffset]     = Clip3( m3 - 2 * tc, m3 + 2 * tc, ( (     m1 + 2 * m2 + 2 * m3 + 2 * m4 +     m5 + 4 ) >> 3 ) );
    piSrc[ 0]           = Clip3( m4 - 2 * tc, m4 + 2 * tc, ( (     m2 + 2 * m3 + 2 * m4 + 2 * m5 +     m6 + 4 ) >> 3 ) );
    piSrc[-iOffset * 2] = Clip3( m2 - 2 * tc, m2 + 2 * tc, ( (     m1 +     m2 +     m3 +     m4 +          2 ) >> 2 ) );
    piSrc[ iOffset]     = Clip3( m5 - 2 * tc, m5 + 2 * tc, ( (     m3 +     m4 +     m5 +     m6 +          2 ) >> 2 ) );
    piSrc[-iOffset * 3] = Clip3( m1 - 2 * tc, m1 + 2 * tc, ( ( 2 * m0 + 3 * m1 +     m2 +     m3 +     m4 + 4 ) >> 3 ) );
    piSrc[ iOffset * 2] = Clip3( m6 - 2 * tc, m6 + 2 * tc, ( (     m3 +     m4 +     m5 + 3 * m6 + 2 * m7 + 4 ) >> 3 ) );
#endif
  }
  else
  {
    /* Weak filter */
    delta = ( 9 * ( m4 - m3 ) - 3 * ( m5 - m2 ) + 8 ) >> 4;

    if ( abs(delta) < iThrCut )
    {
      delta = Clip3( -tc, tc, delta );
      piSrc[-iOffset] = ClipPel( m3 + delta, clpRng);
      piSrc[0]        = ClipPel( m4 - delta, clpRng);

      const int tc2 = tc >> 1;
      if( bFilterSecondP )
      {
        const int delta1 = Clip3( -tc2, tc2, ( ( ( ( m1 + m3 + 1 ) >> 1 ) - m2 + delta ) >> 1 ) );
        piSrc[-iOffset * 2] = ClipPel( m2 + delta1, clpRng);
      }
      if( bFilterSecondQ )
      {
        const int delta2 = Clip3( -tc2, tc2, ( ( ( ( m6 + m4 + 1 ) >> 1 ) - m5 - delta ) >> 1 ) );
        piSrc[iOffset] = ClipPel( m5 + delta2, clpRng);
      }
    }
  }

  if(bPartPNoFilter)
  {
    piSrc[-iOffset    ] = m3;
    piSrc[-iOffset * 2] = m2;
    piSrc[-iOffset * 3] = m1;
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    if (sidePisLarge)
    {
      piSrc[-iOffset * 4] = m0;
      piSrc[-iOffset * 5] = mP1;
      piSrc[-iOffset * 6] = mP2;
      piSrc[-iOffset * 7] = mP3;
    }
#endif
  }

  if(bPartQNoFilter)
  {
    piSrc[ 0          ] = m4;
    piSrc[ iOffset    ] = m5;
    piSrc[ iOffset * 2] = m6;
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    if (sideQisLarge)
    {
      piSrc[iOffset * 3] = m7;
      piSrc[iOffset * 4] = m8;
      piSrc[iOffset * 5] = m9;
      piSrc[iOffset * 6] = m10;
    }
#endif
  }
}

/**
 - Deblocking of one line/column for the chrominance component
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param tc              tc value
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 \param bitDepthChroma  chroma bit depth
 */
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
inline void LoopFilter::xPelFilterChroma( Pel* piSrc, const int iOffset, const int tc, const bool sw, const bool bPartPNoFilter, const bool bPartQNoFilter, const ClpRng& clpRng, const bool largeBoundary ) const
#else
inline void LoopFilter::xPelFilterChroma( Pel* piSrc, const int iOffset, const int tc, const bool bPartPNoFilter, const bool bPartQNoFilter, const ClpRng& clpRng ) const
#endif
{
  int delta;

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  const Pel m0 = piSrc[-iOffset * 4];
  const Pel m1 = piSrc[-iOffset * 3];
  const Pel m2 = piSrc[-iOffset * 2];
  const Pel m3 = piSrc[-iOffset];
  const Pel m4 = piSrc[0];
  const Pel m5 = piSrc[iOffset];
  const Pel m6 = piSrc[iOffset * 2];
  const Pel m7 = piSrc[iOffset * 3];

  if (sw)
  {
      piSrc[-iOffset * 3] = Clip3(m1 - tc, m1 + tc, ((3 * m0 + 2 * m1 + m2 + m3 + m4 + 4) >> 3));       // p2
      piSrc[-iOffset * 2] = Clip3(m2 - tc, m2 + tc, ((2 * m0 + m1 + 2 * m2 + m3 + m4 + m5 + 4) >> 3));  // p1
      piSrc[-iOffset * 1] = Clip3(m3 - tc, m3 + tc, ((m0 + m1 + m2 + 2 * m3 + m4 + m5 + m6 + 4) >> 3)); // p0
      piSrc[0]            = Clip3(m4 - tc, m4 + tc, ((m1 + m2 + m3 + 2 * m4 + m5 + m6 + m7 + 4) >> 3)); // q0
      piSrc[iOffset * 1]  = Clip3(m5 - tc, m5 + tc, ((m2 + m3 + m4 + 2 * m5 + m6 + 2 * m7 + 4) >> 3));  // q1
      piSrc[iOffset * 2]  = Clip3(m6 - tc, m6 + tc, ((m3 + m4 + m5 + 2 * m6 + 3 * m7 + 4) >> 3));       // q2
  }
  else
  {
      delta = Clip3(-tc, tc, ((((m4 - m3) << 2) + m2 - m5 + 4) >> 3));
      piSrc[-iOffset] = ClipPel(m3 + delta, clpRng);
      piSrc[0] = ClipPel(m4 - delta, clpRng);
  }

#else
  const Pel m4  = piSrc[ 0          ];
  const Pel m3  = piSrc[-iOffset    ];
  const Pel m5  = piSrc[ iOffset    ];
  const Pel m2  = piSrc[-iOffset * 2];

  delta           = Clip3( -tc, tc, ( ( ( ( m4 - m3 ) << 2 ) + m2 - m5 + 4 ) >> 3 ) );
  piSrc[-iOffset] = ClipPel( m3 + delta, clpRng );
  piSrc[ 0      ] = ClipPel( m4 - delta, clpRng );
#endif

  if( bPartPNoFilter )
  {
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    if (largeBoundary)
    {
      piSrc[-iOffset * 3] = m1; // p2
      piSrc[-iOffset * 2] = m2; // p1
    }
#endif
    piSrc[-iOffset] = m3;
  }
  if( bPartQNoFilter )
  {
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
    if (largeBoundary)
    {
      piSrc[iOffset * 1] = m5; // q1
      piSrc[iOffset * 2] = m6; // q2
    }
#endif
    piSrc[ 0      ] = m4;
  }
}

/**
 - Decision between strong and weak filter
 .
 \param offset         offset value for picture data
 \param d               d value
 \param beta            beta value
 \param tc              tc value
 \param piSrc           pointer to picture data
 */
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
inline bool LoopFilter::xUseStrongFiltering( Pel* piSrc, const int iOffset, const int d, const int beta, const int tc, bool sidePisLarge, bool sideQisLarge, int maxFilterLengthP, int maxFilterLengthQ ) const
#else
inline bool LoopFilter::xUseStrongFiltering( Pel* piSrc, const int iOffset, const int d, const int beta, const int tc ) const
#endif
{
  const Pel m4 = piSrc[ 0          ];
  const Pel m3 = piSrc[-iOffset    ];
  const Pel m7 = piSrc[ iOffset * 3];
  const Pel m0 = piSrc[-iOffset * 4];
#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  int       sp3      = abs(m0 - m3);
  int       sq3      = abs(m7 - m4);
  const int d_strong = sp3 + sq3;
#else
  const int d_strong = abs( m0 - m3 ) + abs( m7 - m4 );
#endif

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
  if (sidePisLarge || sideQisLarge)
  {
    Pel mP4;
    Pel m11;
    if (maxFilterLengthP == 5)
    {
      mP4 = piSrc[-iOffset * 6];
    }
    else
    {
      mP4 = piSrc[-iOffset * 8];
    }
    if (maxFilterLengthQ == 5)
    {
      m11 = piSrc[iOffset * 5];
    }
    else
    {
      m11 = piSrc[iOffset * 7];
    }

    if (sidePisLarge)
    {
      sp3 = (sp3 + abs(m0 - mP4) + 1) >> 1;
    }
    if (sideQisLarge)
    {
      sq3 = (sq3 + abs(m11 - m7) + 1) >> 1;
    }
    return ((sp3 + sq3) < (beta*3 >> 5)) && (d < (beta >> 2)) && (abs(m3 - m4) < ((tc * 5 + 1) >> 1));
  }
  else
#endif
  return ( ( d_strong < ( beta >> 3 ) ) && ( d < ( beta >> 2 ) ) && ( abs( m3 - m4 ) < ( ( tc * 5 + 1 ) >> 1 ) ) );
}

inline int LoopFilter::xCalcDP( Pel* piSrc, const int iOffset ) const
{
  return abs( piSrc[-iOffset * 3] - 2 * piSrc[-iOffset * 2] + piSrc[-iOffset] );
}

inline int LoopFilter::xCalcDQ( Pel* piSrc, const int iOffset ) const
{
  return abs( piSrc[0] - 2 * piSrc[iOffset] + piSrc[iOffset * 2] );
}

#if JVET_M0471_LONG_DEBLOCKING_FILTERS
inline unsigned LoopFilter::BsSet(unsigned val, const ComponentID compIdx) const { return (val << (compIdx << 1)); }
inline unsigned LoopFilter::BsGet(unsigned val, const ComponentID compIdx) const { return ((val >> (compIdx << 1)) & 3); }
#endif

//! \}
