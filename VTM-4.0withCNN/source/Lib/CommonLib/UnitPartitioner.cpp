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

/** \file     UnitPartitioner.h
 *  \brief    Provides a class for partitioning management
 */

#include "UnitPartitioner.h"

#include "CodingStructure.h"
#include "Unit.h"
#include "Slice.h"
#include "UnitTools.h"
#include "Picture.h"


PartLevel::PartLevel()
: split               ( CU_DONT_SPLIT )
, parts               (               )
, idx                 ( 0u            )
, checkdIfImplicit    ( false         )
, isImplicit          ( false         )
, implicitSplit       ( CU_DONT_SPLIT )
, firstSubPartSplit   ( CU_DONT_SPLIT )
, canQtSplit          ( true          )
{
}

PartLevel::PartLevel( const PartSplit _split, const Partitioning& _parts )
: split               ( _split        )
, parts               ( _parts        )
, idx                 ( 0u            )
, checkdIfImplicit    ( false         )
, isImplicit          ( false         )
, implicitSplit       ( CU_DONT_SPLIT )
, firstSubPartSplit   ( CU_DONT_SPLIT )
, canQtSplit          ( true          )
{
}

PartLevel::PartLevel( const PartSplit _split, Partitioning&& _parts )
: split               ( _split                               )
, parts               ( std::forward<Partitioning>( _parts ) )
, idx                 ( 0u                                   )
, checkdIfImplicit    ( false                                )
, isImplicit          ( false                                )
, implicitSplit       ( CU_DONT_SPLIT                        )
, firstSubPartSplit   ( CU_DONT_SPLIT                        )
, canQtSplit          ( true                                 )
{
}

//////////////////////////////////////////////////////////////////////////
// Partitioner class
//////////////////////////////////////////////////////////////////////////

SplitSeries Partitioner::getSplitSeries() const
{
  SplitSeries splitSeries = 0;
  SplitSeries depth = 0;

  for( const auto &level : m_partStack )
  {
    if( level.split == CTU_LEVEL ) continue;
    else splitSeries += static_cast< SplitSeries >( level.split ) << ( depth * SPLIT_DMULT );

    depth++;
  }

  return splitSeries;
}

void Partitioner::setCUData( CodingUnit& cu )
{
  cu.depth       = currDepth;
  cu.btDepth     = currBtDepth;
  cu.mtDepth     = currMtDepth;
  cu.qtDepth     = currQtDepth;
  cu.splitSeries = getSplitSeries();
}

void Partitioner::copyState( const Partitioner& other )
{
  m_partStack = other.m_partStack;
  currBtDepth = other.currBtDepth;
  currQtDepth = other.currQtDepth;
  currDepth   = other.currDepth;
  currMtDepth = other.currMtDepth;
  currTrDepth = other.currTrDepth;
  currImplicitBtDepth
              = other.currImplicitBtDepth;
  chType      = other.chType;
#ifdef _DEBUG
  m_currArea  = other.m_currArea;
#endif
}

//////////////////////////////////////////////////////////////////////////
// AdaptiveDepthPartitioner class
//////////////////////////////////////////////////////////////////////////

void AdaptiveDepthPartitioner::setMaxMinDepth( unsigned& minDepth, unsigned& maxDepth, const CodingStructure& cs ) const
{
  unsigned          stdMinDepth = 0;
  unsigned          stdMaxDepth = ( g_aucLog2[cs.sps->getCTUSize()] - g_aucLog2[cs.sps->getMinQTSize( cs.slice->getSliceType(), chType )]);
  const Position    pos         = currArea().blocks[chType].pos();
  const unsigned    curSliceIdx = cs.slice->getIndependentSliceIdx();
#if HEVC_TILES_WPP
  const unsigned    curTileIdx  = cs.picture->tileMap->getTileIdxMap( currArea().lumaPos() );

  const CodingUnit* cuLeft        = cs.getCURestricted( pos.offset( -1,                               0 ), curSliceIdx, curTileIdx, chType );
  const CodingUnit* cuBelowLeft   = cs.getCURestricted( pos.offset( -1, currArea().blocks[chType].height), curSliceIdx, curTileIdx, chType );
  const CodingUnit* cuAbove       = cs.getCURestricted( pos.offset(  0,                              -1 ), curSliceIdx, curTileIdx, chType );
  const CodingUnit* cuAboveRight  = cs.getCURestricted( pos.offset( currArea().blocks[chType].width, -1 ), curSliceIdx, curTileIdx, chType );
#else
  const CodingUnit* cuLeft        = cs.getCURestricted( pos.offset( -1,                               0 ), curSliceIdx, chType );
  const CodingUnit* cuBelowLeft   = cs.getCURestricted( pos.offset( -1, currArea().blocks[chType].height), curSliceIdx, chType );
  const CodingUnit* cuAbove       = cs.getCURestricted( pos.offset(  0,                              -1 ), curSliceIdx, chType );
  const CodingUnit* cuAboveRight  = cs.getCURestricted( pos.offset( currArea().blocks[chType].width, -1 ), curSliceIdx, chType );
#endif

  minDepth = stdMaxDepth;
  maxDepth = stdMinDepth;

  if( cuLeft )
  {
    minDepth = std::min<unsigned>( minDepth, cuLeft->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuLeft->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuBelowLeft )
  {
    minDepth = std::min<unsigned>( minDepth, cuBelowLeft->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuBelowLeft->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuAbove )
  {
    minDepth = std::min<unsigned>( minDepth, cuAbove->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuAbove->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuAboveRight )
  {
    minDepth = std::min<unsigned>( minDepth, cuAboveRight->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuAboveRight->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  minDepth = ( minDepth >= 1 ? minDepth - 1 : 0 );
  maxDepth = std::min<unsigned>( stdMaxDepth, maxDepth + 1 );
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// QTBTPartitioner
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void QTBTPartitioner::initCtu( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice )
{
#if _DEBUG
  m_currArea = ctuArea;
#endif
  currDepth   = 0;
  currTrDepth = 0;
  currBtDepth = 0;
  currMtDepth = 0;
  currQtDepth = 0;
  currImplicitBtDepth = 0;
  chType      = _chType;

  m_partStack.clear();
  m_partStack.push_back( PartLevel( CTU_LEVEL, Partitioning{ ctuArea } ) );
}

void QTBTPartitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  CHECKD( !canSplit( split, cs ), "Trying to apply a prohibited split!" );

  bool isImplicit = isSplitImplicit( split, cs );
  bool canQtSplit = canSplit( CU_QUAD_SPLIT, cs );

  switch( split )
  {
  case CU_QUAD_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs ) ) );
    break;
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs, split ) ) );
    break;
  case CU_TRIH_SPLIT:
  case CU_TRIV_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getCUSubPartitions( currArea(), cs, split ) ) );
    break;
  case TU_MAX_TR_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getMaxTuTiling( currArea(), cs ) ) );
    break;
#if JVET_M0140_SBT
  case SBT_VER_HALF_POS0_SPLIT:
  case SBT_VER_HALF_POS1_SPLIT:
  case SBT_HOR_HALF_POS0_SPLIT:
  case SBT_HOR_HALF_POS1_SPLIT:
  case SBT_VER_QUAD_POS0_SPLIT:
  case SBT_VER_QUAD_POS1_SPLIT:
  case SBT_HOR_QUAD_POS0_SPLIT:
  case SBT_HOR_QUAD_POS1_SPLIT:
    m_partStack.push_back( PartLevel( split, PartitionerImpl::getSbtTuTiling( currArea(), cs, split ) ) );
    break;
#endif
  default:
    THROW( "Unknown split mode" );
    break;
  }

  currDepth++;
#if _DEBUG
  m_currArea = m_partStack.back().parts.front();
#endif

  if( split == TU_MAX_TR_SPLIT )
  {
    currTrDepth++;
  }
#if JVET_M0140_SBT
  else if( split >= SBT_VER_HALF_POS0_SPLIT && split <= SBT_HOR_QUAD_POS1_SPLIT )
  {
    currTrDepth++;
  }
#endif
  else
  {
    currTrDepth = 0;
  }

  if( split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT )
  {
    currBtDepth++;
    if( isImplicit ) currImplicitBtDepth++;
    currMtDepth++;

    if( split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT )
    {
      // first and last part of triple split are equivalent to double bt split
      currBtDepth++;
    }
    m_partStack.back().canQtSplit = canQtSplit;
  }
  else if( split == CU_QUAD_SPLIT )
  {
    VTMCHECK( currBtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    VTMCHECK( currMtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    currMtDepth = 0;
    currBtDepth = 0;
    currQtDepth++;
  }
}

#if JVET_M0421_SPLIT_SIG
void QTBTPartitioner::canSplit( const CodingStructure &cs, bool& canNo, bool& canQt, bool& canBh, bool& canBv, bool& canTh, bool& canTv )
{
  const PartSplit implicitSplit = m_partStack.back().checkdIfImplicit ? m_partStack.back().implicitSplit : getImplicitSplit( cs );

  const unsigned maxBTD         = cs.pcv->getMaxBtDepth( *cs.slice, chType ) + currImplicitBtDepth;
  const unsigned maxBtSize      = cs.pcv->getMaxBtSize ( *cs.slice, chType );
  const unsigned minBtSize      = cs.pcv->getMinBtSize ( *cs.slice, chType );
  const unsigned maxTtSize      = cs.pcv->getMaxTtSize ( *cs.slice, chType );
  const unsigned minTtSize      = cs.pcv->getMinTtSize ( *cs.slice, chType );
  const unsigned minQtSize      = cs.pcv->getMinQtSize ( *cs.slice, chType );

  canNo = canQt = canBh = canTh = canBv = canTv = true;
  bool canBtt = currMtDepth < maxBTD;

  // the minimal and maximal sizes are given in luma samples
  const CompArea&  area  = currArea().Y();
        PartLevel& level = m_partStack.back();

  const PartSplit lastSplit = level.split;
  const PartSplit parlSplit = lastSplit == CU_TRIH_SPLIT ? CU_HORZ_SPLIT : CU_VERT_SPLIT;

  // don't allow QT-splitting below a BT split
  if( lastSplit != CTU_LEVEL && lastSplit != CU_QUAD_SPLIT ) canQt = false;
  if( area.width <= minQtSize )                              canQt = false;

  if( implicitSplit != CU_DONT_SPLIT )
  {
    canNo = canTh = canTv = false;

    canBh = implicitSplit == CU_HORZ_SPLIT;
    canBv = implicitSplit == CU_VERT_SPLIT;

    return;
  }

  if( ( lastSplit == CU_TRIH_SPLIT || lastSplit == CU_TRIV_SPLIT ) && currPartIdx() == 1 )
  {
    canBh = parlSplit != CU_HORZ_SPLIT;
    canBv = parlSplit != CU_VERT_SPLIT;
  }

  if( canBtt && ( area.width <= minBtSize && area.height <= minBtSize )
      && ( ( area.width <= minTtSize && area.height <= minTtSize ) ) )
  {
    canBtt = false;
  }
  if( canBtt && ( area.width > maxBtSize || area.height > maxBtSize )
      && ( ( area.width > maxTtSize || area.height > maxTtSize ) ) )
  {
    canBtt = false;
  }

  if( !canBtt )
  {
    canBh = canTh = canBv = canTv = false;

    return;
  }

  // specific check for BT splits
  if( area.height <= minBtSize || area.height > maxBtSize )                            canBh = false;
  if( area.width > MAX_TU_SIZE_FOR_PROFILE && area.height <= MAX_TU_SIZE_FOR_PROFILE ) canBh = false;

  if( area.width <= minBtSize || area.width > maxBtSize )                              canBv = false;
  if( area.width <= MAX_TU_SIZE_FOR_PROFILE && area.height > MAX_TU_SIZE_FOR_PROFILE ) canBv = false;

  if( area.height <= 2 * minTtSize || area.height > maxTtSize || area.width > maxTtSize )
                                                                                       canTh = false;
  if( area.width > MAX_TU_SIZE_FOR_PROFILE || area.height > MAX_TU_SIZE_FOR_PROFILE )  canTh = false;

  if( area.width <= 2 * minTtSize || area.width > maxTtSize || area.height > maxTtSize )
                                                                                       canTv = false;
  if( area.width > MAX_TU_SIZE_FOR_PROFILE || area.height > MAX_TU_SIZE_FOR_PROFILE )  canTv = false;
}

#endif
bool QTBTPartitioner::canSplit( const PartSplit split, const CodingStructure &cs )
{
#if JVET_M0421_SPLIT_SIG
  const CompArea area       = currArea().Y();
  const unsigned maxTrSize  = cs.sps->getMaxTrSize();

  bool canNo, canQt, canBh, canTh, canBv, canTv;

  canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );
#else
  const PartSplit implicitSplit = getImplicitSplit( cs );

  // the minimal and maximal sizes are given in luma samples
  const CompArea area           = currArea().Y();

  const unsigned maxBTD         = cs.pcv->getMaxBtDepth( *cs.slice, chType ) + currImplicitBtDepth;
  const unsigned maxBtSize      = cs.pcv->getMaxBtSize( *cs.slice, chType );
  const unsigned minBtSize      = cs.pcv->getMinBtSize( *cs.slice, chType );
  const unsigned maxTtSize      = cs.pcv->getMaxTtSize( *cs.slice, chType );
  const unsigned minTtSize      = cs.pcv->getMinTtSize( *cs.slice, chType );
  const unsigned maxTrSize      = cs.sps->getMaxTrSize();

  const PartSplit lastSplit = m_partStack.back().split;
  const PartSplit parlSplit = lastSplit == CU_TRIH_SPLIT ? CU_HORZ_SPLIT : CU_VERT_SPLIT;

  if( isNonLog2BlockSize( currArea().Y() ) )
  {
    return false;
  }

#endif
  switch( split )
  {
  case CTU_LEVEL:
    THROW( "Checking if top level split is possible" );
    return true;
    break;
  case TU_MAX_TR_SPLIT:
    return area.width > maxTrSize || area.height > maxTrSize;
    break;
#if JVET_M0140_SBT
  case SBT_VER_HALF_POS0_SPLIT:
  case SBT_VER_HALF_POS1_SPLIT:
  case SBT_HOR_HALF_POS0_SPLIT:
  case SBT_HOR_HALF_POS1_SPLIT:
  case SBT_VER_QUAD_POS0_SPLIT:
  case SBT_VER_QUAD_POS1_SPLIT:
  case SBT_HOR_QUAD_POS0_SPLIT:
  case SBT_HOR_QUAD_POS1_SPLIT:
    return currTrDepth == 0;
    break;
#endif
#if JVET_M0421_SPLIT_SIG
  case CU_QUAD_SPLIT:
    return canQt;
  case CU_DONT_SPLIT:
    return canNo;
  case CU_HORZ_SPLIT:
    return canBh;
  case CU_VERT_SPLIT:
    return canBv;
  case CU_TRIH_SPLIT:
    return canTh;
  case CU_TRIV_SPLIT:
    return canTv;
#else
  case CU_QUAD_SPLIT:
  {
    // don't allow QT-splitting below a BT split
    PartSplit lastSplit = m_partStack.back().split;
    if( lastSplit != CTU_LEVEL && lastSplit != CU_QUAD_SPLIT )                  return false;

    unsigned minQtSize = cs.pcv->getMinQtSize( *cs.slice, chType );
    if( currArea().lwidth() <= minQtSize || currArea().lheight() <= minQtSize ) return false;

    // allowing QT split even if a BT split is implied
    if( implicitSplit != CU_DONT_SPLIT )                                        return true;

    return true;
  }
  break;
  case CU_DONT_SPLIT:
    return implicitSplit == CU_DONT_SPLIT;
    break;
  // general check for BT split, specific checks are done in a separate switch
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
  {
    if( ( lastSplit == CU_TRIH_SPLIT || lastSplit == CU_TRIV_SPLIT ) && currPartIdx() == 1 && split == parlSplit )
    {
      return false;
    }
    if (CS::isDualITree(cs) && (area.width > 64 || area.height > 64))
    {
      return false;
    }
  }
  case CU_TRIH_SPLIT:
  case CU_TRIV_SPLIT:
  {
    if (CS::isDualITree(cs) && (area.width > 64 || area.height > 64))
    {
      return false;
    }
  }
    if( implicitSplit == split )                                   return true;
    if( implicitSplit != CU_DONT_SPLIT && implicitSplit != split ) return false;
#endif
  case CU_MT_SPLIT:
#if JVET_M0421_SPLIT_SIG
    return ( canBh || canTh || canBv || canTv );
#endif
  case CU_BT_SPLIT:
#if JVET_M0421_SPLIT_SIG
    return ( canBh || canBv );
#else
  {
    if( currMtDepth >= maxBTD )                               return false;
    if(      ( area.width <= minBtSize && area.height <= minBtSize )
        && ( ( area.width <= minTtSize && area.height <= minTtSize ) ) ) return false;
    if(      ( area.width > maxBtSize || area.height > maxBtSize )
        && ( ( area.width > maxTtSize || area.height > maxTtSize ) ) ) return false;
    if (CS::isDualITree(cs) && (area.width > 64 || area.height > 64))
    {
      return false;
    }
  }
#endif
  break;
  default:
    THROW( "Unknown split mode" );
    return false;
    break;
  }
#if !JVET_M0421_SPLIT_SIG
  // specific check for BT splits
  switch( split )
  {
  case CU_HORZ_SPLIT:
    if( area.height <= minBtSize || area.height > maxBtSize )     return false;
    if( area.width > MAX_TU_SIZE_FOR_PROFILE && area.height <= MAX_TU_SIZE_FOR_PROFILE ) return false;
    break;
  case CU_VERT_SPLIT:
    if( area.width <= minBtSize || area.width > maxBtSize )       return false;
    if( area.width <= MAX_TU_SIZE_FOR_PROFILE && area.height > MAX_TU_SIZE_FOR_PROFILE ) return false;
    break;
  case CU_TRIH_SPLIT:
    if( area.height <= 2 * minTtSize || area.height > maxTtSize || area.width > maxTtSize) return false;
    if( area.width > MAX_TU_SIZE_FOR_PROFILE || area.height > MAX_TU_SIZE_FOR_PROFILE ) return false;
    break;
  case CU_TRIV_SPLIT:
    if( area.width <= 2 * minTtSize || area.width > maxTtSize || area.height > maxTtSize)  return false;
    if( area.width > MAX_TU_SIZE_FOR_PROFILE || area.height > MAX_TU_SIZE_FOR_PROFILE ) return false;
    break;
  default:
    break;
  }
#endif

  return true;
}

bool QTBTPartitioner::isSplitImplicit( const PartSplit split, const CodingStructure &cs )
{
  return split == getImplicitSplit( cs );
}

PartSplit QTBTPartitioner::getImplicitSplit( const CodingStructure &cs )
{
  if( m_partStack.back().checkdIfImplicit )
  {
    return m_partStack.back().implicitSplit;
  }

  PartSplit split = CU_DONT_SPLIT;

  if( split == CU_DONT_SPLIT )
  {
    const bool isBlInPic = cs.picture->Y().contains( currArea().Y().bottomLeft() );
    const bool isTrInPic = cs.picture->Y().contains( currArea().Y().topRight() );

    const CompArea& area      = currArea().Y();
    const unsigned maxBtSize  = cs.pcv->getMaxBtSize( *cs.slice, chType );
    const bool isBtAllowed    = area.width <= maxBtSize && area.height <= maxBtSize;
    const unsigned minQtSize  = cs.pcv->getMinQtSize( *cs.slice, chType );
    const bool isQtAllowed    = area.width >  minQtSize && area.height >  minQtSize && currBtDepth == 0;

    if( !isBlInPic && !isTrInPic && isQtAllowed )
    {
      split = CU_QUAD_SPLIT;
    }
    else if( !isBlInPic && isBtAllowed )
    {
      split = CU_HORZ_SPLIT;
    }
    else if( !isTrInPic && isBtAllowed )
    {
      split = CU_VERT_SPLIT;
    }
    else if( !isBlInPic || !isTrInPic )
    {
      split = CU_QUAD_SPLIT;
    }
    if (CS::isDualITree(cs) && (currArea().Y().width > 64 || currArea().Y().height > 64))
    {
      split = CU_QUAD_SPLIT;
    }
#if JVET_M0446_M0888_M0905_VPDU_AT_PIC_BOUNDARY
    if ((!isBlInPic || !isTrInPic) && (currArea().Y().width > MAX_TU_SIZE_FOR_PROFILE || currArea().Y().height > MAX_TU_SIZE_FOR_PROFILE))
    {
      split = CU_QUAD_SPLIT;
    }
#endif
  }

  m_partStack.back().checkdIfImplicit = true;
  m_partStack.back().isImplicit = split != CU_DONT_SPLIT;
  m_partStack.back().implicitSplit = split;

  return split;
}

void QTBTPartitioner::exitCurrSplit()
{
  PartSplit currSplit = m_partStack.back().split;
  unsigned  currIdx = m_partStack.back().idx;

  m_partStack.pop_back();

  VTMCHECK( currDepth == 0, "depth is '0', although a split was performed" );
  currDepth--;
#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  if( currSplit == CU_HORZ_SPLIT || currSplit == CU_VERT_SPLIT || currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT )
  {
    VTMCHECK( !m_partStack.back().checkdIfImplicit, "Didn't check if the current split is implicit" );
    VTMCHECK( currBtDepth == 0, "BT depth is '0', athough a BT split was performed" );
    VTMCHECK( currMtDepth == 0, "MT depth is '0', athough a BT split was performed" );
    currMtDepth--;
    if( m_partStack.back().isImplicit ) currImplicitBtDepth--;
    currBtDepth--;
    if( ( currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT ) && currIdx != 1 )
    {
      VTMCHECK( currBtDepth == 0, "BT depth is '0', athough a TT split was performed" );
      currBtDepth--;
    }
  }
  else if( currSplit == TU_MAX_TR_SPLIT )
  {
    VTMCHECK( currTrDepth == 0, "TR depth is '0', although a TU split was performed" );
    currTrDepth--;
  }
#if JVET_M0140_SBT
  else if( currSplit >= SBT_VER_HALF_POS0_SPLIT && currSplit <= SBT_HOR_QUAD_POS1_SPLIT )
  {
    VTMCHECK( currTrDepth == 0, "TR depth is '0', although a TU split was performed" );
    currTrDepth--;
  }
#endif
  else
  {
    VTMCHECK( currTrDepth > 0, "RQT found with QTBT partitioner" );

    VTMCHECK( currQtDepth == 0, "QT depth is '0', although a QT split was performed" );
    currQtDepth--;
  }
}

bool QTBTPartitioner::nextPart( const CodingStructure &cs, bool autoPop /*= false*/ )
{
  const Position &prevPos = currArea().blocks[chType].pos();

  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit = false;
  m_partStack.back().isImplicit = false;

  if( currIdx == 1 )
  {
    const CodingUnit* prevCU = cs.getCU( prevPos, chType );
    m_partStack.back().firstSubPartSplit = prevCU ? CU::getSplitAtDepth( *prevCU, currDepth ) : CU_DONT_SPLIT;
  }

  if( currIdx < m_partStack.back().parts.size() )
  {
    if( m_partStack.back().split == CU_TRIH_SPLIT || m_partStack.back().split == CU_TRIV_SPLIT )
    {
      // adapt the current bt depth
      if( currIdx == 1 ) currBtDepth--;
      else               currBtDepth++;
    }
#if _DEBUG
    m_currArea = m_partStack.back().parts[currIdx];
#endif
    return true;
  }
  else
  {
    if( autoPop ) exitCurrSplit();
    return false;
  }
}

bool QTBTPartitioner::hasNextPart()
{
  return ( ( m_partStack.back().idx + 1 ) < m_partStack.back().parts.size() );
}

#if JVET_M0102_INTRA_SUBPARTITIONS
void TUIntraSubPartitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  switch( split )
  {
    case TU_1D_HORZ_SPLIT:
    case TU_1D_VERT_SPLIT:
    {
      const UnitArea &area = currArea();
      m_partStack.push_back( PartLevel() );
      m_partStack.back().split = split;
      PartitionerImpl::getTUIntraSubPartitions( m_partStack.back().parts, area, cs, split );
      break;
    }
    case TU_MAX_TR_SPLIT: //we need this non ISP split because of the maxTrSize limitation
      m_partStack.push_back( PartLevel( split, PartitionerImpl::getMaxTuTiling( currArea(), cs ) ) );
      break;
    default:
      THROW( "Unknown ISP split mode" );
      break;
  }

  currDepth++;
  currTrDepth++; // we need this to identify the level. since the 1d partitions are forbidden if the RQT is on, there area no compatibility issues

#if _DEBUG
  m_currArea = m_partStack.back().parts.front();
#endif
}

void TUIntraSubPartitioner::exitCurrSplit()
{
  PartSplit currSplit = m_partStack.back().split;

  m_partStack.pop_back();

  VTMCHECK( currDepth == 0, "depth is '0', although a split was performed" );

  currDepth--;
  currTrDepth--;

#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  VTMCHECK( !( currSplit == TU_1D_HORZ_SPLIT || currSplit == TU_1D_VERT_SPLIT || currSplit == TU_MAX_TR_SPLIT ), "Unknown 1D partition split type!" );
}

bool TUIntraSubPartitioner::nextPart( const CodingStructure &cs, bool autoPop /*= false*/ )
{
  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit = false;
  m_partStack.back().isImplicit = false;

  if( currIdx < m_partStack.back().parts.size() )
  {
#if _DEBUG
    m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif
    return true;
  }
  else
  {
    if( autoPop ) exitCurrSplit();
    return false;
  }
}

bool TUIntraSubPartitioner::hasNextPart()
{
  return ( ( m_partStack.back().idx + 1 ) < m_partStack.back().parts.size() );
}

bool TUIntraSubPartitioner::canSplit( const PartSplit split, const CodingStructure &cs )
{
  //const PartSplit implicitSplit = getImplicitSplit(cs);
  const UnitArea &area = currArea();

  switch( split )
  {
    case TU_1D_HORZ_SPLIT:
    {
      return area.lheight() == m_partStack[0].parts[0].lheight();
    }
    case TU_1D_VERT_SPLIT:
    {
      return area.lwidth() == m_partStack[0].parts[0].lwidth();
    }
    case TU_MAX_TR_SPLIT:
    {
      //this split is performed implicitly with the other splits
      return false;
    }
    default:
      THROW( "Unknown 1-D split mode" );
      break;
  }
}
#endif


//////////////////////////////////////////////////////////////////////////
// PartitionerFactory
//////////////////////////////////////////////////////////////////////////

Partitioner* PartitionerFactory::get( const Slice& slice )
{
  return new QTBTPartitioner;
}

//////////////////////////////////////////////////////////////////////////
// Partitioner methods describing the actual partitioning logic
//////////////////////////////////////////////////////////////////////////

Partitioning PartitionerImpl::getCUSubPartitions( const UnitArea &cuArea, const CodingStructure &cs, const PartSplit _splitType /*= CU_QUAD_SPLIT*/ )
{
  const PartSplit splitType = _splitType;

  if( splitType == CU_QUAD_SPLIT )
  {
    if( !cs.pcv->noChroma2x2 )
    {
      Partitioning sub;

      sub.resize( 4, cuArea );

      for( uint32_t i = 0; i < 4; i++ )
      {
        for( auto &blk : sub[i].blocks )
        {
          blk.height >>= 1;
          blk.width  >>= 1;
          if( i >= 2 ) blk.y += blk.height;
          if( i &  1 ) blk.x += blk.width;
        }

        VTMCHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
      }

      return sub;
    }
    else
    {
      const uint32_t minCUSize = ( cs.sps->getMaxCUWidth() >> cs.sps->getMaxCodingDepth() );

      bool canSplit = cuArea.lumaSize().width > minCUSize && cuArea.lumaSize().height > minCUSize;

      Partitioning ret;

      if( cs.slice->getSliceType() == I_SLICE )
      {
        canSplit &= cuArea.lumaSize().width > cs.pcv->minCUWidth && cuArea.lumaSize().height > cs.pcv->minCUHeight;
      }

      if( canSplit )
      {
        ret.resize( 4 );

        if( cuArea.chromaFormat == CHROMA_400 )
        {
          CompArea  blkY = cuArea.Y();
          blkY.width >>= 1;
          blkY.height >>= 1;
          ret[0]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x += blkY.width;
          ret[1]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x -= blkY.width;
          blkY.y += blkY.height;
          ret[2]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x += blkY.width;
          ret[3]  = UnitArea( cuArea.chromaFormat, blkY );
        }
        else
        {
          for( uint32_t i = 0; i < 4; i++ )
          {
            ret[i] = cuArea;

            CompArea &blkY  = ret[i].Y();
            CompArea &blkCb = ret[i].Cb();
            CompArea &blkCr = ret[i].Cr();

            blkY.width  /= 2;
            blkY.height /= 2;

            // TODO: get those params from SPS
            if( blkCb.width > 4 )
            {
              blkCb.width  /= 2;
              blkCb.height /= 2;
              blkCr.width  /= 2;
              blkCr.height /= 2;
            }
            else if( i > 0 )
            {
              blkCb = CompArea();
              blkCr = CompArea();
            }

            if( ( i & 1 ) == 1 )
            {
              blkY.x  += blkY .width;
              blkCb.x += blkCb.width;
              blkCr.x += blkCr.width;
            }

            if( i > 1 )
            {
              blkY.y  += blkY .height;
              blkCb.y += blkCb.height;
              blkCr.y += blkCr.height;
            }
          }
        }
      }

      return ret;
    }
  }
  else if( splitType == CU_HORZ_SPLIT )
  {
    Partitioning sub;

    sub.resize(2, cuArea);

    for (uint32_t i = 0; i < 2; i++)
    {
      for (auto &blk : sub[i].blocks)
      {
        blk.height >>= 1;
        if (i == 1) blk.y += blk.height;
      }

      VTMCHECK(sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size");
    }

    return sub;
  }
  else if( splitType == CU_VERT_SPLIT )
  {
    Partitioning sub;

    sub.resize( 2, cuArea );

    for( uint32_t i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;
        if( i == 1 ) blk.x += blk.width;
      }

      VTMCHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else if( splitType == CU_TRIH_SPLIT )
  {
    Partitioning sub;

    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 1;
        if( ( i + 1 ) & 1 ) blk.height >>= 1;
        if( i == 1 )        blk.y       +=     blk.height / 2;
        if( i == 2 )        blk.y       += 3 * blk.height;
      }

      VTMCHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else if( splitType == CU_TRIV_SPLIT )
  {
    Partitioning sub;

    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;

        if( ( i + 1 ) & 1 ) blk.width >>= 1;
        if( i == 1 )        blk.x      +=     blk.width / 2;
        if( i == 2 )        blk.x      += 3 * blk.width;
      }

      VTMCHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return sub;
  }
  else
  {
    THROW( "Unknown CU sub-partitioning" );
    return Partitioning();
  }
}

#if JVET_M0102_INTRA_SUBPARTITIONS
void PartitionerImpl::getTUIntraSubPartitions( Partitioning &sub, const UnitArea &tuArea, const CodingStructure &cs, const PartSplit splitType )
{
  uint32_t nPartitions;
  uint32_t splitDimensionSize = CU::getISPSplitDim( tuArea.lumaSize().width, tuArea.lumaSize().height, splitType );

  bool isDualTree = CS::isDualITree( cs );

  if( splitType == TU_1D_HORZ_SPLIT )
  {
    nPartitions = tuArea.lumaSize().height >> g_aucLog2[splitDimensionSize];

    sub.resize( nPartitions );

    for( uint32_t i = 0; i < nPartitions; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMPONENT_Y];

      blkY.height = splitDimensionSize;
      blkY.y = i > 0 ? sub[i - 1].blocks[COMPONENT_Y].y + splitDimensionSize : blkY.y;

      VTMCHECK( sub[i].lumaSize().height < 1, "the cs split causes the block to be smaller than the minimal TU size" );
    }
  }
  else if( splitType == TU_1D_VERT_SPLIT )
  {
    nPartitions = tuArea.lumaSize().width >> g_aucLog2[splitDimensionSize];

    sub.resize( nPartitions );

    for( uint32_t i = 0; i < nPartitions; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMPONENT_Y];

      blkY.width = splitDimensionSize;
      blkY.x = i > 0 ? sub[i - 1].blocks[COMPONENT_Y].x + splitDimensionSize : blkY.x;
      VTMCHECK( sub[i].lumaSize().width < 1, "the split causes the block to be smaller than the minimal TU size" );
    }
  }
  else
  {
    THROW( "Unknown TU sub-partitioning" );
  }
  //we only partition luma, so there is going to be only one chroma tu at the end (unless it is dual tree, in which case there won't be any chroma components)
  uint32_t partitionsWithoutChroma = isDualTree ? nPartitions : nPartitions - 1;
  for( uint32_t i = 0; i < partitionsWithoutChroma; i++ )
  {
    CompArea& blkCb = sub[i].blocks[COMPONENT_Cb];
    CompArea& blkCr = sub[i].blocks[COMPONENT_Cr];
    blkCb = CompArea();
    blkCr = CompArea();
  }
}
#endif

static const int g_maxRtGridSize = 3;

static const int g_zScanToX[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  1,  0,  1,  2,  3,  2,  3,
   0,  1,  0,  1,  2,  3,  2,  3,
   4,  5,  4,  5,  6,  7,  6,  7,
   4,  5,  4,  5,  6,  7,  6,  7,
   0,  1,  0,  1,  2,  3,  2,  3,
   0,  1,  0,  1,  2,  3,  2,  3,
   4,  5,  4,  5,  6,  7,  6,  7,
   4,  5,  4,  5,  6,  7,  6,  7,
};
static const int g_zScanToY[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  0,  1,  1,  0,  0,  1,  1,
   2,  2,  3,  3,  2,  2,  3,  3,
   0,  0,  1,  1,  0,  0,  1,  1,
   2,  2,  3,  3,  2,  2,  3,  3,
   4,  4,  5,  5,  4,  4,  5,  5,
   6,  6,  7,  7,  6,  5,  7,  7,
   4,  4,  5,  5,  4,  4,  5,  5,
   6,  6,  7,  7,  6,  5,  7,  7,
};
static const int g_rsScanToZ[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  1,  4,  5, 16, 17, 20, 21,
   2,  3,  6,  7, 18, 19, 22, 23,
   8,  9, 12, 13, 24, 25, 28, 29,
  10, 11, 14, 15, 26, 27, 30, 31,
  32, 33, 36, 37, 48, 49, 52, 53,
  34, 35, 38, 39, 50, 51, 54, 55,
  40, 41, 44, 45, 56, 57, 60, 61,
  42, 43, 46, 47, 58, 59, 62, 63,
};

Partitioning PartitionerImpl::getMaxTuTiling( const UnitArea &cuArea, const CodingStructure &cs )
{
  static_assert( MAX_LOG2_DIFF_CU_TR_SIZE <= g_maxRtGridSize, "Z-scan tables are only provided for MAX_LOG2_DIFF_CU_TR_SIZE for up to 3 (8x8 tiling)!" );

  const CompArea area = cuArea.Y().valid() ? cuArea.Y() : cuArea.Cb();
  const int maxTrSize = cs.sps->getMaxTrSize() >> ( isLuma( area.compID ) ? 0 : 1 );
  const int numTilesH = std::max<int>( 1, area.width  / maxTrSize );
  const int numTilesV = std::max<int>( 1, area.height / maxTrSize );
  const int numTiles  = numTilesH * numTilesV;

  VTMCHECK( numTiles > MAX_CU_TILING_PARTITIONS, "CU partitioning requires more partitions than available" );

  Partitioning ret;
  ret.resize( numTiles, cuArea );

  for( int i = 0; i < numTiles; i++ )
  {
    const int rsy = i / numTilesH;
    const int rsx = i % numTilesH;

    const int x = g_zScanToX[g_rsScanToZ[( rsy << g_maxRtGridSize ) + rsx]];
    const int y = g_zScanToY[g_rsScanToZ[( rsy << g_maxRtGridSize ) + rsx]];

    UnitArea& tile = ret[i];

    for( CompArea &comp : tile.blocks )
    {
      if( !comp.valid() ) continue;

      comp.width  /= numTilesH;
      comp.height /= numTilesV;

      comp.x += comp.width  * x;
      comp.y += comp.height * y;
    }
  }

  return ret;
}

#if JVET_M0140_SBT
Partitioning PartitionerImpl::getSbtTuTiling( const UnitArea& cuArea, const CodingStructure &cs, const PartSplit splitType )
{
  Partitioning ret;
  int numTiles = 2;
  int widthFactor, heightFactor, xOffsetFactor, yOffsetFactor; // y = (x * factor) >> 2;
  assert( splitType >= SBT_VER_HALF_POS0_SPLIT && splitType <= SBT_HOR_QUAD_POS1_SPLIT );

  ret.resize( numTiles, cuArea );
  for( int i = 0; i < numTiles; i++ )
  {
    if( splitType >= SBT_VER_QUAD_POS0_SPLIT )
    {
      if( splitType == SBT_HOR_QUAD_POS0_SPLIT || splitType == SBT_HOR_QUAD_POS1_SPLIT )
      {
        widthFactor = 4;
        xOffsetFactor = 0;
        heightFactor = ( ( i == 0 && splitType == SBT_HOR_QUAD_POS0_SPLIT ) || ( i == 1 && splitType == SBT_HOR_QUAD_POS1_SPLIT ) ) ? 1 : 3;
        yOffsetFactor = ( i == 0 ) ? 0 : ( splitType == SBT_HOR_QUAD_POS0_SPLIT ? 1 : 3 );
      }
      else
      {
        widthFactor = ( ( i == 0 && splitType == SBT_VER_QUAD_POS0_SPLIT ) || ( i == 1 && splitType == SBT_VER_QUAD_POS1_SPLIT ) ) ? 1 : 3;
        xOffsetFactor = ( i == 0 ) ? 0 : ( splitType == SBT_VER_QUAD_POS0_SPLIT ? 1 : 3 );
        heightFactor = 4;
        yOffsetFactor = 0;
      }
    }
    else
    {
      if( splitType == SBT_HOR_HALF_POS0_SPLIT || splitType == SBT_HOR_HALF_POS1_SPLIT )
      {
        widthFactor = 4;
        xOffsetFactor = 0;
        heightFactor = 2;
        yOffsetFactor = ( i == 0 ) ? 0 : 2;
      }
      else
      {
        widthFactor = 2;
        xOffsetFactor = ( i == 0 ) ? 0 : 2;
        heightFactor = 4;
        yOffsetFactor = 0;
      }
    }

    UnitArea& tile = ret[i];
    for( CompArea &comp : tile.blocks )
    {
      if( !comp.valid() ) continue;
      comp.x += ( comp.width  * xOffsetFactor ) >> 2;
      comp.y += ( comp.height * yOffsetFactor ) >> 2;
      comp.width = ( comp.width  * widthFactor ) >> 2;
      comp.height = ( comp.height * heightFactor ) >> 2;
    }
  }

  return ret;
}
#endif
