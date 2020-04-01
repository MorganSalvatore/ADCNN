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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#include "CodingStructure.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"


XUCache g_globalUnitCache = XUCache();

const UnitScale UnitScaleArray[NUM_CHROMA_FORMAT][MAX_NUM_COMPONENT] =
{
  { {2,2}, {0,0}, {0,0} },  // 4:0:0
  { {2,2}, {1,1}, {1,1} },  // 4:2:0
  { {2,2}, {1,2}, {1,2} },  // 4:2:2
  { {2,2}, {2,2}, {2,2} }   // 4:4:4
};

// ---------------------------------------------------------------------------
// coding structure method definitions
// ---------------------------------------------------------------------------

CodingStructure::CodingStructure(CUCache& cuCache, PUCache& puCache, TUCache& tuCache)
  : area      ()
  , picture   ( nullptr )
  , parent    ( nullptr )
#if JVET_M0246_AFFINE_AMVR
  , bestCS    ( nullptr )
#endif
  , m_isTuEnc ( false )
  , m_cuCache ( cuCache )
  , m_puCache ( puCache )
  , m_tuCache ( tuCache )
{
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    m_coeffs[ i ] = nullptr;
    m_pcmbuf[ i ] = nullptr;

    m_offsets[ i ] = 0;
  }

  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_cuIdx   [ i ] = nullptr;
    m_puIdx   [ i ] = nullptr;
    m_tuIdx   [ i ] = nullptr;
    m_isDecomp[ i ] = nullptr;
  }

  m_motionBuf     = nullptr;
  features.resize( NUM_ENC_FEATURES );

}

void CodingStructure::destroy()
{
  picture   = nullptr;
  parent    = nullptr;

  m_pred.destroy();
  m_resi.destroy();
  m_reco.destroy();
  m_orgr.destroy();

  destroyCoeffs();

  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    delete[] m_isDecomp[ i ];
    m_isDecomp[ i ] = nullptr;

    delete[] m_cuIdx[ i ];
    m_cuIdx[ i ] = nullptr;

    delete[] m_puIdx[ i ];
    m_puIdx[ i ] = nullptr;

    delete[] m_tuIdx[ i ];
    m_tuIdx[ i ] = nullptr;
  }

  delete[] m_motionBuf;
  m_motionBuf = nullptr;


  m_tuCache.cache( tus );
  m_puCache.cache( pus );
  m_cuCache.cache( cus );
}

void CodingStructure::releaseIntermediateData()
{
  clearTUs();
  clearPUs();
  clearCUs();
}

bool CodingStructure::isDecomp( const Position &pos, const ChannelType effChType )
{
  if( area.blocks[effChType].contains( pos ) )
  {
    return m_isDecomp[effChType][rsAddr( pos, area.blocks[effChType], area.blocks[effChType].width, unitScale[effChType] )];
  }
  else if( parent )
  {
    return parent->isDecomp( pos, effChType );
  }
  else
  {
    return false;
  }
}

bool CodingStructure::isDecomp( const Position &pos, const ChannelType effChType ) const
{
  if( area.blocks[effChType].contains( pos ) )
  {
    return m_isDecomp[effChType][rsAddr( pos, area.blocks[effChType], area.blocks[effChType].width, unitScale[effChType] )];
  }
  else if( parent )
  {
    return parent->isDecomp( pos, effChType );
  }
  else
  {
    return false;
  }
}

void CodingStructure::setDecomp(const CompArea &_area, const bool _isCoded /*= true*/)
{
  const UnitScale& scale = unitScale[_area.compID];

  AreaBuf<bool> isCodedBlk( m_isDecomp[toChannelType( _area.compID )] + rsAddr( _area, area.blocks[_area.compID].pos(), area.blocks[_area.compID].width, scale ),
                            area.blocks[_area.compID].width >> scale.posx,
                            _area.width                     >> scale.posx,
                            _area.height                    >> scale.posy);
  isCodedBlk.fill( _isCoded );
}

void CodingStructure::setDecomp(const UnitArea &_area, const bool _isCoded /*= true*/)
{
  for( uint32_t i = 0; i < _area.blocks.size(); i++ )
  {
    if( _area.blocks[i].valid() ) setDecomp( _area.blocks[i], _isCoded );
  }
}



CodingUnit* CodingStructure::getCU( const Position &pos, const ChannelType effChType )
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if( parent ) return parent->getCU( pos, effChType );
    else         return nullptr;
  }
  else
  {
    const unsigned idx = m_cuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if( idx != 0 ) return cus[ idx - 1 ];
    else           return nullptr;
  }
}

const CodingUnit* CodingStructure::getCU( const Position &pos, const ChannelType effChType ) const
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if( parent ) return parent->getCU( pos, effChType );
    else         return nullptr;
  }
  else
  {
    const unsigned idx = m_cuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if( idx != 0 ) return cus[ idx - 1 ];
    else           return nullptr;
  }
}

PredictionUnit* CodingStructure::getPU( const Position &pos, const ChannelType effChType )
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if( parent ) return parent->getPU( pos, effChType );
    else         return nullptr;
  }
  else
  {
    const unsigned idx = m_puIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if( idx != 0 ) return pus[ idx - 1 ];
    else           return nullptr;
  }
}

const PredictionUnit * CodingStructure::getPU( const Position &pos, const ChannelType effChType ) const
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if( parent ) return parent->getPU( pos, effChType );
    else         return nullptr;
  }
  else
  {
    const unsigned idx = m_puIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if( idx != 0 ) return pus[ idx - 1 ];
    else           return nullptr;
  }
}

#if JVET_M0102_INTRA_SUBPARTITIONS
TransformUnit* CodingStructure::getTU( const Position &pos, const ChannelType effChType, const int subTuIdx )
#else
TransformUnit* CodingStructure::getTU( const Position &pos, const ChannelType effChType )
#endif
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if( parent ) return parent->getTU( pos, effChType );
    else         return nullptr;
  }
  else
  {
    const unsigned idx = m_tuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

#if JVET_M0102_INTRA_SUBPARTITIONS
    if( idx != 0 )
    {
      unsigned extraIdx = 0;
      if( isLuma( effChType ) )
      {
        const TransformUnit& tu = *tus[idx - 1];

        if( tu.cu->ispMode ) // Intra SubPartitions mode
        {
          //we obtain the offset to index the corresponding sub-partition
          if( subTuIdx != -1 )
          {
            extraIdx = subTuIdx;
          }
          else
          {
            while( pos != tus[idx - 1 + extraIdx]->blocks[getFirstComponentOfChannel( effChType )].pos() )
            {
              extraIdx++;
            }
          }
        }
      }
      return tus[idx - 1 + extraIdx];
    }
#else
    if( idx != 0 )       return tus[ idx - 1 ];
#endif
    else if( m_isTuEnc ) return parent->getTU( pos, effChType );
    else                 return nullptr;
  }
}

#if JVET_M0102_INTRA_SUBPARTITIONS
const TransformUnit * CodingStructure::getTU( const Position &pos, const ChannelType effChType, const int subTuIdx ) const
#else
const TransformUnit * CodingStructure::getTU( const Position &pos, const ChannelType effChType ) const
#endif
{
  const CompArea &_blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if( parent ) return parent->getTU( pos, effChType );
    else         return nullptr;
  }
  else
  {
    const unsigned idx = m_tuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];
#if JVET_M0102_INTRA_SUBPARTITIONS
    if( idx != 0 )
    {
      unsigned extraIdx = 0;
      if( isLuma( effChType ) )
      {
        const TransformUnit& tu = *tus[idx - 1];
        if( tu.cu->ispMode ) // Intra SubPartitions mode
        {
          //we obtain the offset to index the corresponding sub-partition
          if( subTuIdx != -1 )
          {
            extraIdx = subTuIdx;
          }
          else
          {
            while( pos != tus[idx - 1 + extraIdx]->blocks[effChType].pos() )
            {
              extraIdx++;
            }
          }
        }
      }
      return tus[idx - 1 + extraIdx];
    }
#else
    if( idx != 0 )       return tus[idx - 1];
#endif
    else if( m_isTuEnc ) return parent->getTU( pos, effChType );
    else                 return nullptr;
  }
}

CodingUnit& CodingStructure::addCU( const UnitArea &unit, const ChannelType chType )
{
  CodingUnit *cu = m_cuCache.get();

  cu->UnitArea::operator=( unit );
  cu->initData();
  cu->cs        = this;
  cu->slice     = nullptr;
  cu->next      = nullptr;
  cu->firstPU   = nullptr;
  cu->lastPU    = nullptr;
  cu->firstTU   = nullptr;
  cu->lastTU    = nullptr;
  cu->chType    = chType;

  CodingUnit *prevCU = m_numCUs > 0 ? cus.back() : nullptr;

  if( prevCU )
  {
    prevCU->next = cu;
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

    VTMCHECK( prevCU->cacheId != cu->cacheId, "Inconsintent cacheId between previous and current CU" );
#endif
  }

  cus.push_back( cu );

  uint32_t idx = ++m_numCUs;
  cu->idx  = idx;

  uint32_t numCh = ::getNumberValidChannels( area.chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !cu->blocks[i].valid() )
    {
      continue;
    }

    const CompArea &_selfBlk = area.blocks[i];
    const CompArea     &_blk = cu-> blocks[i];

    const UnitScale& scale = unitScale[_blk.compID];
    const Area scaledSelf  = scale.scale( _selfBlk );
    const Area scaledBlk   = scale.scale(     _blk );
    unsigned *idxPtr       = m_cuIdx[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
    VTMCHECK( *idxPtr, "Overwriting a pre-existing value, should be '0'!" );
    AreaBuf<uint32_t>( idxPtr, scaledSelf.width, scaledBlk.size() ).fill( idx );
  }

  return *cu;
}

PredictionUnit& CodingStructure::addPU( const UnitArea &unit, const ChannelType chType )
{
  PredictionUnit *pu = m_puCache.get();

  pu->UnitArea::operator=( unit );
  pu->initData();
  pu->next   = nullptr;
  pu->cs     = this;
  pu->cu     = m_isTuEnc ? cus[0] : getCU( unit.blocks[chType].pos(), chType );
  pu->chType = chType;
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

  VTMCHECK( pu->cacheId != pu->cu->cacheId, "Inconsintent cacheId between the PU and assigned CU" );
  VTMCHECK( pu->cu->firstPU != nullptr, "Without an RQT the firstPU should be null" );
#endif

  PredictionUnit *prevPU = m_numPUs > 0 ? pus.back() : nullptr;

  if( prevPU && prevPU->cu == pu->cu )
  {
    prevPU->next = pu;
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

    VTMCHECK( prevPU->cacheId != pu->cacheId, "Inconsintent cacheId between previous and current PU" );
#endif
  }

  pus.push_back( pu );

  if( pu->cu->firstPU == nullptr )
  {
    pu->cu->firstPU = pu;
  }
  pu->cu->lastPU = pu;

  uint32_t idx = ++m_numPUs;
  pu->idx  = idx;

  uint32_t numCh = ::getNumberValidChannels( area.chromaFormat );
  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !pu->blocks[i].valid() )
    {
      continue;
    }

    const CompArea &_selfBlk = area.blocks[i];
    const CompArea     &_blk = pu-> blocks[i];

    const UnitScale& scale = unitScale[_blk.compID];
    const Area scaledSelf  = scale.scale( _selfBlk );
    const Area scaledBlk   = scale.scale(     _blk );
    unsigned *idxPtr       = m_puIdx[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
    VTMCHECK( *idxPtr, "Overwriting a pre-existing value, should be '0'!" );
    AreaBuf<uint32_t>( idxPtr, scaledSelf.width, scaledBlk.size() ).fill( idx );
  }

  return *pu;
}

TransformUnit& CodingStructure::addTU( const UnitArea &unit, const ChannelType chType )
{
  TransformUnit *tu = m_tuCache.get();

  tu->UnitArea::operator=( unit );
  tu->initData();
  tu->next   = nullptr;
#if JVET_M0102_INTRA_SUBPARTITIONS
  tu->prev   = nullptr;
#endif
  tu->cs     = this;
  tu->cu     = m_isTuEnc ? cus[0] : getCU( unit.blocks[chType].pos(), chType );
  tu->chType = chType;
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

  if( tu->cu )
    VTMCHECK( tu->cacheId != tu->cu->cacheId, "Inconsintent cacheId between the TU and assigned CU" );
#endif


  TransformUnit *prevTU = m_numTUs > 0 ? tus.back() : nullptr;

  if( prevTU && prevTU->cu == tu->cu )
  {
    prevTU->next = tu;
#if JVET_M0102_INTRA_SUBPARTITIONS
    tu->prev     = prevTU;
#endif
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM

    VTMCHECK( prevTU->cacheId != tu->cacheId, "Inconsintent cacheId between previous and current TU" );
#endif
  }

  tus.push_back( tu );

  if( tu->cu )
  {
    if( tu->cu->firstTU == nullptr )
    {
      tu->cu->firstTU = tu;
    }
    tu->cu->lastTU = tu;
  }

  uint32_t idx = ++m_numTUs;
  tu->idx  = idx;

  TCoeff *coeffs[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
  Pel    *pcmbuf[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };

  uint32_t numCh = ::getNumberValidComponents( area.chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !tu->blocks[i].valid() )
    {
      continue;
    }

    if (i < ::getNumberValidChannels(area.chromaFormat))
    {
      const CompArea &_selfBlk = area.blocks[i];
      const CompArea     &_blk = tu-> blocks[i];

#if JVET_M0102_INTRA_SUBPARTITIONS
      bool isIspTu = tu->cu != nullptr && tu->cu->ispMode && isLuma( _blk.compID );

      bool isFirstIspTu = false;
      if( isIspTu )
      {
        isFirstIspTu = CU::isISPFirst( *tu->cu, _blk, getFirstComponentOfChannel( ChannelType( i ) ) );
      }
      if( !isIspTu || isFirstIspTu )
#endif
      {
        const UnitScale& scale = unitScale[_blk.compID];

        const Area scaledSelf  = scale.scale( _selfBlk );
#if JVET_M0102_INTRA_SUBPARTITIONS
        const Area scaledBlk   = isIspTu ? scale.scale( tu->cu->blocks[i] ) : scale.scale( _blk );
#else
        const Area scaledBlk   = scale.scale(     _blk );
#endif
        unsigned *idxPtr       = m_tuIdx[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
        VTMCHECK( *idxPtr, "Overwriting a pre-existing value, should be '0'!" );
        AreaBuf<uint32_t>( idxPtr, scaledSelf.width, scaledBlk.size() ).fill( idx );
      }
    }

    coeffs[i] = m_coeffs[i] + m_offsets[i];
    pcmbuf[i] = m_pcmbuf[i] + m_offsets[i];

    unsigned areaSize = tu->blocks[i].area();
    m_offsets[i] += areaSize;
  }

  tu->init( coeffs, pcmbuf );

  return *tu;
}

CUTraverser CodingStructure::traverseCUs( const UnitArea& unit, const ChannelType effChType )
{
  CodingUnit* firstCU = getCU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  CodingUnit* lastCU = firstCU;

  do { } while( lastCU && ( lastCU = lastCU->next ) && unit.contains( *lastCU ) );

  return CUTraverser( firstCU, lastCU );
}

PUTraverser CodingStructure::traversePUs( const UnitArea& unit, const ChannelType effChType )
{
  PredictionUnit* firstPU = getPU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  PredictionUnit* lastPU  = firstPU;

  do { } while( lastPU && ( lastPU = lastPU->next ) && unit.contains( *lastPU ) );

  return PUTraverser( firstPU, lastPU );
}

TUTraverser CodingStructure::traverseTUs( const UnitArea& unit, const ChannelType effChType )
{
  TransformUnit* firstTU = getTU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  TransformUnit* lastTU  = firstTU;

  do { } while( lastTU && ( lastTU = lastTU->next ) && unit.contains( *lastTU ) );

  return TUTraverser( firstTU, lastTU );
}

cCUTraverser CodingStructure::traverseCUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const CodingUnit* firstCU = getCU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const CodingUnit* lastCU  = firstCU;

  do { } while( lastCU && ( lastCU = lastCU->next ) && unit.contains( *lastCU ) );

  return cCUTraverser( firstCU, lastCU );
}

cPUTraverser CodingStructure::traversePUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const PredictionUnit* firstPU = getPU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const PredictionUnit* lastPU  = firstPU;

  do { } while( lastPU && ( lastPU = lastPU->next ) && unit.contains( *lastPU ) );

  return cPUTraverser( firstPU, lastPU );
}

cTUTraverser CodingStructure::traverseTUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const TransformUnit* firstTU = getTU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const TransformUnit* lastTU  = firstTU;

  do { } while( lastTU && ( lastTU = lastTU->next ) && unit.contains( *lastTU ) );

  return cTUTraverser( firstTU, lastTU );
}

// coding utilities

void CodingStructure::allocateVectorsAtPicLevel()
{
  const int  twice = ( !pcv->ISingleTree && slice->isIRAP() && pcv->chrFormat != CHROMA_400 ) ? 2 : 1;
  size_t allocSize = twice * unitScale[0].scale( area.blocks[0].size() ).area();

  cus.reserve( allocSize );
  pus.reserve( allocSize );
  tus.reserve( allocSize );
}



void CodingStructure::create(const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer)
{
  createInternals( UnitArea( _chromaFormat, _area ), isTopLayer );

  if( isTopLayer ) return;

  m_reco.create( area );
  m_pred.create( area );
  m_resi.create( area );
  m_orgr.create( area );
}

void CodingStructure::create(const UnitArea& _unit, const bool isTopLayer)
{
  createInternals( _unit, isTopLayer );

  if( isTopLayer ) return;

  m_reco.create( area );
  m_pred.create( area );
  m_resi.create( area );
  m_orgr.create( area );
}

void CodingStructure::createInternals( const UnitArea& _unit, const bool isTopLayer )
{
  area = _unit;

  memcpy( unitScale, UnitScaleArray[area.chromaFormat], sizeof( unitScale ) );

  picture = nullptr;
  parent  = nullptr;

  unsigned numCh = ::getNumberValidChannels(area.chromaFormat);

  for (unsigned i = 0; i < numCh; i++)
  {
    unsigned _area = unitScale[i].scale( area.blocks[i].size() ).area();

    m_cuIdx[i]    = _area > 0 ? new unsigned[_area] : nullptr;
    m_puIdx[i]    = _area > 0 ? new unsigned[_area] : nullptr;
    m_tuIdx[i]    = _area > 0 ? new unsigned[_area] : nullptr;
    m_isDecomp[i] = _area > 0 ? new bool    [_area] : nullptr;
  }

  numCh = getNumberValidComponents(area.chromaFormat);

  for (unsigned i = 0; i < numCh; i++)
  {
    m_offsets[i] = 0;
  }

  if( !isTopLayer ) createCoeffs();

  unsigned _lumaAreaScaled = g_miScaling.scale( area.lumaSize() ).area();
  m_motionBuf       = new MotionInfo[_lumaAreaScaled];
  initStructData();
}

void CodingStructure::rebindPicBufs()
{
  VTMCHECK( parent, "rebindPicBufs can only be used for the top level CodingStructure" );

  if( !picture->M_BUFS( 0, PIC_RECONSTRUCTION ).bufs.empty() ) m_reco.createFromBuf( picture->M_BUFS( 0, PIC_RECONSTRUCTION ) );
  else                                                         m_reco.destroy();
  if( !picture->M_BUFS( 0, PIC_PREDICTION     ).bufs.empty() ) m_pred.createFromBuf( picture->M_BUFS( 0, PIC_PREDICTION ) );
  else                                                         m_pred.destroy();
  if( !picture->M_BUFS( 0, PIC_RESIDUAL       ).bufs.empty() ) m_resi.createFromBuf( picture->M_BUFS( 0, PIC_RESIDUAL ) );
  else                                                         m_resi.destroy();
  if( pcv->isEncoder )
  {
    if( !picture->M_BUFS( 0, PIC_RESIDUAL     ).bufs.empty() ) m_orgr.create( area.chromaFormat, area.blocks[0], pcv->maxCUWidth );
    else                                                       m_orgr.destroy();
  }
}

void CodingStructure::createCoeffs()
{
  const unsigned numCh = getNumberValidComponents( area.chromaFormat );

  for( unsigned i = 0; i < numCh; i++ )
  {
    unsigned _area = area.blocks[i].area();

    m_coeffs[i] = _area > 0 ? ( TCoeff* ) xMalloc( TCoeff, _area ) : nullptr;
    m_pcmbuf[i] = _area > 0 ? ( Pel*    ) xMalloc( Pel,    _area ) : nullptr;
  }
}

void CodingStructure::destroyCoeffs()
{
  for( uint32_t i = 0; i < MAX_NUM_COMPONENT; i++ )
  {
    if( m_coeffs[i] ) { xFree( m_coeffs[i] ); m_coeffs[i] = nullptr; }
    if( m_pcmbuf[i] ) { xFree( m_pcmbuf[i] ); m_pcmbuf[i] = nullptr; }
  }
}

void CodingStructure::initSubStructure( CodingStructure& subStruct, const ChannelType _chType, const UnitArea &subArea, const bool &isTuEnc )
{
  VTMCHECK( this == &subStruct, "Trying to init self as sub-structure" );

  for( uint32_t i = 0; i < subStruct.area.blocks.size(); i++ )
  {
    CHECKD( subStruct.area.blocks[i].size() != subArea.blocks[i].size(), "Trying to init sub-structure of incompatible size" );

    subStruct.area.blocks[i].pos() = subArea.blocks[i].pos();
  }

  if( parent )
  {
    // allow this to be false at the top level (need for edge CTU's)
    CHECKD( !area.contains( subStruct.area ), "Trying to init sub-structure not contained in the parent" );
  }

  subStruct.parent    = this;
  subStruct.picture   = picture;

  subStruct.sps       = sps;
#if HEVC_VPS
  subStruct.vps       = vps;
#endif
  subStruct.pps       = pps;
  subStruct.slice     = slice;
  subStruct.baseQP    = baseQP;
  subStruct.prevQP[_chType]
                      = prevQP[_chType];
  subStruct.pcv       = pcv;

  subStruct.m_isTuEnc = isTuEnc;

  subStruct.initStructData( currQP[_chType], isLossless );

  if( isTuEnc )
  {
    CHECKD( area != subStruct.area, "Trying to init sub-structure for TU-encoding of incompatible size" );

    for( const auto &pcu : cus )
    {
      CodingUnit &cu = subStruct.addCU( *pcu, _chType );

      cu = *pcu;
    }

    for( const auto &ppu : pus )
    {
      PredictionUnit &pu = subStruct.addPU( *ppu, _chType );

      pu = *ppu;
    }

    unsigned numComp = ::getNumberValidChannels( area.chromaFormat );
    for( unsigned i = 0; i < numComp; i++)
    {
      ::memcpy( subStruct.m_isDecomp[i], m_isDecomp[i], (unitScale[i].scale( area.blocks[i].size() ).area() * sizeof( bool ) ) );
    }
  }
}

void CodingStructure::useSubStructure( const CodingStructure& subStruct, const ChannelType chType, const UnitArea &subArea, const bool cpyPred /*= true*/, const bool cpyReco /*= true*/, const bool cpyOrgResi /*= true*/, const bool cpyResi /*= true*/ )
{
  UnitArea clippedArea = clipArea( subArea, *picture );

  setDecomp( clippedArea );

  CPelUnitBuf subPredBuf = cpyPred ? subStruct.getPredBuf( clippedArea ) : CPelUnitBuf();
  CPelUnitBuf subResiBuf = cpyResi ? subStruct.getResiBuf( clippedArea ) : CPelUnitBuf();
  CPelUnitBuf subRecoBuf = cpyReco ? subStruct.getRecoBuf( clippedArea ) : CPelUnitBuf();

  if( parent )
  {
    // copy data to picture
    if( cpyPred )    getPredBuf   ( clippedArea ).copyFrom( subPredBuf );
    if( cpyResi )    getResiBuf   ( clippedArea ).copyFrom( subResiBuf );
    if( cpyReco )    getRecoBuf   ( clippedArea ).copyFrom( subRecoBuf );
    if( cpyOrgResi ) getOrgResiBuf( clippedArea ).copyFrom( subStruct.getOrgResiBuf( clippedArea ) );
  }

  if( cpyPred ) picture->getPredBuf( clippedArea ).copyFrom( subPredBuf );
  if( cpyResi ) picture->getResiBuf( clippedArea ).copyFrom( subResiBuf );
  if( cpyReco ) picture->getRecoBuf( clippedArea ).copyFrom( subRecoBuf );

#if JVET_M0483_IBC
  if (!subStruct.m_isTuEnc && ((!slice->isIntra() || slice->getSPS()->getIBCFlag()) && subStruct.chType != CHANNEL_TYPE_CHROMA))
#else
  if (!subStruct.m_isTuEnc && (!slice->isIntra() && subStruct.chType != CHANNEL_TYPE_CHROMA))
#endif
  {
    // copy motion buffer
    MotionBuf ownMB  = getMotionBuf          ( clippedArea );
    CMotionBuf subMB = subStruct.getMotionBuf( clippedArea );

    ownMB.copyFrom( subMB );
  }
#if ENABLE_WPP_PARALLELISM

  if( nullptr == parent )
  {
#pragma omp critical
    {
      fracBits += subStruct.fracBits;
      dist     += subStruct.dist;
      cost     += subStruct.cost;

      if( parent )
      {
        // allow this to be false at the top level
        CHECKD( !area.contains( subArea ), "Trying to use a sub-structure not contained in self" );
      }

      // copy the CUs over
      if( subStruct.m_isTuEnc )
      {
        // don't copy if the substruct was created for encoding of the TUs
      }
      else
      {
        for( const auto &pcu : subStruct.cus )
        {
          // add an analogue CU into own CU store
          const UnitArea &cuPatch = *pcu;

          CodingUnit &cu = addCU( cuPatch, chType );

          // copy the CU info from subPatch
          cu = *pcu;
        }
      }

      // copy the PUs over
      if( subStruct.m_isTuEnc )
      {
        // don't copy if the substruct was created for encoding of the TUs
      }
      else
      {
        for( const auto &ppu : subStruct.pus )
        {
          // add an analogue PU into own PU store
          const UnitArea &puPatch = *ppu;

          PredictionUnit &pu = addPU( puPatch, chType );

          // copy the PU info from subPatch
          pu = *ppu;
        }
      }
      // copy the TUs over
      for( const auto &ptu : subStruct.tus )
      {
        // add an analogue TU into own TU store
        const UnitArea &tuPatch = *ptu;

        TransformUnit &tu = addTU( tuPatch, chType );

        // copy the TU info from subPatch
        tu = *ptu;
      }
    }

    return;
  }
#endif

  fracBits += subStruct.fracBits;
  dist     += subStruct.dist;
  cost     += subStruct.cost;

  if( parent )
  {
    // allow this to be false at the top level
    CHECKD( !area.contains( subArea ), "Trying to use a sub-structure not contained in self" );
  }

  // copy the CUs over
  if( subStruct.m_isTuEnc )
  {
    // don't copy if the substruct was created for encoding of the TUs
  }
  else
  {
    for( const auto &pcu : subStruct.cus )
    {
      // add an analogue CU into own CU store
      const UnitArea &cuPatch = *pcu;

      CodingUnit &cu = addCU( cuPatch, chType );

      // copy the CU info from subPatch
      cu = *pcu;
    }
  }

  // copy the PUs over
  if( subStruct.m_isTuEnc )
  {
    // don't copy if the substruct was created for encoding of the TUs
  }
  else
  {
    for( const auto &ppu : subStruct.pus )
    {
      // add an analogue PU into own PU store
      const UnitArea &puPatch = *ppu;

      PredictionUnit &pu = addPU( puPatch, chType );

      // copy the PU info from subPatch
      pu = *ppu;
    }
  }
  // copy the TUs over
  for( const auto &ptu : subStruct.tus )
  {
    // add an analogue TU into own TU store
    const UnitArea &tuPatch = *ptu;

    TransformUnit &tu = addTU( tuPatch, chType );

    // copy the TU info from subPatch
    tu = *ptu;
  }
}

void CodingStructure::copyStructure( const CodingStructure& other, const ChannelType chType, const bool copyTUs, const bool copyRecoBuf )
{
  fracBits = other.fracBits;
  dist     = other.dist;
  cost     = other.cost;

  CHECKD( area != other.area, "Incompatible sizes" );

  const UnitArea dualITreeArea = CS::getArea( *this, this->area, chType );

  // copy the CUs over
  for (const auto &pcu : other.cus)
  {
    if( !dualITreeArea.contains( *pcu ) )
    {
      continue;
    }
    // add an analogue CU into own CU store
    const UnitArea &cuPatch = *pcu;

    CodingUnit &cu = addCU(cuPatch, chType);

    // copy the CU info from subPatch
    cu = *pcu;
  }

  // copy the PUs over
  for (const auto &ppu : other.pus)
  {
    if( !dualITreeArea.contains( *ppu ) )
    {
      continue;
    }
    // add an analogue PU into own PU store
    const UnitArea &puPatch = *ppu;

    PredictionUnit &pu = addPU(puPatch, chType);

    // copy the PU info from subPatch
    pu = *ppu;
  }

#if JVET_M0483_IBC
  if (!other.slice->isIntra() || other.slice->getSPS()->getIBCFlag())
#else
  if( !other.slice->isIntra() )
#endif
  {
    // copy motion buffer
    MotionBuf  ownMB = getMotionBuf();
    CMotionBuf subMB = other.getMotionBuf();

    ownMB.copyFrom( subMB );
  }

  if( copyTUs )
  {
    // copy the TUs over
    for( const auto &ptu : other.tus )
    {
      if( !dualITreeArea.contains( *ptu ) )
      {
        continue;
      }
      // add an analogue TU into own TU store
      const UnitArea &tuPatch = *ptu;

      TransformUnit &tu = addTU( tuPatch, chType );

      // copy the TU info from subPatch
      tu = *ptu;
    }
  }

  if( copyRecoBuf )
  {
    CPelUnitBuf recoBuf = other.getRecoBuf( area );

    if( parent )
    {
      // copy data to self for neighbors
      getRecoBuf( area ).copyFrom( recoBuf );
    }

    // copy data to picture
    picture->getRecoBuf( area ).copyFrom( recoBuf );
#if JVET_M0427_INLOOP_RESHAPER
    if (other.pcv->isEncoder)
    {
      CPelUnitBuf predBuf = other.getPredBuf(area);
      if (parent)
      {
        getPredBuf(area).copyFrom(predBuf);
      }
      picture->getPredBuf(area).copyFrom(predBuf);
    }
#endif
#if JVET_M0055_DEBUG_CTU

    // required for DebugCTU
    int numCh = ::getNumberValidChannels( area.chromaFormat );
    for( int i = 0; i < numCh; i++ )
    {
      const size_t _area = unitScale[i].scaleArea( area.blocks[i].area() );

      memcpy( m_isDecomp[i], other.m_isDecomp[i], sizeof( *m_isDecomp[0] ) * _area );
    }
#endif
  }
}

void CodingStructure::initStructData( const int &QP, const bool &_isLosses, const bool &skipMotBuf )
{
  clearPUs();
  clearTUs();
  clearCUs();

  if( QP < MAX_INT )
  {
    currQP[0] = currQP[1] = QP;
    isLossless            = _isLosses;
  }

#if JVET_M0483_IBC
  if (!skipMotBuf && (!parent || ((!slice->isIntra() || slice->getSPS()->getIBCFlag()) && !m_isTuEnc)))
#else
  if( !skipMotBuf && ( !parent || ( ( slice->getSliceType() != I_SLICE ) && !m_isTuEnc ) ) )
#endif
  {
    getMotionBuf()      .memset( 0 );
  }

  fracBits = 0;
  dist     = 0;
  cost     = MAX_DOUBLE;
#if JVET_M0102_INTRA_SUBPARTITIONS
  lumaCost = MAX_DOUBLE;
#endif
  interHad = std::numeric_limits<Distortion>::max();
}


void CodingStructure::clearTUs()
{
  int numCh = ::getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    size_t _area = ( area.blocks[i].area() >> unitScale[i].area );

    memset( m_isDecomp[i], false, sizeof( *m_isDecomp[0] ) * _area );
    memset( m_tuIdx   [i],     0, sizeof( *m_tuIdx   [0] ) * _area );
  }

  numCh = getNumberValidComponents( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    m_offsets[i] = 0;
  }

  for( auto &pcu : cus )
  {
    pcu->firstTU = pcu->lastTU = nullptr;
  }

  m_tuCache.cache( tus );
  m_numTUs = 0;
}

void CodingStructure::clearPUs()
{
  int numCh = ::getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    memset( m_puIdx[i], 0, sizeof( *m_puIdx[0] ) * unitScale[i].scaleArea( area.blocks[i].area() ) );
  }

  m_puCache.cache( pus );
  m_numPUs = 0;

  for( auto &pcu : cus )
  {
    pcu->firstPU = pcu->lastPU = nullptr;
  }
}

void CodingStructure::clearCUs()
{
  int numCh = ::getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    memset( m_cuIdx[i], 0, sizeof( *m_cuIdx[0] ) * unitScale[i].scaleArea( area.blocks[i].area() ) );
  }

  m_cuCache.cache( cus );
  m_numCUs = 0;
}

MotionBuf CodingStructure::getMotionBuf( const Area& _area )
{
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

const CMotionBuf CodingStructure::getMotionBuf( const Area& _area ) const
{
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

MotionInfo& CodingStructure::getMotionInfo( const Position& pos )
{
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_motionBuf + miPos.y * stride + miPos.x );
}

const MotionInfo& CodingStructure::getMotionInfo( const Position& pos ) const
{
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_motionBuf + miPos.y * stride + miPos.x );
}


// data accessors
       PelBuf     CodingStructure::getPredBuf(const CompArea &blk)           { return getBuf(blk,  PIC_PREDICTION); }
const CPelBuf     CodingStructure::getPredBuf(const CompArea &blk)     const { return getBuf(blk,  PIC_PREDICTION); }
       PelUnitBuf CodingStructure::getPredBuf(const UnitArea &unit)          { return getBuf(unit, PIC_PREDICTION); }
const CPelUnitBuf CodingStructure::getPredBuf(const UnitArea &unit)    const { return getBuf(unit, PIC_PREDICTION); }

       PelBuf     CodingStructure::getResiBuf(const CompArea &blk)           { return getBuf(blk,  PIC_RESIDUAL); }
const CPelBuf     CodingStructure::getResiBuf(const CompArea &blk)     const { return getBuf(blk,  PIC_RESIDUAL); }
       PelUnitBuf CodingStructure::getResiBuf(const UnitArea &unit)          { return getBuf(unit, PIC_RESIDUAL); }
const CPelUnitBuf CodingStructure::getResiBuf(const UnitArea &unit)    const { return getBuf(unit, PIC_RESIDUAL); }

       PelBuf     CodingStructure::getRecoBuf(const CompArea &blk)           { return getBuf(blk,  PIC_RECONSTRUCTION); }
const CPelBuf     CodingStructure::getRecoBuf(const CompArea &blk)     const { return getBuf(blk,  PIC_RECONSTRUCTION); }
       PelUnitBuf CodingStructure::getRecoBuf(const UnitArea &unit)          { return getBuf(unit, PIC_RECONSTRUCTION); }
const CPelUnitBuf CodingStructure::getRecoBuf(const UnitArea &unit)    const { return getBuf(unit, PIC_RECONSTRUCTION); }

       PelBuf     CodingStructure::getOrgResiBuf(const CompArea &blk)        { return getBuf(blk,  PIC_ORG_RESI); }
const CPelBuf     CodingStructure::getOrgResiBuf(const CompArea &blk)  const { return getBuf(blk,  PIC_ORG_RESI); }
       PelUnitBuf CodingStructure::getOrgResiBuf(const UnitArea &unit)       { return getBuf(unit, PIC_ORG_RESI); }
const CPelUnitBuf CodingStructure::getOrgResiBuf(const UnitArea &unit) const { return getBuf(unit, PIC_ORG_RESI); }

       PelBuf     CodingStructure::getOrgBuf(const CompArea &blk)            { return getBuf(blk,  PIC_ORIGINAL); }
const CPelBuf     CodingStructure::getOrgBuf(const CompArea &blk)      const { return getBuf(blk,  PIC_ORIGINAL); }
       PelUnitBuf CodingStructure::getOrgBuf(const UnitArea &unit)           { return getBuf(unit, PIC_ORIGINAL); }
const CPelUnitBuf CodingStructure::getOrgBuf(const UnitArea &unit)     const { return getBuf(unit, PIC_ORIGINAL); }

       PelBuf     CodingStructure::getOrgBuf(const ComponentID &compID)      { return picture->getBuf(area.blocks[compID], PIC_ORIGINAL); }
const CPelBuf     CodingStructure::getOrgBuf(const ComponentID &compID)const { return picture->getBuf(area.blocks[compID], PIC_ORIGINAL); }
       PelUnitBuf CodingStructure::getOrgBuf()                               { return picture->getBuf(area, PIC_ORIGINAL); }
const CPelUnitBuf CodingStructure::getOrgBuf()                         const { return picture->getBuf(area, PIC_ORIGINAL); }

PelBuf CodingStructure::getBuf( const CompArea &blk, const PictureType &type )
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  if (type == PIC_ORIGINAL)
  {
    return picture->getBuf(blk, type);
  }

  const ComponentID compID = blk.compID;

  PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : ( type == PIC_RECONSTRUCTION ? &m_reco : ( type == PIC_ORG_RESI ? &m_orgr : nullptr ) ) );

  VTMCHECK( !buf, "Unknown buffer requested" );

  CHECKD( !area.blocks[compID].contains( blk ), "Buffer not contained in self requested" );

  CompArea cFinal = blk;
  cFinal.relativeTo( area.blocks[compID] );

#if !KEEP_PRED_AND_RESI_SIGNALS
  if( !parent && ( type == PIC_RESIDUAL || type == PIC_PREDICTION ) )
  {
    cFinal.x &= ( pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    cFinal.y &= ( pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );
  }
#endif

  return buf->getBuf( cFinal );
}

const CPelBuf CodingStructure::getBuf( const CompArea &blk, const PictureType &type ) const
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  if (type == PIC_ORIGINAL)
  {
    return picture->getBuf(blk, type);
  }

  const ComponentID compID = blk.compID;

  const PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : ( type == PIC_RECONSTRUCTION ? &m_reco : ( type == PIC_ORG_RESI ? &m_orgr : nullptr ) ) );

  VTMCHECK( !buf, "Unknown buffer requested" );

  CHECKD( !area.blocks[compID].contains( blk ), "Buffer not contained in self requested" );

  CompArea cFinal = blk;
  cFinal.relativeTo( area.blocks[compID] );

#if !KEEP_PRED_AND_RESI_SIGNALS
  if( !parent && ( type == PIC_RESIDUAL || type == PIC_PREDICTION ) )
  {
    cFinal.x &= ( pcv->maxCUWidthMask  >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    cFinal.y &= ( pcv->maxCUHeightMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );
  }
#endif

  return buf->getBuf( cFinal );
}

PelUnitBuf CodingStructure::getBuf( const UnitArea &unit, const PictureType &type )
{
  // no parent fetching for buffers
  if( area.chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf CodingStructure::getBuf( const UnitArea &unit, const PictureType &type ) const
{
  // no parent fetching for buffers
  if( area.chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CodingUnit* CodingStructure::getCURestricted( const Position &pos, const CodingUnit& curCu, const ChannelType _chType ) const
{
  const CodingUnit* cu = getCU( pos, _chType );
#if HEVC_TILES_WPP
  // exists       same slice and tile                  cu precedes curCu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  if( cu && CU::isSameSliceAndTile( *cu, curCu ) && ( cu->cs != curCu.cs || cu->idx <= curCu.idx ) )
#else
  // exists       same slice                          cu precedes curCu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  if(cu && CU::isSameSlice(*cu, curCu) && (cu->cs != curCu.cs || cu->idx <= curCu.idx))
#endif
  {
    return cu;
  }
  else
  {
    return nullptr;
  }
}

#if HEVC_TILES_WPP
const CodingUnit* CodingStructure::getCURestricted( const Position &pos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType ) const
{
  const CodingUnit* cu = getCU( pos, _chType );
  return ( cu && cu->slice->getIndependentSliceIdx() == curSliceIdx && cu->tileIdx == curTileIdx ) ? cu : nullptr;
}
#else
const CodingUnit* CodingStructure::getCURestricted(const Position &pos, const unsigned curSliceIdx, const ChannelType _chType) const
{
  const CodingUnit* cu = getCU(pos, _chType);
  return (cu && cu->slice->getIndependentSliceIdx() == curSliceIdx ) ? cu : nullptr;
}
#endif

const PredictionUnit* CodingStructure::getPURestricted( const Position &pos, const PredictionUnit& curPu, const ChannelType _chType ) const
{
  const PredictionUnit* pu = getPU( pos, _chType );
#if HEVC_TILES_WPP
  // exists       same slice and tile                  pu precedes curPu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  if( pu && CU::isSameSliceAndTile( *pu->cu, *curPu.cu ) && ( pu->cs != curPu.cs || pu->idx <= curPu.idx ) )
#else
  // exists       same slice                           pu precedes curPu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  if(pu && CU::isSameSlice(*pu->cu, *curPu.cu) && (pu->cs != curPu.cs || pu->idx <= curPu.idx))
#endif
  {
    return pu;
  }
  else
  {
    return nullptr;
  }
}

const TransformUnit* CodingStructure::getTURestricted( const Position &pos, const TransformUnit& curTu, const ChannelType _chType ) const
{
  const TransformUnit* tu = getTU( pos, _chType );
#if HEVC_TILES_WPP
  // exists       same slice and tile                  tu precedes curTu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  if( tu && CU::isSameSliceAndTile( *tu->cu, *curTu.cu ) && ( tu->cs != curTu.cs || tu->idx <= curTu.idx ) )
#else
  // exists       same slice                           tu precedes curTu in encoding order
  //                                                  (thus, is either from parent CS in RD-search or its index is lower)
  if(tu && CU::isSameSlice(*tu->cu, *curTu.cu) && (tu->cs != curTu.cs || tu->idx <= curTu.idx))
#endif
  {
    return tu;
  }
  else
  {
    return nullptr;
  }
}

IbcLumaCoverage CodingStructure::getIbcLumaCoverage(const CompArea& chromaArea) const
{
  VTMCHECK(chType != CHANNEL_TYPE_CHROMA, "Error");

  const unsigned int unitAreaSubBlock = MIN_PU_SIZE * MIN_PU_SIZE;
  CompArea lumaArea = CompArea(COMPONENT_Y, chromaArea.chromaFormat, chromaArea.lumaPos(), recalcSize(chromaArea.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, chromaArea.size()));
  lumaArea = clipArea(lumaArea, picture->block(COMPONENT_Y));
  const unsigned int fullArea = lumaArea.area();
  unsigned int ibcArea = 0;
  for (SizeType y = 0; y < lumaArea.height; y += MIN_PU_SIZE)
  {
    for (SizeType x = 0; x < lumaArea.width; x += MIN_PU_SIZE)
    {
      Position pos = lumaArea.offset(x, y);
      if (picture->cs->getMotionInfo(pos).isInter) // need to change if inter slice allows dualtree
      {
        ibcArea += unitAreaSubBlock;
      }
    }
  }

  IbcLumaCoverage coverage = IBC_LUMA_COVERAGE_FULL;
  if (ibcArea == 0)
  {
    coverage = IBC_LUMA_COVERAGE_NONE;
  }
  else if (ibcArea < fullArea)
  {
    coverage = IBC_LUMA_COVERAGE_PARTIAL;
  }

  return coverage;
}
