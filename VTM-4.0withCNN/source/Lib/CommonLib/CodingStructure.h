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

#ifndef __CODINGSTRUCTURE__
#define __CODINGSTRUCTURE__

#include "Unit.h"
#include "Buffer.h"
#include "CommonDef.h"
#include "UnitPartitioner.h"
#include "Slice.h"
#include <vector>


struct Picture;


enum PictureType
{
  PIC_RECONSTRUCTION = 0,
  PIC_ORIGINAL,
#if JVET_M0427_INLOOP_RESHAPER
  PIC_TRUE_ORIGINAL,
#endif
  PIC_PREDICTION,
  PIC_RESIDUAL,
  PIC_ORG_RESI,
  NUM_PIC_TYPES
};
enum IbcLumaCoverage
{
  IBC_LUMA_COVERAGE_FULL = 0,
  IBC_LUMA_COVERAGE_PARTIAL,
  IBC_LUMA_COVERAGE_NONE,
  NUM_IBC_LUMA_COVERAGE,
};
extern XUCache g_globalUnitCache;

// ---------------------------------------------------------------------------
// coding structure
// ---------------------------------------------------------------------------

class CodingStructure
{
public:

  UnitArea         area;

  Picture         *picture;
  CodingStructure *parent;
#if JVET_M0246_AFFINE_AMVR
  CodingStructure *bestCS;
#endif
  Slice           *slice;

  UnitScale        unitScale[MAX_NUM_COMPONENT];
  ChannelType chType;

  int         baseQP;
  int         prevQP[MAX_NUM_CHANNEL_TYPE];
  int         currQP[MAX_NUM_CHANNEL_TYPE];
  int         chromaQpAdj;
#if JVET_M0170_MRG_SHARELIST
  Position    sharedBndPos;
  Size        sharedBndSize;
#endif
  bool        isLossless;
  const SPS *sps;
  const PPS *pps;
#if HEVC_VPS
  const VPS *vps;
#endif
  const PreCalcValues* pcv;

  CodingStructure(CUCache&, PUCache&, TUCache&);
  void create( const UnitArea &_unit, const bool isTopLayer );
  void create( const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer );
  void destroy();
  void releaseIntermediateData();

  void rebindPicBufs();
  void createCoeffs();
  void destroyCoeffs();

  void allocateVectorsAtPicLevel();

  // ---------------------------------------------------------------------------
  // global accessors
  // ---------------------------------------------------------------------------

  bool isDecomp (const Position &pos, const ChannelType _chType) const;
  bool isDecomp (const Position &pos, const ChannelType _chType);
  void setDecomp(const CompArea &area, const bool _isCoded = true);
  void setDecomp(const UnitArea &area, const bool _isCoded = true);

  const CodingUnit     *getCU(const Position &pos, const ChannelType _chType) const;
  const PredictionUnit *getPU(const Position &pos, const ChannelType _chType) const;
#if JVET_M0102_INTRA_SUBPARTITIONS
  const TransformUnit  *getTU(const Position &pos, const ChannelType _chType, const int subTuIdx = -1) const;
#else
  const TransformUnit  *getTU(const Position &pos, const ChannelType _chType) const;
#endif

  CodingUnit     *getCU(const Position &pos, const ChannelType _chType);
  PredictionUnit *getPU(const Position &pos, const ChannelType _chType);
#if JVET_M0102_INTRA_SUBPARTITIONS
  TransformUnit  *getTU(const Position &pos, const ChannelType _chType, const int subTuIdx = -1);
#else
  TransformUnit  *getTU(const Position &pos, const ChannelType _chType);
#endif

  const CodingUnit     *getCU(const ChannelType &_chType) const { return getCU(area.blocks[_chType].pos(), _chType); }
  const PredictionUnit *getPU(const ChannelType &_chType) const { return getPU(area.blocks[_chType].pos(), _chType); }
  const TransformUnit  *getTU(const ChannelType &_chType) const { return getTU(area.blocks[_chType].pos(), _chType); }

  CodingUnit     *getCU(const ChannelType &_chType ) { return getCU(area.blocks[_chType].pos(), _chType); }
  PredictionUnit *getPU(const ChannelType &_chType ) { return getPU(area.blocks[_chType].pos(), _chType); }
  TransformUnit  *getTU(const ChannelType &_chType ) { return getTU(area.blocks[_chType].pos(), _chType); }

#if HEVC_TILES_WPP
  const CodingUnit     *getCURestricted(const Position &pos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType) const;
#else
  const CodingUnit     *getCURestricted(const Position &pos, const unsigned curSliceIdx,                            const ChannelType _chType) const;
#endif
  const CodingUnit     *getCURestricted(const Position &pos, const CodingUnit& curCu,                               const ChannelType _chType) const;
  const PredictionUnit *getPURestricted(const Position &pos, const PredictionUnit& curPu,                           const ChannelType _chType) const;
  const TransformUnit  *getTURestricted(const Position &pos, const TransformUnit& curTu,                            const ChannelType _chType) const;

  CodingUnit&     addCU(const UnitArea &unit, const ChannelType _chType);
  PredictionUnit& addPU(const UnitArea &unit, const ChannelType _chType);
  TransformUnit&  addTU(const UnitArea &unit, const ChannelType _chType);

  CUTraverser     traverseCUs(const UnitArea& _unit, const ChannelType _chType);
  PUTraverser     traversePUs(const UnitArea& _unit, const ChannelType _chType);
  TUTraverser     traverseTUs(const UnitArea& _unit, const ChannelType _chType);

  cCUTraverser    traverseCUs(const UnitArea& _unit, const ChannelType _chType) const;
  cPUTraverser    traversePUs(const UnitArea& _unit, const ChannelType _chType) const;
  cTUTraverser    traverseTUs(const UnitArea& _unit, const ChannelType _chType) const;
  IbcLumaCoverage getIbcLumaCoverage(const CompArea& chromaArea) const;
  // ---------------------------------------------------------------------------
  // encoding search utilities
  // ---------------------------------------------------------------------------

  static_vector<double, NUM_ENC_FEATURES> features;

  double      cost;
#if JVET_M0102_INTRA_SUBPARTITIONS
  double      lumaCost;
#endif
  uint64_t      fracBits;
  Distortion  dist;
  Distortion  interHad;

  void initStructData  (const int &QP = MAX_INT, const bool &_isLosses = false, const bool &skipMotBuf = false);
  void initSubStructure(      CodingStructure& cs, const ChannelType chType, const UnitArea &subArea, const bool &isTuEnc);

  void copyStructure   (const CodingStructure& cs, const ChannelType chType, const bool copyTUs = false, const bool copyRecoBuffer = false);
  void useSubStructure (const CodingStructure& cs, const ChannelType chType, const UnitArea &subArea, const bool cpyPred, const bool cpyReco, const bool cpyOrgResi, const bool cpyResi);
  void useSubStructure (const CodingStructure& cs, const ChannelType chType,                          const bool cpyPred, const bool cpyReco, const bool cpyOrgResi, const bool cpyResi) { useSubStructure(cs, chType, cs.area, cpyPred, cpyReco, cpyOrgResi, cpyResi); }

  void clearTUs();
  void clearPUs();
  void clearCUs();


private:
  void createInternals(const UnitArea& _unit, const bool isTopLayer);

public:

  std::vector<    CodingUnit*> cus;
  std::vector<PredictionUnit*> pus;
  std::vector< TransformUnit*> tus;

private:

  // needed for TU encoding
  bool m_isTuEnc;

  unsigned *m_cuIdx   [MAX_NUM_CHANNEL_TYPE];
  unsigned *m_puIdx   [MAX_NUM_CHANNEL_TYPE];
  unsigned *m_tuIdx   [MAX_NUM_CHANNEL_TYPE];
  bool     *m_isDecomp[MAX_NUM_CHANNEL_TYPE];

  unsigned m_numCUs;
  unsigned m_numPUs;
  unsigned m_numTUs;

  CUCache& m_cuCache;
  PUCache& m_puCache;
  TUCache& m_tuCache;

  std::vector<SAOBlkParam> m_sao;

  PelStorage m_pred;
  PelStorage m_resi;
  PelStorage m_reco;
  PelStorage m_orgr;

  TCoeff *m_coeffs [ MAX_NUM_COMPONENT ];
  Pel    *m_pcmbuf [ MAX_NUM_COMPONENT ];

  int     m_offsets[ MAX_NUM_COMPONENT ];

  MotionInfo *m_motionBuf;

public:

  MotionBuf getMotionBuf( const     Area& _area );
  MotionBuf getMotionBuf( const UnitArea& _area ) { return getMotionBuf( _area.Y() ); }
  MotionBuf getMotionBuf()                        { return getMotionBuf(  area.Y() ); }

  const CMotionBuf getMotionBuf( const     Area& _area ) const;
  const CMotionBuf getMotionBuf( const UnitArea& _area ) const { return getMotionBuf( _area.Y() ); }
  const CMotionBuf getMotionBuf()                        const { return getMotionBuf(  area.Y() ); }

  MotionInfo& getMotionInfo( const Position& pos );
  const MotionInfo& getMotionInfo( const Position& pos ) const;


public:
  // ---------------------------------------------------------------------------
  // temporary (shadowed) data accessors
  // ---------------------------------------------------------------------------
         PelBuf       getPredBuf(const CompArea &blk);
  const CPelBuf       getPredBuf(const CompArea &blk) const;
         PelUnitBuf   getPredBuf(const UnitArea &unit);
  const CPelUnitBuf   getPredBuf(const UnitArea &unit) const;

         PelBuf       getResiBuf(const CompArea &blk);
  const CPelBuf       getResiBuf(const CompArea &blk) const;
         PelUnitBuf   getResiBuf(const UnitArea &unit);
  const CPelUnitBuf   getResiBuf(const UnitArea &unit) const;

         PelBuf       getRecoBuf(const CompArea &blk);
  const CPelBuf       getRecoBuf(const CompArea &blk) const;
         PelUnitBuf   getRecoBuf(const UnitArea &unit);
  const CPelUnitBuf   getRecoBuf(const UnitArea &unit) const;

         PelBuf       getOrgResiBuf(const CompArea &blk);
  const CPelBuf       getOrgResiBuf(const CompArea &blk) const;
         PelUnitBuf   getOrgResiBuf(const UnitArea &unit);
  const CPelUnitBuf   getOrgResiBuf(const UnitArea &unit) const;

         PelBuf       getOrgBuf(const CompArea &blk);
  const CPelBuf       getOrgBuf(const CompArea &blk) const;
         PelUnitBuf   getOrgBuf(const UnitArea &unit);
  const CPelUnitBuf   getOrgBuf(const UnitArea &unit) const;

         PelBuf       getOrgBuf(const ComponentID &compID);
  const CPelBuf       getOrgBuf(const ComponentID &compID) const;
         PelUnitBuf   getOrgBuf();
  const CPelUnitBuf   getOrgBuf() const;


  // pred buffer
         PelBuf       getPredBuf(const ComponentID &compID)       { return m_pred.get(compID); }
  const CPelBuf       getPredBuf(const ComponentID &compID) const { return m_pred.get(compID); }
         PelUnitBuf   getPredBuf()                                { return m_pred; }
  const CPelUnitBuf   getPredBuf()                          const { return m_pred; }

  // resi buffer
         PelBuf       getResiBuf(const ComponentID compID)        { return m_resi.get(compID); }
  const CPelBuf       getResiBuf(const ComponentID compID)  const { return m_resi.get(compID); }
         PelUnitBuf   getResiBuf()                                { return m_resi; }
  const CPelUnitBuf   getResiBuf()                          const { return m_resi; }

  // org-resi buffer
         PelBuf       getOrgResiBuf(const ComponentID &compID)       { return m_orgr.get(compID); }
  const CPelBuf       getOrgResiBuf(const ComponentID &compID) const { return m_orgr.get(compID); }
         PelUnitBuf   getOrgResiBuf()                                { return m_orgr; }
  const CPelUnitBuf   getOrgResiBuf()                          const { return m_orgr; }

  // reco buffer
         PelBuf       getRecoBuf(const ComponentID compID)         { return m_reco.get(compID); }
  const CPelBuf       getRecoBuf(const ComponentID compID)   const { return m_reco.get(compID); }
         PelUnitBuf   getRecoBuf()                                 { return m_reco; }
  const CPelUnitBuf   getRecoBuf()                           const { return m_reco; }

private:

  inline        PelBuf       getBuf(const CompArea &blk,  const PictureType &type);
  inline const CPelBuf       getBuf(const CompArea &blk,  const PictureType &type) const;
  inline        PelUnitBuf   getBuf(const UnitArea &unit, const PictureType &type);
  inline const CPelUnitBuf   getBuf(const UnitArea &unit, const PictureType &type) const;
};


static inline uint32_t getNumberValidTBlocks(const PreCalcValues& pcv) { return (pcv.chrFormat==CHROMA_400) ? 1 : ( pcv.multiBlock422 ? MAX_NUM_TBLOCKS : MAX_NUM_COMPONENT ); }

inline unsigned toWSizeIdx( const CodingStructure* cs ) { return gp_sizeIdxInfo->idxFrom( cs->area.lwidth() ); }
inline unsigned toHSizeIdx( const CodingStructure* cs ) { return gp_sizeIdxInfo->idxFrom( cs->area.lheight() ); }

#endif

