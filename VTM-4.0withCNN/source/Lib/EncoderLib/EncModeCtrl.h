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

/** \file     EncModeCtrl.h
    \brief    Encoder controller for trying out specific modes
*/

#ifndef __ENCMODECTRL__
#define __ENCMODECTRL__

// Include files
#include "EncCfg.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/CodingStructure.h"

#include <typeinfo>
#include <vector>

//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////


enum EncTestModeType
{
#if JVET_M0253_HASH_ME
  ETM_HASH_INTER,
#endif
  ETM_MERGE_SKIP,
  ETM_INTER_ME,
  ETM_AFFINE,
  ETM_MERGE_TRIANGLE,
  ETM_INTRA,
  ETM_IPCM,
  ETM_SPLIT_QT,
  ETM_SPLIT_BT_H,
  ETM_SPLIT_BT_V,
  ETM_SPLIT_TT_H,
  ETM_SPLIT_TT_V,
  ETM_POST_DONT_SPLIT, // dummy mode to collect the data from the unsplit coding
#if REUSE_CU_RESULTS
  ETM_RECO_CACHED,
#endif
  ETM_TRIGGER_IMV_LIST,
  ETM_IBC,    // ibc mode
  ETM_IBC_MERGE, // ibc merge mode
  ETM_INVALID
};

enum EncTestModeOpts
{
  ETO_STANDARD    =  0,                   // empty      (standard option)
  ETO_FORCE_MERGE =  1<<0,                // bit   0    (indicates forced merge)
  ETO_IMV_SHIFT   =     1,                // bits  1-3  (imv parameter starts at bit 1)
  ETO_IMV         =  7<<ETO_IMV_SHIFT,    // bits  1-3  (imv parameter uses 3 bits)
  ETO_DUMMY       =  1<<5,                // bit   5    (dummy)
  ETO_INVALID     = 0xffffffff            // bits 0-31  (invalid option)
};

static void getAreaIdx(const Area& area, const PreCalcValues &pcv, unsigned &idx1, unsigned &idx2, unsigned &idx3, unsigned &idx4)
{
  idx1 = (area.x & pcv.maxCUWidthMask)  >> MIN_CU_LOG2;
  idx2 = (area.y & pcv.maxCUHeightMask) >> MIN_CU_LOG2;
  idx3 = gp_sizeIdxInfo->idxFrom( area.width  );
  idx4 = gp_sizeIdxInfo->idxFrom( area.height );
}

struct EncTestMode
{
  EncTestMode()
    : type( ETM_INVALID ), opts( ETO_INVALID  ), qp( -1  ), lossless( false ) {}
  EncTestMode( EncTestModeType _type )
    : type( _type       ), opts( ETO_STANDARD ), qp( -1  ), lossless( false ) {}
  EncTestMode( EncTestModeType _type, int _qp, bool _lossless )
    : type( _type       ), opts( ETO_STANDARD ), qp( _qp ), lossless( _lossless ) {}
  EncTestMode( EncTestModeType _type, EncTestModeOpts _opts, int _qp, bool _lossless )
    : type( _type       ), opts( _opts        ), qp( _qp ), lossless( _lossless ) {}

  EncTestModeType type;
  EncTestModeOpts opts;
  int             qp;
  bool            lossless;
};


inline bool isModeSplit( const EncTestMode& encTestmode )
{
  switch( encTestmode.type )
  {
  case ETM_SPLIT_QT     :
  case ETM_SPLIT_BT_H   :
  case ETM_SPLIT_BT_V   :
  case ETM_SPLIT_TT_H   :
  case ETM_SPLIT_TT_V   :
    return true;
  default:
    return false;
  }
}

inline bool isModeNoSplit( const EncTestMode& encTestmode )
{
  return !isModeSplit( encTestmode ) && encTestmode.type != ETM_POST_DONT_SPLIT;
}

inline bool isModeInter( const EncTestMode& encTestmode ) // perhaps remove
{
  return (   encTestmode.type == ETM_INTER_ME
          || encTestmode.type == ETM_MERGE_SKIP
          || encTestmode.type == ETM_AFFINE
          || encTestmode.type == ETM_MERGE_TRIANGLE
#if JVET_M0253_HASH_ME
          || encTestmode.type == ETM_HASH_INTER
#endif
         );
}

inline PartSplit getPartSplit( const EncTestMode& encTestmode )
{
  switch( encTestmode.type )
  {
  case ETM_SPLIT_QT     : return CU_QUAD_SPLIT;
  case ETM_SPLIT_BT_H   : return CU_HORZ_SPLIT;
  case ETM_SPLIT_BT_V   : return CU_VERT_SPLIT;
  case ETM_SPLIT_TT_H   : return CU_TRIH_SPLIT;
  case ETM_SPLIT_TT_V   : return CU_TRIV_SPLIT;
  default:                return CU_DONT_SPLIT;
  }
}

inline EncTestMode getCSEncMode( const CodingStructure& cs )
{
  return EncTestMode( EncTestModeType( (unsigned)cs.features[ENC_FT_ENC_MODE_TYPE] ),
                      EncTestModeOpts( (unsigned)cs.features[ENC_FT_ENC_MODE_OPTS] ),
                      false);
}



//////////////////////////////////////////////////////////////////////////
// EncModeCtrl controls if specific modes should be tested
//////////////////////////////////////////////////////////////////////////

struct ComprCUCtx
{
  ComprCUCtx() : testModes(), extraFeatures()
  {
  }

  ComprCUCtx( const CodingStructure& cs, const uint32_t _minDepth, const uint32_t _maxDepth, const uint32_t numExtraFeatures )
    : minDepth      ( _minDepth  )
    , maxDepth      ( _maxDepth  )
    , testModes     (            )
    , lastTestMode  (            )
    , earlySkip     ( false      )
#if JVET_M0253_HASH_ME
    , isHashPerfectMatch
                    ( false      )
#endif
    , bestCS        ( nullptr    )
    , bestCU        ( nullptr    )
    , bestTU        ( nullptr    )
    , extraFeatures (            )
    , extraFeaturesd(            )
    , bestInterCost ( MAX_DOUBLE )
#if !JVET_M0464_UNI_MTS
    , bestEmtSize2Nx2N1stPass
                    ( MAX_DOUBLE )
    , skipSecondEMTPass
                    ( false   )
#endif
    , interHad      (std::numeric_limits<Distortion>::max())
#if ENABLE_SPLIT_PARALLELISM
    , isLevelSplitParallel
                    ( false )
#endif
#if JVET_M0102_INTRA_SUBPARTITIONS
    , bestCostWithoutSplitFlags( MAX_DOUBLE )
#if !JVET_M0464_UNI_MTS
    , bestCostEmtFirstPassNoIsp( MAX_DOUBLE )
#endif
#endif
  {
    getAreaIdx( cs.area.Y(), *cs.pcv, cuX, cuY, cuW, cuH );
    partIdx = ( ( cuX << 8 ) | cuY );

    extraFeatures.reserve( numExtraFeatures );
    extraFeatures.resize ( numExtraFeatures, 0 );

    extraFeaturesd.reserve( numExtraFeatures );
    extraFeaturesd.resize ( numExtraFeatures, 0.0 );
  }

  unsigned                          minDepth;
  unsigned                          maxDepth;
  unsigned                          cuX, cuY, cuW, cuH, partIdx;
  std::vector<EncTestMode>          testModes;
  EncTestMode                       lastTestMode;
  bool                              earlySkip;
#if JVET_M0253_HASH_ME
  bool                              isHashPerfectMatch;
#endif
  CodingStructure                  *bestCS;
  CodingUnit                       *bestCU;
  TransformUnit                    *bestTU;
  static_vector<int64_t,  30>         extraFeatures;
  static_vector<double, 30>         extraFeaturesd;
  double                            bestInterCost;
#if !JVET_M0464_UNI_MTS
  double                            bestEmtSize2Nx2N1stPass;
  bool                              skipSecondEMTPass;
#endif
  Distortion                        interHad;
#if ENABLE_SPLIT_PARALLELISM
  bool                              isLevelSplitParallel;
#endif
#if JVET_M0102_INTRA_SUBPARTITIONS
  double                            bestCostWithoutSplitFlags;
#if !JVET_M0464_UNI_MTS
  double                            bestCostEmtFirstPassNoIsp;
#endif
#endif

  template<typename T> T    get( int ft )       const { return typeid(T) == typeid(double) ? (T&)extraFeaturesd[ft] : T(extraFeatures[ft]); }
  template<typename T> void set( int ft, T val )      { extraFeatures [ft] = int64_t( val ); }
  void                      set( int ft, double val ) { extraFeaturesd[ft] = val; }
};

//////////////////////////////////////////////////////////////////////////
// EncModeCtrl - abstract class specifying the general flow of mode control
//////////////////////////////////////////////////////////////////////////

class EncModeCtrl
{
protected:

  const EncCfg         *m_pcEncCfg;
  const class RateCtrl *m_pcRateCtrl;
        class RdCost   *m_pcRdCost;
  const Slice          *m_slice;
#if SHARP_LUMA_DELTA_QP
  int                   m_lumaLevelToDeltaQPLUT[LUMA_LEVEL_TO_DQP_LUT_MAXSIZE];
  int                   m_lumaQPOffset;
#endif
  bool                  m_fastDeltaQP;
  static_vector<ComprCUCtx, ( MAX_CU_DEPTH << 2 )> m_ComprCUCtxList;
#if ENABLE_SPLIT_PARALLELISM
  int                   m_runNextInParallel;
#endif

public:

  virtual ~EncModeCtrl              () {}

#if REUSE_CU_RESULTS
  virtual void create               ( const EncCfg& cfg )                                                                   = 0;
  virtual void destroy              ()                                                                                      = 0;
#endif
  virtual void initCTUEncoding      ( const Slice &slice )                                                                  = 0;
  virtual void initCULevel          ( Partitioner &partitioner, const CodingStructure& cs )                                 = 0;
  virtual void finishCULevel        ( Partitioner &partitioner )                                                            = 0;

protected:

  virtual bool tryMode              ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) = 0;

public:

  virtual bool useModeResult        ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner ) = 0;
#if ENABLE_SPLIT_PARALLELISM
  virtual void copyState            ( const EncModeCtrl& other, const UnitArea& area );
  virtual int  getNumParallelJobs   ( const CodingStructure &cs, Partitioner& partitioner )                                 const { return 1;     }
  virtual bool isParallelSplit      ( const CodingStructure &cs, Partitioner& partitioner )                                 const { return false; }
  virtual bool parallelJobSelector  ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) const { return true;  }
          void setParallelSplit     ( bool val ) { m_runNextInParallel = val; }
#endif

  void         init                 ( EncCfg *pCfg, RateCtrl *pRateCtrl, RdCost *pRdCost );
  bool         tryModeMaster        ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  bool         nextMode             ( const CodingStructure &cs, Partitioner &partitioner );
  EncTestMode  currTestMode         () const;
  EncTestMode  lastTestMode         () const;
  void         setEarlySkipDetected ();
#if JVET_M0253_HASH_ME
  void         setIsHashPerfectMatch( bool b ) { m_ComprCUCtxList.back().isHashPerfectMatch = b; }
  bool         getIsHashPerfectMatch() { return m_ComprCUCtxList.back().isHashPerfectMatch; }
#endif
  virtual void setBest              ( CodingStructure& cs );
  bool         anyMode              () const;

  const ComprCUCtx& getComprCUCtx   () { VTMCHECK( m_ComprCUCtxList.empty(), "Accessing empty list!"); return m_ComprCUCtxList.back(); }

#if SHARP_LUMA_DELTA_QP
  void                  initLumaDeltaQpLUT();
  int                   calculateLumaDQP  ( const CPelBuf& rcOrg );
#endif
  void setFastDeltaQp                 ( bool b )                {        m_fastDeltaQP = b;                               }
  bool getFastDeltaQp                 ()                  const { return m_fastDeltaQP;                                   }

  double getBestInterCost             ()                  const { return m_ComprCUCtxList.back().bestInterCost;           }
  Distortion getInterHad              ()                  const { return m_ComprCUCtxList.back().interHad;                }
  void enforceInterHad                ( Distortion had )        {        m_ComprCUCtxList.back().interHad = had;          }
#if !JVET_M0464_UNI_MTS
  double getEmtSize2Nx2NFirstPassCost ()                  const { return m_ComprCUCtxList.back().bestEmtSize2Nx2N1stPass; }
  bool getSkipSecondEMTPass           ()                  const { return m_ComprCUCtxList.back().skipSecondEMTPass;       }
  void setSkipSecondEMTPass           ( bool b )                {        m_ComprCUCtxList.back().skipSecondEMTPass = b;   }
#endif
#if JVET_M0102_INTRA_SUBPARTITIONS
  double getBestCostWithoutSplitFlags ()                  const { return m_ComprCUCtxList.back().bestCostWithoutSplitFlags;         }
  void   setBestCostWithoutSplitFlags ( double cost )           { m_ComprCUCtxList.back().bestCostWithoutSplitFlags = cost;         }
#if !JVET_M0464_UNI_MTS
  double getEmtFirstPassNoIspCost     ()                  const { return m_ComprCUCtxList.back().bestCostEmtFirstPassNoIsp; }
  void   setEmtFirstPassNoIspCost     ( double cost )           { m_ComprCUCtxList.back().bestCostEmtFirstPassNoIsp = cost; }
#endif
#endif

protected:
  void xExtractFeatures ( const EncTestMode encTestmode, CodingStructure& cs );
  void xGetMinMaxQP     ( int& iMinQP, int& iMaxQP, const CodingStructure& cs, const Partitioner &pm, const int baseQP, const SPS& sps, const PPS& pps, const bool splitMode );
  int  xComputeDQP      ( const CodingStructure &cs, const Partitioner &pm );
};


//////////////////////////////////////////////////////////////////////////
// some utility interfaces that expose some functionality that can be used without concerning about which particular controller is used
//////////////////////////////////////////////////////////////////////////
#if JVET_M0140_SBT
struct SaveLoadStructSbt
{
  uint8_t  numPuInfoStored;
  uint32_t puSse[SBT_NUM_SL];
  uint8_t  puSbt[SBT_NUM_SL];
  uint8_t  puTrs[SBT_NUM_SL];
};

class SaveLoadEncInfoSbt
{
protected:
  void init( const Slice &slice );
  void create();
  void destroy();

private:
  SaveLoadStructSbt ****m_saveLoadSbt;
  Slice const       *m_sliceSbt;

public:
  virtual  ~SaveLoadEncInfoSbt() { }
  void     resetSaveloadSbt( int maxSbtSize );
  uint16_t findBestSbt( const UnitArea& area, const uint32_t curPuSse );
  bool     saveBestSbt( const UnitArea& area, const uint32_t curPuSse, const uint8_t curPuSbt, const uint8_t curPuTrs );
};
#endif

static const int MAX_STORED_CU_INFO_REFS = 4;

struct CodedCUInfo
{
  bool isInter;
  bool isIntra;
  bool isSkip;
  bool isMMVDSkip;
#if JVET_M0483_IBC
  bool isIBC;
#endif
  bool validMv[NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];
  Mv   saveMv [NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];

  uint8_t GBiIdx;

#if ENABLE_SPLIT_PARALLELISM

  uint64_t
       temporalId;
#endif
};

class CacheBlkInfoCtrl
{
private:

  unsigned         m_numWidths, m_numHeights;
  Slice const     *m_slice_chblk;
  // x in CTU, y in CTU, width, height
  CodedCUInfo   ***m_codedCUInfo[MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];

protected:

  void create   ();
  void destroy  ();
#if ENABLE_SPLIT_PARALLELISM
public:
#endif
  void init     ( const Slice &slice );
#if ENABLE_SPLIT_PARALLELISM
private:
  uint64_t
       m_currTemporalId;
public:
  void tick     () { m_currTemporalId++; VTMCHECK( m_currTemporalId <= 0, "Problem with integer overflow!" ); }
  // mark the state of the blk as changed within the current temporal id
  void copyState( const CacheBlkInfoCtrl &other, const UnitArea& area );
protected:
  void touch    ( const UnitArea& area );
#endif

  CodedCUInfo& getBlkInfo( const UnitArea& area );

public:

  virtual ~CacheBlkInfoCtrl() {}

  bool isSkip ( const UnitArea& area );
  bool isMMVDSkip(const UnitArea& area);
  bool getMv  ( const UnitArea& area, const RefPicList refPicList, const int iRefIdx,       Mv& rMv ) const;
  void setMv  ( const UnitArea& area, const RefPicList refPicList, const int iRefIdx, const Mv& rMv );

  bool  getInter( const UnitArea& area );
  void  setGbiIdx( const UnitArea& area, uint8_t gBiIdx );
  uint8_t getGbiIdx( const UnitArea& area );
};

#if REUSE_CU_RESULTS
struct BestEncodingInfo
{
  CodingUnit     cu;
  PredictionUnit pu;
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  TransformUnit  tus[MAX_NUM_TUS];
  size_t         numTus;
#else
  TransformUnit  tu;
#endif
  EncTestMode    testMode;

  int            poc;
};

class BestEncInfoCache
{
private:

  unsigned            m_numWidths, m_numHeights;
  const Slice        *m_slice_bencinf;
  BestEncodingInfo ***m_bestEncInfo[MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
  TCoeff             *m_pCoeff;
  Pel                *m_pPcmBuf;
  CodingStructure     m_dummyCS;
  XUCache             m_dummyCache;

protected:

  void create   ( const ChromaFormat chFmt );
  void destroy  ();
  void init     ( const Slice &slice );

  bool setFromCs( const CodingStructure& cs, const Partitioner& partitioner );
  bool isValid  ( const CodingStructure& cs, const Partitioner& partitioner, int qp );

  // TODO: implement copyState

public:

  BestEncInfoCache() : m_slice_bencinf( nullptr ), m_dummyCS( m_dummyCache.cuCache, m_dummyCache.puCache, m_dummyCache.tuCache ) {}
  virtual ~BestEncInfoCache() {}

  bool     setCsFrom( CodingStructure& cs, EncTestMode& testMode, const Partitioner& partitioner ) const;
};

#endif
//////////////////////////////////////////////////////////////////////////
// EncModeCtrlMTnoRQT - allows and controls modes introduced by QTBT (inkl. multi-type-tree)
//                    - only 2Nx2N, no RQT, additional binary/triary CU splits
//////////////////////////////////////////////////////////////////////////

class EncModeCtrlMTnoRQT : public EncModeCtrl, public CacheBlkInfoCtrl
#if REUSE_CU_RESULTS
  , public BestEncInfoCache
#endif
#if JVET_M0140_SBT
  , public SaveLoadEncInfoSbt
#endif
{
  enum ExtraFeatures
  {
    DID_HORZ_SPLIT = 0,
    DID_VERT_SPLIT,
    DID_QUAD_SPLIT,
    BEST_HORZ_SPLIT_COST,
    BEST_VERT_SPLIT_COST,
    BEST_TRIH_SPLIT_COST,
    BEST_TRIV_SPLIT_COST,
    DO_TRIH_SPLIT,
    DO_TRIV_SPLIT,
    BEST_NON_SPLIT_COST,
    BEST_NO_IMV_COST,
    BEST_IMV_COST,
    QT_BEFORE_BT,
    IS_BEST_NOSPLIT_SKIP,
    MAX_QT_SUB_DEPTH,
#if REUSE_CU_RESULTS
    IS_REUSING_CU,
#endif
    NUM_EXTRA_FEATURES
  };

  unsigned m_skipThreshold;

public:

  EncModeCtrlMTnoRQT ();
  ~EncModeCtrlMTnoRQT();

#if REUSE_CU_RESULTS
  virtual void create             ( const EncCfg& cfg );
  virtual void destroy            ();
#endif
  virtual void initCTUEncoding    ( const Slice &slice );
  virtual void initCULevel        ( Partitioner &partitioner, const CodingStructure& cs );
  virtual void finishCULevel      ( Partitioner &partitioner );

  virtual bool tryMode            ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  virtual bool useModeResult      ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner );

#if ENABLE_SPLIT_PARALLELISM
  virtual void copyState          ( const EncModeCtrl& other, const UnitArea& area );

  virtual int  getNumParallelJobs ( const CodingStructure &cs, Partitioner& partitioner ) const;
  virtual bool isParallelSplit    ( const CodingStructure &cs, Partitioner& partitioner ) const;
  virtual bool parallelJobSelector( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner ) const;
#endif
};


//! \}

#endif // __ENCMODECTRL__
