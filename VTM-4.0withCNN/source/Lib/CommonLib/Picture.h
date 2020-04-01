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

/** \file     Picture.h
 *  \brief    Description of a coded picture
 */

#ifndef __PICTURE__
#define __PICTURE__

#include "CommonDef.h"

#include "Common.h"
#include "Unit.h"
#include "Buffer.h"
#include "Unit.h"
#include "Slice.h"
#include "CodingStructure.h"
#if JVET_M0253_HASH_ME
#include "Hash.h"
#endif
#include <deque>

#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
#if ENABLE_WPP_PARALLELISM
#include <mutex>
class SyncObj;
#endif

#define CURR_THREAD_ID -1

class Scheduler
{
public:
  Scheduler();
  ~Scheduler();

#if ENABLE_SPLIT_PARALLELISM
  unsigned getSplitDataId( int jobId = CURR_THREAD_ID ) const;
  unsigned getSplitPicId ( int tId   = CURR_THREAD_ID ) const;
  unsigned getSplitJobId () const;
  void     setSplitJobId ( const int jobId );
  void     startParallel ();
  void     finishParallel();
  void     setSplitThreadId( const int tId = CURR_THREAD_ID );
  unsigned getNumSplitThreads() const { return m_numSplitThreads; };
#endif
#if ENABLE_WPP_PARALLELISM
  unsigned getWppDataId  ( int lId = CURR_THREAD_ID ) const;
  unsigned getWppThreadId() const;
  void     setWppThreadId( const int tId = CURR_THREAD_ID );
#endif
  unsigned getDataId     () const;
  bool init              ( const int ctuYsize, const int ctuXsize, const int numWppThreadsRunning, const int numWppExtraLines, const int numSplitThreads );
  int  getNumPicInstances() const;
#if ENABLE_WPP_PARALLELISM
  void setReady          ( const int ctuPosX, const int ctuPosY );
  void wait              ( const int ctuPosX, const int ctuPosY );

private:
  bool getNextCtu( Position& pos, int ctuLine, int offset );

private:
  int m_firstNonFinishedLine;
  int m_numWppThreads;
  int m_numWppThreadsRunning;
  int m_numWppDataInstances;
  int m_ctuYsize;
  int m_ctuXsize;

  std::vector<int>         m_LineDone;
  std::vector<bool>        m_LineProc;
  std::mutex               m_mutex;
  std::vector<SyncObj*>    m_SyncObjs;
#endif
#if ENABLE_SPLIT_PARALLELISM

  int   m_numSplitThreads;
  bool  m_hasParallelBuffer;
#endif
};
#endif

class SEI;
class AQpLayer;

typedef std::list<SEI*> SEIMessages;

#if HEVC_TILES_WPP
class Tile
{
private:
  uint32_t      m_tileWidthInCtus;
  uint32_t      m_tileHeightInCtus;
  uint32_t      m_rightEdgePosInCtus;
  uint32_t      m_bottomEdgePosInCtus;
  uint32_t      m_firstCtuRsAddr;

public:
  Tile();
  virtual ~Tile();

  void      setTileWidthInCtus     ( uint32_t i )            { m_tileWidthInCtus = i; }
  uint32_t      getTileWidthInCtus     () const              { return m_tileWidthInCtus; }
  void      setTileHeightInCtus    ( uint32_t i )            { m_tileHeightInCtus = i; }
  uint32_t      getTileHeightInCtus    () const              { return m_tileHeightInCtus; }
  void      setRightEdgePosInCtus  ( uint32_t i )            { m_rightEdgePosInCtus = i; }
  uint32_t      getRightEdgePosInCtus  () const              { return m_rightEdgePosInCtus; }
  void      setBottomEdgePosInCtus ( uint32_t i )            { m_bottomEdgePosInCtus = i; }
  uint32_t      getBottomEdgePosInCtus () const              { return m_bottomEdgePosInCtus; }
  void      setFirstCtuRsAddr      ( uint32_t i )            { m_firstCtuRsAddr = i; }
  uint32_t      getFirstCtuRsAddr      () const              { return m_firstCtuRsAddr; }
};


struct TileMap
{
  TileMap();

  void create( const SPS& sps, const PPS& pps );
  void destroy();

  uint32_t getTileIdxMap( uint32_t ctuRsAddr )       const { return *(tileIdxMap + ctuRsAddr); }
  uint32_t getTileIdxMap( const Position& pos )  const { return getTileIdxMap( ( pos.x / pcv->maxCUWidth ) + ( pos.y / pcv->maxCUHeight ) * pcv->widthInCtus ); };
  uint32_t getCtuTsToRsAddrMap( uint32_t ctuTsAddr ) const { return *(ctuTsToRsAddrMap + (ctuTsAddr>=pcv->sizeInCtus ? pcv->sizeInCtus : ctuTsAddr)); }
  uint32_t getCtuRsToTsAddrMap( uint32_t ctuRsAddr ) const { return *(ctuRsToTsAddrMap + (ctuRsAddr>=pcv->sizeInCtus ? pcv->sizeInCtus : ctuRsAddr)); }
  uint32_t getSubstreamForCtuAddr(const uint32_t ctuAddr, const bool bAddressInRaster, Slice *pcSlice) const;

  const PreCalcValues* pcv;
  std::vector<Tile> tiles;
  uint32_t  numTiles;
  uint32_t  numTileColumns;
  uint32_t  numTileRows;
  uint32_t* tileIdxMap;
  uint32_t* ctuTsToRsAddrMap;
  uint32_t* ctuRsToTsAddrMap;

  void initTileMap( const SPS& sps, const PPS& pps );
  void initCtuTsRsAddrMap();
  uint32_t calculateNextCtuRSAddr( const uint32_t currCtuRsAddr ) const;
};
#endif

#if ENABLE_SPLIT_PARALLELISM
#define M_BUFS(JID,PID) m_bufs[JID][PID]
#else
#define M_BUFS(JID,PID) m_bufs[PID]
#endif

struct Picture : public UnitArea
{
  uint32_t margin;
  Picture();

  void create(const ChromaFormat &_chromaFormat, const Size &size, const unsigned _maxCUSize, const unsigned margin, const bool bDecoder);
  void destroy();

  void createTempBuffers( const unsigned _maxCUSize );
  void destroyTempBuffers();

         PelBuf     getOrigBuf(const CompArea &blk);
  const CPelBuf     getOrigBuf(const CompArea &blk) const;
         PelUnitBuf getOrigBuf(const UnitArea &unit);
  const CPelUnitBuf getOrigBuf(const UnitArea &unit) const;
         PelUnitBuf getOrigBuf();
  const CPelUnitBuf getOrigBuf() const;
#if JVET_M0427_INLOOP_RESHAPER
         PelBuf     getOrigBuf(const ComponentID compID);
  const CPelBuf     getOrigBuf(const ComponentID compID) const;
         PelUnitBuf getTrueOrigBuf();
  const CPelUnitBuf getTrueOrigBuf() const;
        PelBuf      getTrueOrigBuf(const CompArea &blk);
  const CPelBuf     getTrueOrigBuf(const CompArea &blk) const;
#endif

         PelBuf     getPredBuf(const CompArea &blk);
  const CPelBuf     getPredBuf(const CompArea &blk) const;
         PelUnitBuf getPredBuf(const UnitArea &unit);
  const CPelUnitBuf getPredBuf(const UnitArea &unit) const;

         PelBuf     getResiBuf(const CompArea &blk);
  const CPelBuf     getResiBuf(const CompArea &blk) const;
         PelUnitBuf getResiBuf(const UnitArea &unit);
  const CPelUnitBuf getResiBuf(const UnitArea &unit) const;

         PelBuf     getRecoBuf(const ComponentID compID);
  const CPelBuf     getRecoBuf(const ComponentID compID) const;
         PelBuf     getRecoBuf(const CompArea &blk);
  const CPelBuf     getRecoBuf(const CompArea &blk) const;
         PelUnitBuf getRecoBuf(const UnitArea &unit);
  const CPelUnitBuf getRecoBuf(const UnitArea &unit) const;
         PelUnitBuf getRecoBuf();
  const CPelUnitBuf getRecoBuf() const;

         PelBuf     getBuf(const ComponentID compID, const PictureType &type);
  const CPelBuf     getBuf(const ComponentID compID, const PictureType &type) const;
         PelBuf     getBuf(const CompArea &blk,      const PictureType &type);
  const CPelBuf     getBuf(const CompArea &blk,      const PictureType &type) const;
         PelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type);
  const CPelUnitBuf getBuf(const UnitArea &unit,     const PictureType &type) const;

  void extendPicBorder();
  void finalInit( const SPS& sps, const PPS& pps );

  int  getPOC()                               const { return poc; }
  void setBorderExtension( bool bFlag)              { m_bIsBorderExtended = bFlag;}
  Pel* getOrigin( const PictureType &type, const ComponentID compID ) const;

  int           getSpliceIdx(uint32_t idx) const { return m_spliceIdx[idx]; }
  void          setSpliceIdx(uint32_t idx, int poc) { m_spliceIdx[idx] = poc; }
  void          createSpliceIdx(int nums);
  bool          getSpliceFull();

public:
  bool m_bIsBorderExtended;
  bool referenced;
  bool reconstructed;
  bool neededForOutput;
  bool usedByCurr;
  bool longTerm;
  bool topField;
  bool fieldPic;
  int  m_prevQP[MAX_NUM_CHANNEL_TYPE];

  int  poc;
  uint32_t layer;
  uint32_t depth;

  int* m_spliceIdx;
  int  m_ctuNums;

#if ENABLE_SPLIT_PARALLELISM
#if ENABLE_WPP_PARALLELISM
  PelStorage m_bufs[( PARL_SPLIT_MAX_NUM_JOBS * PARL_WPP_MAX_NUM_THREADS )][NUM_PIC_TYPES];
#else
  PelStorage m_bufs[PARL_SPLIT_MAX_NUM_JOBS][NUM_PIC_TYPES];
#endif
#else
  PelStorage m_bufs[NUM_PIC_TYPES];
#endif

#if JVET_M0253_HASH_ME
  TComHash           m_hashMap;
  TComHash*          getHashMap() { return &m_hashMap; }
  const TComHash*    getHashMap() const { return &m_hashMap; }
  void               addPictureToHashMapForInter();
#endif

  CodingStructure*   cs;
  std::deque<Slice*> slices;
  SEIMessages        SEIs;

  void         allocateNewSlice();
  Slice        *swapSliceObject(Slice * p, uint32_t i);
  void         clearSliceBuffer();

#if HEVC_TILES_WPP
  TileMap*     tileMap;
#endif
  std::vector<AQpLayer*> aqlayer;

#if !KEEP_PRED_AND_RESI_SIGNALS
private:
  UnitArea m_ctuArea;
#endif

#if ENABLE_SPLIT_PARALLELISM
public:
  void finishParallelPart   ( const UnitArea& ctuArea );
#if ENABLE_WPP_PARALLELISM
  void finishCtuPart        ( const UnitArea& ctuArea );
#endif
#endif
#if ENABLE_WPP_PARALLELISM || ENABLE_SPLIT_PARALLELISM
public:
  Scheduler                  scheduler;
#endif

public:
  SAOBlkParam    *getSAO(int id = 0)                        { return &m_sao[id][0]; };
  void            resizeSAO(unsigned numEntries, int dstid) { m_sao[dstid].resize(numEntries); }
  void            copySAO(const Picture& src, int dstid)    { std::copy(src.m_sao[0].begin(), src.m_sao[0].end(), m_sao[dstid].begin()); }

#if ENABLE_QPA
  std::vector<double>     m_uEnerHpCtu;                         ///< CTU-wise L2 or squared L1 norm of high-passed luma input
  std::vector<Pel>        m_iOffsetCtu;                         ///< CTU-wise DC offset (later QP index offset) of luma input
#endif

  std::vector<SAOBlkParam> m_sao[2];

  std::vector<uint8_t> m_alfCtuEnableFlag[MAX_NUM_COMPONENT];
  uint8_t* getAlfCtuEnableFlag( int compIdx ) { return m_alfCtuEnableFlag[compIdx].data(); }
  std::vector<uint8_t>* getAlfCtuEnableFlag() { return m_alfCtuEnableFlag; }
  void resizeAlfCtuEnableFlag( int numEntries )
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      m_alfCtuEnableFlag[compIdx].resize( numEntries );
      std::fill( m_alfCtuEnableFlag[compIdx].begin(), m_alfCtuEnableFlag[compIdx].end(), 0 );
    }
  }

#if ADCNN
  std::vector<uint8_t> m_cnnlfCtuEnableFlag[MAX_NUM_COMPONENT];
  uint8_t* getCnnlfCtuEnableFlag(int compIdx) { return m_cnnlfCtuEnableFlag[compIdx].data(); }
  std::vector<uint8_t>* getCnnlfCtuEnableFlag() { return m_cnnlfCtuEnableFlag; }
  void resizeCnnlfCtuEnableFlag(int numEntries)
  {
	  for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
	  {
		  m_cnnlfCtuEnableFlag[compIdx].resize(numEntries);
		  std::fill(m_cnnlfCtuEnableFlag[compIdx].begin(), m_cnnlfCtuEnableFlag[compIdx].end(), 0);
	  }
  }
#endif

};

int calcAndPrintHashStatus(const CPelUnitBuf& pic, const class SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const MsgLevel msgl);


typedef std::list<Picture*> PicList;

#endif
