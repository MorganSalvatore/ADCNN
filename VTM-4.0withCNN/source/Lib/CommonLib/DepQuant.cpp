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

#include "DepQuant.h"
#include "TrQuant.h"
#include "CodingStructure.h"
#include "UnitTools.h"

#include <bitset>






namespace DQIntern
{
  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   R A T E   E S T I M A T O R                                        =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  struct NbInfoSbb
  {
    uint8_t   num;
    uint8_t   inPos[5];
  };
  struct NbInfoOut
  {
    uint16_t  maxDist;
    uint16_t  num;
    uint16_t  outPos[5];
  };
  struct CoeffFracBits
  {
    int32_t   bits[6];
  };


  enum ScanPosType { SCAN_ISCSBB = 0, SCAN_SOCSBB = 1, SCAN_EOCSBB = 2 };

  struct ScanInfo
  {
    ScanInfo() {}
    int           sbbSize;
    int           numSbb;
    int           scanIdx;
    int           rasterPos;
    int           sbbPos;
    int           insidePos;
    bool          eosbb;
    ScanPosType   spt;
    unsigned      sigCtxOffsetNext;
    unsigned      gtxCtxOffsetNext;
    int           nextInsidePos;
    NbInfoSbb     nextNbInfoSbb;
    int           nextSbbRight;
    int           nextSbbBelow;
#if JVET_M0297_32PT_MTS_ZERO_OUT
    int           posX;
    int           posY;
#endif
  };

  class Rom;
  struct TUParameters
  {
    TUParameters ( const Rom& rom, const unsigned width, const unsigned height, const ChannelType chType );
    ~TUParameters()
    {
      delete [] m_scanInfo;
    }

    ChannelType       m_chType;
    unsigned          m_width;
    unsigned          m_height;
    unsigned          m_numCoeff;
    unsigned          m_numSbb;
    unsigned          m_log2SbbWidth;
    unsigned          m_log2SbbHeight;
    unsigned          m_log2SbbSize;
    unsigned          m_sbbSize;
    unsigned          m_sbbMask;
    unsigned          m_widthInSbb;
    unsigned          m_heightInSbb;
    CoeffScanType     m_scanType;
    const ScanElement *m_scanSbbId2SbbPos;
    const ScanElement *m_scanId2BlkPos;
    const NbInfoSbb*  m_scanId2NbInfoSbb;
    const NbInfoOut*  m_scanId2NbInfoOut;
    ScanInfo*         m_scanInfo;
  private:
    void xSetScanInfo( ScanInfo& scanInfo, int scanIdx );
  };

  class Rom
  {
  public:
    Rom() : m_scansInitialized(false) {}
    ~Rom() { xUninitScanArrays(); }
    void                init        ()                       { xInitScanArrays(); }
#if JVET_M0102_INTRA_SUBPARTITIONS
    const NbInfoSbb*    getNbInfoSbb( int hd, int vd, int ch ) const { return m_scanId2NbInfoSbbArray[hd][vd][ch]; }
    const NbInfoOut*    getNbInfoOut( int hd, int vd, int ch ) const { return m_scanId2NbInfoOutArray[hd][vd][ch]; }
#else
    const NbInfoSbb*    getNbInfoSbb( int hd, int vd ) const { return m_scanId2NbInfoSbbArray[hd][vd]; }
    const NbInfoOut*    getNbInfoOut( int hd, int vd ) const { return m_scanId2NbInfoOutArray[hd][vd]; }
#endif
    const TUParameters* getTUPars   ( const CompArea& area, const ComponentID compID ) const
    {
      return m_tuParameters[g_aucLog2[area.width]][g_aucLog2[area.height]][toChannelType(compID)];
    }
  private:
    void  xInitScanArrays   ();
    void  xUninitScanArrays ();
  private:
    bool          m_scansInitialized;
#if JVET_M0102_INTRA_SUBPARTITIONS
    NbInfoSbb*    m_scanId2NbInfoSbbArray[ MAX_CU_DEPTH+1 ][ MAX_CU_DEPTH+1 ][ MAX_NUM_CHANNEL_TYPE ];
    NbInfoOut*    m_scanId2NbInfoOutArray[ MAX_CU_DEPTH+1 ][ MAX_CU_DEPTH+1 ][ MAX_NUM_CHANNEL_TYPE ];
#else
    NbInfoSbb*    m_scanId2NbInfoSbbArray[ MAX_CU_DEPTH+1 ][ MAX_CU_DEPTH+1 ];
    NbInfoOut*    m_scanId2NbInfoOutArray[ MAX_CU_DEPTH+1 ][ MAX_CU_DEPTH+1 ];
#endif
    TUParameters* m_tuParameters         [ MAX_CU_DEPTH+1 ][ MAX_CU_DEPTH+1 ][ MAX_NUM_CHANNEL_TYPE ];
  };

  void Rom::xInitScanArrays()
  {
    if( m_scansInitialized )
    {
      return;
    }
    ::memset( m_scanId2NbInfoSbbArray, 0, sizeof(m_scanId2NbInfoSbbArray) );
    ::memset( m_scanId2NbInfoOutArray, 0, sizeof(m_scanId2NbInfoOutArray) );
    ::memset( m_tuParameters,          0, sizeof(m_tuParameters) );

    uint32_t raster2id[ MAX_CU_SIZE * MAX_CU_SIZE ];
    ::memset(raster2id, 0, sizeof(raster2id));

#if JVET_M0102_INTRA_SUBPARTITIONS
    for( int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++ )
    {
    for( int hd = 0; hd <= MAX_CU_DEPTH; hd++ )
    {
      for( int vd = 0; vd <= MAX_CU_DEPTH; vd++ )
      {
        if( (hd == 0 && vd <= 1) || (hd <= 1 && vd == 0) )
        {
          continue;
        }
#else
    for( int hd = 1; hd <= MAX_CU_DEPTH; hd++ )
    {
      for( int vd = 1; vd <= MAX_CU_DEPTH; vd++ )
      {
#endif
        const uint32_t      blockWidth    = (1 << hd);
        const uint32_t      blockHeight   = (1 << vd);
#if JVET_M0102_INTRA_SUBPARTITIONS
        const uint32_t      log2CGWidth   = g_log2SbbSize[ch][hd][vd][0];
        const uint32_t      log2CGHeight  = g_log2SbbSize[ch][hd][vd][1];
#else
        const uint32_t      log2CGWidth   = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;
        const uint32_t      log2CGHeight  = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;
#endif
        const uint32_t      groupWidth    = 1 << log2CGWidth;
        const uint32_t      groupHeight   = 1 << log2CGHeight;
        const uint32_t      groupSize     = groupWidth * groupHeight;
        const CoeffScanType scanType      = SCAN_DIAG;
        const SizeType      blkWidthIdx   = gp_sizeIdxInfo->idxFrom( blockWidth  );
        const SizeType      blkHeightIdx  = gp_sizeIdxInfo->idxFrom( blockHeight );
#if JVET_M0102_INTRA_SUBPARTITIONS
        const ScanElement * scanId2RP     = g_scanOrder[ch][SCAN_GROUPED_4x4][scanType][blkWidthIdx][blkHeightIdx];
        NbInfoSbb*&         sId2NbSbb     = m_scanId2NbInfoSbbArray[hd][vd][ch];
        NbInfoOut*&         sId2NbOut     = m_scanId2NbInfoOutArray[hd][vd][ch];
#else
        const ScanElement * scanId2RP     = g_scanOrder[SCAN_GROUPED_4x4][scanType][blkWidthIdx][blkHeightIdx];
        NbInfoSbb*&         sId2NbSbb     = m_scanId2NbInfoSbbArray[hd][vd];
        NbInfoOut*&         sId2NbOut     = m_scanId2NbInfoOutArray[hd][vd];
#endif
        // consider only non-zero-out region
#if JVET_M0257
        const uint32_t      blkWidthNZOut = std::min<unsigned>( JVET_C0024_ZERO_OUT_TH, blockWidth  );
        const uint32_t      blkHeightNZOut= std::min<unsigned>( JVET_C0024_ZERO_OUT_TH, blockHeight );
#else
        const uint32_t      blkWidthNZOut = blockWidth;
        const uint32_t      blkHeightNZOut= blockHeight;
#endif
        const uint32_t      totalValues   = blkWidthNZOut * blkHeightNZOut;

        sId2NbSbb = new NbInfoSbb[ totalValues ];
        sId2NbOut = new NbInfoOut[ totalValues ];

        for( uint32_t scanId = 0; scanId < totalValues; scanId++ )
        {
          raster2id[scanId2RP[scanId].idx] = scanId;
        }

        for( unsigned scanId = 0; scanId < totalValues; scanId++ )
        {
          const int posX = scanId2RP[scanId].x;
          const int posY = scanId2RP[scanId].y;
          const int rpos = scanId2RP[scanId].idx;
          {
            //===== inside subband neighbours =====
            NbInfoSbb&     nbSbb  = sId2NbSbb[ scanId ];
            const int      begSbb = scanId - ( scanId & (groupSize-1) ); // first pos in current subblock
            int            cpos[5];

            cpos[0] = ( posX + 1 < blkWidthNZOut                              ? ( raster2id[rpos+1           ] < groupSize + begSbb ? raster2id[rpos+1           ] - begSbb : 0 ) : 0 );
            cpos[1] = ( posX + 2 < blkWidthNZOut                              ? ( raster2id[rpos+2           ] < groupSize + begSbb ? raster2id[rpos+2           ] - begSbb : 0 ) : 0 );
            cpos[2] = ( posX + 1 < blkWidthNZOut && posY + 1 < blkHeightNZOut ? ( raster2id[rpos+1+blockWidth] < groupSize + begSbb ? raster2id[rpos+1+blockWidth] - begSbb : 0 ) : 0 );
            cpos[3] = ( posY + 1 < blkHeightNZOut                             ? ( raster2id[rpos+  blockWidth] < groupSize + begSbb ? raster2id[rpos+  blockWidth] - begSbb : 0 ) : 0 );
            cpos[4] = ( posY + 2 < blkHeightNZOut                             ? ( raster2id[rpos+2*blockWidth] < groupSize + begSbb ? raster2id[rpos+2*blockWidth] - begSbb : 0 ) : 0 );

            for( nbSbb.num = 0; true; )
            {
              int nk = -1;
              for( int k = 0; k < 5; k++ )
              {
                if( cpos[k] != 0 && ( nk < 0 || cpos[k] < cpos[nk] ) )
                {
                  nk = k;
                }
              }
              if( nk < 0 )
              {
                break;
              }
              nbSbb.inPos[ nbSbb.num++ ] = uint8_t( cpos[nk] );
              cpos[nk] = 0;
            }
            for( int k = nbSbb.num; k < 5; k++ )
            {
              nbSbb.inPos[k] = 0;
            }
          }
          {
            //===== outside subband neighbours =====
            NbInfoOut&     nbOut  = sId2NbOut[ scanId ];
            const int      begSbb = scanId - ( scanId & (groupSize-1) ); // first pos in current subblock
            int            cpos[5];

            cpos[0] = ( posX + 1 < blkWidthNZOut                              ? ( raster2id[rpos+1           ] >= groupSize + begSbb ? raster2id[rpos+1           ] : 0 ) : 0 );
            cpos[1] = ( posX + 2 < blkWidthNZOut                              ? ( raster2id[rpos+2           ] >= groupSize + begSbb ? raster2id[rpos+2           ] : 0 ) : 0 );
            cpos[2] = ( posX + 1 < blkWidthNZOut && posY + 1 < blkHeightNZOut ? ( raster2id[rpos+1+blockWidth] >= groupSize + begSbb ? raster2id[rpos+1+blockWidth] : 0 ) : 0 );
            cpos[3] = ( posY + 1 < blkHeightNZOut                             ? ( raster2id[rpos+  blockWidth] >= groupSize + begSbb ? raster2id[rpos+  blockWidth] : 0 ) : 0 );
            cpos[4] = ( posY + 2 < blkHeightNZOut                             ? ( raster2id[rpos+2*blockWidth] >= groupSize + begSbb ? raster2id[rpos+2*blockWidth] : 0 ) : 0 );

            for( nbOut.num = 0; true; )
            {
              int nk = -1;
              for( int k = 0; k < 5; k++ )
              {
                if( cpos[k] != 0 && ( nk < 0 || cpos[k] < cpos[nk] ) )
                {
                  nk = k;
                }
              }
              if( nk < 0 )
              {
                break;
              }
              nbOut.outPos[ nbOut.num++ ] = uint16_t( cpos[nk] );
              cpos[nk] = 0;
            }
            for( int k = nbOut.num; k < 5; k++ )
            {
              nbOut.outPos[k] = 0;
            }
            nbOut.maxDist = ( scanId == 0 ? 0 : sId2NbOut[scanId-1].maxDist );
            for( int k = 0; k < nbOut.num; k++ )
            {
              if( nbOut.outPos[k] > nbOut.maxDist )
              {
                nbOut.maxDist = nbOut.outPos[k];
              }
            }
          }
        }

        // make it relative
        for( unsigned scanId = 0; scanId < totalValues; scanId++ )
        {
          NbInfoOut& nbOut  = sId2NbOut[scanId];
          const int  begSbb = scanId - ( scanId & (groupSize-1) ); // first pos in current subblock
          for( int k = 0; k < nbOut.num; k++ )
          {
            VTMCHECK(begSbb > nbOut.outPos[k], "Position must be past sub block begin");
            nbOut.outPos[k] -= begSbb;
          }
          nbOut.maxDist -= scanId;
        }

#if JVET_M0102_INTRA_SUBPARTITIONS
        m_tuParameters[hd][vd][ch] = new TUParameters( *this, blockWidth, blockHeight, ChannelType(ch) );
#else
        for( int chId = 0; chId < MAX_NUM_CHANNEL_TYPE; chId++ )
        {
          m_tuParameters[hd][vd][chId] = new TUParameters( *this, blockWidth, blockHeight, ChannelType(chId) );
        }
#endif
      }
    }
#if JVET_M0102_INTRA_SUBPARTITIONS
    }
#endif
    m_scansInitialized = true;
  }

  void Rom::xUninitScanArrays()
  {
    if( !m_scansInitialized )
    {
      return;
    }
#if JVET_M0102_INTRA_SUBPARTITIONS
    for( int hd = 0; hd <= MAX_CU_DEPTH; hd++ )
    {
      for( int vd = 0; vd <= MAX_CU_DEPTH; vd++ )
      {
        for( int ch = 0; ch < 2; ch++ )
        {
          NbInfoSbb*&     sId2NbSbb = m_scanId2NbInfoSbbArray[hd][vd][ch];
          NbInfoOut*&     sId2NbOut = m_scanId2NbInfoOutArray[hd][vd][ch];
          TUParameters*&  tuPars    = m_tuParameters         [hd][vd][ch];
          if( sId2NbSbb )
          {
            delete [] sId2NbSbb;
          }
          if( sId2NbOut )
          {
            delete [] sId2NbOut;
          }
          if( tuPars )
          {
            delete tuPars;
          }
        }
      }
    }
#else
    for( int hd = 0; hd <= MAX_CU_DEPTH; hd++ )
    {
      for( int vd = 0; vd <= MAX_CU_DEPTH; vd++ )
      {
        NbInfoSbb*& sId2NbSbb = m_scanId2NbInfoSbbArray[hd][vd];
        NbInfoOut*& sId2NbOut = m_scanId2NbInfoOutArray[hd][vd];
        if( sId2NbSbb )
        {
          delete [] sId2NbSbb;
        }
        if( sId2NbOut )
        {
          delete [] sId2NbOut;
        }
        for( int chId = 0; chId < MAX_NUM_CHANNEL_TYPE; chId++ )
        {
          TUParameters*& tuPars = m_tuParameters[hd][vd][chId];
          if( tuPars )
          {
            delete tuPars;
          }
        }
      }
    }
#endif
    m_scansInitialized = false;
  }


  static Rom g_Rom;


  TUParameters::TUParameters( const Rom& rom, const unsigned width, const unsigned height, const ChannelType chType )
  {
    m_chType              = chType;
    m_width               = width;
    m_height              = height;
#if JVET_M0257
    const uint32_t nonzeroWidth  = std::min<uint32_t>(JVET_C0024_ZERO_OUT_TH, m_width);
    const uint32_t nonzeroHeight = std::min<uint32_t>(JVET_C0024_ZERO_OUT_TH, m_height);
    m_numCoeff                   = nonzeroWidth * nonzeroHeight;
#else
    m_numCoeff            = m_width * m_height;
#endif
#if JVET_M0102_INTRA_SUBPARTITIONS
    m_log2SbbWidth        = g_log2SbbSize[m_chType][ g_aucLog2[m_width] ][ g_aucLog2[m_height] ][0];
    m_log2SbbHeight       = g_log2SbbSize[m_chType][ g_aucLog2[m_width] ][ g_aucLog2[m_height] ][1];
#else
    const bool      no4x4 = ( ( m_width & 3 ) != 0 || ( m_height & 3 ) != 0 );
    m_log2SbbWidth        = ( no4x4 ? 1 : 2 );
    m_log2SbbHeight       = ( no4x4 ? 1 : 2 );
#endif
    m_log2SbbSize         = m_log2SbbWidth + m_log2SbbHeight;
    m_sbbSize             = ( 1 << m_log2SbbSize );
    m_sbbMask             = m_sbbSize - 1;
#if JVET_M0257
    m_widthInSbb  = nonzeroWidth >> m_log2SbbWidth;
    m_heightInSbb = nonzeroHeight >> m_log2SbbHeight;
#else
    m_widthInSbb          = m_width  >> m_log2SbbWidth;
    m_heightInSbb         = m_height >> m_log2SbbHeight;
#endif
    m_numSbb              = m_widthInSbb * m_heightInSbb;
#if HEVC_USE_MDCS
#error "MDCS is not supported" // use different function...
    //  m_scanType            = CoeffScanType( TU::getCoefScanIdx( tu, m_compID ) );
#else
    m_scanType            = SCAN_DIAG;
#endif
    SizeType        hsbb  = gp_sizeIdxInfo->idxFrom( m_widthInSbb  );
    SizeType        vsbb  = gp_sizeIdxInfo->idxFrom( m_heightInSbb );
    SizeType        hsId  = gp_sizeIdxInfo->idxFrom( m_width  );
    SizeType        vsId  = gp_sizeIdxInfo->idxFrom( m_height );
#if JVET_M0102_INTRA_SUBPARTITIONS
    m_scanSbbId2SbbPos    = g_scanOrder     [ chType ][ SCAN_UNGROUPED   ][ m_scanType ][ hsbb ][ vsbb ];
    m_scanId2BlkPos       = g_scanOrder     [ chType ][ SCAN_GROUPED_4x4 ][ m_scanType ][ hsId ][ vsId ];
    int log2W             = g_aucLog2[ m_width  ];
    int log2H             = g_aucLog2[ m_height ];
    m_scanId2NbInfoSbb    = rom.getNbInfoSbb( log2W, log2H, chType );
    m_scanId2NbInfoOut    = rom.getNbInfoOut( log2W, log2H, chType );
#else
    m_scanSbbId2SbbPos    = g_scanOrder     [ SCAN_UNGROUPED   ][ m_scanType ][ hsbb ][ vsbb ];
    m_scanId2BlkPos       = g_scanOrder     [ SCAN_GROUPED_4x4 ][ m_scanType ][ hsId ][ vsId ];
    int log2W             = g_aucLog2[ m_width  ];
    int log2H             = g_aucLog2[ m_height ];
    m_scanId2NbInfoSbb    = rom.getNbInfoSbb( log2W, log2H );
    m_scanId2NbInfoOut    = rom.getNbInfoOut( log2W, log2H );
#endif
    m_scanInfo            = new ScanInfo[ m_numCoeff ];
    for( int scanIdx = 0; scanIdx < m_numCoeff; scanIdx++ )
    {
      xSetScanInfo( m_scanInfo[scanIdx], scanIdx );
    }
  }


  void TUParameters::xSetScanInfo( ScanInfo& scanInfo, int scanIdx )
  {
    scanInfo.sbbSize    = m_sbbSize;
    scanInfo.numSbb     = m_numSbb;
    scanInfo.scanIdx    = scanIdx;
    scanInfo.rasterPos  = m_scanId2BlkPos[scanIdx].idx;
    scanInfo.sbbPos     = m_scanSbbId2SbbPos[scanIdx >> m_log2SbbSize].idx;
    scanInfo.insidePos  = scanIdx & m_sbbMask;
    scanInfo.eosbb      = ( scanInfo.insidePos == 0 );
    scanInfo.spt        = SCAN_ISCSBB;
    if(  scanInfo.insidePos == m_sbbMask && scanIdx > scanInfo.sbbSize && scanIdx < m_numCoeff - 1 )
      scanInfo.spt      = SCAN_SOCSBB;
    else if( scanInfo.eosbb && scanIdx > 0 && scanIdx < m_numCoeff - m_sbbSize )
      scanInfo.spt      = SCAN_EOCSBB;
#if JVET_M0297_32PT_MTS_ZERO_OUT
    scanInfo.posX = m_scanId2BlkPos[scanIdx].x;
    scanInfo.posY = m_scanId2BlkPos[scanIdx].y;
#endif
    if( scanIdx )
    {
      const int nextScanIdx = scanIdx - 1;
      const int diag        = m_scanId2BlkPos[nextScanIdx].x + m_scanId2BlkPos[nextScanIdx].y;
      if( m_chType == CHANNEL_TYPE_LUMA )
      {
        scanInfo.sigCtxOffsetNext = ( diag < 2 ? 12 : diag < 5 ?  6 : 0 );
        scanInfo.gtxCtxOffsetNext = ( diag < 1 ? 16 : diag < 3 ? 11 : diag < 10 ? 6 : 1 );
      }
      else
      {
        scanInfo.sigCtxOffsetNext = ( diag < 2 ? 6 : 0 );
        scanInfo.gtxCtxOffsetNext = ( diag < 1 ? 6 : 1 );
      }
      scanInfo.nextInsidePos      = nextScanIdx & m_sbbMask;
      scanInfo.nextNbInfoSbb      = m_scanId2NbInfoSbb[ nextScanIdx ];
      if( scanInfo.eosbb )
      {
        const int nextSbbPos  = m_scanSbbId2SbbPos[nextScanIdx >> m_log2SbbSize].idx;
        const int nextSbbPosY = nextSbbPos               / m_widthInSbb;
        const int nextSbbPosX = nextSbbPos - nextSbbPosY * m_widthInSbb;
        scanInfo.nextSbbRight = ( nextSbbPosX < m_widthInSbb  - 1 ? nextSbbPos + 1            : 0 );
        scanInfo.nextSbbBelow = ( nextSbbPosY < m_heightInSbb - 1 ? nextSbbPos + m_widthInSbb : 0 );
      }
    }
  }



  class RateEstimator
  {
  public:
    RateEstimator () {}
    ~RateEstimator() {}
    void initCtx  ( const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID, const FracBitsAccess& fracBitsAccess );

    inline const BinFracBits *sigSbbFracBits() const { return m_sigSbbFracBits; }
    inline const BinFracBits *sigFlagBits(unsigned stateId) const
    {
      return m_sigFracBits[std::max(((int) stateId) - 1, 0)];
    }
    inline const CoeffFracBits *gtxFracBits(unsigned stateId) const { return m_gtxFracBits; }
    inline int32_t              lastOffset(unsigned scanIdx) const
    {
      return m_lastBitsX[m_scanId2Pos[scanIdx].x] + m_lastBitsY[m_scanId2Pos[scanIdx].y];
    }

  private:
    void  xSetLastCoeffOffset ( const FracBitsAccess& fracBitsAccess, const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID );
    void  xSetSigSbbFracBits  ( const FracBitsAccess& fracBitsAccess, ChannelType chType );
    void  xSetSigFlagBits     ( const FracBitsAccess& fracBitsAccess, ChannelType chType );
    void  xSetGtxFlagBits     ( const FracBitsAccess& fracBitsAccess, ChannelType chType );

  private:
    static const unsigned sm_numCtxSetsSig    = 3;
    static const unsigned sm_numCtxSetsGtx    = 2;
    static const unsigned sm_maxNumSigSbbCtx  = 2;
    static const unsigned sm_maxNumSigCtx     = 18;
    static const unsigned sm_maxNumGtxCtx     = 21;

  private:
    const ScanElement * m_scanId2Pos;
    int32_t             m_lastBitsX      [ MAX_TU_SIZE ];
    int32_t             m_lastBitsY      [ MAX_TU_SIZE ];
    BinFracBits         m_sigSbbFracBits [ sm_maxNumSigSbbCtx ];
    BinFracBits         m_sigFracBits    [ sm_numCtxSetsSig   ][ sm_maxNumSigCtx ];
    CoeffFracBits       m_gtxFracBits                          [ sm_maxNumGtxCtx ];
  };

  void RateEstimator::initCtx( const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID, const FracBitsAccess& fracBitsAccess )
  {
    m_scanId2Pos = tuPars.m_scanId2BlkPos;
    xSetSigSbbFracBits  ( fracBitsAccess, tuPars.m_chType );
    xSetSigFlagBits     ( fracBitsAccess, tuPars.m_chType );
    xSetGtxFlagBits     ( fracBitsAccess, tuPars.m_chType );
    xSetLastCoeffOffset ( fracBitsAccess, tuPars, tu, compID );
  }

  void RateEstimator::xSetLastCoeffOffset( const FracBitsAccess& fracBitsAccess, const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID )
  {
    const ChannelType chType = ( compID == COMPONENT_Y ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA );
    int32_t cbfDeltaBits = 0;
    if( compID == COMPONENT_Y && !CU::isIntra(*tu.cu) && !tu.depth )
    {
      const BinFracBits bits  = fracBitsAccess.getFracBitsArray( Ctx::QtRootCbf() );
      cbfDeltaBits            = int32_t( bits.intBits[1] ) - int32_t( bits.intBits[0] );
    }
    else
    {
#if JVET_M0102_INTRA_SUBPARTITIONS
      BinFracBits bits;
      bool prevLumaCbf           = false;
      bool lastCbfIsInferred     = false;
      bool useIntraSubPartitions = tu.cu->ispMode && isLuma(chType);
      if( useIntraSubPartitions )
      {
        bool rootCbfSoFar = false;
        bool isLastSubPartition = CU::isISPLast(*tu.cu, tu.Y(), compID);
        uint32_t nTus = tu.cu->ispMode == HOR_INTRA_SUBPARTITIONS ? tu.cu->lheight() >> g_aucLog2[tu.lheight()] : tu.cu->lwidth() >> g_aucLog2[tu.lwidth()];
        if( isLastSubPartition )
        {
          TransformUnit* tuPointer = tu.cu->firstTU;
          for( int tuIdx = 0; tuIdx < nTus - 1; tuIdx++ )
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMPONENT_Y, tu.depth);
            tuPointer     = tuPointer->next;
          }
          if( !rootCbfSoFar )
          {
            lastCbfIsInferred = true;
          }
        }
        if( !lastCbfIsInferred )
        {
          prevLumaCbf = TU::getPrevTuCbfAtDepth(tu, compID, tu.depth);
        }
        bits = fracBitsAccess.getFracBitsArray(Ctx::QtCbf[compID](DeriveCtx::CtxQtCbf(compID, tu.depth, prevLumaCbf, true)));
      }
      else
      {
        bits = fracBitsAccess.getFracBitsArray(Ctx::QtCbf[compID](DeriveCtx::CtxQtCbf(compID, tu.depth, tu.cbf[COMPONENT_Cb])));
      }
      cbfDeltaBits = lastCbfIsInferred ? 0 : int32_t(bits.intBits[1]) - int32_t(bits.intBits[0]);
#else
      BinFracBits bits = fracBitsAccess.getFracBitsArray( Ctx::QtCbf[compID]( DeriveCtx::CtxQtCbf( compID, tu.depth, tu.cbf[COMPONENT_Cb] ) ) );
      cbfDeltaBits = int32_t( bits.intBits[1] ) - int32_t( bits.intBits[0] );
#endif
    }

    static const unsigned prefixCtx[] = { 0, 0, 0, 3, 6, 10, 15, 21 };
    uint32_t              ctxBits  [ LAST_SIGNIFICANT_GROUPS ];
    for( unsigned xy = 0; xy < 2; xy++ )
    {
      int32_t             bitOffset   = ( xy ? cbfDeltaBits : 0 );
      int32_t*            lastBits    = ( xy ? m_lastBitsY : m_lastBitsX );
      const unsigned      size        = ( xy ? tuPars.m_height : tuPars.m_width );
      const unsigned      log2Size    = g_aucNextLog2[ size ];
#if HEVC_USE_MDCS
      const bool          useYCtx     = ( m_scanType == SCAN_VER ? ( xy == 0 ) : ( xy != 0 ) );
#else
      const bool          useYCtx     = ( xy != 0 );
#endif
      const CtxSet&       ctxSetLast  = ( useYCtx ? Ctx::LastY : Ctx::LastX )[ chType ];
      const unsigned      lastShift   = ( compID == COMPONENT_Y ? (log2Size+1)>>2 : Clip3<unsigned>(0,2,size>>3) );
      const unsigned      lastOffset  = ( compID == COMPONENT_Y ? ( prefixCtx[log2Size] ) : 0 );
      uint32_t            sumFBits    = 0;
#if JVET_M0257
      unsigned            maxCtxId    = g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, size) - 1];
#else
      unsigned            maxCtxId    = g_uiGroupIdx[ size - 1 ];
#endif
      for( unsigned ctxId = 0; ctxId < maxCtxId; ctxId++ )
      {
        const BinFracBits bits  = fracBitsAccess.getFracBitsArray( ctxSetLast( lastOffset + ( ctxId >> lastShift ) ) );
        ctxBits[ ctxId ]        = sumFBits + bits.intBits[0] + ( ctxId>3 ? ((ctxId-2)>>1)<<SCALE_BITS : 0 ) + bitOffset;
        sumFBits               +=            bits.intBits[1];
      }
      ctxBits  [ maxCtxId ]     = sumFBits + ( maxCtxId>3 ? ((maxCtxId-2)>>1)<<SCALE_BITS : 0 ) + bitOffset;
#if JVET_M0257
      for (unsigned pos = 0; pos < std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, size); pos++)
#else
      for( unsigned pos = 0; pos < size; pos++ )
#endif
      {
        lastBits[ pos ]         = ctxBits[ g_uiGroupIdx[ pos ] ];
      }
    }
  }

  void RateEstimator::xSetSigSbbFracBits( const FracBitsAccess& fracBitsAccess, ChannelType chType )
  {
    const CtxSet& ctxSet = Ctx::SigCoeffGroup[ chType ];
    for( unsigned ctxId = 0; ctxId < sm_maxNumSigSbbCtx; ctxId++ )
    {
      m_sigSbbFracBits[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
    }
  }

  void RateEstimator::xSetSigFlagBits( const FracBitsAccess& fracBitsAccess, ChannelType chType )
  {
    for( unsigned ctxSetId = 0; ctxSetId < sm_numCtxSetsSig; ctxSetId++ )
    {
      BinFracBits*    bits    = m_sigFracBits [ ctxSetId ];
      const CtxSet&   ctxSet  = Ctx::SigFlag  [ chType + 2*ctxSetId ];
      const unsigned  numCtx  = ( chType == CHANNEL_TYPE_LUMA ? 18 : 12 );
      for( unsigned ctxId = 0; ctxId < numCtx; ctxId++ )
      {
        bits[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
      }
    }
  }

  void RateEstimator::xSetGtxFlagBits( const FracBitsAccess& fracBitsAccess, ChannelType chType )
  {
    const CtxSet&   ctxSetPar   = Ctx::ParFlag [     chType ];
    const CtxSet&   ctxSetGt1   = Ctx::GtxFlag [ 2 + chType ];
    const CtxSet&   ctxSetGt2   = Ctx::GtxFlag [     chType ];
    const unsigned  numCtx      = ( chType == CHANNEL_TYPE_LUMA ? 21 : 11 );
    for( unsigned ctxId = 0; ctxId < numCtx; ctxId++ )
    {
      BinFracBits     fbPar = fracBitsAccess.getFracBitsArray( ctxSetPar( ctxId ) );
      BinFracBits     fbGt1 = fracBitsAccess.getFracBitsArray( ctxSetGt1( ctxId ) );
      BinFracBits     fbGt2 = fracBitsAccess.getFracBitsArray( ctxSetGt2( ctxId ) );
      CoeffFracBits&  cb    = m_gtxFracBits[ ctxId ];
      int32_t         par0  = (1<<SCALE_BITS) + int32_t(fbPar.intBits[0]);
      int32_t         par1  = (1<<SCALE_BITS) + int32_t(fbPar.intBits[1]);
      cb.bits[0] = 0;
      cb.bits[1] = fbGt1.intBits[0] + (1 << SCALE_BITS);
      cb.bits[2] = fbGt1.intBits[1] + par0 + fbGt2.intBits[0];
      cb.bits[3] = fbGt1.intBits[1] + par1 + fbGt2.intBits[0];
      cb.bits[4] = fbGt1.intBits[1] + par0 + fbGt2.intBits[1];
      cb.bits[5] = fbGt1.intBits[1] + par1 + fbGt2.intBits[1];
    }
  }





  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   D A T A   S T R U C T U R E S                                      =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/


  struct PQData
  {
    TCoeff  absLevel;
    int64_t deltaDist;
  };


  struct Decision
  {
    int64_t rdCost;
    TCoeff  absLevel;
    int     prevId;
  };




  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   P R E - Q U A N T I Z E R                                          =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  class Quantizer
  {
  public:
    Quantizer() {}

    void  dequantBlock  ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff   ) const;
    void  initQuantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda  );

    inline void   preQuantCoeff(const TCoeff absCoeff, PQData *pqData) const;
    inline TCoeff getLastThreshold() const { return m_thresLast; }
    inline TCoeff getSSbbThreshold() const { return m_thresSSbb; }

  private:
    // quantization
    int               m_QShift;
    int64_t           m_QAdd;
    int64_t           m_QScale;
    TCoeff            m_maxQIdx;
    TCoeff            m_thresLast;
    TCoeff            m_thresSSbb;
    // distortion normalization
    int               m_DistShift;
    int64_t           m_DistAdd;
    int64_t           m_DistStepAdd;
    int64_t           m_DistOrgFact;
  };

  inline int ceil_log2(uint64_t x)
  {
    static const uint64_t t[6] = { 0xFFFFFFFF00000000ull, 0x00000000FFFF0000ull, 0x000000000000FF00ull, 0x00000000000000F0ull, 0x000000000000000Cull, 0x0000000000000002ull };
    int y = (((x & (x - 1)) == 0) ? 0 : 1);
    int j = 32;
    for( int i = 0; i < 6; i++)
    {
      int k = (((x & t[i]) == 0) ? 0 : j);
      y += k;
      x >>= k;
      j >>= 1;
    }
    return y;
  }

  void Quantizer::initQuantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda )
  {
#if HEVC_USE_SCALING_LISTS
    VTMCHECK ( tu.cs->sps->getScalingListFlag(), "Scaling lists not supported" );
#endif
    CHECKD( lambda <= 0.0, "Lambda must be greater than 0" );

    const int         qpDQ                  = cQP.Qp + 1;
    const int         qpPer                 = qpDQ / 6;
    const int         qpRem                 = qpDQ - 6 * qpPer;
    const SPS&        sps                   = *tu.cs->sps;
    const CompArea&   area                  = tu.blocks[ compID ];
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.getBitDepth( chType );
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange( chType );
    const int         nomTransformShift     = getTransformShift( channelBitDepth, area.size(), maxLog2TrDynamicRange );
#if JVET_M0464_UNI_MTS
    const bool        clipTransformShift    = ( tu.mtsIdx==1 && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() );
#else
    const bool        clipTransformShift    = ( tu.transformSkip[ compID ] && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() );
#endif
    const int         transformShift        = ( clipTransformShift ? std::max<int>( 0, nomTransformShift ) : nomTransformShift );

    // quant parameters
    m_QShift                    = QUANT_SHIFT  - 1 + qpPer + transformShift;
    m_QAdd                      = -( ( 3 << m_QShift ) >> 1 );
#if HM_QTBT_AS_IN_JEM_QUANT
#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
    Intermediate_Int  invShift  = IQUANT_SHIFT + 1 - qpPer - transformShift + ( TU::needsBlockSizeTrafoScale( tu, compID ) ? ADJ_DEQUANT_SHIFT : 0 );
    m_QScale                    = ( TU::needsSqrt2Scale( tu, compID ) ? ( g_quantScales[ qpRem ] * 181 ) >> 7 : g_quantScales[ qpRem ] );
#else
    Intermediate_Int  invShift  = IQUANT_SHIFT + 1 - qpPer - transformShift + ( TU::needsBlockSizeTrafoScale( area ) ? ADJ_DEQUANT_SHIFT : 0 );
    m_QScale                    = ( TU::needsSqrt2Scale( area ) ? ( g_quantScales[ qpRem ] * 181 ) >> 7 : g_quantScales[ qpRem ] );
#endif
#else
    Intermediate_Int  invShift  = IQUANT_SHIFT + 1 - qpPer - transformShift;
    m_QScale                    = g_quantScales   [ qpRem ];
#endif
    const unsigned    qIdxBD    = std::min<unsigned>( maxLog2TrDynamicRange + 1, 8*sizeof(Intermediate_Int) + invShift - IQUANT_SHIFT - 1 );
    m_maxQIdx                   = ( 1 << (qIdxBD-1) ) - 4;
    m_thresLast                 = TCoeff( ( int64_t(3) << m_QShift ) / ( 4 * m_QScale ) );
    m_thresSSbb                 = TCoeff( ( int64_t(3) << m_QShift ) / ( 4 * m_QScale ) );

    // distortion calculation parameters
    const int64_t qScale        = g_quantScales[ qpRem ];
#if HM_QTBT_AS_IN_JEM_QUANT
    const int nomDShift =
      SCALE_BITS - 2 * (nomTransformShift + DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth)) + m_QShift;
#else
    const int nomDShift = SCALE_BITS - 2 * (nomTransformShift + DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth))
                          + m_QShift + (TU::needsQP3Offset(tu, compID) ? 1 : 0);
#endif
    const double  qScale2       = double( qScale * qScale );
    const double  nomDistFactor = ( nomDShift < 0 ? 1.0/(double(int64_t(1)<<(-nomDShift))*qScale2*lambda) : double(int64_t(1)<<nomDShift)/(qScale2*lambda) );
    const int64_t pow2dfShift   = (int64_t)( nomDistFactor * qScale2 ) + 1;
    const int     dfShift       = ceil_log2( pow2dfShift );
    m_DistShift                 = 62 + m_QShift - 2*maxLog2TrDynamicRange - dfShift;
    m_DistAdd                   = (int64_t(1) << m_DistShift) >> 1;
    m_DistStepAdd               = (int64_t)( nomDistFactor * double(int64_t(1)<<(m_DistShift+m_QShift)) + .5 );
    m_DistOrgFact               = (int64_t)( nomDistFactor * double(int64_t(1)<<(m_DistShift+1       )) + .5 );
  }

  void Quantizer::dequantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff ) const
  {
#if HEVC_USE_SCALING_LISTS
    VTMCHECK ( tu.cs->sps->getScalingListFlag(), "Scaling lists not supported" );
#endif

    //----- set basic parameters -----
    const CompArea&     area      = tu.blocks[ compID ];
    const int           numCoeff  = area.area();
    const SizeType      hsId      = gp_sizeIdxInfo->idxFrom( area.width  );
    const SizeType      vsId      = gp_sizeIdxInfo->idxFrom( area.height );
#if HEVC_USE_MDCS
    const CoeffScanType scanType  = CoeffScanType( TU::getCoefScanIdx( tu, compID ) );
#else
    const CoeffScanType scanType  = SCAN_DIAG;
#endif
#if JVET_M0102_INTRA_SUBPARTITIONS
    const ScanElement *scan = g_scanOrder[toChannelType(compID)][SCAN_GROUPED_4x4][scanType][hsId][vsId];
#else
    const ScanElement * scan      = g_scanOrder[SCAN_GROUPED_4x4][scanType][hsId][vsId];
#endif
    const TCoeff*       qCoeff    = tu.getCoeffs( compID ).buf;
          TCoeff*       tCoeff    = recCoeff.buf;

    //----- reset coefficients and get last scan index -----
    ::memset( tCoeff, 0, numCoeff * sizeof(TCoeff) );
    int lastScanIdx = -1;
    for( int scanIdx = numCoeff - 1; scanIdx >= 0; scanIdx-- )
    {
      if (qCoeff[scan[scanIdx].idx])
      {
        lastScanIdx = scanIdx;
        break;
      }
    }
    if( lastScanIdx < 0 )
    {
      return;
    }

    //----- set dequant parameters -----
    const int         qpDQ                  = cQP.Qp + 1;
    const int         qpPer                 = qpDQ / 6;
    const int         qpRem                 = qpDQ - 6 * qpPer;
    const SPS&        sps                   = *tu.cs->sps;
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.getBitDepth( chType );
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange( chType );
    const TCoeff      minTCoeff             = -( 1 << maxLog2TrDynamicRange );
    const TCoeff      maxTCoeff             =  ( 1 << maxLog2TrDynamicRange ) - 1;
    const int         nomTransformShift     = getTransformShift( channelBitDepth, area.size(), maxLog2TrDynamicRange );
#if JVET_M0464_UNI_MTS
    const bool        clipTransformShift    = ( tu.mtsIdx==1 && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() );
#else
    const bool        clipTransformShift    = ( tu.transformSkip[ compID ] && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag() );
#endif
    const int         transformShift        = ( clipTransformShift ? std::max<int>( 0, nomTransformShift ) : nomTransformShift );
#if HM_QTBT_AS_IN_JEM_QUANT
#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
    Intermediate_Int  shift                 = IQUANT_SHIFT + 1 - qpPer - transformShift + ( TU::needsBlockSizeTrafoScale( tu, compID ) ? ADJ_DEQUANT_SHIFT : 0 );
    Intermediate_Int  invQScale             = g_invQuantScales[ qpRem ] * ( TU::needsSqrt2Scale( tu, compID ) ? 181 : 1 );
#else
    Intermediate_Int  shift                 = IQUANT_SHIFT + 1 - qpPer - transformShift + ( TU::needsBlockSizeTrafoScale( area ) ? ADJ_DEQUANT_SHIFT : 0 );
    Intermediate_Int  invQScale             = g_invQuantScales[ qpRem ] * ( TU::needsSqrt2Scale( area ) ? 181 : 1 );
#endif
#else
    Intermediate_Int  shift                 = IQUANT_SHIFT + 1 - qpPer - transformShift;
    Intermediate_Int  invQScale             = g_invQuantScales[ qpRem ];
#endif
    if( shift < 0 )
    {
      invQScale <<= -shift;
      shift       = 0;
    }
    Intermediate_Int  add       = ( 1 << shift ) >> 1;

    //----- dequant coefficients -----
    for( int state = 0, scanIdx = lastScanIdx; scanIdx >= 0; scanIdx-- )
    {
      const unsigned  rasterPos = scan[scanIdx].idx;
      const TCoeff&   level     = qCoeff[ rasterPos ];
      if( level )
      {
        Intermediate_Int  qIdx      = ( level << 1 ) + ( level > 0 ? -(state>>1) : (state>>1) );
        Intermediate_Int  nomTCoeff = ( qIdx * invQScale + add ) >> shift;
        tCoeff[ rasterPos ]         = (TCoeff)Clip3<Intermediate_Int>( minTCoeff, maxTCoeff, nomTCoeff );
      }
      state = ( 32040 >> ((state<<2)+((level&1)<<1)) ) & 3;   // the 16-bit value "32040" represent the state transition table
    }
  }

  inline void Quantizer::preQuantCoeff(const TCoeff absCoeff, PQData *pqData) const
  {
    int64_t scaledOrg = int64_t( absCoeff ) * m_QScale;
    TCoeff  qIdx      = std::max<TCoeff>( 1, std::min<TCoeff>( m_maxQIdx, TCoeff( ( scaledOrg + m_QAdd ) >> m_QShift ) ) );
    int64_t scaledAdd = qIdx * m_DistStepAdd - scaledOrg * m_DistOrgFact;
    PQData& pq_a      = pqData[ qIdx & 3 ];
    pq_a.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_a.absLevel     = ( ++qIdx ) >> 1;
    scaledAdd        += m_DistStepAdd;
    PQData& pq_b      = pqData[ qIdx & 3 ];
    pq_b.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_b.absLevel     = ( ++qIdx ) >> 1;
    scaledAdd        += m_DistStepAdd;
    PQData& pq_c      = pqData[ qIdx & 3 ];
    pq_c.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_c.absLevel     = ( ++qIdx ) >> 1;
    scaledAdd        += m_DistStepAdd;
    PQData& pq_d      = pqData[ qIdx & 3 ];
    pq_d.deltaDist    = ( scaledAdd * qIdx + m_DistAdd ) >> m_DistShift;
    pq_d.absLevel     = ( ++qIdx ) >> 1;
  }







  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q   S T A T E                                                  =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  class State;

  struct SbbCtx
  {
    uint8_t*  sbbFlags;
    uint8_t*  levels;
  };

  class CommonCtx
  {
  public:
    CommonCtx() : m_currSbbCtx( m_allSbbCtx ), m_prevSbbCtx( m_currSbbCtx + 4 ) {}

    inline void swap() { std::swap(m_currSbbCtx, m_prevSbbCtx); }

    inline void reset( const TUParameters& tuPars, const RateEstimator &rateEst)
    {
      m_nbInfo = tuPars.m_scanId2NbInfoOut;
      ::memcpy( m_sbbFlagBits, rateEst.sigSbbFracBits(), 2*sizeof(BinFracBits) );
      const int numSbb    = tuPars.m_numSbb;
      const int chunkSize = numSbb + tuPars.m_numCoeff;
      uint8_t*  nextMem   = m_memory;
      for( int k = 0; k < 8; k++, nextMem += chunkSize )
      {
        m_allSbbCtx[k].sbbFlags = nextMem;
        m_allSbbCtx[k].levels   = nextMem + numSbb;
      }
    }

    inline void update(const ScanInfo &scanInfo, const State *prevState, State &currState);

  private:
    const NbInfoOut*            m_nbInfo;
    BinFracBits                 m_sbbFlagBits[2];
    SbbCtx                      m_allSbbCtx  [8];
    SbbCtx*                     m_currSbbCtx;
    SbbCtx*                     m_prevSbbCtx;
    uint8_t                     m_memory[ 8 * ( MAX_TU_SIZE * MAX_TU_SIZE + MLS_GRP_NUM ) ];
  };

#define RICEMAX 32
#if JVET_M0470
  const int32_t g_goRiceBits[4][RICEMAX] =
  {
      { 32768,	65536,	98304,	131072,	163840,	196608,	262144,	262144,	327680,	327680,	327680,	327680,	393216,	393216,	393216,	393216,	393216,	393216,	393216,	393216,	458752,	458752,	458752,	458752,	458752,	458752,	458752,	458752,	458752,	458752,	458752,	458752},
      { 65536,	65536,	98304,	98304,	131072,	131072,	163840,	163840,	196608,	196608,	229376,	229376,	294912,	294912,	294912,	294912,	360448,	360448,	360448,	360448,	360448,	360448,	360448,	360448,	425984,	425984,	425984,	425984,	425984,	425984,	425984,	425984},
      { 98304,	98304,	98304,	98304,	131072,	131072,	131072,	131072,	163840,	163840,	163840,	163840,	196608,	196608,	196608,	196608,	229376,	229376,	229376,	229376,	262144,	262144,	262144,	262144,	327680,	327680,	327680,	327680,	327680,	327680,	327680,	327680},
      { 131072,	131072,	131072,	131072,	131072,	131072,	131072,	131072,	163840,	163840,	163840,	163840,	163840,	163840,	163840,	163840,	196608,	196608,	196608,	196608,	196608,	196608,	196608,	196608,	229376,	229376,	229376,	229376,	229376,	229376,	229376,	229376}
  };
#else
  const int32_t g_goRiceBits[4][RICEMAX] =
  {
    {  32768,  65536,  98304, 131072, 163840, 196608, 229376, 294912, 294912, 360448, 360448, 360448, 360448, 425984, 425984, 425984, 425984, 425984, 425984, 425984, 425984, 491520, 491520, 491520, 491520, 491520, 491520, 491520, 491520, 491520, 491520, 491520 },
    {  65536,  65536,  98304,  98304, 131072, 131072, 163840, 163840, 196608, 196608, 229376, 229376, 294912, 294912, 294912, 294912, 360448, 360448, 360448, 360448, 360448, 360448, 360448, 360448, 425984, 425984, 425984, 425984, 425984, 425984, 425984, 425984 },
    {  98304,  98304,  98304,  98304, 131072, 131072, 131072, 131072, 163840, 163840, 163840, 163840, 196608, 196608, 196608, 196608, 229376, 229376, 229376, 229376, 262144, 262144, 262144, 262144, 294912, 294912, 294912, 294912, 360448, 360448, 360448, 360448 },
    { 131072, 131072, 131072, 131072, 131072, 131072, 131072, 131072, 163840, 163840, 163840, 163840, 163840, 163840, 163840, 163840, 196608, 196608, 196608, 196608, 196608, 196608, 196608, 196608, 229376, 229376, 229376, 229376, 229376, 229376, 229376, 229376 }
  };
#endif

  class State
  {
    friend class CommonCtx;
  public:
    State( const RateEstimator& rateEst, CommonCtx& commonCtx, const int stateId );

    template<uint8_t numIPos>
    inline void updateState(const ScanInfo &scanInfo, const State *prevStates, const Decision &decision);
    inline void updateStateEOS(const ScanInfo &scanInfo, const State *prevStates, const State *skipStates,
                               const Decision &decision);

    inline void init()
    {
      m_rdCost        = std::numeric_limits<int64_t>::max()>>1;
      m_numSigSbb     = 0;
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
      m_remRegBins    = 4;  // just large enough for last scan pos
#else
      m_remRegBins    = 3;  // just large enough for last scan pos
#endif
      m_refSbbCtxId   = -1;
      m_sigFracBits   = m_sigFracBitsArray[ 0 ];
      m_coeffFracBits = m_gtxFracBitsArray[ 0 ];
      m_goRicePar     = 0;
      m_goRiceZero    = 0;
    }

    void checkRdCosts( const ScanPosType spt, const PQData& pqDataA, const PQData& pqDataB, Decision& decisionA, Decision& decisionB) const
    {
      const int32_t*  goRiceTab = g_goRiceBits[m_goRicePar];
      int64_t         rdCostA   = m_rdCost + pqDataA.deltaDist;
      int64_t         rdCostB   = m_rdCost + pqDataB.deltaDist;
      int64_t         rdCostZ   = m_rdCost;
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
      if( m_remRegBins >= 4 )
#else
      if( m_remRegBins >= 3 )
#endif
      {
        if( pqDataA.absLevel < 4 )
          rdCostA += m_coeffFracBits.bits[pqDataA.absLevel];
        else
        {
          const unsigned value = (pqDataA.absLevel - 4) >> 1;
          rdCostA += m_coeffFracBits.bits[pqDataA.absLevel - (value << 1)] + goRiceTab[value<RICEMAX ? value : RICEMAX-1];
        }
        if( pqDataB.absLevel < 4 )
          rdCostB += m_coeffFracBits.bits[pqDataB.absLevel];
        else
        {
          const unsigned value = (pqDataB.absLevel - 4) >> 1;
          rdCostB += m_coeffFracBits.bits[pqDataB.absLevel - (value << 1)] + goRiceTab[value<RICEMAX ? value : RICEMAX-1];
        }
        if( spt == SCAN_ISCSBB )
        {
          rdCostA += m_sigFracBits.intBits[1];
          rdCostB += m_sigFracBits.intBits[1];
          rdCostZ += m_sigFracBits.intBits[0];
        }
        else if( spt == SCAN_SOCSBB )
        {
          rdCostA += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
          rdCostB += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
          rdCostZ += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[0];
        }
        else if( m_numSigSbb )
        {
          rdCostA += m_sigFracBits.intBits[1];
          rdCostB += m_sigFracBits.intBits[1];
          rdCostZ += m_sigFracBits.intBits[0];
        }
        else
        {
          rdCostZ = decisionA.rdCost;
        }
      }
      else
      {
        rdCostA += (1 << SCALE_BITS) + goRiceTab[pqDataA.absLevel <= m_goRiceZero ? pqDataA.absLevel - 1 : (pqDataA.absLevel<RICEMAX ? pqDataA.absLevel : RICEMAX-1)];
        rdCostB += (1 << SCALE_BITS) + goRiceTab[pqDataB.absLevel <= m_goRiceZero ? pqDataB.absLevel - 1 : (pqDataB.absLevel<RICEMAX ? pqDataB.absLevel : RICEMAX-1)];
        rdCostZ += goRiceTab[m_goRiceZero];
      }
      if( rdCostA < decisionA.rdCost )
      {
        decisionA.rdCost   = rdCostA;
        decisionA.absLevel = pqDataA.absLevel;
        decisionA.prevId   = m_stateId;
      }
      if( rdCostZ < decisionA.rdCost )
      {
        decisionA.rdCost   = rdCostZ;
        decisionA.absLevel = 0;
        decisionA.prevId   = m_stateId;
      }
      if( rdCostB < decisionB.rdCost )
      {
        decisionB.rdCost   = rdCostB;
        decisionB.absLevel = pqDataB.absLevel;
        decisionB.prevId   = m_stateId;
      }
    }

    inline void checkRdCostStart(int32_t lastOffset, const PQData &pqData, Decision &decision) const
    {
      int64_t rdCost = pqData.deltaDist + lastOffset;
      if (pqData.absLevel < 4)
      {
        rdCost += m_coeffFracBits.bits[pqData.absLevel];
      }
      else
      {
        const unsigned value = (pqData.absLevel - 4) >> 1;
        rdCost += m_coeffFracBits.bits[pqData.absLevel - (value << 1)] + g_goRiceBits[m_goRicePar][value < RICEMAX ? value : RICEMAX-1];
      }
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = pqData.absLevel;
        decision.prevId   = -1;
      }
    }

    inline void checkRdCostSkipSbb(Decision &decision) const
    {
      int64_t rdCost = m_rdCost + m_sbbFracBits.intBits[0];
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = 0;
        decision.prevId   = 4+m_stateId;
      }
    }

#if JVET_M0297_32PT_MTS_ZERO_OUT
    inline void checkRdCostSkipSbbZeroOut(Decision &decision) const
    {
      int64_t rdCost = m_rdCost + m_sbbFracBits.intBits[0];
      decision.rdCost = rdCost;
      decision.absLevel = 0;
      decision.prevId = 4 + m_stateId;
    }
#endif

  private:
    int64_t                   m_rdCost;
    uint16_t                  m_absLevelsAndCtxInit[24];  // 16x8bit for abs levels + 16x16bit for ctx init id
    int8_t                    m_numSigSbb;
    int8_t                    m_remRegBins;
    int8_t                    m_refSbbCtxId;
    BinFracBits               m_sbbFracBits;
    BinFracBits               m_sigFracBits;
    CoeffFracBits             m_coeffFracBits;
    int8_t                    m_goRicePar;
    int8_t                    m_goRiceZero;
    const int8_t              m_stateId;
    const BinFracBits*const   m_sigFracBitsArray;
    const CoeffFracBits*const m_gtxFracBitsArray;
    const uint32_t*const      m_goRiceZeroArray;
    CommonCtx&                m_commonCtx;
  };


  State::State( const RateEstimator& rateEst, CommonCtx& commonCtx, const int stateId )
    : m_sbbFracBits     { { 0, 0 } }
    , m_stateId         ( stateId )
    , m_sigFracBitsArray( rateEst.sigFlagBits(stateId) )
    , m_gtxFracBitsArray( rateEst.gtxFracBits(stateId) )
    , m_goRiceZeroArray ( g_auiGoRicePosCoeff0[std::max(0,stateId-1)] )
    , m_commonCtx       ( commonCtx )
  {
  }

  template<uint8_t numIPos>
  inline void State::updateState(const ScanInfo &scanInfo, const State *prevStates, const Decision &decision)
  {
    m_rdCost = decision.rdCost;
    if( decision.prevId > -2 )
    {
      if( decision.prevId >= 0 )
      {
        const State*  prvState  = prevStates            +   decision.prevId;
        m_numSigSbb             = prvState->m_numSigSbb + !!decision.absLevel;
        m_refSbbCtxId           = prvState->m_refSbbCtxId;
        m_sbbFracBits           = prvState->m_sbbFracBits;
        m_remRegBins            = prvState->m_remRegBins - 1;
        m_goRicePar             = prvState->m_goRicePar;
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
        if( m_remRegBins >= 4 )
#else
        if( m_remRegBins >= 3 )
#endif
        {
          TCoeff rem = (decision.absLevel - 4) >> 1;
          if( m_goRicePar < 3 && rem > (3<<m_goRicePar)-1 )
          {
            m_goRicePar++;
          }
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
          m_remRegBins -= (decision.absLevel < 2 ? decision.absLevel : 3);
#else
          m_remRegBins -= std::min<TCoeff>( decision.absLevel, 2 );
#endif
        }
        ::memcpy( m_absLevelsAndCtxInit, prvState->m_absLevelsAndCtxInit, 48*sizeof(uint8_t) );
      }
      else
      {
        m_numSigSbb     =  1;
        m_refSbbCtxId   = -1;
        if ( scanInfo.sbbSize == 4 )
        {
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
          m_remRegBins = MAX_NUM_REG_BINS_2x2SUBBLOCK - (decision.absLevel < 2 ? decision.absLevel : 3);
#else
          m_remRegBins  = MAX_NUM_REG_BINS_2x2SUBBLOCK - MAX_NUM_GT2_BINS_2x2SUBBLOCK - std::min<TCoeff>( decision.absLevel, 2 );
#endif
        }
        else
        {
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
          m_remRegBins = MAX_NUM_REG_BINS_4x4SUBBLOCK - (decision.absLevel < 2 ? decision.absLevel : 3);
#else
          m_remRegBins  = MAX_NUM_REG_BINS_4x4SUBBLOCK - MAX_NUM_GT2_BINS_4x4SUBBLOCK - std::min<TCoeff>( decision.absLevel, 2 );
#endif
        }
        m_goRicePar     = ( ((decision.absLevel - 4) >> 1) > (3<<0)-1 ? 1 : 0 );
        ::memset( m_absLevelsAndCtxInit, 0, 48*sizeof(uint8_t) );
      }

      uint8_t* levels               = reinterpret_cast<uint8_t*>(m_absLevelsAndCtxInit);
      levels[ scanInfo.insidePos ]  = (uint8_t)std::min<TCoeff>( 255, decision.absLevel );

#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
      if (m_remRegBins >= 4)
#else
      if (m_remRegBins >= 3)
#endif
      {
        TCoeff  tinit = m_absLevelsAndCtxInit[8 + scanInfo.nextInsidePos];
        TCoeff  sumAbs1 = (tinit >> 3) & 31;
        TCoeff  sumNum = tinit & 7;
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
#define UPDATE(k) {TCoeff t=levels[scanInfo.nextNbInfoSbb.inPos[k]]; sumAbs1+=std::min<TCoeff>(4+(t&1),t); sumNum+=!!t; }
#else
#define UPDATE(k) {TCoeff t=levels[scanInfo.nextNbInfoSbb.inPos[k]]; sumAbs1+=std::min<TCoeff>(2+(t&1),t); sumNum+=!!t; }
#endif
        if (numIPos == 1)
        {
          UPDATE(0);
        }
        else if (numIPos == 2)
        {
          UPDATE(0);
          UPDATE(1);
        }
        else if (numIPos == 3)
        {
          UPDATE(0);
          UPDATE(1);
          UPDATE(2);
        }
        else if (numIPos == 4)
        {
          UPDATE(0);
          UPDATE(1);
          UPDATE(2);
          UPDATE(3);
        }
        else if (numIPos == 5)
        {
          UPDATE(0);
          UPDATE(1);
          UPDATE(2);
          UPDATE(3);
          UPDATE(4);
        }
#undef UPDATE
        TCoeff sumGt1 = sumAbs1 - sumNum;
        m_sigFracBits = m_sigFracBitsArray[scanInfo.sigCtxOffsetNext + (sumAbs1 < 5 ? sumAbs1 : 5)];
        m_coeffFracBits = m_gtxFracBitsArray[scanInfo.gtxCtxOffsetNext + (sumGt1 < 4 ? sumGt1 : 4)];
      }
      else
      {
        TCoeff  sumAbs = m_absLevelsAndCtxInit[8 + scanInfo.nextInsidePos] >> 8;
#define UPDATE(k) {TCoeff t=levels[scanInfo.nextNbInfoSbb.inPos[k]]; sumAbs+=t; }
        if (numIPos == 1)
        {
          UPDATE(0);
        }
        else if (numIPos == 2)
        {
          UPDATE(0);
          UPDATE(1);
        }
        else if (numIPos == 3)
        {
          UPDATE(0);
          UPDATE(1);
          UPDATE(2);
        }
        else if (numIPos == 4)
        {
          UPDATE(0);
          UPDATE(1);
          UPDATE(2);
          UPDATE(3);
        }
        else if (numIPos == 5)
        {
          UPDATE(0);
          UPDATE(1);
          UPDATE(2);
          UPDATE(3);
          UPDATE(4);
        }
#undef UPDATE
        sumAbs = std::min(31, sumAbs);
        m_goRicePar = g_auiGoRiceParsCoeff[sumAbs];
        m_goRiceZero = m_goRiceZeroArray[sumAbs];
      }
    }
  }

  inline void State::updateStateEOS(const ScanInfo &scanInfo, const State *prevStates, const State *skipStates,
                                    const Decision &decision)
  {
    m_rdCost = decision.rdCost;
    if( decision.prevId > -2 )
    {
      const State* prvState = 0;
      if( decision.prevId  >= 4 )
      {
        VTMCHECK( decision.absLevel != 0, "cannot happen" );
        prvState    = skipStates + ( decision.prevId - 4 );
        m_numSigSbb = 0;
        ::memset( m_absLevelsAndCtxInit, 0, 16*sizeof(uint8_t) );
      }
      else if( decision.prevId  >= 0 )
      {
        prvState    = prevStates            +   decision.prevId;
        m_numSigSbb = prvState->m_numSigSbb + !!decision.absLevel;
        ::memcpy( m_absLevelsAndCtxInit, prvState->m_absLevelsAndCtxInit, 16*sizeof(uint8_t) );
      }
      else
      {
        m_numSigSbb = 1;
        ::memset( m_absLevelsAndCtxInit, 0, 16*sizeof(uint8_t) );
      }
      reinterpret_cast<uint8_t*>(m_absLevelsAndCtxInit)[ scanInfo.insidePos ] = (uint8_t)std::min<TCoeff>( 255, decision.absLevel );

      m_commonCtx.update( scanInfo, prvState, *this );

      TCoeff  tinit   = m_absLevelsAndCtxInit[ 8 + scanInfo.nextInsidePos ];
      TCoeff  sumNum  =   tinit        & 7;
      TCoeff  sumAbs1 = ( tinit >> 3 ) & 31;
      TCoeff  sumGt1  = sumAbs1        - sumNum;
      m_sigFracBits   = m_sigFracBitsArray[ scanInfo.sigCtxOffsetNext + ( sumAbs1 < 5 ? sumAbs1 : 5 ) ];
      m_coeffFracBits = m_gtxFracBitsArray[ scanInfo.gtxCtxOffsetNext + ( sumGt1  < 4 ? sumGt1  : 4 ) ];
    }
  }

  inline void CommonCtx::update(const ScanInfo &scanInfo, const State *prevState, State &currState)
  {
    uint8_t*    sbbFlags  = m_currSbbCtx[ currState.m_stateId ].sbbFlags;
    uint8_t*    levels    = m_currSbbCtx[ currState.m_stateId ].levels;
    std::size_t setCpSize = m_nbInfo[ scanInfo.scanIdx - 1 ].maxDist * sizeof(uint8_t);
    if( prevState && prevState->m_refSbbCtxId >= 0 )
    {
      ::memcpy( sbbFlags,                  m_prevSbbCtx[prevState->m_refSbbCtxId].sbbFlags,                  scanInfo.numSbb*sizeof(uint8_t) );
      ::memcpy( levels + scanInfo.scanIdx, m_prevSbbCtx[prevState->m_refSbbCtxId].levels + scanInfo.scanIdx, setCpSize );
    }
    else
    {
      ::memset( sbbFlags,                  0, scanInfo.numSbb*sizeof(uint8_t) );
      ::memset( levels + scanInfo.scanIdx, 0, setCpSize );
    }
    sbbFlags[ scanInfo.sbbPos ] = !!currState.m_numSigSbb;
    ::memcpy( levels + scanInfo.scanIdx, currState.m_absLevelsAndCtxInit, scanInfo.sbbSize*sizeof(uint8_t) );

    const int       sigNSbb   = ( ( scanInfo.nextSbbRight ? sbbFlags[ scanInfo.nextSbbRight ] : false ) || ( scanInfo.nextSbbBelow ? sbbFlags[ scanInfo.nextSbbBelow ] : false ) ? 1 : 0 );
    currState.m_numSigSbb     = 0;
    if (scanInfo.sbbSize == 4)
    {
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
      currState.m_remRegBins  = MAX_NUM_REG_BINS_2x2SUBBLOCK;
#else
      currState.m_remRegBins  = MAX_NUM_REG_BINS_2x2SUBBLOCK - MAX_NUM_GT2_BINS_2x2SUBBLOCK;
#endif
    }
    else
    {
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
      currState.m_remRegBins  = MAX_NUM_REG_BINS_4x4SUBBLOCK;
#else
      currState.m_remRegBins  = MAX_NUM_REG_BINS_4x4SUBBLOCK - MAX_NUM_GT2_BINS_4x4SUBBLOCK;
#endif
    }
    currState.m_goRicePar     = 0;
    currState.m_refSbbCtxId   = currState.m_stateId;
    currState.m_sbbFracBits   = m_sbbFlagBits[ sigNSbb ];

    uint16_t          templateCtxInit[16];
    const int         scanBeg   = scanInfo.scanIdx - scanInfo.sbbSize;
    const NbInfoOut*  nbOut     = m_nbInfo + scanBeg;
    const uint8_t*    absLevels = levels   + scanBeg;
    for( int id = 0; id < scanInfo.sbbSize; id++, nbOut++ )
    {
      if( nbOut->num )
      {
        TCoeff sumAbs = 0, sumAbs1 = 0, sumNum = 0;
#if JVET_M0173_MOVE_GT2_TO_FIRST_PASS
#define UPDATE(k) {TCoeff t=absLevels[nbOut->outPos[k]]; sumAbs+=t; sumAbs1+=std::min<TCoeff>(4+(t&1),t); sumNum+=!!t; }
#else
#define UPDATE(k) {TCoeff t=absLevels[nbOut->outPos[k]]; sumAbs+=t; sumAbs1+=std::min<TCoeff>(2+(t&1),t); sumNum+=!!t; }
#endif
        UPDATE(0);
        if( nbOut->num > 1 )
        {
          UPDATE(1);
          if( nbOut->num > 2 )
          {
            UPDATE(2);
            if( nbOut->num > 3 )
            {
              UPDATE(3);
              if( nbOut->num > 4 )
              {
                UPDATE(4);
              }
            }
          }
        }
#undef UPDATE
        templateCtxInit[id] = uint16_t(sumNum) + ( uint16_t(sumAbs1) << 3 ) + ( (uint16_t)std::min<TCoeff>( 127, sumAbs ) << 8 );
      }
      else
      {
        templateCtxInit[id] = 0;
      }
    }
    ::memset( currState.m_absLevelsAndCtxInit,     0,               16*sizeof(uint8_t) );
    ::memcpy( currState.m_absLevelsAndCtxInit + 8, templateCtxInit, 16*sizeof(uint16_t) );
  }



  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q                                                              =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/
  class DepQuant : private RateEstimator
  {
  public:
    DepQuant();

    void    quant   ( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum );
    void    dequant ( const TransformUnit& tu,  CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP )  const;

  private:
#if JVET_M0297_32PT_MTS_ZERO_OUT
    void    xDecideAndUpdate  ( const TCoeff absCoeff, const ScanInfo& scanInfo, bool zeroOut );
    void    xDecide           ( const ScanPosType spt, const TCoeff absCoeff, const int lastOffset, Decision* decisions, bool zeroOut );
#else
    void    xDecideAndUpdate  ( const TCoeff absCoeff, const ScanInfo& scanInfo );
    void    xDecide           ( const ScanPosType spt, const TCoeff absCoeff, const int lastOffset, Decision* decisions );
#endif

  private:
    CommonCtx   m_commonCtx;
    State       m_allStates[ 12 ];
    State*      m_currStates;
    State*      m_prevStates;
    State*      m_skipStates;
    State       m_startState;
    Quantizer   m_quant;
    Decision    m_trellis[ MAX_TU_SIZE * MAX_TU_SIZE ][ 8 ];
  };


#define TINIT(x) {*this,m_commonCtx,x}
  DepQuant::DepQuant()
    : RateEstimator ()
    , m_commonCtx   ()
    , m_allStates   {TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3)}
    , m_currStates  (  m_allStates      )
    , m_prevStates  (  m_currStates + 4 )
    , m_skipStates  (  m_prevStates + 4 )
    , m_startState  TINIT(0)
  {}
#undef TINIT


  void DepQuant::dequant( const TransformUnit& tu,  CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP ) const
  {
    m_quant.dequantBlock( tu, compID, cQP, recCoeff );
  }


#define DINIT(l,p) {std::numeric_limits<int64_t>::max()>>2,l,p}
  static const Decision startDec[8] = {DINIT(-1,-2),DINIT(-1,-2),DINIT(-1,-2),DINIT(-1,-2),DINIT(0,4),DINIT(0,5),DINIT(0,6),DINIT(0,7)};
#undef  DINIT


#if JVET_M0297_32PT_MTS_ZERO_OUT
  void DepQuant::xDecide( const ScanPosType spt, const TCoeff absCoeff, const int lastOffset, Decision* decisions, bool zeroOut)
#else
  void DepQuant::xDecide( const ScanPosType spt, const TCoeff absCoeff, const int lastOffset, Decision* decisions)
#endif
  {
    ::memcpy( decisions, startDec, 8*sizeof(Decision) );

#if JVET_M0297_32PT_MTS_ZERO_OUT
    if( zeroOut )
    {
      if( spt==SCAN_EOCSBB )
      {
        m_skipStates[0].checkRdCostSkipSbbZeroOut( decisions[0] );
        m_skipStates[1].checkRdCostSkipSbbZeroOut( decisions[1] );
        m_skipStates[2].checkRdCostSkipSbbZeroOut( decisions[2] );
        m_skipStates[3].checkRdCostSkipSbbZeroOut( decisions[3] );
      }
      return;
    }
#endif

    PQData  pqData[4];
    m_quant.preQuantCoeff( absCoeff, pqData );
    m_prevStates[0].checkRdCosts( spt, pqData[0], pqData[2], decisions[0], decisions[2]);
    m_prevStates[1].checkRdCosts( spt, pqData[0], pqData[2], decisions[2], decisions[0]);
    m_prevStates[2].checkRdCosts( spt, pqData[3], pqData[1], decisions[1], decisions[3]);
    m_prevStates[3].checkRdCosts( spt, pqData[3], pqData[1], decisions[3], decisions[1]);
    if( spt==SCAN_EOCSBB )
    {
      m_skipStates[0].checkRdCostSkipSbb( decisions[0] );
      m_skipStates[1].checkRdCostSkipSbb( decisions[1] );
      m_skipStates[2].checkRdCostSkipSbb( decisions[2] );
      m_skipStates[3].checkRdCostSkipSbb( decisions[3] );
    }
    m_startState.checkRdCostStart( lastOffset, pqData[0], decisions[0] );
    m_startState.checkRdCostStart( lastOffset, pqData[2], decisions[2] );
  }

#if JVET_M0297_32PT_MTS_ZERO_OUT
  void DepQuant::xDecideAndUpdate( const TCoeff absCoeff, const ScanInfo& scanInfo, bool zeroOut )
#else
  void DepQuant::xDecideAndUpdate( const TCoeff absCoeff, const ScanInfo& scanInfo )
#endif
  {
    Decision* decisions = m_trellis[ scanInfo.scanIdx ];

    std::swap( m_prevStates, m_currStates );

#if JVET_M0297_32PT_MTS_ZERO_OUT
    xDecide( scanInfo.spt, absCoeff, lastOffset(scanInfo.scanIdx), decisions, zeroOut );
#else
    xDecide( scanInfo.spt, absCoeff, lastOffset(scanInfo.scanIdx), decisions);
#endif

    if( scanInfo.scanIdx )
    {
      if( scanInfo.eosbb )
      {
        m_commonCtx.swap();
        m_currStates[0].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[0] );
        m_currStates[1].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[1] );
        m_currStates[2].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[2] );
        m_currStates[3].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[3] );
        ::memcpy( decisions+4, decisions, 4*sizeof(Decision) );
      }
#if JVET_M0297_32PT_MTS_ZERO_OUT
      else if( !zeroOut )
#else
      else
#endif
      {
        switch( scanInfo.nextNbInfoSbb.num )
        {
        case 0:
          m_currStates[0].updateState<0>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<0>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<0>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<0>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 1:
          m_currStates[0].updateState<1>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<1>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<1>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<1>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 2:
          m_currStates[0].updateState<2>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<2>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<2>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<2>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 3:
          m_currStates[0].updateState<3>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<3>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<3>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<3>( scanInfo, m_prevStates, decisions[3] );
          break;
        case 4:
          m_currStates[0].updateState<4>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<4>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<4>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<4>( scanInfo, m_prevStates, decisions[3] );
          break;
        default:
          m_currStates[0].updateState<5>( scanInfo, m_prevStates, decisions[0] );
          m_currStates[1].updateState<5>( scanInfo, m_prevStates, decisions[1] );
          m_currStates[2].updateState<5>( scanInfo, m_prevStates, decisions[2] );
          m_currStates[3].updateState<5>( scanInfo, m_prevStates, decisions[3] );
        }
      }

      if( scanInfo.spt == SCAN_SOCSBB )
      {
        std::swap( m_prevStates, m_skipStates );
      }
    }
  }


  void DepQuant::quant( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum )
  {
    CHECKD( tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag(), "ext precision is not supported" );

    //===== reset / pre-init =====
    const TUParameters& tuPars  = *g_Rom.getTUPars( tu.blocks[compID], compID );
    m_quant.initQuantBlock    ( tu, compID, cQP, lambda );
    TCoeff*       qCoeff      = tu.getCoeffs( compID ).buf;
    const TCoeff* tCoeff      = srcCoeff.buf;
    const int     numCoeff    = tu.blocks[compID].area();
    ::memset( tu.getCoeffs( compID ).buf, 0x00, numCoeff*sizeof(TCoeff) );
    absSum          = 0;

    //===== find first test position =====
    int   firstTestPos = numCoeff - 1;
    const TCoeff thres = m_quant.getLastThreshold();
    for( ; firstTestPos >= 0; firstTestPos-- )
    {
      if (abs(tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx]) > thres)
      {
        break;
      }
    }
    if( firstTestPos < 0 )
    {
      return;
    }

    //===== real init =====
    RateEstimator::initCtx( tuPars, tu, compID, ctx.getFracBitsAcess() );
    m_commonCtx.reset( tuPars, *this );
    for( int k = 0; k < 12; k++ )
    {
      m_allStates[k].init();
    }
    m_startState.init();

#if JVET_M0297_32PT_MTS_ZERO_OUT
    int effWidth = tuPars.m_width, effHeight = tuPars.m_height;
    bool zeroOut = false;
#if JVET_M0140_SBT
#if JVET_M0464_UNI_MTS
    if( ( tu.mtsIdx > 1 || ( tu.cu->sbtInfo != 0 && tuPars.m_height <= 32 && tuPars.m_width <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
    if( ( ( tu.cu->emtFlag && !tu.transformSkip[ compID ] ) || ( tu.cu->sbtInfo != 0 && tuPars.m_height <= 32 && tuPars.m_width <= 32 ) ) && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
#else
#if JVET_M0464_UNI_MTS
    if( tu.mtsIdx > 1 && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#else
    if( tu.cu->emtFlag && !tu.transformSkip[compID] && !tu.cu->transQuantBypass && compID == COMPONENT_Y )
#endif
#endif
    {
      effHeight = ( tuPars.m_height == 32 ) ? 16 : tuPars.m_height;
      effWidth = ( tuPars.m_width == 32 ) ? 16 : tuPars.m_width;
      zeroOut  = ( effHeight < tuPars.m_height || effWidth < tuPars.m_width );
    }
#endif

    //===== populate trellis =====
    for( int scanIdx = firstTestPos; scanIdx >= 0; scanIdx-- )
    {
      const ScanInfo& scanInfo = tuPars.m_scanInfo[ scanIdx ];
#if JVET_M0297_32PT_MTS_ZERO_OUT
      xDecideAndUpdate( abs( tCoeff[ scanInfo.rasterPos ] ), scanInfo, zeroOut && ( scanInfo.posX >= effWidth || scanInfo.posY >= effHeight ) );
#else
      xDecideAndUpdate( abs( tCoeff[ scanInfo.rasterPos ] ), scanInfo );
#endif
    }

    //===== find best path =====
    Decision  decision    = { std::numeric_limits<int64_t>::max(), -1, -2 };
    int64_t   minPathCost =  0;
    for( int8_t stateId = 0; stateId < 4; stateId++ )
    {
      int64_t pathCost = m_trellis[0][stateId].rdCost;
      if( pathCost < minPathCost )
      {
        decision.prevId = stateId;
        minPathCost     = pathCost;
      }
    }

    //===== backward scanning =====
    int scanIdx = 0;
    for( ; decision.prevId >= 0; scanIdx++ )
    {
      decision          = m_trellis[ scanIdx ][ decision.prevId ];
      int32_t blkpos    = tuPars.m_scanId2BlkPos[scanIdx].idx;
      qCoeff[ blkpos ]  = ( tCoeff[ blkpos ] < 0 ? -decision.absLevel : decision.absLevel );
      absSum           += decision.absLevel;
    }
  }

}; // namespace DQIntern




//===== interface class =====
DepQuant::DepQuant( const Quant* other, bool enc ) : QuantRDOQ( other )
{
  const DepQuant* dq = dynamic_cast<const DepQuant*>( other );
  VTMCHECK( other && !dq, "The DepQuant cast must be successfull!" );
  p = new DQIntern::DepQuant();
  if( enc )
  {
    DQIntern::g_Rom.init();
  }
}

DepQuant::~DepQuant()
{
  delete static_cast<DQIntern::DepQuant*>(p);
}

void DepQuant::quant( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx )
{
  if( tu.cs->slice->getDepQuantEnabledFlag() )
  {
    static_cast<DQIntern::DepQuant*>(p)->quant( tu, pSrc, compID, cQP, Quant::m_dLambda, ctx, uiAbsSum );
  }
  else
  {
    QuantRDOQ::quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
  }
}

void DepQuant::dequant( const TransformUnit &tu, CoeffBuf &dstCoeff, const ComponentID &compID, const QpParam &cQP )
{
  if( tu.cs->slice->getDepQuantEnabledFlag() )
  {
    static_cast<DQIntern::DepQuant*>(p)->dequant( tu, dstCoeff, compID, cQP );
  }
  else
  {
    QuantRDOQ::dequant( tu, dstCoeff, compID, cQP );
  }
}



