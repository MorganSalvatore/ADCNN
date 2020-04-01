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

/** \file     VLCWriter.h
 *  \brief    Writer for high level syntax
 */

#ifndef __VLCWRITER__
#define __VLCWRITER__

#include "CommonLib/CommonDef.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Slice.h"
#include "CABACWriter.h"

//! \ingroup EncoderLib
//! \{

#if ENABLE_TRACING

#define WRITE_CODE( value, length, name)    xWriteCodeTr ( value, length, name )
#define WRITE_UVLC( value,         name)    xWriteUvlcTr ( value,         name )
#define WRITE_SVLC( value,         name)    xWriteSvlcTr ( value,         name )
#define WRITE_FLAG( value,         name)    xWriteFlagTr ( value,         name )

extern bool g_HLSTraceEnable;
#else

#define WRITE_CODE( value, length, name)     xWriteCode ( value, length )
#define WRITE_UVLC( value,         name)     xWriteUvlc ( value )
#define WRITE_SVLC( value,         name)     xWriteSvlc ( value )
#define WRITE_FLAG( value,         name)     xWriteFlag ( value )

#endif



class VLCWriter
{
protected:

  OutputBitstream*    m_pcBitIf;

  VLCWriter() : m_pcBitIf(NULL) {}
  virtual ~VLCWriter() {}

  void  setBitstream          ( OutputBitstream* p )  { m_pcBitIf = p;  }

  void  xWriteCode            ( uint32_t uiCode, uint32_t uiLength );
  void  xWriteUvlc            ( uint32_t uiCode );
  void  xWriteSvlc            ( int  iCode   );
  void  xWriteFlag            ( uint32_t uiCode );
#if ENABLE_TRACING
  void  xWriteCodeTr          ( uint32_t value, uint32_t  length, const char *pSymbolName);
  void  xWriteUvlcTr          ( uint32_t value,               const char *pSymbolName);
  void  xWriteSvlcTr          ( int  value,               const char *pSymbolName);
  void  xWriteFlagTr          ( uint32_t value,               const char *pSymbolName);
#endif
  void  xWriteRbspTrailingBits();
};



class AUDWriter : public VLCWriter
{
public:
  AUDWriter() {};
  virtual ~AUDWriter() {};

  void  codeAUD(OutputBitstream& bs, const int pictureType);
};



class HLSWriter : public VLCWriter
{
public:
  HLSWriter() {}
  virtual ~HLSWriter() {}

private:
  void xCodeShortTermRefPicSet  ( const ReferencePictureSet* pcRPS, bool calledFromSliceHeader, int idx );
  bool xFindMatchingLTRP        ( Slice* pcSlice, uint32_t *ltrpsIndex, int ltrpPOC, bool usedFlag );
  void xCodePredWeightTable     ( Slice* pcSlice );
#if HEVC_USE_SCALING_LISTS
  void xCodeScalingList         ( const ScalingList* scalingList, uint32_t sizeId, uint32_t listId);
#endif
public:
  void  setBitstream            ( OutputBitstream* p )  { m_pcBitIf = p;  }
  uint32_t  getNumberOfWrittenBits  ()                      { return m_pcBitIf->getNumberOfWrittenBits();  }
  void  codeVUI                 ( const VUI *pcVUI, const SPS* pcSPS );
  void  codeSPS                 ( const SPS* pcSPS );
  void  codePPS                 ( const PPS* pcPPS );
#if HEVC_VPS
  void  codeVPS                 ( const VPS* pcVPS );
#endif
  void  codeSliceHeader         ( Slice* pcSlice );
  void  codePTL                 ( const PTL* pcPTL, bool profilePresentFlag, int maxNumSubLayersMinus1);
  void  codeProfileTier         ( const ProfileTierLevel* ptl, const bool bIsSubLayer );
  void  codeHrdParameters       ( const HRD *hrd, bool commonInfPresentFlag, uint32_t maxNumSubLayersMinus1 );
#if HEVC_TILES_WPP
  void  codeTilesWPPEntryPoint  ( Slice* pSlice );
#endif
#if HEVC_USE_SCALING_LISTS
  void  codeScalingList         ( const ScalingList &scalingList );
#endif

  void alf( const AlfSliceParam& alfSliceParam );
  void alfFilter( const AlfSliceParam& alfSliceParam, const bool isChroma );

#if ADCNN
  void cnnlf(const CnnlfSliceParam& cnnlfSliceParam);
#endif

private:
  void xWriteTruncBinCode( uint32_t uiSymbol, const int uiMaxSymbol );
  void alfGolombEncode( const int coeff, const int k );
  void truncatedUnaryEqProb( int symbol, int maxSymbol );

#if JVET_M0427_INLOOP_RESHAPER
  void  codeReshaper            ( const SliceReshapeInfo& pSliceReshaperInfo, const SPS* pcSPS, const bool isIntra);
#endif
};

//! \}

#endif
