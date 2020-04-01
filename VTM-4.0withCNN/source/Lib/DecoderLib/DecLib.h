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

/** \file     DecLib.h
    \brief    decoder class (header)
*/

#ifndef __DECLIB__
#define __DECLIB__

#include "DecSlice.h"
#include "CABACReader.h"
#include "VLCReader.h"
#include "SEIread.h"
#include "CacheModel.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Unit.h"
#if JVET_M0427_INLOOP_RESHAPER
#include "CommonLib/Reshape.h"
#endif
#if WMZ_CNNLF
#include "CommonLib/CNNLoopFilter.h"
#endif

class InputNALUnit;

//! \ingroup DecoderLib
//! \{

#if JVET_M0055_DEBUG_CTU
bool tryDecodePicture( Picture* pcPic, const int expectedPoc, const std::string& bitstreamFileName, bool bDecodeUntilPocFound = false, int debugCTU = -1, int debugPOC = -1 );
#else
 bool tryDecodePicture( Picture* pcPic, const int expectedPoc, const std::string& bitstreamFileName, bool bDecodeUntilPocFound = false );
#endif// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// decoder class
class DecLib
{
private:
  int                     m_iMaxRefPicNum;

  NalUnitType             m_associatedIRAPType; ///< NAL unit type of the associated IRAP picture
  int                     m_pocCRA;            ///< POC number of the latest CRA picture
  int                     m_pocRandomAccess;   ///< POC number of the random access point (the first IDR or CRA picture)
  int                     m_lastRasPoc;

  PicList                 m_cListPic;         //  Dynamic buffer
  ParameterSetManager     m_parameterSetManager;  // storage for parameter sets
  Slice*                  m_apcSlicePilot;


  SEIMessages             m_SEIs; ///< List of SEI messages that have been received before the first slice and between slices, excluding prefix SEIs...

  // functional classes
  IntraPrediction         m_cIntraPred;
  InterPrediction         m_cInterPred;
  TrQuant                 m_cTrQuant;
  DecSlice                m_cSliceDecoder;
  DecCu                   m_cCuDecoder;
  HLSyntaxReader          m_HLSReader;
  CABACDecoder            m_CABACDecoder;
  SEIReader               m_seiReader;
  LoopFilter              m_cLoopFilter;
  SampleAdaptiveOffset    m_cSAO;
  AdaptiveLoopFilter      m_cALF;
#if WMZ_CNNLF
  CNNLoopFilter           m_cCNNLoopFilter;
#endif
#if JVET_M0427_INLOOP_RESHAPER
  Reshape                 m_cReshaper;                        ///< reshaper class
#endif
  // decoder side RD cost computation
  RdCost                  m_cRdCost;                      ///< RD cost computation class
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel              m_cacheModel;
#endif

  bool isSkipPictureForBLA(int& iPOCLastDisplay);
  bool isRandomAccessSkipPicture(int& iSkipFrame,  int& iPOCLastDisplay);
  Picture*                m_pcPic;
  uint32_t                    m_uiSliceSegmentIdx;
  int                     m_prevPOC;
  int                     m_prevTid0POC;
  bool                    m_bFirstSliceInPicture;
  bool                    m_bFirstSliceInSequence;
  bool                    m_prevSliceSkipped;
  int                     m_skippedPOC;
  bool                    m_bFirstSliceInBitstream;
  int                     m_lastPOCNoOutputPriorPics;
  bool                    m_isNoOutputPriorPics;
  bool                    m_craNoRaslOutputFlag;    //value of variable NoRaslOutputFlag of the last CRA pic
  std::ostream           *m_pDecodedSEIOutputStream;

  int                     m_decodedPictureHashSEIEnabled;  ///< Checksum(3)/CRC(2)/MD5(1)/disable(0) acting on decoded picture hash SEI message
  uint32_t                    m_numberOfChecksumErrorsDetected;

  bool                    m_warningMessageSkipPicture;

  std::list<InputNALUnit*> m_prefixSEINALUs; /// Buffered up prefix SEI NAL Units.
#if WMZ_CNNLF
  std::string		m_pbpath;
  std::string		m_sWorkingMode;
  std::string		m_iGPUid;
#endif
#if JVET_M0055_DEBUG_CTU
  int                     m_debugPOC;
  int                     m_debugCTU;
#endif
public:
  DecLib();
  virtual ~DecLib();

  void  create  ();
  void  destroy ();

  void  setDecodedPictureHashSEIEnabled(int enabled) { m_decodedPictureHashSEIEnabled=enabled; }

#if WMZ_CNNLF
  void					setpbpath(std::string a)		{ m_pbpath = a; }
  std::string			getpbpath()						const { return m_pbpath; }
  void					setworkingmode(std::string a)	{ m_sWorkingMode = a; }
  std::string			getworkingmode()				const { return m_sWorkingMode; }
  void					setgpuid(std::string a)			{ m_iGPUid = a; }
  std::string			getgpuid()						const { return m_iGPUid; }

#endif

  void  init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
    const std::string& cacheCfgFileName
#endif
  );
  bool  decode(InputNALUnit& nalu, int& iSkipFrame, int& iPOCLastDisplay);
  void  deletePicBuffer();

  void  executeLoopFilters();
  void  finishPicture(int& poc, PicList*& rpcListPic, MsgLevel msgl = VTMINFO);
  void  finishPictureLight(int& poc, PicList*& rpcListPic );
  void  checkNoOutputPriorPics (PicList* rpcListPic);

  bool  getNoOutputPriorPicsFlag () const   { return m_isNoOutputPriorPics; }
  void  setNoOutputPriorPicsFlag (bool val) { m_isNoOutputPriorPics = val; }
  void  setFirstSliceInPicture (bool val)  { m_bFirstSliceInPicture = val; }
  bool  getFirstSliceInSequence () const   { return m_bFirstSliceInSequence; }
  void  setFirstSliceInSequence (bool val) { m_bFirstSliceInSequence = val; }
  void  setDecodedSEIMessageOutputStream(std::ostream *pOpStream) { m_pDecodedSEIOutputStream = pOpStream; }
  uint32_t  getNumberOfChecksumErrorsDetected() const { return m_numberOfChecksumErrorsDetected; }

#if JVET_M0055_DEBUG_CTU
  int  getDebugCTU( )               const { return m_debugCTU; }
  void setDebugCTU( int debugCTU )        { m_debugCTU = debugCTU; }
  int  getDebugPOC( )               const { return m_debugPOC; };
  void setDebugPOC( int debugPOC )        { m_debugPOC = debugPOC; };
#endif
protected:
  void  xUpdateRasInit(Slice* slice);

  Picture * xGetNewPicBuffer(const SPS &sps, const PPS &pps, const uint32_t temporalLayer);
  void  xCreateLostPicture (int iLostPOC);

  void      xActivateParameterSets();
  bool      xDecodeSlice(InputNALUnit &nalu, int &iSkipFrame, int iPOCLastDisplay);
#if HEVC_VPS
  void      xDecodeVPS( InputNALUnit& nalu );
#endif
  void      xDecodeSPS( InputNALUnit& nalu );
  void      xDecodePPS( InputNALUnit& nalu );
  void      xUpdatePreviousTid0POC( Slice *pSlice ) { if ((pSlice->getTLayer()==0) && (pSlice->isReferenceNalu() && (pSlice->getNalUnitType()!=NAL_UNIT_CODED_SLICE_RASL_R)&& (pSlice->getNalUnitType()!=NAL_UNIT_CODED_SLICE_RADL_R))) { m_prevTid0POC=pSlice->getPOC(); } }
  void      xParsePrefixSEImessages();
  void      xParsePrefixSEIsForUnknownVCLNal();

};// END CLASS DEFINITION DecLib


//! \}

#endif // __DECTOP__

