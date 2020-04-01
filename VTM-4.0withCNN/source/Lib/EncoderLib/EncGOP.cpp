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

/** \file     EncGOP.cpp
    \brief    GOP encoder class
*/

#include <list>
#include <algorithm>
#include <functional>

#include "EncLib.h"
#include "EncGOP.h"
#include "Analyze.h"
#include "libmd5/MD5.h"
#include "CommonLib/SEI.h"
#include "CommonLib/NAL.h"
#include "NALwrite.h"

#include <math.h>
#include <deque>
#include <chrono>
#include <cinttypes>

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"

#include "DecoderLib/DecLib.h"

#define ENCODE_SUB_SET 0

using namespace std;

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
int getLSB(int poc, int maxLSB)
{
  if (poc >= 0)
  {
    return poc % maxLSB;
  }
  else
  {
    return (maxLSB - ((-poc) % maxLSB)) % maxLSB;
  }
}


EncGOP::EncGOP()
{
  m_iLastIDR            = 0;
  m_iGopSize            = 0;
  m_iNumPicCoded        = 0; //Niko
  m_bFirst              = true;
  m_iLastRecoveryPicPOC = 0;
  m_lastRasPoc          = MAX_INT;

  m_pcCfg               = NULL;
  m_pcSliceEncoder      = NULL;
  m_pcListPic           = NULL;
  m_HLSWriter           = NULL;
  m_bSeqFirst           = true;

  m_bRefreshPending     = 0;
  m_pocCRA              = 0;
  m_numLongTermRefPicSPS = 0;
  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_ltRefPicUsedByCurrPicFlag, 0, sizeof(m_ltRefPicUsedByCurrPicFlag));
  m_lastBPSEI           = 0;
  m_bufferingPeriodSEIPresentInAU = false;
  m_associatedIRAPType  = NAL_UNIT_CODED_SLICE_IDR_N_LP;
  m_associatedIRAPPOC   = 0;
#if W0038_DB_OPT
  m_pcDeblockingTempPicYuv = NULL;
#endif

  m_bInitAMaxBT         = true;
  m_bgPOC = -1;
  m_picBg = NULL;
  m_picOrig = NULL;
  m_isEncodedLTRef = false;
  m_isUseLTRef = false;
  m_isPrepareLTRef = true;
  m_lastLTRefPoc = 0;
}

EncGOP::~EncGOP()
{
  if( !m_pcCfg->getDecodeBitstream(0).empty() || !m_pcCfg->getDecodeBitstream(1).empty() )
  {
    // reset potential decoder resources
    tryDecodePicture( NULL, 0, std::string("") );
  }
}

/** Create list to contain pointers to CTU start addresses of slice.
 */
void  EncGOP::create()
{
  m_bLongtermTestPictureHasBeenCoded = 0;
  m_bLongtermTestPictureHasBeenCoded2 = 0;
}

void  EncGOP::destroy()
{
#if W0038_DB_OPT
  if (m_pcDeblockingTempPicYuv)
  {
    m_pcDeblockingTempPicYuv->destroy();
    delete m_pcDeblockingTempPicYuv;
    m_pcDeblockingTempPicYuv = NULL;
  }
#endif
  if (m_picBg)
  {
    m_picBg->destroy();
    delete m_picBg;
    m_picBg = NULL;
  }
  if (m_picOrig)
  {
    m_picOrig->destroy();
    delete m_picOrig;
    m_picOrig = NULL;
  }
}

void EncGOP::init ( EncLib* pcEncLib )
{
  m_pcEncLib     = pcEncLib;
  m_pcCfg                = pcEncLib;
  m_seiEncoder.init(m_pcCfg, pcEncLib, this);
  m_pcSliceEncoder       = pcEncLib->getSliceEncoder();
  m_pcListPic            = pcEncLib->getListPic();
  m_HLSWriter            = pcEncLib->getHLSWriter();
  m_pcLoopFilter         = pcEncLib->getLoopFilter();
  m_pcSAO                = pcEncLib->getSAO();
  m_pcALF = pcEncLib->getALF();
#if ADCNN
  m_pcCNNLF = pcEncLib->getCNNLF();
#endif
  m_pcRateCtrl           = pcEncLib->getRateCtrl();
  m_lastBPSEI          = 0;
  m_totalCoded         = 0;

  m_AUWriterIf = pcEncLib->getAUWriterIf();

#if WCG_EXT
#if JVET_M0427_INLOOP_RESHAPER
  if (m_pcCfg->getReshaper())
  {
    pcEncLib->getRdCost()->setReshapeInfo(m_pcCfg->getReshapeSignalType(), m_pcCfg->getBitDepth(CHANNEL_TYPE_LUMA));
    pcEncLib->getRdCost()->initLumaLevelToWeightTableReshape();
  }
  else if (m_pcCfg->getLumaLevelToDeltaQPMapping().mode)
  {
    pcEncLib->getRdCost()->setReshapeInfo(RESHAPE_SIGNAL_PQ, m_pcCfg->getBitDepth(CHANNEL_TYPE_LUMA));
    pcEncLib->getRdCost()->initLumaLevelToWeightTableReshape();
#else
  pcEncLib->getRdCost()->initLumaLevelToWeightTable();
#endif
#if JVET_M0427_INLOOP_RESHAPER
  }
  pcEncLib->getALF()->getLumaLevelWeightTable() = pcEncLib->getRdCost()->getLumaLevelWeightTable();
  int alfWSSD = 0;
  if (m_pcCfg->getReshaper() && m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ )
  {
    alfWSSD = 1;
  }
  pcEncLib->getALF()->setAlfWSSD(alfWSSD);
#endif
#endif
#if JVET_M0427_INLOOP_RESHAPER
  m_pcReshaper = pcEncLib->getReshaper();
#endif
}

#if HEVC_VPS
int EncGOP::xWriteVPS (AccessUnit &accessUnit, const VPS *vps)
{
  OutputNALUnit nalu(NAL_UNIT_VPS);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  m_HLSWriter->codeVPS( vps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}
#endif

int EncGOP::xWriteSPS (AccessUnit &accessUnit, const SPS *sps)
{
  OutputNALUnit nalu(NAL_UNIT_SPS);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  m_HLSWriter->codeSPS( sps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;

}

int EncGOP::xWritePPS (AccessUnit &accessUnit, const PPS *pps)
{
  OutputNALUnit nalu(NAL_UNIT_PPS);
  m_HLSWriter->setBitstream( &nalu.m_Bitstream );
  m_HLSWriter->codePPS( pps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}


int EncGOP::xWriteParameterSets (AccessUnit &accessUnit, Slice *slice, const bool bSeqFirst)
{
  int actualTotalBits = 0;

#if HEVC_VPS
  if (bSeqFirst)
  {
    actualTotalBits += xWriteVPS(accessUnit, m_pcEncLib->getVPS());
  }
#endif
  if (m_pcEncLib->SPSNeedsWriting(slice->getSPS()->getSPSId())) // Note this assumes that all changes to the SPS are made at the EncLib level prior to picture creation (EncLib::xGetNewPicBuffer).
  {
    VTMCHECK(!(bSeqFirst), "Unspecified error"); // Implementations that use more than 1 SPS need to be aware of activation issues.
    actualTotalBits += xWriteSPS(accessUnit, slice->getSPS());
  }
  if (m_pcEncLib->PPSNeedsWriting(slice->getPPS()->getPPSId())) // Note this assumes that all changes to the PPS are made at the EncLib level prior to picture creation (EncLib::xGetNewPicBuffer).
  {
    actualTotalBits += xWritePPS(accessUnit, slice->getPPS());
  }

  return actualTotalBits;
}

void EncGOP::xWriteAccessUnitDelimiter (AccessUnit &accessUnit, Slice *slice)
{
  AUDWriter audWriter;
  OutputNALUnit nalu(NAL_UNIT_ACCESS_UNIT_DELIMITER);

  int picType = slice->isIntra() ? 0 : (slice->isInterP() ? 1 : 2);

  audWriter.codeAUD(nalu.m_Bitstream, picType);
  accessUnit.push_front(new NALUnitEBSP(nalu));
}

// write SEI list into one NAL unit and add it to the Access unit at auPos
void EncGOP::xWriteSEI (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, int temporalId, const SPS *sps)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  OutputNALUnit nalu(naluType, temporalId);
  m_seiWriter.writeSEImessages(nalu.m_Bitstream, seiMessages, sps, false);
  auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
  auPos++;
}

void EncGOP::xWriteSEISeparately (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, int temporalId, const SPS *sps)
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  for (SEIMessages::const_iterator sei = seiMessages.begin(); sei!=seiMessages.end(); sei++ )
  {
    SEIMessages tmpMessages;
    tmpMessages.push_back(*sei);
    OutputNALUnit nalu(naluType, temporalId);
    m_seiWriter.writeSEImessages(nalu.m_Bitstream, tmpMessages, sps, false);
    auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
    auPos++;
  }
}

void EncGOP::xClearSEIs(SEIMessages& seiMessages, bool deleteMessages)
{
  if (deleteMessages)
  {
    deleteSEIs(seiMessages);
  }
  else
  {
    seiMessages.clear();
  }
}

// write SEI messages as separate NAL units ordered
void EncGOP::xWriteLeadingSEIOrdered (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, int temporalId, const SPS *sps, bool testWrite)
{
  AccessUnit::iterator itNalu = accessUnit.begin();

  while ( (itNalu!=accessUnit.end())&&
    ( (*itNalu)->m_nalUnitType==NAL_UNIT_ACCESS_UNIT_DELIMITER
#if HEVC_VPS
    || (*itNalu)->m_nalUnitType==NAL_UNIT_VPS
#endif
    || (*itNalu)->m_nalUnitType==NAL_UNIT_SPS
    || (*itNalu)->m_nalUnitType==NAL_UNIT_PPS
    ))
  {
    itNalu++;
  }

  SEIMessages localMessages = seiMessages;
  SEIMessages currentMessages;

#if ENABLE_TRACING
  g_HLSTraceEnable = !testWrite;
#endif
  // The case that a specific SEI is not present is handled in xWriteSEI (empty list)

  // Active parameter sets SEI must always be the first SEI
  currentMessages = extractSeisByType(localMessages, SEI::ACTIVE_PARAMETER_SETS);
  VTMCHECK(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // Buffering period SEI must always be following active parameter sets
  currentMessages = extractSeisByType(localMessages, SEI::BUFFERING_PERIOD);
  VTMCHECK(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // Picture timing SEI must always be following buffering period
  currentMessages = extractSeisByType(localMessages, SEI::PICTURE_TIMING);
  VTMCHECK(!(currentMessages.size() <= 1), "Unspecified error");
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // Decoding unit info SEI must always be following picture timing
  if (!duInfoSeiMessages.empty())
  {
    currentMessages.push_back(duInfoSeiMessages.front());
    if (!testWrite)
    {
      duInfoSeiMessages.pop_front();
    }
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
    xClearSEIs(currentMessages, !testWrite);
  }

  // Scalable nesting SEI must always be the following DU info
  currentMessages = extractSeisByType(localMessages, SEI::SCALABLE_NESTING);
  xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(currentMessages, !testWrite);

  // And finally everything else one by one
  xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, localMessages, accessUnit, itNalu, temporalId, sps);
  xClearSEIs(localMessages, !testWrite);

  if (!testWrite)
  {
    seiMessages.clear();
  }
}


void EncGOP::xWriteLeadingSEIMessages (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, int temporalId, const SPS *sps, std::deque<DUData> &duData)
{
  AccessUnit testAU;
  SEIMessages picTimingSEIs = getSeisByType(seiMessages, SEI::PICTURE_TIMING);
  VTMCHECK(!(picTimingSEIs.size() < 2), "Unspecified error");
  SEIPictureTiming * picTiming = picTimingSEIs.empty() ? NULL : (SEIPictureTiming*) picTimingSEIs.front();

  // test writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, testAU, temporalId, sps, true);
  // update Timing and DU info SEI
  xUpdateDuData(testAU, duData);
  xUpdateTimingSEI(picTiming, duData, sps);
  xUpdateDuInfoSEI(duInfoSeiMessages, picTiming);
  // actual writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, accessUnit, temporalId, sps, false);

  // testAU will automatically be cleaned up when losing scope
}

void EncGOP::xWriteTrailingSEIMessages (SEIMessages& seiMessages, AccessUnit &accessUnit, int temporalId, const SPS *sps)
{
  // Note: using accessUnit.end() works only as long as this function is called after slice coding and before EOS/EOB NAL units
  AccessUnit::iterator pos = accessUnit.end();
  xWriteSEISeparately(NAL_UNIT_SUFFIX_SEI, seiMessages, accessUnit, pos, temporalId, sps);
  deleteSEIs(seiMessages);
}

void EncGOP::xWriteDuSEIMessages (SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, int temporalId, const SPS *sps, std::deque<DUData> &duData)
{
  const HRD *hrd = sps->getVuiParameters()->getHrdParameters();

  if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
  {
    int naluIdx = 0;
    AccessUnit::iterator nalu = accessUnit.begin();

    // skip over first DU, we have a DU info SEI there already
    while (naluIdx < duData[0].accumNalsDU && nalu!=accessUnit.end())
    {
      naluIdx++;
      nalu++;
    }

    SEIMessages::iterator duSEI = duInfoSeiMessages.begin();
    // loop over remaining DUs
    for (int duIdx = 1; duIdx < duData.size(); duIdx++)
    {
      if (duSEI == duInfoSeiMessages.end())
      {
        // if the number of generated SEIs matches the number of DUs, this should not happen
        VTMCHECK(!(false), "Unspecified error");
        return;
      }
      // write the next SEI
      SEIMessages tmpSEI;
      tmpSEI.push_back(*duSEI);
      xWriteSEI(NAL_UNIT_PREFIX_SEI, tmpSEI, accessUnit, nalu, temporalId, sps);
      // nalu points to the position after the SEI, so we have to increase the index as well
      naluIdx++;
      while ((naluIdx < duData[duIdx].accumNalsDU) && nalu!=accessUnit.end())
      {
        naluIdx++;
        nalu++;
      }
      duSEI++;
    }
  }
  deleteSEIs(duInfoSeiMessages);
}


void EncGOP::xCreateIRAPLeadingSEIMessages (SEIMessages& seiMessages, const SPS *sps, const PPS *pps)
{
  OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

  if(m_pcCfg->getActiveParameterSetsSEIEnabled())
  {
    SEIActiveParameterSets *sei = new SEIActiveParameterSets;
#if HEVC_VPS
    m_seiEncoder.initSEIActiveParameterSets (sei, m_pcCfg->getVPS(), sps);
#else
    m_seiEncoder.initSEIActiveParameterSets(sei, sps);
#endif
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getFramePackingArrangementSEIEnabled())
  {
    SEIFramePacking *sei = new SEIFramePacking;
    m_seiEncoder.initSEIFramePacking (sei, m_iNumPicCoded);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getSegmentedRectFramePackingArrangementSEIEnabled())
  {
    SEISegmentedRectFramePacking *sei = new SEISegmentedRectFramePacking;
    m_seiEncoder.initSEISegmentedRectFramePacking(sei);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getDisplayOrientationSEIAngle())
  {
    SEIDisplayOrientation *sei = new SEIDisplayOrientation;
    m_seiEncoder.initSEIDisplayOrientation(sei);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getToneMappingInfoSEIEnabled())
  {
    SEIToneMappingInfo *sei = new SEIToneMappingInfo;
    m_seiEncoder.initSEIToneMappingInfo (sei);
    seiMessages.push_back(sei);
  }

#if HEVC_TILES_WPP
  if(m_pcCfg->getTMCTSSEIEnabled())
  {
    SEITempMotionConstrainedTileSets *sei = new SEITempMotionConstrainedTileSets;
    m_seiEncoder.initSEITempMotionConstrainedTileSets(sei, pps);
    seiMessages.push_back(sei);
  }
#endif

  if(m_pcCfg->getTimeCodeSEIEnabled())
  {
    SEITimeCode *seiTimeCode = new SEITimeCode;
    m_seiEncoder.initSEITimeCode(seiTimeCode);
    seiMessages.push_back(seiTimeCode);
  }

  if(m_pcCfg->getKneeSEIEnabled())
  {
    SEIKneeFunctionInfo *sei = new SEIKneeFunctionInfo;
    m_seiEncoder.initSEIKneeFunctionInfo(sei);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getMasteringDisplaySEI().colourVolumeSEIEnabled)
  {
    const SEIMasteringDisplay &seiCfg=m_pcCfg->getMasteringDisplaySEI();
    SEIMasteringDisplayColourVolume *sei = new SEIMasteringDisplayColourVolume;
    sei->values = seiCfg;
    seiMessages.push_back(sei);
  }
  if(m_pcCfg->getChromaResamplingFilterHintEnabled())
  {
    SEIChromaResamplingFilterHint *seiChromaResamplingFilterHint = new SEIChromaResamplingFilterHint;
    m_seiEncoder.initSEIChromaResamplingFilterHint(seiChromaResamplingFilterHint, m_pcCfg->getChromaResamplingHorFilterIdc(), m_pcCfg->getChromaResamplingVerFilterIdc());
    seiMessages.push_back(seiChromaResamplingFilterHint);
  }
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  if(m_pcCfg->getSEIAlternativeTransferCharacteristicsSEIEnable())
  {
    SEIAlternativeTransferCharacteristics *seiAlternativeTransferCharacteristics = new SEIAlternativeTransferCharacteristics;
    m_seiEncoder.initSEIAlternativeTransferCharacteristics(seiAlternativeTransferCharacteristics);
    seiMessages.push_back(seiAlternativeTransferCharacteristics);
  }
#endif
}

void EncGOP::xCreatePerPictureSEIMessages (int picInGOP, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, Slice *slice)
{
  if( ( m_pcCfg->getBufferingPeriodSEIEnabled() ) && ( slice->getSliceType() == I_SLICE ) &&
    ( slice->getSPS()->getVuiParametersPresentFlag() ) &&
    ( ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
    || ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
  {
    SEIBufferingPeriod *bufferingPeriodSEI = new SEIBufferingPeriod();
    m_seiEncoder.initSEIBufferingPeriod(bufferingPeriodSEI, slice);
    seiMessages.push_back(bufferingPeriodSEI);
    m_bufferingPeriodSEIPresentInAU = true;

    if (m_pcCfg->getScalableNestingSEIEnabled())
    {
      SEIBufferingPeriod *bufferingPeriodSEIcopy = new SEIBufferingPeriod();
      bufferingPeriodSEI->copyTo(*bufferingPeriodSEIcopy);
      nestedSeiMessages.push_back(bufferingPeriodSEIcopy);
    }
  }

  if (picInGOP ==0 && m_pcCfg->getSOPDescriptionSEIEnabled() ) // write SOP description SEI (if enabled) at the beginning of GOP
  {
    SEISOPDescription* sopDescriptionSEI = new SEISOPDescription();
    m_seiEncoder.initSEISOPDescription(sopDescriptionSEI, slice, picInGOP, m_iLastIDR, m_iGopSize);
    seiMessages.push_back(sopDescriptionSEI);
  }

  if( ( m_pcEncLib->getRecoveryPointSEIEnabled() ) && ( slice->getSliceType() == I_SLICE ) )
  {
    if( m_pcEncLib->getGradualDecodingRefreshInfoEnabled() && !slice->getRapPicFlag() )
    {
      // Gradual decoding refresh SEI
      SEIGradualDecodingRefreshInfo *gradualDecodingRefreshInfoSEI = new SEIGradualDecodingRefreshInfo();
      gradualDecodingRefreshInfoSEI->m_gdrForegroundFlag = true; // Indicating all "foreground"
      seiMessages.push_back(gradualDecodingRefreshInfoSEI);
    }
    // Recovery point SEI
    SEIRecoveryPoint *recoveryPointSEI = new SEIRecoveryPoint();
    m_seiEncoder.initSEIRecoveryPoint(recoveryPointSEI, slice);
    seiMessages.push_back(recoveryPointSEI);
  }
  if (m_pcCfg->getTemporalLevel0IndexSEIEnabled())
  {
    SEITemporalLevel0Index *temporalLevel0IndexSEI = new SEITemporalLevel0Index();
    m_seiEncoder.initTemporalLevel0IndexSEI(temporalLevel0IndexSEI, slice);
    seiMessages.push_back(temporalLevel0IndexSEI);
  }

  if( m_pcEncLib->getNoDisplaySEITLayer() && ( slice->getTLayer() >= m_pcEncLib->getNoDisplaySEITLayer() ) )
  {
    SEINoDisplay *seiNoDisplay = new SEINoDisplay;
    seiNoDisplay->m_noDisplay = true;
    seiMessages.push_back(seiNoDisplay);
  }

  // insert one Colour Remapping Info SEI for the picture (if the file exists)
  if (!m_pcCfg->getColourRemapInfoSEIFileRoot().empty())
  {
    SEIColourRemappingInfo *seiColourRemappingInfo = new SEIColourRemappingInfo();
    const bool success = m_seiEncoder.initSEIColourRemappingInfo(seiColourRemappingInfo, slice->getPOC() );

    if(success)
    {
      seiMessages.push_back(seiColourRemappingInfo);
    }
    else
    {
      delete seiColourRemappingInfo;
    }
  }
}

void EncGOP::xCreateScalableNestingSEI (SEIMessages& seiMessages, SEIMessages& nestedSeiMessages)
{
  SEIMessages tmpMessages;
  while (!nestedSeiMessages.empty())
  {
    SEI* sei=nestedSeiMessages.front();
    nestedSeiMessages.pop_front();
    tmpMessages.push_back(sei);
    SEIScalableNesting *nestingSEI = new SEIScalableNesting();
    m_seiEncoder.initSEIScalableNesting(nestingSEI, tmpMessages);
    seiMessages.push_back(nestingSEI);
    tmpMessages.clear();
  }
}

void EncGOP::xCreatePictureTimingSEI  (int IRAPGOPid, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, Slice *slice, bool isField, std::deque<DUData> &duData)
{
  const VUI *vui = slice->getSPS()->getVuiParameters();
  const HRD *hrd = vui->getHrdParameters();

  // update decoding unit parameters
  if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
    ( slice->getSPS()->getVuiParametersPresentFlag() ) &&
    (  hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() ) )
  {
    int picSptDpbOutputDuDelay = 0;
    SEIPictureTiming *pictureTimingSEI = new SEIPictureTiming();

    // DU parameters
    if( hrd->getSubPicCpbParamsPresentFlag() )
    {
      uint32_t numDU = (uint32_t) duData.size();
      pictureTimingSEI->m_numDecodingUnitsMinus1     = ( numDU - 1 );
      pictureTimingSEI->m_duCommonCpbRemovalDelayFlag = false;
      pictureTimingSEI->m_numNalusInDuMinus1.resize( numDU );
      pictureTimingSEI->m_duCpbRemovalDelayMinus1.resize( numDU );
    }
    pictureTimingSEI->m_auCpbRemovalDelay = std::min<int>(std::max<int>(1, m_totalCoded - m_lastBPSEI), static_cast<int>(pow(2, static_cast<double>(hrd->getCpbRemovalDelayLengthMinus1()+1)))); // Syntax element signalled as minus, hence the .
    pictureTimingSEI->m_picDpbOutputDelay = slice->getSPS()->getNumReorderPics(slice->getSPS()->getMaxTLayers()-1) + slice->getPOC() - m_totalCoded;
    if(m_pcCfg->getEfficientFieldIRAPEnabled() && IRAPGOPid > 0 && IRAPGOPid < m_iGopSize)
    {
      // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
      pictureTimingSEI->m_picDpbOutputDelay ++;
    }
    int factor = hrd->getTickDivisorMinus2() + 2;
    pictureTimingSEI->m_picDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() )
    {
      picSptDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    }
    if (m_bufferingPeriodSEIPresentInAU)
    {
      m_lastBPSEI = m_totalCoded;
    }

    if( hrd->getSubPicCpbParamsPresentFlag() )
    {
      int i;
      uint64_t ui64Tmp;
      uint32_t uiPrev = 0;
      uint32_t numDU = ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 );
      std::vector<uint32_t> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
      uint32_t maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

      for( i = 0; i < numDU; i ++ )
      {
        pictureTimingSEI->m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( duData[i].accumNalsDU - 1 ) : ( duData[i].accumNalsDU- duData[i-1].accumNalsDU - 1 );
      }

      if( numDU == 1 )
      {
        rDuCpbRemovalDelayMinus1[ 0 ] = 0; /* don't care */
      }
      else
      {
        rDuCpbRemovalDelayMinus1[ numDU - 1 ] = 0;/* by definition */
        uint32_t tmp = 0;
        uint32_t accum = 0;

        for( i = ( numDU - 2 ); i >= 0; i -- )
        {
          ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
          if( (uint32_t)ui64Tmp > maxDiff )
          {
            tmp ++;
          }
        }
        uiPrev = 0;

        uint32_t flag = 0;
        for( i = ( numDU - 2 ); i >= 0; i -- )
        {
          flag = 0;
          ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );

          if( (uint32_t)ui64Tmp > maxDiff )
          {
            if(uiPrev >= maxDiff - tmp)
            {
              ui64Tmp = uiPrev + 1;
              flag = 1;
            }
            else                            ui64Tmp = maxDiff - tmp + 1;
          }
          rDuCpbRemovalDelayMinus1[ i ] = (uint32_t)ui64Tmp - uiPrev - 1;
          if( (int)rDuCpbRemovalDelayMinus1[ i ] < 0 )
          {
            rDuCpbRemovalDelayMinus1[ i ] = 0;
          }
          else if (tmp > 0 && flag == 1)
          {
            tmp --;
          }
          accum += rDuCpbRemovalDelayMinus1[ i ] + 1;
          uiPrev = accum;
        }
      }
    }

    if( m_pcCfg->getPictureTimingSEIEnabled() )
    {
      pictureTimingSEI->m_picStruct = (isField && slice->getPic()->topField)? 1 : isField? 2 : 0;
      seiMessages.push_back(pictureTimingSEI);

      if ( m_pcCfg->getScalableNestingSEIEnabled() ) // put picture timing SEI into scalable nesting SEI
      {
        SEIPictureTiming *pictureTimingSEIcopy = new SEIPictureTiming();
        pictureTimingSEI->copyTo(*pictureTimingSEIcopy);
        nestedSeiMessages.push_back(pictureTimingSEIcopy);
      }
    }

    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
    {
      for( int i = 0; i < ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 ); i ++ )
      {
        SEIDecodingUnitInfo *duInfoSEI = new SEIDecodingUnitInfo();
        duInfoSEI->m_decodingUnitIdx = i;
        duInfoSEI->m_duSptCpbRemovalDelay = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i] + 1;
        duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
        duInfoSEI->m_picSptDpbOutputDuDelay = picSptDpbOutputDuDelay;

        duInfoSeiMessages.push_back(duInfoSEI);
      }
    }

    if( !m_pcCfg->getPictureTimingSEIEnabled() && pictureTimingSEI )
    {
      delete pictureTimingSEI;
    }
  }
}

void EncGOP::xUpdateDuData(AccessUnit &testAU, std::deque<DUData> &duData)
{
  if (duData.empty())
  {
    return;
  }
  // fix first
  uint32_t numNalUnits = (uint32_t)testAU.size();
  uint32_t numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = testAU.begin(); it != testAU.end(); it++)
  {
    numRBSPBytes += uint32_t((*it)->m_nalUnitData.str().size());
  }
  duData[0].accumBitsDU += ( numRBSPBytes << 3 );
  duData[0].accumNalsDU += numNalUnits;

  // adapt cumulative sums for all following DUs
  // and add one DU info SEI, if enabled
  for (int i=1; i<duData.size(); i++)
  {
    if (m_pcCfg->getDecodingUnitInfoSEIEnabled())
    {
      numNalUnits  += 1;
      numRBSPBytes += ( 5 << 3 );
    }
    duData[i].accumBitsDU += numRBSPBytes; // probably around 5 bytes
    duData[i].accumNalsDU += numNalUnits;
  }

  // The last DU may have a trailing SEI
  if (m_pcCfg->getDecodedPictureHashSEIType()!=HASHTYPE_NONE)
  {
    duData.back().accumBitsDU += ( 20 << 3 ); // probably around 20 bytes - should be further adjusted, e.g. by type
    duData.back().accumNalsDU += 1;
  }

}
void EncGOP::xUpdateTimingSEI(SEIPictureTiming *pictureTimingSEI, std::deque<DUData> &duData, const SPS *sps)
{
  if (!pictureTimingSEI)
  {
    return;
  }
  const VUI *vui = sps->getVuiParameters();
  const HRD *hrd = vui->getHrdParameters();
  if( hrd->getSubPicCpbParamsPresentFlag() )
  {
    int i;
    uint64_t ui64Tmp;
    uint32_t uiPrev = 0;
    uint32_t numDU = ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 );
    std::vector<uint32_t> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
    uint32_t maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

    for( i = 0; i < numDU; i ++ )
    {
      pictureTimingSEI->m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( duData[i].accumNalsDU - 1 ) : ( duData[i].accumNalsDU- duData[i-1].accumNalsDU - 1 );
    }

    if( numDU == 1 )
    {
      rDuCpbRemovalDelayMinus1[ 0 ] = 0; /* don't care */
    }
    else
    {
      rDuCpbRemovalDelayMinus1[ numDU - 1 ] = 0;/* by definition */
      uint32_t tmp = 0;
      uint32_t accum = 0;

      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
        if( (uint32_t)ui64Tmp > maxDiff )
        {
          tmp ++;
        }
      }
      uiPrev = 0;

      uint32_t flag = 0;
      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        flag = 0;
        ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );

        if( (uint32_t)ui64Tmp > maxDiff )
        {
          if(uiPrev >= maxDiff - tmp)
          {
            ui64Tmp = uiPrev + 1;
            flag = 1;
          }
          else                            ui64Tmp = maxDiff - tmp + 1;
        }
        rDuCpbRemovalDelayMinus1[ i ] = (uint32_t)ui64Tmp - uiPrev - 1;
        if( (int)rDuCpbRemovalDelayMinus1[ i ] < 0 )
        {
          rDuCpbRemovalDelayMinus1[ i ] = 0;
        }
        else if (tmp > 0 && flag == 1)
        {
          tmp --;
        }
        accum += rDuCpbRemovalDelayMinus1[ i ] + 1;
        uiPrev = accum;
      }
    }
  }
}
void EncGOP::xUpdateDuInfoSEI(SEIMessages &duInfoSeiMessages, SEIPictureTiming *pictureTimingSEI)
{
  if (duInfoSeiMessages.empty() || (pictureTimingSEI == NULL))
  {
    return;
  }

  int i=0;

  for (SEIMessages::iterator du = duInfoSeiMessages.begin(); du!= duInfoSeiMessages.end(); du++)
  {
    SEIDecodingUnitInfo *duInfoSEI = (SEIDecodingUnitInfo*) (*du);
    duInfoSEI->m_decodingUnitIdx = i;
    duInfoSEI->m_duSptCpbRemovalDelay = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i] + 1;
    duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
    i++;
  }
}

static void
cabac_zero_word_padding(Slice *const pcSlice, Picture *const pcPic, const std::size_t binCountsInNalUnits, const std::size_t numBytesInVclNalUnits, std::ostringstream &nalUnitData, const bool cabacZeroWordPaddingEnabled)
{
  const SPS &sps=*(pcSlice->getSPS());
  const ChromaFormat format = sps.getChromaFormatIdc();
  const int log2subWidthCxsubHeightC = (::getComponentScaleX(COMPONENT_Cb, format)+::getComponentScaleY(COMPONENT_Cb, format));
  const int minCuWidth  = pcPic->cs->pcv->minCUWidth;
  const int minCuHeight = pcPic->cs->pcv->minCUHeight;
  const int paddedWidth = ((sps.getPicWidthInLumaSamples()  + minCuWidth  - 1) / minCuWidth) * minCuWidth;
  const int paddedHeight= ((sps.getPicHeightInLumaSamples() + minCuHeight - 1) / minCuHeight) * minCuHeight;
  const int rawBits = paddedWidth * paddedHeight *
                         (sps.getBitDepth(CHANNEL_TYPE_LUMA) + 2*(sps.getBitDepth(CHANNEL_TYPE_CHROMA)>>log2subWidthCxsubHeightC));
  const std::size_t threshold = (32/3)*numBytesInVclNalUnits + (rawBits/32);
  if (binCountsInNalUnits >= threshold)
  {
    // need to add additional cabac zero words (each one accounts for 3 bytes (=00 00 03)) to increase numBytesInVclNalUnits
    const std::size_t targetNumBytesInVclNalUnits = ((binCountsInNalUnits - (rawBits/32))*3+31)/32;

    if (targetNumBytesInVclNalUnits>numBytesInVclNalUnits) // It should be!
    {
      const std::size_t numberOfAdditionalBytesNeeded=targetNumBytesInVclNalUnits - numBytesInVclNalUnits;
      const std::size_t numberOfAdditionalCabacZeroWords=(numberOfAdditionalBytesNeeded+2)/3;
      const std::size_t numberOfAdditionalCabacZeroBytes=numberOfAdditionalCabacZeroWords*3;
      if (cabacZeroWordPaddingEnabled)
      {
        std::vector<uint8_t> zeroBytesPadding(numberOfAdditionalCabacZeroBytes, uint8_t(0));
        for(std::size_t i=0; i<numberOfAdditionalCabacZeroWords; i++)
        {
          zeroBytesPadding[i*3+2]=3;  // 00 00 03
        }
        nalUnitData.write(reinterpret_cast<const char*>(&(zeroBytesPadding[0])), numberOfAdditionalCabacZeroBytes);
        msg( NOTICE, "Adding %d bytes of padding\n", uint32_t( numberOfAdditionalCabacZeroWords * 3 ) );
      }
      else
      {
        msg( NOTICE, "Standard would normally require adding %d bytes of padding\n", uint32_t( numberOfAdditionalCabacZeroWords * 3 ) );
      }
    }
  }
}

class EfficientFieldIRAPMapping
{
  private:
    int  IRAPGOPid;
    bool IRAPtoReorder;
    bool swapIRAPForward;

  public:
    EfficientFieldIRAPMapping() :
      IRAPGOPid(-1),
      IRAPtoReorder(false),
      swapIRAPForward(false)
    { }

    void initialize(const bool isField, const int gopSize, const int POCLast, const int numPicRcvd, const int lastIDR, EncGOP *pEncGop, EncCfg *pCfg);

    int adjustGOPid(const int gopID);
    int restoreGOPid(const int gopID);
    int GetIRAPGOPid() const { return IRAPGOPid; }
};

void EfficientFieldIRAPMapping::initialize(const bool isField, const int gopSize, const int POCLast, const int numPicRcvd, const int lastIDR, EncGOP *pEncGop, EncCfg *pCfg )
{
  if(isField)
  {
    int pocCurr;
    for ( int iGOPid=0; iGOPid < gopSize; iGOPid++ )
    {
      // determine actual POC
      if(POCLast == 0) //case first frame or first top field
      {
        pocCurr=0;
      }
      else if(POCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
      {
        pocCurr = 1;
      }
      else
      {
        pocCurr = POCLast - numPicRcvd + pCfg->getGOPEntry(iGOPid).m_POC - isField;
      }

      // check if POC corresponds to IRAP
      NalUnitType tmpUnitType = pEncGop->getNalUnitType(pocCurr, lastIDR, isField);
      if(tmpUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA) // if picture is an IRAP
      {
        if(pocCurr%2 == 0 && iGOPid < gopSize-1 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid+1).m_POC-1)
        { // if top field and following picture in enc order is associated bottom field
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = true;
          break;
        }
        if(pocCurr%2 != 0 && iGOPid > 0 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid-1).m_POC+1)
        {
          // if picture is an IRAP remember to process it first
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true;
          swapIRAPForward = false;
          break;
        }
      }
    }
  }
}

int EfficientFieldIRAPMapping::adjustGOPid(const int GOPid)
{
  if(IRAPtoReorder)
  {
    if(swapIRAPForward)
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return IRAPGOPid;
      }
    }
    else
    {
      if(GOPid == IRAPGOPid -1)
      {
        return IRAPGOPid;
      }
      else if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
    }
  }
  return GOPid;
}

int EfficientFieldIRAPMapping::restoreGOPid(const int GOPid)
{
  if(IRAPtoReorder)
  {
    if(swapIRAPForward)
    {
      if(GOPid == IRAPGOPid)
      {
        IRAPtoReorder = false;
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return GOPid -1;
      }
    }
    else
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
      else if(GOPid == IRAPGOPid -1)
      {
        IRAPtoReorder = false;
        return IRAPGOPid;
      }
    }
  }
  return GOPid;
}


#if X0038_LAMBDA_FROM_QP_CAPABILITY
static uint32_t calculateCollocatedFromL0Flag(const Slice *pSlice)
{
  const int refIdx = 0; // Zero always assumed
  const Picture *refPicL0 = pSlice->getRefPic(REF_PIC_LIST_0, refIdx);
  const Picture *refPicL1 = pSlice->getRefPic(REF_PIC_LIST_1, refIdx);
  return refPicL0->slices[0]->getSliceQp() > refPicL1->slices[0]->getSliceQp();
}
#else
static uint32_t calculateCollocatedFromL1Flag(EncCfg *pCfg, const int GOPid, const int gopSize)
{
  int iCloseLeft=1, iCloseRight=-1;
  for(int i = 0; i<pCfg->getGOPEntry(GOPid).m_numRefPics; i++)
  {
    int iRef = pCfg->getGOPEntry(GOPid).m_referencePics[i];
    if(iRef>0&&(iRef<iCloseRight||iCloseRight==-1))
    {
      iCloseRight=iRef;
    }
    else if(iRef<0&&(iRef>iCloseLeft||iCloseLeft==1))
    {
      iCloseLeft=iRef;
    }
  }
  if(iCloseRight>-1)
  {
    iCloseRight=iCloseRight+pCfg->getGOPEntry(GOPid).m_POC-1;
  }
  if(iCloseLeft<1)
  {
    iCloseLeft=iCloseLeft+pCfg->getGOPEntry(GOPid).m_POC-1;
    while(iCloseLeft<0)
    {
      iCloseLeft+=gopSize;
    }
  }
  int iLeftQP=0, iRightQP=0;
  for(int i=0; i<gopSize; i++)
  {
    if(pCfg->getGOPEntry(i).m_POC==(iCloseLeft%gopSize)+1)
    {
      iLeftQP= pCfg->getGOPEntry(i).m_QPOffset;
    }
    if (pCfg->getGOPEntry(i).m_POC==(iCloseRight%gopSize)+1)
    {
      iRightQP=pCfg->getGOPEntry(i).m_QPOffset;
    }
  }
  if(iCloseRight>-1&&iRightQP<iLeftQP)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}
#endif


static void
printHash(const HashType hashType, const std::string &digestStr)
{
  const char *decodedPictureHashModeName;
  switch (hashType)
  {
    case HASHTYPE_MD5:
      decodedPictureHashModeName = "MD5";
      break;
    case HASHTYPE_CRC:
      decodedPictureHashModeName = "CRC";
      break;
    case HASHTYPE_CHECKSUM:
      decodedPictureHashModeName = "Checksum";
      break;
    default:
      decodedPictureHashModeName = NULL;
      break;
  }
  if (decodedPictureHashModeName != NULL)
  {
    if (digestStr.empty())
    {
      msg( NOTICE, " [%s:%s]", decodedPictureHashModeName, "?");
    }
    else
    {
      msg( NOTICE, " [%s:%s]", decodedPictureHashModeName, digestStr.c_str());
    }
  }
}

bool isPicEncoded( int targetPoc, int curPoc, int curTLayer, int gopSize, int intraPeriod )
{
  int  tarGop = targetPoc / gopSize;
  int  curGop = curPoc / gopSize;

  if( tarGop + 1 == curGop )
  {
    // part of next GOP only for tl0 pics
    return curTLayer == 0;
  }

  int  tarIFr = ( targetPoc / intraPeriod ) * intraPeriod;
  int  curIFr = ( curPoc / intraPeriod ) * intraPeriod;

  if( curIFr != tarIFr )
  {
    return false;
  }

  int  tarId = targetPoc - tarGop * gopSize;

  if( tarGop > curGop )
  {
    return ( tarId == 0 ) ? ( 0 == curTLayer ) : ( 1 >= curTLayer );
  }

  if( tarGop + 1 < curGop )
  {
    return false;
  }

  int  curId = curPoc - curGop * gopSize;
  int  tarTL = 0;

  while( tarId != 0 )
  {
    gopSize /= 2;
    if( tarId >= gopSize )
    {
      tarId -= gopSize;
      if( curId != 0 ) curId -= gopSize;
    }
    else if( curId == gopSize )
    {
      curId = 0;
    }
    tarTL++;
  }

  return curTLayer <= tarTL && curId == 0;
}

void trySkipOrDecodePicture( bool& decPic, bool& encPic, const EncCfg& cfg, Picture* pcPic )
{
  // check if we should decode a leading bitstream
  if( !cfg.getDecodeBitstream( 0 ).empty() )
  {
    static bool bDecode1stPart = true; /* TODO: MT */
    if( bDecode1stPart )
    {
      if( cfg.getForceDecodeBitstream1() )
      {
        if( ( bDecode1stPart = tryDecodePicture( pcPic, pcPic->getPOC(), cfg.getDecodeBitstream( 0 ), false ) ) )
        {
          decPic = bDecode1stPart;
        }
      }
      else
      {
        // update decode decision
#if JVET_M0055_DEBUG_CTU
        bool dbgCTU = cfg.getDebugCTU() != -1 && cfg.getSwitchPOC() == pcPic->getPOC();

        if( ( bDecode1stPart = ( cfg.getSwitchPOC() != pcPic->getPOC() ) || dbgCTU ) && ( bDecode1stPart = tryDecodePicture( pcPic, pcPic->getPOC(), cfg.getDecodeBitstream( 0 ), false, cfg.getDebugCTU(), cfg.getSwitchPOC() ) ) )
        {
          if( dbgCTU )
          {
            encPic = true;
            decPic = false;
            bDecode1stPart = false;

            return;
          }
          decPic = bDecode1stPart;
          return;
        }
#else
        if( ( bDecode1stPart = ( cfg.getSwitchPOC() != pcPic->getPOC() )) && ( bDecode1stPart = tryDecodePicture( pcPic, pcPic->getPOC(), cfg.getDecodeBitstream( 0 ), false ) ) )
        {
          decPic = bDecode1stPart;
          return;
        }
#endif
        else if( pcPic->getPOC() )
        {
          // reset decoder if used and not required any further
          tryDecodePicture( NULL, 0, std::string( "" ) );
        }
      }
    }

    encPic |= cfg.getForceDecodeBitstream1() && !decPic;
    if( cfg.getForceDecodeBitstream1() ) { return; }
  }


  // check if we should decode a trailing bitstream
  if( ! cfg.getDecodeBitstream(1).empty() )
  {
    const int  iNextKeyPOC    = (1+cfg.getSwitchPOC()  / cfg.getGOPSize())     *cfg.getGOPSize();
    const int  iNextIntraPOC  = (1+(cfg.getSwitchPOC() / cfg.getIntraPeriod()))*cfg.getIntraPeriod();
    const int  iRestartIntraPOC   = iNextIntraPOC + (((iNextKeyPOC == iNextIntraPOC) && cfg.getSwitchDQP() ) ? cfg.getIntraPeriod() : 0);

    bool bDecode2ndPart = (pcPic->getPOC() >= iRestartIntraPOC);
    int expectedPoc = pcPic->getPOC();
    Slice slice0;
    if ( cfg.getBs2ModPOCAndType() )
    {
      expectedPoc = pcPic->getPOC() - iRestartIntraPOC;
      slice0.copySliceInfo( pcPic->slices[ 0 ], false );
    }
    if( bDecode2ndPart && (bDecode2ndPart = tryDecodePicture( pcPic, expectedPoc, cfg.getDecodeBitstream(1), true )) )
    {
      decPic = bDecode2ndPart;
      if ( cfg.getBs2ModPOCAndType() )
      {
        for( int i = 0; i < pcPic->slices.size(); i++ )
        {
          pcPic->slices[ i ]->setPOC              ( slice0.getPOC()            );
          if ( pcPic->slices[ i ]->getNalUnitType() != slice0.getNalUnitType()
              && pcPic->slices[ i ]->getIdrPicFlag()
              && slice0.getRapPicFlag()
              && slice0.isIntra() )
          {
            // patch IDR-slice to CRA-Intra-slice
            pcPic->slices[ i ]->setNalUnitType    ( slice0.getNalUnitType()    );
            pcPic->slices[ i ]->setLastIDR        ( slice0.getLastIDR()        );
            pcPic->slices[ i ]->setEnableTMVPFlag ( slice0.getEnableTMVPFlag() );
            if ( slice0.getEnableTMVPFlag() )
            {
              pcPic->slices[ i ]->setColFromL0Flag( slice0.getColFromL0Flag()  );
              pcPic->slices[ i ]->setColRefIdx    ( slice0.getColRefIdx()      );
            }
          }
        }
      }
      return;
    }
  }

  // leave here if we do not use forward to poc
  if( ! cfg.useFastForwardToPOC() )
  {
    // let's encode
    encPic   = true;
    return;
  }

  // this is the forward to poc section
  static bool bHitFastForwardPOC = false; /* TODO: MT */
  if( bHitFastForwardPOC || isPicEncoded( cfg.getFastForwardToPOC(), pcPic->getPOC(), pcPic->layer, cfg.getGOPSize(), cfg.getIntraPeriod() ) )
  {
    bHitFastForwardPOC |= cfg.getFastForwardToPOC() == pcPic->getPOC(); // once we hit the poc we continue encoding

    if( bHitFastForwardPOC && cfg.getStopAfterFFtoPOC() && cfg.getFastForwardToPOC() != pcPic->getPOC() )
    {
      return;
    }

    //except if FastForwardtoPOC is meant to be a SwitchPOC in thist case drop all preceding pictures
    if( bHitFastForwardPOC && ( cfg.getSwitchPOC() == cfg.getFastForwardToPOC() ) && ( cfg.getFastForwardToPOC() > pcPic->getPOC() ) )
    {
      return;
    }
    // let's encode
    encPic   = true;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
void EncGOP::compressGOP( int iPOCLast, int iNumPicRcvd, PicList& rcListPic,
                          std::list<PelUnitBuf*>& rcListPicYuvRecOut,
                          bool isField, bool isTff, const InputColourSpaceConversion snr_conversion, const bool printFrameMSE
                        , bool isEncodeLtRef
)
{
  // TODO: Split this function up.

  Picture*        pcPic = NULL;
  Slice*      pcSlice;
  OutputBitstream  *pcBitstreamRedirect;
  pcBitstreamRedirect = new OutputBitstream;
  AccessUnit::iterator  itLocationToPushSliceHeaderNALU; // used to store location where NALU containing slice header is to be inserted

  xInitGOP(iPOCLast, iNumPicRcvd, isField
         , isEncodeLtRef
  );

  m_iNumPicCoded = 0;
  SEIMessages leadingSeiMessages;
  SEIMessages nestedSeiMessages;
  SEIMessages duInfoSeiMessages;
  SEIMessages trailingSeiMessages;
  std::deque<DUData> duData;
  SEIDecodingUnitInfo decodingUnitInfoSEI;

  EfficientFieldIRAPMapping effFieldIRAPMap;
  if (m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    effFieldIRAPMap.initialize(isField, m_iGopSize, iPOCLast, iNumPicRcvd, m_iLastIDR, this, m_pcCfg);
  }

  // reset flag indicating whether pictures have been encoded
  for ( int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
  {
    m_pcCfg->setEncodedFlag(iGOPid, false);
  }

  for ( int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )
  {
    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      iGOPid=effFieldIRAPMap.adjustGOPid(iGOPid);
    }

    //-- For time output for each slice
    auto beforeTime = std::chrono::steady_clock::now();

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
    uint32_t uiColDir = calculateCollocatedFromL1Flag(m_pcCfg, iGOPid, m_iGopSize);
#endif

    /////////////////////////////////////////////////////////////////////////////////////////////////// Initial to start encoding
    int iTimeOffset;
    int pocCurr;
    int multipleFactor = m_pcCfg->getUseCompositeRef() ? 2 : 1;

    if(iPOCLast == 0) //case first frame or first top field
    {
      pocCurr=0;
      iTimeOffset = multipleFactor;
    }
    else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
    {
      pocCurr = 1;
      iTimeOffset = 1;
    }
    else
    {
      pocCurr = iPOCLast - iNumPicRcvd * multipleFactor + m_pcCfg->getGOPEntry(iGOPid).m_POC - ((isField && m_iGopSize>1) ? 1 : 0);
      iTimeOffset = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    }

    if (m_pcCfg->getUseCompositeRef() && isEncodeLtRef)
    {
      pocCurr++;
      iTimeOffset--;
    }
    if (pocCurr / multipleFactor >= m_pcCfg->getFramesToBeEncoded())
    {
      if (m_pcCfg->getEfficientFieldIRAPEnabled())
      {
        iGOPid=effFieldIRAPMap.restoreGOPid(iGOPid);
      }
      continue;
    }

    if( getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP )
    {
      m_iLastIDR = pocCurr;
    }

    // start a new access unit: create an entry in the list of output access units
    AccessUnit accessUnit;
    xGetBuffer( rcListPic, rcListPicYuvRecOut,
                iNumPicRcvd, iTimeOffset, pcPic, pocCurr, isField );

    // th this is a hot fix for the choma qp control
    if( m_pcEncLib->getWCGChromaQPControl().isEnabled() && m_pcEncLib->getSwitchPOC() != -1 )
    {
      static int usePPS = 0; /* TODO: MT */
      if( pocCurr == m_pcEncLib->getSwitchPOC() )
      {
        usePPS = 1;
      }
      const PPS *pPPS = m_pcEncLib->getPPS(usePPS);
      // replace the pps with a more appropriated one
      pcPic->cs->pps = pPPS;
    }

#if ENABLE_SPLIT_PARALLELISM && ENABLE_WPP_PARALLELISM
    pcPic->scheduler.init( pcPic->cs->pcv->heightInCtus, pcPic->cs->pcv->widthInCtus, m_pcCfg->getNumWppThreads(), m_pcCfg->getNumWppExtraLines(), m_pcCfg->getNumSplitThreads() );
#elif ENABLE_SPLIT_PARALLELISM
    pcPic->scheduler.init( pcPic->cs->pcv->heightInCtus, pcPic->cs->pcv->widthInCtus, 1                          , 0                             , m_pcCfg->getNumSplitThreads() );
#elif ENABLE_WPP_PARALLELISM
    pcPic->scheduler.init( pcPic->cs->pcv->heightInCtus, pcPic->cs->pcv->widthInCtus, m_pcCfg->getNumWppThreads(), m_pcCfg->getNumWppExtraLines(), 1                             );
#endif
    pcPic->createTempBuffers( pcPic->cs->pps->pcv->maxCUWidth );
    pcPic->cs->createCoeffs();

    //  Slice data initialization
    pcPic->clearSliceBuffer();
    pcPic->allocateNewSlice();
    m_pcSliceEncoder->setSliceSegmentIdx(0);

    m_pcSliceEncoder->initEncSlice(pcPic, iPOCLast, pocCurr, iGOPid, pcSlice, isField
      , isEncodeLtRef
    );

    DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", pocCurr ) ) );
    DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "final", 0 ) ) );

#if !SHARP_LUMA_DELTA_QP
    //Set Frame/Field coding
    pcPic->fieldPic = isField;
#endif

    int pocBits = pcSlice->getSPS()->getBitsForPOC();
    int pocMask = (1 << pocBits) - 1;
    pcSlice->setLastIDR(m_iLastIDR & ~pocMask);
#if HEVC_DEPENDENT_SLICES
    pcSlice->setSliceSegmentIdx(0);
#endif
    pcSlice->setIndependentSliceIdx(0);
    //set default slice level flag to the same as SPS level flag
    pcSlice->setLFCrossSliceBoundaryFlag(  pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()  );

    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='P')
    {
      pcSlice->setSliceType(P_SLICE);
    }
    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='I')
    {
      pcSlice->setSliceType(I_SLICE);
    }
#if JVET_M0483_IBC ==0
    if (pcSlice->getSliceType() == I_SLICE && pcSlice->getSPS()->getIBCMode())
    {
      pcSlice->setSliceType(P_SLICE);
    }
#endif
    // Set the nal unit type
    pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));
    if(pcSlice->getTemporalLayerNonReferenceFlag())
    {
      if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_R &&
          !(m_iGopSize == 1 && pcSlice->getSliceType() == I_SLICE))
        // Add this condition to avoid POC issues with encoder_intra_main.cfg configuration (see #1127 in bug tracker)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TRAIL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RADL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RADL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RASL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RASL_N);
      }
    }

    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
      {
        m_associatedIRAPType = pcSlice->getNalUnitType();
        m_associatedIRAPPOC = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
    }

    pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic, m_pcCfg->getEfficientFieldIRAPEnabled());
    if (m_pcCfg->getUseCompositeRef() && isEncodeLtRef)
    {
      setUseLTRef(true);
      setPrepareLTRef(false);
      setNewestBgPOC(pocCurr);
      setLastLTRefPoc(pocCurr);
    }
    else if (pcPic->cs->sps->getUseCompositeRef() && getLastLTRefPoc() >= 0 && getEncodedLTRef()==false && !getPicBg()->getSpliceFull() && (pocCurr - getLastLTRefPoc()) > (m_pcCfg->getFrameRate() * 2))
    {
      setUseLTRef(false);
      setPrepareLTRef(false);
      setEncodedLTRef(true);
      setNewestBgPOC(-1);
      setLastLTRefPoc(-1);
    }

    if (pcPic->cs->sps->getUseCompositeRef() && m_picBg->getSpliceFull() && getUseLTRef())
    {
      m_pcEncLib->selectReferencePictureSet(pcSlice, pocCurr, iGOPid, m_bgPOC);
    }
    else
    {
      m_pcEncLib->selectReferencePictureSet(pcSlice, pocCurr, iGOPid, -1);
    }
    if (!m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
      {
        m_associatedIRAPType = pcSlice->getNalUnitType();
        m_associatedIRAPPOC = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
    }

    if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false, m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3) != 0) || (pcSlice->isIRAP())
      || (m_pcCfg->getEfficientFieldIRAPEnabled() && isField && pcSlice->getAssociatedIRAPType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getAssociatedIRAPType() <= NAL_UNIT_CODED_SLICE_CRA && pcSlice->getAssociatedIRAPPOC() == pcSlice->getPOC()+1)
      )
    {
      pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP(), m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3, m_pcCfg->getEfficientFieldIRAPEnabled()
                                                            , isEncodeLtRef, m_pcCfg->getUseCompositeRef()
      );
    }

    pcSlice->applyReferencePictureSet(rcListPic, pcSlice->getRPS());

    if(pcSlice->getTLayer() > 0
      &&  !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N     // Check if not a leading picture
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R )
        )
    {
      if(pcSlice->isTemporalLayerSwitchingPoint(rcListPic) || pcSlice->getSPS()->getTemporalIdNestingFlag())
      {
        if(pcSlice->getTemporalLayerNonReferenceFlag())
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
        }
        else
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R);
        }
      }
      else if(pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))
      {
        bool isSTSA=true;
        for(int ii=iGOPid+1;(ii<m_pcCfg->getGOPSize() && isSTSA==true);ii++)
        {
          int lTid= m_pcCfg->getGOPEntry(ii).m_temporalId;
          if(lTid==pcSlice->getTLayer())
          {
            const ReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);
            for(int jj=0;jj<nRPS->getNumberOfPictures();jj++)
            {
              if(nRPS->getUsed(jj))
              {
                int tPoc=m_pcCfg->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);
                int kk=0;
                for(kk=0;kk<m_pcCfg->getGOPSize();kk++)
                {
                  if(m_pcCfg->getGOPEntry(kk).m_POC==tPoc)
                  {
                    break;
                  }
                }
                int tTid=m_pcCfg->getGOPEntry(kk).m_temporalId;
                if(tTid >= pcSlice->getTLayer())
                {
                  isSTSA=false;
                  break;
                }
              }
            }
          }
        }
        if(isSTSA==true)
        {
          if(pcSlice->getTemporalLayerNonReferenceFlag())
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
          }
          else
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R);
          }
        }
      }
    }
    if (pcSlice->getRPSidx() == -1)
      arrangeLongtermPicturesInRPS(pcSlice, rcListPic);
    RefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    refPicListModification->setRefPicListModificationFlagL0(0);
    refPicListModification->setRefPicListModificationFlagL1(0);

    if (m_pcCfg->getUseCompositeRef() && getUseLTRef() && (pocCurr > getLastLTRefPoc()))
    {
      pcSlice->setNumRefIdx(REF_PIC_LIST_0, min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive + 1, pcSlice->getRPS()->getNumberOfPictures()));
      pcSlice->setNumRefIdx(REF_PIC_LIST_1, min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive + 1, pcSlice->getRPS()->getNumberOfPictures()));
    }
    else
    {
      pcSlice->setNumRefIdx(REF_PIC_LIST_0, std::min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive, pcSlice->getRPS()->getNumberOfPictures()));
      pcSlice->setNumRefIdx(REF_PIC_LIST_1, std::min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive, pcSlice->getRPS()->getNumberOfPictures()));
    }
    if (pcPic->cs->sps->getUseCompositeRef() && getPrepareLTRef()) {
      arrangeCompositeReference(pcSlice, rcListPic, pocCurr);
    }
#if JVET_M0483_IBC==0
    if (pcSlice->getSPS()->getIBCMode())
    {
      if (m_pcCfg->getIntraPeriod() > 0 && pcSlice->getPOC() % m_pcCfg->getIntraPeriod() == 0)
      {
        pcSlice->setNumRefIdx(REF_PIC_LIST_0, 0);
        pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
      }

      pcSlice->setNumRefIdx(REF_PIC_LIST_0, pcSlice->getNumRefIdx(REF_PIC_LIST_0) + 1);
    }
#endif
    //  Set reference list
    pcSlice->setRefPicList ( rcListPic );

#if JVET_M0253_HASH_ME
    if (m_pcCfg->getUseHashME())
    {
      PicList::iterator iterPic = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        Picture* refPic = *(iterPic++);

        if (refPic->poc != pcPic->poc && refPic->referenced)
        {
          if (!refPic->getHashMap()->isInitial())
          {
            if (refPic->getPOC() == 0)
            {
              Pel* picSrc = refPic->getOrigBuf().get(COMPONENT_Y).buf;
              int stridePic = refPic->getOrigBuf().get(COMPONENT_Y).stride;
              int picWidth = pcSlice->getSPS()->getPicWidthInLumaSamples();
              int picHeight = pcSlice->getSPS()->getPicHeightInLumaSamples();
              int blockSize = 4;
              int allNum = 0;
              int simpleNum = 0;
              for (int j = 0; j <= picHeight - blockSize; j += blockSize)
              {
                for (int i = 0; i <= picWidth - blockSize; i += blockSize)
                {
                  Pel* curBlock = picSrc + j * stridePic + i;
                  bool isHorSame = true;
                  for (int m = 0; m < blockSize&&isHorSame; m++)
                  {
                    for (int n = 1; n < blockSize&&isHorSame; n++)
                    {
                      if (curBlock[m*stridePic] != curBlock[m*stridePic + n])
                      {
                        isHorSame = false;
                      }
                    }
                  }
                  bool isVerSame = true;
                  for (int m = 1; m < blockSize&&isVerSame; m++)
                  {
                    for (int n = 0; n < blockSize&&isVerSame; n++)
                    {
                      if (curBlock[n] != curBlock[m*stridePic + n])
                      {
                        isVerSame = false;
                      }
                    }
                  }
                  allNum++;
                  if (isHorSame || isVerSame)
                  {
                    simpleNum++;
                  }
                }
              }

              if (simpleNum < 0.3*allNum)
              {
                m_pcCfg->setUseHashME(false);
                break;
              }
            }
            refPic->addPictureToHashMapForInter();
          }
        }
      }
    }
#endif
    if( m_pcCfg->getUseAMaxBT() )
    {
      if( !pcSlice->isIRAP() )
      {
        int refLayer = pcSlice->getDepth();
        if( refLayer > 9 ) refLayer = 9; // Max layer is 10

        if( m_bInitAMaxBT && pcSlice->getPOC() > m_uiPrevISlicePOC )
        {
          ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
          ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
          m_bInitAMaxBT = false;
        }

        if( refLayer >= 0 && m_uiNumBlk[refLayer] != 0 )
        {
          pcSlice->setSplitConsOverrideFlag(true);
          double dBlkSize = sqrt( ( double ) m_uiBlkSize[refLayer] / m_uiNumBlk[refLayer] );
          if( dBlkSize < AMAXBT_TH32 )
          {
            pcSlice->setMaxBTSize( 32 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 32 );
          }
          else if( dBlkSize < AMAXBT_TH64 )
          {
            pcSlice->setMaxBTSize( 64 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 64 );
          }
          else
          {
            pcSlice->setMaxBTSize( 128 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 128 );
          }

          m_uiBlkSize[refLayer] = 0;
          m_uiNumBlk [refLayer] = 0;
        }
      }
      else
      {
        if( m_bInitAMaxBT )
        {
          ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
          ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
        }

        m_uiPrevISlicePOC = pcSlice->getPOC();
        m_bInitAMaxBT = true;
      }
    }

    //  Slice info. refinement
    if ( (pcSlice->getSliceType() == B_SLICE) && (pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0) )
    {
      pcSlice->setSliceType ( P_SLICE );
    }
#if JVET_M0483_IBC==0
    if (pcSlice->getSPS()->getIBCMode() && pcSlice->getNumRefIdx(REF_PIC_LIST_0) == 1)
    {
      m_pcSliceEncoder->setEncCABACTableIdx(P_SLICE);
    }
#endif
    xUpdateRasInit( pcSlice );

    // Do decoding refresh marking if any
#if COM16_C806_ALF_TEMPPRED_NUM
    if ( pcSlice->getPendingRasInit() || pcSlice->isIDRorBLA() )
    {
      m_pcALF->refreshAlfTempPred();
    }
#endif

    if ( pcSlice->getPendingRasInit() )
    {
      // this ensures that independently encoded bitstream chunks can be combined to bit-equal
      pcSlice->setEncCABACTableIdx( pcSlice->getSliceType() );
    }
    else
    {
      pcSlice->setEncCABACTableIdx( m_pcSliceEncoder->getEncCABACTableIdx() );
    }

    if (pcSlice->getSliceType() == B_SLICE)
    {
#if X0038_LAMBDA_FROM_QP_CAPABILITY
      const uint32_t uiColFromL0 = calculateCollocatedFromL0Flag(pcSlice);
      pcSlice->setColFromL0Flag(uiColFromL0);
#else
      pcSlice->setColFromL0Flag(1-uiColDir);
#endif
      bool bLowDelay = true;
      int  iCurrPOC  = pcSlice->getPOC();
      int iRefIdx = 0;

      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }
    else
    {
      pcSlice->setCheckLDC(true);
    }

#if !X0038_LAMBDA_FROM_QP_CAPABILITY
    uiColDir = 1-uiColDir;
#endif

    //-------------------------------------------------------------
    pcSlice->setRefPOCList();


    pcSlice->setList1IdxToList0Idx();

    if (m_pcEncLib->getTMVPModeId() == 2)
    {
      if (iGOPid == 0) // first picture in SOP (i.e. forward B)
      {
        pcSlice->setEnableTMVPFlag(0);
      }
      else
      {
        // Note: pcSlice->getColFromL0Flag() is assumed to be always 0 and getcolRefIdx() is always 0.
        pcSlice->setEnableTMVPFlag(1);
      }
    }
    else if (m_pcEncLib->getTMVPModeId() == 1)
    {
      pcSlice->setEnableTMVPFlag(1);
    }
    else
    {
      pcSlice->setEnableTMVPFlag(0);
    }

    // disable TMVP when current picture is the only ref picture
#if JVET_M0483_IBC
    if (pcSlice->isIRAP() && pcSlice->getSPS()->getIBCFlag())
#else
    if (pcSlice->isIRAP() && pcSlice->getSPS()->getIBCMode())
#endif
    {
      pcSlice->setEnableTMVPFlag(0);
    }

    // set adaptive search range for non-intra-slices
    if (m_pcCfg->getUseASR() && !pcSlice->isIRAP())
    {
      m_pcSliceEncoder->setSearchRange(pcSlice);
    }

    bool bGPBcheck=false;
    if ( pcSlice->getSliceType() == B_SLICE)
    {
#if JVET_M0483_IBC==0
      if (pcSlice->getSPS()->getIBCMode())
      {
        if (pcSlice->getNumRefIdx(RefPicList(0)) - 1 == pcSlice->getNumRefIdx(RefPicList(1)))
        {
          bGPBcheck = true;
          for (int i = 0; i < pcSlice->getNumRefIdx(RefPicList(1)); i++)
          {
            if (pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i))
            {
              bGPBcheck = false;
              break;
            }
          }
        }
      }
      else
#endif
      if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
      {
        bGPBcheck=true;
        int i;
        for ( i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
        {
          if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) )
          {
            bGPBcheck=false;
            break;
          }
        }
      }
    }
    if(bGPBcheck)
    {
      pcSlice->setMvdL1ZeroFlag(true);
    }
    else
    {
      pcSlice->setMvdL1ZeroFlag(false);
    }
#if HEVC_DEPENDENT_SLICES
    pcPic->slices[pcSlice->getSliceSegmentIdx()]->setMvdL1ZeroFlag(pcSlice->getMvdL1ZeroFlag());
#endif

#if JVET_M0444_SMVD
    if ( pcSlice->getCheckLDC() == false && pcSlice->getMvdL1ZeroFlag() == false )
    {
      int currPOC = pcSlice->getPOC();

      int forwardPOC = currPOC;
      int backwardPOC = currPOC;
      int ref = 0, refIdx0 = -1, refIdx1 = -1;

      // search nearest forward POC in List 0
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
        if ( poc < currPOC && (poc > forwardPOC || refIdx0 == -1) )
        {
          forwardPOC = poc;
          refIdx0 = ref;
        }
      }

      // search nearest backward POC in List 1
      for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
      {
        int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
        if ( poc > currPOC && (poc < backwardPOC || refIdx1 == -1) )
        {
          backwardPOC = poc;
          refIdx1 = ref;
        }
      }

      if ( !(forwardPOC < currPOC && backwardPOC > currPOC) )
      {
        forwardPOC = currPOC;
        backwardPOC = currPOC;
        refIdx0 = -1;
        refIdx1 = -1;

        // search nearest backward POC in List 0
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_0, ref )->getPOC();
          if ( poc > currPOC && (poc < backwardPOC || refIdx0 == -1) )
          {
            backwardPOC = poc;
            refIdx0 = ref;
          }
        }

        // search nearest forward POC in List 1
        for ( ref = 0; ref < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); ref++ )
        {
          int poc = pcSlice->getRefPic( REF_PIC_LIST_1, ref )->getPOC();
          if ( poc < currPOC && (poc > forwardPOC || refIdx1 == -1) )
          {
            forwardPOC = poc;
            refIdx1 = ref;
          }
        }
      }

      if ( forwardPOC < currPOC && backwardPOC > currPOC )
      {
        pcSlice->setBiDirPred( true, refIdx0, refIdx1 );
      }
      else
      {
        pcSlice->setBiDirPred( false, -1, -1 );
      }
    }
    else
    {
      pcSlice->setBiDirPred( false, -1, -1 );
    }
#endif

    double lambda            = 0.0;
    int actualHeadBits       = 0;
    int actualTotalBits      = 0;
    int estimatedBits        = 0;
    int tmpBitsBeforeWriting = 0;
    if ( m_pcCfg->getUseRateCtrl() ) // TODO: does this work with multiple slices and slice-segments?
    {
      int frameLevel = m_pcRateCtrl->getRCSeq()->getGOPID2Level( iGOPid );
      if ( pcPic->slices[0]->isIRAP() )
      {
        frameLevel = 0;
      }
      m_pcRateCtrl->initRCPic( frameLevel );
      estimatedBits = m_pcRateCtrl->getRCPic()->getTargetBits();

#if U0132_TARGET_BITS_SATURATION
      if (m_pcRateCtrl->getCpbSaturationEnabled() && frameLevel != 0)
      {
        int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

        // prevent overflow
        if (estimatedCpbFullness - estimatedBits > (int)(m_pcRateCtrl->getCpbSize()*0.9f))
        {
          estimatedBits = estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize()*0.9f);
        }

        estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
        // prevent underflow
#if V0078_ADAPTIVE_LOWER_BOUND
        if (estimatedCpbFullness - estimatedBits < m_pcRateCtrl->getRCPic()->getLowerBound())
        {
          estimatedBits = std::max(200, estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound());
        }
#else
        if (estimatedCpbFullness - estimatedBits < (int)(m_pcRateCtrl->getCpbSize()*0.1f))
        {
          estimatedBits = std::max(200, estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize()*0.1f));
        }
#endif

        m_pcRateCtrl->getRCPic()->setTargetBits(estimatedBits);
      }
#endif

      int sliceQP = m_pcCfg->getInitialQP();
      if ( ( pcSlice->getPOC() == 0 && m_pcCfg->getInitialQP() > 0 ) || ( frameLevel == 0 && m_pcCfg->getForceIntraQP() ) ) // QP is specified
      {
        int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(double)NumberBFrames );
        double dQPFactor     = 0.57*dLambda_scale;
        int    SHIFT_QP      = 12;
        int bitdepth_luma_qp_scale =
          6
          * (pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8
             - DISTORTION_PRECISION_ADJUSTMENT(pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)));
        double qp_temp = (double) sliceQP + bitdepth_luma_qp_scale - SHIFT_QP;
        lambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
      }
      else if ( frameLevel == 0 )   // intra case, but use the model
      {
        m_pcSliceEncoder->calCostSliceI(pcPic); // TODO: This only analyses the first slice segment - what about the others?

        if ( m_pcCfg->getIntraPeriod() != 1 )   // do not refine allocated bits for all intra case
        {
          int bits = m_pcRateCtrl->getRCSeq()->getLeftAverageBits();
          bits = m_pcRateCtrl->getRCPic()->getRefineBitsForIntra( bits );

#if U0132_TARGET_BITS_SATURATION
          if (m_pcRateCtrl->getCpbSaturationEnabled() )
          {
            int estimatedCpbFullness = m_pcRateCtrl->getCpbState() + m_pcRateCtrl->getBufferingRate();

            // prevent overflow
            if (estimatedCpbFullness - bits > (int)(m_pcRateCtrl->getCpbSize()*0.9f))
            {
              bits = estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize()*0.9f);
            }

            estimatedCpbFullness -= m_pcRateCtrl->getBufferingRate();
            // prevent underflow
#if V0078_ADAPTIVE_LOWER_BOUND
            if (estimatedCpbFullness - bits < m_pcRateCtrl->getRCPic()->getLowerBound())
            {
              bits = estimatedCpbFullness - m_pcRateCtrl->getRCPic()->getLowerBound();
            }
#else
            if (estimatedCpbFullness - bits < (int)(m_pcRateCtrl->getCpbSize()*0.1f))
            {
              bits = estimatedCpbFullness - (int)(m_pcRateCtrl->getCpbSize()*0.1f);
            }
#endif
          }
#endif

          if ( bits < 200 )
          {
            bits = 200;
          }
          m_pcRateCtrl->getRCPic()->setTargetBits( bits );
        }

        list<EncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        m_pcRateCtrl->getRCPic()->getLCUInitTargetBits();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->isIRAP());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }
      else    // normal case
      {
        list<EncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->isIRAP());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }

      sliceQP = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, sliceQP );
      m_pcRateCtrl->getRCPic()->setPicEstQP( sliceQP );

      m_pcSliceEncoder->resetQP( pcPic, sliceQP, lambda );
    }

    uint32_t uiNumSliceSegments = 1;

    {
      pcSlice->setDefaultClpRng( *pcSlice->getSPS() );
    }

    // Allocate some coders, now the number of tiles are known.
    const uint32_t numberOfCtusInFrame = pcPic->cs->pcv->sizeInCtus;
#if HEVC_TILES_WPP
    const int numSubstreamsColumns = (pcSlice->getPPS()->getNumTileColumnsMinus1() + 1);
    const int numSubstreamRows     = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag() ? pcPic->cs->pcv->heightInCtus : (pcSlice->getPPS()->getNumTileRowsMinus1() + 1);
    const int numSubstreams        = numSubstreamRows * numSubstreamsColumns;
#else
    const int numSubstreams        = 1;
#endif
    std::vector<OutputBitstream> substreamsOut(numSubstreams);

#if ENABLE_QPA
    pcPic->m_uEnerHpCtu.resize( numberOfCtusInFrame );
    pcPic->m_iOffsetCtu.resize( numberOfCtusInFrame );
#endif

#if ADCNN
	if (pcSlice->getSPS()->getCNNLFEnabledFlag())
	{
		pcPic->resizeCnnlfCtuEnableFlag(numberOfCtusInFrame);
		std::memset(pcSlice->getCnnlfSliceParam().enabledFlag, false, sizeof(pcSlice->getCnnlfSliceParam().enabledFlag));
	}
#endif

    if (pcSlice->getSPS()->getSAOEnabledFlag())
    {
      pcPic->resizeSAO( numberOfCtusInFrame, 0 );
      pcPic->resizeSAO( numberOfCtusInFrame, 1 );
    }

    // it is used for signalling during CTU mode decision, i.e. before ALF processing
    if( pcSlice->getSPS()->getALFEnabledFlag() )
    {
      pcPic->resizeAlfCtuEnableFlag( numberOfCtusInFrame );
      std::memset( pcSlice->getAlfSliceParam().enabledFlag, false, sizeof( pcSlice->getAlfSliceParam().enabledFlag ) );
    }

    bool decPic = false;
    bool encPic = false;
    // test if we can skip the picture entirely or decode instead of encoding
    trySkipOrDecodePicture( decPic, encPic, *m_pcCfg, pcPic );

    pcPic->cs->slice = pcSlice; // please keep this
#if ENABLE_QPA
    if (pcSlice->getPPS()->getSliceChromaQpFlag() && CS::isDualITree (*pcSlice->getPic()->cs) && !m_pcCfg->getUsePerceptQPA() && (m_pcCfg->getSliceChromaOffsetQpPeriodicity() == 0))
#else
    if (pcSlice->getPPS()->getSliceChromaQpFlag() && CS::isDualITree (*pcSlice->getPic()->cs))
#endif
    {
      // overwrite chroma qp offset for dual tree
      pcSlice->setSliceChromaQpDelta(COMPONENT_Cb, m_pcCfg->getChromaCbQpOffsetDualTree());
      pcSlice->setSliceChromaQpDelta(COMPONENT_Cr, m_pcCfg->getChromaCrQpOffsetDualTree());
      m_pcSliceEncoder->setUpLambda(pcSlice, pcSlice->getLambdas()[0], pcSlice->getSliceQp());
    }
#if JVET_M0427_INLOOP_RESHAPER
    if (pcSlice->getSPS()->getUseReshaper())
    {
      m_pcReshaper->getReshapeCW()->rspTid = pcSlice->getTLayer() + (pcSlice->isIntra() ? 0 : 1);
      m_pcReshaper->getReshapeCW()->rspSliceQP = pcSlice->getSliceQp();

      m_pcReshaper->setSrcReshaped(false);
      m_pcReshaper->setRecReshaped(true);

      if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ)
      {
#if JVET_M0483_IBC
        m_pcReshaper->preAnalyzerHDR(pcPic, pcSlice->getSliceType(), m_pcCfg->getReshapeCW(), m_pcCfg->getDualITree());
#else
        m_pcReshaper->preAnalyzerHDR(pcPic, pcSlice->getSliceType(), m_pcCfg->getReshapeCW(), m_pcCfg->getDualITree(), m_pcCfg->getIBCMode());
#endif
      }
      else if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_SDR)
      {
#if JVET_M0483_IBC
        m_pcReshaper->preAnalyzerSDR(pcPic, pcSlice->getSliceType(), m_pcCfg->getReshapeCW(), m_pcCfg->getDualITree());
#else
        m_pcReshaper->preAnalyzerSDR(pcPic, pcSlice->getSliceType(), m_pcCfg->getReshapeCW(), m_pcCfg->getDualITree(), m_pcCfg->getIBCMode());
#endif
      }
      else
      {
        THROW("Reshaper for other signal currently not defined!");
      }

#if JVET_M0483_IBC
      if (pcSlice->getSliceType() == I_SLICE )
#else
      if (pcSlice->getSliceType() == I_SLICE || (pcSlice->getSliceType() == P_SLICE && m_pcCfg->getIBCMode()))
#endif
      {
        if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ)
        {
          m_pcReshaper->initLUTfromdQPModel();
          m_pcEncLib->getRdCost()->updateReshapeLumaLevelToWeightTableChromaMD(m_pcReshaper->getInvLUT());
        }
        else if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_SDR)
        {
          if (m_pcReshaper->getReshapeFlag())
          {
            m_pcReshaper->constructReshaperSDR();
            m_pcEncLib->getRdCost()->updateReshapeLumaLevelToWeightTable(m_pcReshaper->getSliceReshaperInfo(), m_pcReshaper->getWeightTable(), m_pcReshaper->getCWeight());
          }
        }
        else
        {
          THROW("Reshaper for other signal currently not defined!");
        }

        m_pcReshaper->setCTUFlag(false);

        //reshape original signal
        if (m_pcReshaper->getSliceReshaperInfo().getUseSliceReshaper())
        {
          pcPic->getOrigBuf(COMPONENT_Y).rspSignal(m_pcReshaper->getFwdLUT());
          m_pcReshaper->setSrcReshaped(true);
          m_pcReshaper->setRecReshaped(true);
        }
      }
      else
      {
        if (!m_pcReshaper->getReshapeFlag())
        {
          m_pcReshaper->setCTUFlag(false);
        }
        else
          m_pcReshaper->setCTUFlag(true);

        m_pcReshaper->getSliceReshaperInfo().setSliceReshapeModelPresentFlag(false);

        if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ)
        {
          m_pcEncLib->getRdCost()->restoreReshapeLumaLevelToWeightTable();
        }
        else if (m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_SDR)
        {
          int modIP = pcPic->getPOC() - pcPic->getPOC() / m_pcCfg->getReshapeCW().rspFpsToIp * m_pcCfg->getReshapeCW().rspFpsToIp;
          if (m_pcReshaper->getReshapeFlag() && m_pcCfg->getReshapeCW().rspIntraPeriod == -1 && modIP == 0)           // for LDB, update reshaping curve every second
          {
            m_pcReshaper->getSliceReshaperInfo().setSliceReshapeModelPresentFlag(true);
            m_pcReshaper->constructReshaperSDR();
            m_pcEncLib->getRdCost()->updateReshapeLumaLevelToWeightTable(m_pcReshaper->getSliceReshaperInfo(), m_pcReshaper->getWeightTable(), m_pcReshaper->getCWeight());
          }
        }
        else
        {
          THROW("Reshaper for other signal currently not defined!");
        }
      }

      m_pcReshaper->copySliceReshaperInfo(pcSlice->getReshapeInfo(), m_pcReshaper->getSliceReshaperInfo());
    }
    else
    {
      m_pcReshaper->setCTUFlag(false);
    }
#endif

    if( encPic )
    // now compress (trial encode) the various slice segments (slices, and dependent slices)
    {
      DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "poc", pocCurr ) ) );

      pcSlice->setSliceCurStartCtuTsAddr( 0 );
#if HEVC_DEPENDENT_SLICES
      pcSlice->setSliceSegmentCurStartCtuTsAddr( 0 );
#endif

      for(uint32_t nextCtuTsAddr = 0; nextCtuTsAddr < numberOfCtusInFrame; )
      {
        m_pcSliceEncoder->precompressSlice( pcPic );
        m_pcSliceEncoder->compressSlice   ( pcPic, false, false );

#if HEVC_DEPENDENT_SLICES
        const uint32_t curSliceSegmentEnd = pcSlice->getSliceSegmentCurEndCtuTsAddr();
        if (curSliceSegmentEnd < numberOfCtusInFrame)
        {
          const bool bNextSegmentIsDependentSlice = curSliceSegmentEnd < pcSlice->getSliceCurEndCtuTsAddr();
          const uint32_t sliceBits                    = pcSlice->getSliceBits();
          uint32_t independentSliceIdx                = pcSlice->getIndependentSliceIdx();
          pcPic->allocateNewSlice();
          // prepare for next slice
          m_pcSliceEncoder->setSliceSegmentIdx      ( uiNumSliceSegments   );
          pcSlice = pcPic->slices                   [ uiNumSliceSegments   ];
          VTMCHECK(!(pcSlice->getPPS()!=0), "Unspecified error");
          pcSlice->copySliceInfo                    ( pcPic->slices[uiNumSliceSegments-1]  );
          pcSlice->setSliceSegmentIdx               ( uiNumSliceSegments   );
          if (bNextSegmentIsDependentSlice)
          {
            pcSlice->setSliceBits(sliceBits);
          }
          else
          {
            pcSlice->setSliceCurStartCtuTsAddr      ( curSliceSegmentEnd );
            pcSlice->setSliceBits(0);
            independentSliceIdx ++;
          }
          pcSlice->setIndependentSliceIdx( independentSliceIdx );
          pcSlice->setDependentSliceSegmentFlag( bNextSegmentIsDependentSlice );
          pcSlice->setSliceSegmentCurStartCtuTsAddr ( curSliceSegmentEnd );
          // TODO: optimise cabac_init during compress slice to improve multi-slice operation
          // pcSlice->setEncCABACTableIdx(m_pcSliceEncoder->getEncCABACTableIdx());
          uiNumSliceSegments ++;
        }
        nextCtuTsAddr = curSliceSegmentEnd;
#else
        const uint32_t curSliceEnd = pcSlice->getSliceCurEndCtuTsAddr();
        if(curSliceEnd < numberOfCtusInFrame)
        {
          uint32_t independentSliceIdx = pcSlice->getIndependentSliceIdx();
          pcPic->allocateNewSlice();
          m_pcSliceEncoder->setSliceSegmentIdx      (uiNumSliceSegments);
          // prepare for next slice
          pcSlice = pcPic->slices[uiNumSliceSegments];
          VTMCHECK(!(pcSlice->getPPS() != 0), "Unspecified error");
          pcSlice->copySliceInfo(pcPic->slices[uiNumSliceSegments - 1]);
          pcSlice->setSliceCurStartCtuTsAddr(curSliceEnd);
          pcSlice->setSliceBits(0);
          independentSliceIdx++;
          pcSlice->setIndependentSliceIdx(independentSliceIdx);
          uiNumSliceSegments++;
        }
        nextCtuTsAddr = curSliceEnd;
#endif
      }

      duData.clear();

      CodingStructure& cs = *pcPic->cs;
      pcSlice = pcPic->slices[0];

#if JVET_M0427_INLOOP_RESHAPER
      if (pcSlice->getSPS()->getUseReshaper() && m_pcReshaper->getSliceReshaperInfo().getUseSliceReshaper())
      {
          VTMCHECK((m_pcReshaper->getRecReshaped() == false), "Rec picture is not reshaped!");
          pcPic->getRecoBuf(COMPONENT_Y).rspSignal(m_pcReshaper->getInvLUT());
          m_pcReshaper->setRecReshaped(false);

          pcPic->getOrigBuf().copyFrom(pcPic->getTrueOrigBuf());
      }
#endif

      // SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
      if( pcSlice->getSPS()->getSAOEnabledFlag() && m_pcCfg->getSaoCtuBoundary() )
      {
        m_pcSAO->getPreDBFStatistics( cs );
      }

      //-- Loop filter
      if ( m_pcCfg->getDeblockingFilterMetric() )
      {
  #if W0038_DB_OPT
        if ( m_pcCfg->getDeblockingFilterMetric()==2 )
        {
          applyDeblockingFilterParameterSelection(pcPic, uiNumSliceSegments, iGOPid);
        }
        else
        {
  #endif
          applyDeblockingFilterMetric(pcPic, uiNumSliceSegments);
  #if W0038_DB_OPT
        }
  #endif
      }

#if ADCNN
	  if (pcSlice->getSPS()->getCNNLFEnabledFlag())
	  {
		  CnnlfSliceParam cnnlfSliceParam;
		  m_pcCNNLF->initCABACEstimator(m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice);
		  m_pcCNNLF->CNNLFProcess(cs, pcSlice->getLambdas(), cnnlfSliceParam, pcSlice->getSliceQp());
		  // assign CNNLF slice header
		  for (int s = 0; s< uiNumSliceSegments; s++)
		  {
			  pcPic->slices[s]->setCnnlfSliceParam(cnnlfSliceParam);
		  }
	  }
#endif

      m_pcLoopFilter->loopFilterPic( cs );

#if JVET_M0147_DMVR
      CS::setRefinedMotionField(cs);
#endif
      DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "final", 1 ) ) );

      if( pcSlice->getSPS()->getSAOEnabledFlag() )
      {
        bool sliceEnabled[MAX_NUM_COMPONENT];
        m_pcSAO->initCABACEstimator( m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice );

        m_pcSAO->SAOProcess( cs, sliceEnabled, pcSlice->getLambdas(),
#if ENABLE_QPA
                             (m_pcCfg->getUsePerceptQPA() && !m_pcCfg->getUseRateCtrl() && pcSlice->getPPS()->getUseDQP() ? m_pcEncLib->getRdCost (PARL_PARAM0 (0))->getChromaWeight() : 0.0),
#endif
#if K0238_SAO_GREEDY_MERGE_ENCODING
                             m_pcCfg->getTestSAODisableAtPictureLevel(), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma(), m_pcCfg->getSaoCtuBoundary(), m_pcCfg->getSaoGreedyMergeEnc() );
#else
                             m_pcCfg->getTestSAODisableAtPictureLevel(), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma(), m_pcCfg->getSaoCtuBoundary() );
#endif
        //assign SAO slice header
        for(int s=0; s< uiNumSliceSegments; s++)
        {
          pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
          VTMCHECK(!(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]), "Unspecified error");
          pcPic->slices[s]->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);
        }
      }

      if( pcSlice->getSPS()->getALFEnabledFlag() )
      {
        AlfSliceParam alfSliceParam;
        m_pcALF->initCABACEstimator( m_pcEncLib->getCABACEncoder(), m_pcEncLib->getCtxCache(), pcSlice );

        m_pcALF->ALFProcess( cs, pcSlice->getLambdas(),
#if ENABLE_QPA
                             (m_pcCfg->getUsePerceptQPA() && !m_pcCfg->getUseRateCtrl() && pcSlice->getPPS()->getUseDQP() ? m_pcEncLib->getRdCost (PARL_PARAM0 (0))->getChromaWeight() : 0.0),
#endif
                             alfSliceParam );
        //assign ALF slice header
        for( int s = 0; s< uiNumSliceSegments; s++ )
        {
          pcPic->slices[s]->setAlfSliceParam( alfSliceParam );
        }
      }
      if (pcPic->cs->sps->getUseCompositeRef() && getPrepareLTRef())
      {
        updateCompositeReference(pcSlice, rcListPic, pocCurr);
      }
    }
    else // skip enc picture
    {
      pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

#if ENABLE_QPA
      if (m_pcCfg->getUsePerceptQPA() && !m_pcCfg->getUseRateCtrl() && pcSlice->getPPS()->getUseDQP())
      {
        const double picLambda = pcSlice->getLambdas()[0];

        for (uint32_t ctuRsAddr = 0; ctuRsAddr < numberOfCtusInFrame; ctuRsAddr++)
        {
          pcPic->m_uEnerHpCtu[ctuRsAddr] = picLambda;  // initialize to slice lambda (just for safety)
        }
      }
#endif
      if( pcSlice->getSPS()->getSAOEnabledFlag() )
      {
        m_pcSAO->disabledRate( *pcPic->cs, pcPic->getSAO(1), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma());
      }
    }

    if( m_pcCfg->getUseAMaxBT() )
    {
      for( const CodingUnit *cu : pcPic->cs->cus )
      {
        if( !pcSlice->isIRAP() )
        {
          m_uiBlkSize[pcSlice->getDepth()] += cu->Y().area();
          m_uiNumBlk [pcSlice->getDepth()]++;
        }
      }
    }

    if( encPic || decPic )
    {
      pcSlice = pcPic->slices[0];

      /////////////////////////////////////////////////////////////////////////////////////////////////// File writing

      // write various parameter sets
      actualTotalBits += xWriteParameterSets( accessUnit, pcSlice, m_bSeqFirst );

      if ( m_bSeqFirst )
      {
        // create prefix SEI messages at the beginning of the sequence
        VTMCHECK(!(leadingSeiMessages.empty()), "Unspecified error");
        xCreateIRAPLeadingSEIMessages(leadingSeiMessages, pcSlice->getSPS(), pcSlice->getPPS());

        m_bSeqFirst = false;
      }
      if (m_pcCfg->getAccessUnitDelimiter())
      {
        xWriteAccessUnitDelimiter(accessUnit, pcSlice);
      }

      // reset presence of BP SEI indication
      m_bufferingPeriodSEIPresentInAU = false;
      // create prefix SEI associated with a picture
      xCreatePerPictureSEIMessages(iGOPid, leadingSeiMessages, nestedSeiMessages, pcSlice);

      // pcSlice is currently slice 0.
      std::size_t binCountsInNalUnits   = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
      std::size_t numBytesInVclNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)

#if HEVC_DEPENDENT_SLICES
      for( uint32_t sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount=0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr=pcSlice->getSliceSegmentCurEndCtuTsAddr() )
#else
      for(uint32_t sliceSegmentStartCtuTsAddr = 0, sliceSegmentIdxCount = 0; sliceSegmentStartCtuTsAddr < numberOfCtusInFrame; sliceSegmentIdxCount++, sliceSegmentStartCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr())
#endif
      {
        pcSlice = pcPic->slices[sliceSegmentIdxCount];
        if(sliceSegmentIdxCount > 0 && pcSlice->getSliceType()!= I_SLICE)
        {
          pcSlice->checkColRefIdx(sliceSegmentIdxCount, pcPic);
        }
        m_pcSliceEncoder->setSliceSegmentIdx(sliceSegmentIdxCount);

        pcSlice->setRPS   (pcPic->slices[0]->getRPS());
        pcSlice->setRPSidx(pcPic->slices[0]->getRPSidx());

        for ( uint32_t ui = 0 ; ui < numSubstreams; ui++ )
        {
          substreamsOut[ui].clear();
        }

        /* start slice NALunit */
        OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer() );
        m_HLSWriter->setBitstream( &nalu.m_Bitstream );

        pcSlice->setNoRaslOutputFlag(false);
        if (pcSlice->isIRAP())
        {
          if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)
          {
            pcSlice->setNoRaslOutputFlag(true);
          }
          //the inference for NoOutputPriorPicsFlag
          // KJS: This cannot happen at the encoder
          if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())
          {
            if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)
            {
              pcSlice->setNoOutputPriorPicsFlag(true);
            }
          }
        }

        tmpBitsBeforeWriting = m_HLSWriter->getNumberOfWrittenBits();
        m_HLSWriter->codeSliceHeader( pcSlice );
        actualHeadBits += ( m_HLSWriter->getNumberOfWrittenBits() - tmpBitsBeforeWriting );

        pcSlice->setFinalized(true);

        pcSlice->clearSubstreamSizes(  );
        {
          uint32_t numBinsCoded = 0;
          m_pcSliceEncoder->encodeSlice(pcPic, &(substreamsOut[0]), numBinsCoded);
          binCountsInNalUnits+=numBinsCoded;
        }
        {
          // Construct the final bitstream by concatenating substreams.
          // The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
          // Complete the slice header info.
          m_HLSWriter->setBitstream( &nalu.m_Bitstream );
#if HEVC_TILES_WPP
          m_HLSWriter->codeTilesWPPEntryPoint( pcSlice );
#endif

          // Append substreams...
          OutputBitstream *pcOut = pcBitstreamRedirect;
#if HEVC_TILES_WPP
#if HEVC_DEPENDENT_SLICES

          const int numZeroSubstreamsAtStartOfSlice = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceSegmentCurStartCtuTsAddr(), false, pcSlice);
#else
          const int numZeroSubstreamsAtStartOfSlice  = pcPic->tileMap->getSubstreamForCtuAddr(pcSlice->getSliceCurStartCtuTsAddr(), false, pcSlice);
#endif
          const int numSubstreamsToCode  = pcSlice->getNumberOfSubstreamSizes()+1;
#else
          const int numZeroSubstreamsAtStartOfSlice  = 0;
          const int numSubstreamsToCode  = pcSlice->getNumberOfSubstreamSizes()+1;
#endif
          for ( uint32_t ui = 0 ; ui < numSubstreamsToCode; ui++ )
          {
            pcOut->addSubstream(&(substreamsOut[ui+numZeroSubstreamsAtStartOfSlice]));
          }
        }

        // If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
        // If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
        bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
        xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);
        accessUnit.push_back(new NALUnitEBSP(nalu));
        actualTotalBits += uint32_t(accessUnit.back()->m_nalUnitData.str().size()) * 8;
        numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());
        bNALUAlignedWrittenToList = true;

        if (!bNALUAlignedWrittenToList)
        {
          nalu.m_Bitstream.writeAlignZero();
          accessUnit.push_back(new NALUnitEBSP(nalu));
        }

        if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
            ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
            ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
           || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) &&
            ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag() ) )
        {
            uint32_t numNalus = 0;
          uint32_t numRBSPBytes = 0;
          for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
          {
            numRBSPBytes += uint32_t((*it)->m_nalUnitData.str().size());
            numNalus ++;
          }
          duData.push_back(DUData());
          duData.back().accumBitsDU = ( numRBSPBytes << 3 );
          duData.back().accumNalsDU = numNalus;
        }
      } // end iteration over slices


      // cabac_zero_words processing
      cabac_zero_word_padding(pcSlice, pcPic, binCountsInNalUnits, numBytesInVclNalUnits, accessUnit.back()->m_nalUnitData, m_pcCfg->getCabacZeroWordPaddingEnabled());

      //-- For time output for each slice
      auto elapsed = std::chrono::steady_clock::now() - beforeTime;
      auto encTime = std::chrono::duration_cast<std::chrono::seconds>( elapsed ).count();

      std::string digestStr;
      if (m_pcCfg->getDecodedPictureHashSEIType()!=HASHTYPE_NONE)
      {
        SEIDecodedPictureHash *decodedPictureHashSei = new SEIDecodedPictureHash();
        PelUnitBuf recoBuf = pcPic->cs->getRecoBuf();
        m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSei, recoBuf, digestStr, pcSlice->getSPS()->getBitDepths());
        trailingSeiMessages.push_back(decodedPictureHashSei);
      }

      m_pcCfg->setEncodedFlag(iGOPid, true);

      double PSNR_Y;
      xCalculateAddPSNRs(isField, isTff, iGOPid, pcPic, accessUnit, rcListPic, encTime, snr_conversion, printFrameMSE, &PSNR_Y
                       , isEncodeLtRef
      );

      // Only produce the Green Metadata SEI message with the last picture.
      if( m_pcCfg->getSEIGreenMetadataInfoSEIEnable() && pcSlice->getPOC() == ( m_pcCfg->getFramesToBeEncoded() - 1 )  )
      {
        SEIGreenMetadataInfo *seiGreenMetadataInfo = new SEIGreenMetadataInfo;
        m_seiEncoder.initSEIGreenMetadataInfo(seiGreenMetadataInfo, (uint32_t)(PSNR_Y * 100 + 0.5));
        trailingSeiMessages.push_back(seiGreenMetadataInfo);
      }

      xWriteTrailingSEIMessages(trailingSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS());

      printHash(m_pcCfg->getDecodedPictureHashSEIType(), digestStr);

      if ( m_pcCfg->getUseRateCtrl() )
      {
        double avgQP     = m_pcRateCtrl->getRCPic()->calAverageQP();
        double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
        if ( avgLambda < 0.0 )
        {
          avgLambda = lambda;
        }

        m_pcRateCtrl->getRCPic()->updateAfterPicture( actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->isIRAP());
        m_pcRateCtrl->getRCPic()->addToPictureLsit( m_pcRateCtrl->getPicList() );

        m_pcRateCtrl->getRCSeq()->updateAfterPic( actualTotalBits );
        if ( !pcSlice->isIRAP() )
        {
          m_pcRateCtrl->getRCGOP()->updateAfterPicture( actualTotalBits );
        }
        else    // for intra picture, the estimated bits are used to update the current status in the GOP
        {
          m_pcRateCtrl->getRCGOP()->updateAfterPicture( estimatedBits );
        }
  #if U0132_TARGET_BITS_SATURATION
        if (m_pcRateCtrl->getCpbSaturationEnabled())
        {
          m_pcRateCtrl->updateCpbState(actualTotalBits);
          msg( NOTICE, " [CPB %6d bits]", m_pcRateCtrl->getCpbState() );
        }
  #endif
      }

      xCreatePictureTimingSEI( m_pcCfg->getEfficientFieldIRAPEnabled() ? effFieldIRAPMap.GetIRAPGOPid() : 0, leadingSeiMessages, nestedSeiMessages, duInfoSeiMessages, pcSlice, isField, duData );
      if( m_pcCfg->getScalableNestingSEIEnabled() )
      {
        xCreateScalableNestingSEI( leadingSeiMessages, nestedSeiMessages );
      }
      xWriteLeadingSEIMessages( leadingSeiMessages, duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData );
      xWriteDuSEIMessages( duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData );

      m_AUWriterIf->outputAU( accessUnit );

      msg( NOTICE, "\n" );
      fflush( stdout );
    }


    DTRACE_UPDATE( g_trace_ctx, ( std::make_pair( "final", 0 ) ) );

    pcPic->reconstructed = true;
#if JVET_M0483_IBC ==0
    pcPic->longTerm = false;
#endif
    m_bFirst = false;
    m_iNumPicCoded++;
    if (!(pcPic->cs->sps->getUseCompositeRef() && isEncodeLtRef))
      m_totalCoded ++;
    /* logging: insert a newline at end of picture period */

    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      iGOPid=effFieldIRAPMap.restoreGOPid(iGOPid);
    }

    pcPic->destroyTempBuffers();
    pcPic->cs->destroyCoeffs();
    pcPic->cs->releaseIntermediateData();
  } // iGOPid-loop

  delete pcBitstreamRedirect;

  VTMCHECK(!( (m_iNumPicCoded == iNumPicRcvd) ), "Unspecified error");

}

void EncGOP::printOutSummary(uint32_t uiNumAllPicCoded, bool isField, const bool printMSEBasedSNR, const bool printSequenceMSE, const bool printHexPsnr, const BitDepths &bitDepths)
{
#if ENABLE_QPA
  const bool    useWPSNR = m_pcEncLib->getUseWPSNR();
#endif
#if WCG_WPSNR
#if JVET_M0427_INLOOP_RESHAPER
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcCfg->getReshaper() && m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ);
#else
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled();
#endif
#endif

  if( m_pcCfg->getDecodeBitstream(0).empty() && m_pcCfg->getDecodeBitstream(1).empty() && !m_pcCfg->useFastForwardToPOC() )
  {
    VTMCHECK( !( uiNumAllPicCoded == m_gcAnalyzeAll.getNumPic() ), "Unspecified error" );
  }

  //--CFG_KDY
  const int rateMultiplier=(isField?2:1);
  m_gcAnalyzeAll.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
  m_gcAnalyzeI.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
  m_gcAnalyzeP.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
  m_gcAnalyzeB.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.setFrmRate(m_pcCfg->getFrameRate()*rateMultiplier / (double)m_pcCfg->getTemporalSubsampleRatio());
  }
#endif

  const ChromaFormat chFmt = m_pcCfg->getChromaFormatIdc();

  //-- all
  msg( VTMINFO, "\n" );
  msg( DETAILS,"\nSUMMARY --------------------------------------------------------\n" );
#if ENABLE_QPA
#if SUMMARY_OUT
  m_gcAnalyzeAll.printOut_summary('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths, useWPSNR);
#else
  m_gcAnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths, useWPSNR);
#endif
#else
#if SUMMARY_OUT
  _gcAnalyzeAll.printOut_summary('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);
#else
  m_gcAnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);
#endif
#endif
  msg( DETAILS,"\n\nI Slices--------------------------------------------------------\n" );
  m_gcAnalyzeI.printOut('i', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);

  msg( DETAILS,"\n\nP Slices--------------------------------------------------------\n" );
  m_gcAnalyzeP.printOut('p', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);

  msg( DETAILS,"\n\nB Slices--------------------------------------------------------\n" );
  m_gcAnalyzeB.printOut('b', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);

#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    msg(DETAILS, "\nWPSNR SUMMARY --------------------------------------------------------\n");
    m_gcAnalyzeWPSNR.printOut('w', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths, useLumaWPSNR);
  }
#endif
  if (!m_pcCfg->getSummaryOutFilename().empty())
  {
    m_gcAnalyzeAll.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryOutFilename());
  }

  if (!m_pcCfg->getSummaryPicFilenameBase().empty())
  {
    m_gcAnalyzeI.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"I.txt");
    m_gcAnalyzeP.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"P.txt");
    m_gcAnalyzeB.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"B.txt");
  }

#if WCG_WPSNR
  if (!m_pcCfg->getSummaryOutFilename().empty() && useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryOutFilename());
  }
#endif
  if(isField)
  {
    //-- interlaced summary
    m_gcAnalyzeAll_in.setFrmRate( m_pcCfg->getFrameRate() / (double)m_pcCfg->getTemporalSubsampleRatio());
    m_gcAnalyzeAll_in.setBits(m_gcAnalyzeAll.getBits());
    // prior to the above statement, the interlace analyser does not contain the correct total number of bits.

    msg( DETAILS,"\n\nSUMMARY INTERLACED ---------------------------------------------\n" );
#if ENABLE_QPA
#if SUMMARY_OUT
	m_gcAnalyzeAll_in.printOut_summary('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths, useWPSNR);
#else
    m_gcAnalyzeAll_in.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths, useWPSNR);
#endif
#else
#if SUMMARY_OUT
	m_gcAnalyzeAll_in.printOut_summary('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);
#else
	m_gcAnalyzeAll_in.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);
#endif
#endif
    if (!m_pcCfg->getSummaryOutFilename().empty())
    {
      m_gcAnalyzeAll_in.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryOutFilename());
#if WCG_WPSNR
      if (useLumaWPSNR)
      {
        m_gcAnalyzeWPSNR.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, m_pcCfg->getSummaryOutFilename());
      }
#endif
    }
  }

  msg( DETAILS,"\nRVM: %.3lf\n", xCalculateRVM() );
}

#if W0038_DB_OPT
uint64_t EncGOP::preLoopFilterPicAndCalcDist( Picture* pcPic )
{
  CodingStructure& cs = *pcPic->cs;
  m_pcLoopFilter->loopFilterPic( cs );

  const CPelUnitBuf picOrg = pcPic->getRecoBuf();
  const CPelUnitBuf picRec = cs.getRecoBuf();

  uint64_t uiDist = 0;
  for( uint32_t comp = 0; comp < (uint32_t)picRec.bufs.size(); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    const uint32_t rshift = 2 * DISTORTION_PRECISION_ADJUSTMENT(cs.sps->getBitDepth(toChannelType(compID)));
#if ENABLE_QPA
    VTMCHECK( rshift >= 8, "shifts greater than 7 are not supported." );
#endif
    uiDist += xFindDistortionPlane( picOrg.get(compID), picRec.get(compID), rshift );
  }
  return uiDist;
}
#endif

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
void EncGOP::xInitGOP( int iPOCLast, int iNumPicRcvd, bool isField
  , bool isEncodeLtRef
)
{
  VTMCHECK(!( iNumPicRcvd > 0 ), "Unspecified error");
  //  Exception for the first frames
  if ((isField && (iPOCLast == 0 || iPOCLast == 1)) || (!isField && (iPOCLast == 0)) || isEncodeLtRef)
  {
    m_iGopSize    = 1;
  }
  else
  {
    m_iGopSize    = m_pcCfg->getGOPSize();
  }
  VTMCHECK(!(m_iGopSize > 0), "Unspecified error");

  return;
}


void EncGOP::xGetBuffer( PicList&                  rcListPic,
                         std::list<PelUnitBuf*>&   rcListPicYuvRecOut,
                         int                       iNumPicRcvd,
                         int                       iTimeOffset,
                         Picture*&                 rpcPic,
                         int                       pocCurr,
                         bool                      isField )
{
  int i;
  //  Rec. output
  std::list<PelUnitBuf*>::iterator     iterPicYuvRec = rcListPicYuvRecOut.end();

  if (isField && pocCurr > 1 && m_iGopSize!=1)
  {
    iTimeOffset--;
  }

  int multipleFactor = m_pcCfg->getUseCompositeRef() ? 2 : 1;
  for (i = 0; i < (iNumPicRcvd * multipleFactor - iTimeOffset + 1); i += multipleFactor)
  {
    iterPicYuvRec--;
  }

  //  Current pic.
  PicList::iterator        iterPic       = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    rpcPic = *(iterPic);
    if (rpcPic->getPOC() == pocCurr)
    {
      break;
    }
    iterPic++;
  }

  VTMCHECK(!(rpcPic != NULL), "Unspecified error");
  VTMCHECK(!(rpcPic->getPOC() == pocCurr), "Unspecified error");

  (**iterPicYuvRec) = rpcPic->getRecoBuf();
  return;
}

#if ENABLE_QPA

#ifndef BETA
  #define BETA 0.5 // value between 0.0 and 1; use 0.0 to obtain traditional PSNR
#endif

static inline double calcWeightedSquaredError(const CPelBuf& org,        const CPelBuf& rec,
                                              double &sumAct,            const uint32_t bitDepth,
                                              const uint32_t imageWidth, const uint32_t imageHeight,
                                              const uint32_t offsetX,    const uint32_t offsetY,
                                              int blockWidth,            int blockHeight)
{
  const int    O = org.stride;
  const int    R = rec.stride;
  const Pel   *o = org.bufAt(offsetX, offsetY);
  const Pel   *r = rec.bufAt(offsetX, offsetY);
  const int yAct = offsetY > 0 ? 0 : 1;
  const int xAct = offsetX > 0 ? 0 : 1;

  if (offsetY + (uint32_t)blockHeight > imageHeight) blockHeight = imageHeight - offsetY;
  if (offsetX + (uint32_t)blockWidth  > imageWidth ) blockWidth  = imageWidth  - offsetX;

  const int hAct = offsetY + (uint32_t)blockHeight < imageHeight ? blockHeight : blockHeight - 1;
  const int wAct = offsetX + (uint32_t)blockWidth  < imageWidth  ? blockWidth  : blockWidth  - 1;
  uint64_t ssErr = 0; // sum of squared diffs
  uint64_t saAct = 0; // sum of abs. activity
  double msAct;
  int x, y;

  // calculate image differences and activity
  for (y = 0; y < blockHeight; y++)  // error
  {
    for (x = 0; x < blockWidth; x++)
    {
      const     int64_t iDiff = (int64_t)o[y*O + x] - (int64_t)r[y*R + x];
      ssErr += uint64_t(iDiff * iDiff);
    }
  }
  if (wAct <= xAct || hAct <= yAct) return (double)ssErr;

  for (y = yAct; y < hAct; y++)   // activity
  {
    for (x = xAct; x < wAct; x++)
    {
      const int f = 12 * (int)o[y*O + x] - 2 * ((int)o[y*O + x-1] + (int)o[y*O + x+1] + (int)o[(y-1)*O + x] + (int)o[(y+1)*O + x])
                       - (int)o[(y-1)*O + x-1] - (int)o[(y-1)*O + x+1] - (int)o[(y+1)*O + x-1] - (int)o[(y+1)*O + x+1];
      saAct += abs(f);
    }
  }

  // calculate weight (mean squared activity)
  msAct = (double)saAct / (double(wAct - xAct) * double(hAct - yAct));

  // lower limit, accounts for high-pass gain
  if (msAct < double(1 << (bitDepth - 4))) msAct = double(1 << (bitDepth - 4));

  msAct *= msAct; // because ssErr is squared

  sumAct += msAct; // includes high-pass gain

  // calculate activity weighted error square
  return (double)ssErr * pow(msAct, -1.0 * BETA);
}
#endif // ENABLE_QPA

uint64_t EncGOP::xFindDistortionPlane(const CPelBuf& pic0, const CPelBuf& pic1, const uint32_t rshift
#if ENABLE_QPA
                                    , const uint32_t chromaShift /*= 0*/
#endif
                                      )
{
  uint64_t uiTotalDiff;
  const  Pel*  pSrc0 = pic0.bufAt(0, 0);
  const  Pel*  pSrc1 = pic1.bufAt(0, 0);

  VTMCHECK(pic0.width  != pic1.width , "Unspecified error");
  VTMCHECK(pic0.height != pic1.height, "Unspecified error");

  if( rshift > 0 )
  {
#if ENABLE_QPA
    const   uint32_t  BD = rshift;      // image bit-depth
    if (BD >= 8)
    {
      const uint32_t   W = pic0.width;  // image width
      const uint32_t   H = pic0.height; // image height
      const double     R = double(W * H) / (1920.0 * 1080.0);
      const uint32_t   B = Clip3<uint32_t>(0, 128 >> chromaShift, 4 * uint32_t(16.0 * sqrt(R) + 0.5)); // WPSNR block size in integer multiple of 4 (for SIMD, = 64 at full-HD)

      uint32_t x, y;

      if (B < 4) // image is too small to use WPSNR, resort to traditional PSNR
      {
        uiTotalDiff = 0;
        for (y = 0; y < H; y++)
        {
          for (x = 0; x < W; x++)
          {
            const           int64_t iDiff = (int64_t)pSrc0[x] - (int64_t)pSrc1[x];
            uiTotalDiff += uint64_t(iDiff * iDiff);
          }
          pSrc0 += pic0.stride;
          pSrc1 += pic1.stride;
        }
        return uiTotalDiff;
      }

      double wmse = 0.0, sumAct = 0.0; // compute activity normalized SNR value

      for (y = 0; y < H; y += B)
      {
        for (x = 0; x < W; x += B)
        {
          wmse += calcWeightedSquaredError(pic1,   pic0,
                                           sumAct, BD,
                                           W,      H,
                                           x,      y,
                                           B,      B);
        }
      }

      // integer weighted distortion
      sumAct = 16.0 * sqrt ((3840.0 * 2160.0) / double((W << chromaShift) * (H << chromaShift))) * double(1 << BD);

      return (wmse <= 0.0) ? 0 : uint64_t(wmse * pow(sumAct, BETA) + 0.5);
    }
#endif // ENABLE_QPA
    uiTotalDiff = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += uint64_t((iTemp * iTemp) >> rshift);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }
  else
  {
    uiTotalDiff = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += uint64_t(iTemp * iTemp);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }

  return uiTotalDiff;
}
#if WCG_WPSNR
double EncGOP::xFindDistortionPlaneWPSNR(const CPelBuf& pic0, const CPelBuf& pic1, const uint32_t rshift, const CPelBuf& picLuma0,
  ComponentID compID, const ChromaFormat chfmt    )
{
#if JVET_M0427_INLOOP_RESHAPER
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcCfg->getReshaper() && m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ);
#else
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled();
#endif
  if (!useLumaWPSNR)
  {
    return 0;
  }

  double uiTotalDiffWPSNR;
  const  Pel*  pSrc0 = pic0.bufAt(0, 0);
  const  Pel*  pSrc1 = pic1.bufAt(0, 0);
  const  Pel*  pSrcLuma = picLuma0.bufAt(0, 0);
  VTMCHECK(pic0.width  != pic1.width , "Unspecified error");
  VTMCHECK(pic0.height != pic1.height, "Unspecified error");

  if( rshift > 0 )
  {
    uiTotalDiffWPSNR = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        double dW = m_pcEncLib->getRdCost()->getWPSNRLumaLevelWeight(pSrcLuma[(x << getComponentScaleX(compID, chfmt))]);
        uiTotalDiffWPSNR += ((dW * (double)iTemp * (double)iTemp)) * (double)(1 >> rshift);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
      pSrcLuma += picLuma0.stride << getComponentScaleY(compID, chfmt);
    }
  }
  else
  {
    uiTotalDiffWPSNR = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        double dW = m_pcEncLib->getRdCost()->getWPSNRLumaLevelWeight(pSrcLuma[x << getComponentScaleX(compID, chfmt)]);
        uiTotalDiffWPSNR += dW * (double)iTemp * (double)iTemp;
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
      pSrcLuma += picLuma0.stride << getComponentScaleY(compID, chfmt);
    }
  }

  return uiTotalDiffWPSNR;
}
#endif

void EncGOP::xCalculateAddPSNRs( const bool isField, const bool isFieldTopFieldFirst, const int iGOPid, Picture* pcPic, const AccessUnit&accessUnit, PicList &rcListPic, const int64_t dEncTime, const InputColourSpaceConversion snr_conversion, const bool printFrameMSE, double* PSNR_Y
                               , bool isEncodeLtRef
)
{
  xCalculateAddPSNR(pcPic, pcPic->getRecoBuf(), accessUnit, (double)dEncTime, snr_conversion, printFrameMSE, PSNR_Y
                  , isEncodeLtRef
  );

  //In case of field coding, compute the interlaced PSNR for both fields
  if(isField)
  {
    bool bothFieldsAreEncoded = false;
    int correspondingFieldPOC = pcPic->getPOC();
    int currentPicGOPPoc = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    if(pcPic->getPOC() == 0)
    {
      // particular case for POC 0 and 1.
      // If they are not encoded first and separately from other pictures, we need to change this
      // POC 0 is always encoded first then POC 1 is encoded
      bothFieldsAreEncoded = false;
    }
    else if(pcPic->getPOC() == 1)
    {
      // if we are at POC 1, POC 0 has been encoded for sure
      correspondingFieldPOC = 0;
      bothFieldsAreEncoded = true;
    }
    else
    {
      if(pcPic->getPOC()%2 == 1)
      {
        correspondingFieldPOC -= 1; // all odd POC are associated with the preceding even POC (e.g poc 1 is associated to poc 0)
        currentPicGOPPoc      -= 1;
      }
      else
      {
        correspondingFieldPOC += 1; // all even POC are associated with the following odd POC (e.g poc 0 is associated to poc 1)
        currentPicGOPPoc      += 1;
      }
      for(int i = 0; i < m_iGopSize; i ++)
      {
        if(m_pcCfg->getGOPEntry(i).m_POC == currentPicGOPPoc)
        {
          bothFieldsAreEncoded = m_pcCfg->getGOPEntry(i).m_isEncoded;
          break;
        }
      }
    }

    if(bothFieldsAreEncoded)
    {
      //get complementary top field
      PicList::iterator   iterPic = rcListPic.begin();
      while ((*iterPic)->getPOC() != correspondingFieldPOC)
      {
        iterPic ++;
      }
      Picture* correspondingFieldPic = *(iterPic);

      if ((pcPic->topField && isFieldTopFieldFirst) || (!pcPic->topField && !isFieldTopFieldFirst))
      {
        xCalculateInterlacedAddPSNR(pcPic, correspondingFieldPic, pcPic->getRecoBuf(), correspondingFieldPic->getRecoBuf(), snr_conversion, printFrameMSE, PSNR_Y
          , isEncodeLtRef
        );
      }
      else
      {
        xCalculateInterlacedAddPSNR(correspondingFieldPic, pcPic, correspondingFieldPic->getRecoBuf(), pcPic->getRecoBuf(), snr_conversion, printFrameMSE, PSNR_Y
          , isEncodeLtRef
        );
      }
    }
  }
}

void EncGOP::xCalculateAddPSNR(Picture* pcPic, PelUnitBuf cPicD, const AccessUnit& accessUnit, double dEncTime, const InputColourSpaceConversion conversion, const bool printFrameMSE, double* PSNR_Y
                              , bool isEncodeLtRef
)
{
  const SPS&         sps = *pcPic->cs->sps;
  const CPelUnitBuf& pic = cPicD;
  VTMCHECK(!(conversion == IPCOLOURSPACE_UNCHANGED), "Unspecified error");
//  const CPelUnitBuf& org = (conversion != IPCOLOURSPACE_UNCHANGED) ? pcPic->getPicYuvTrueOrg()->getBuf() : pcPic->getPicYuvOrg()->getBuf();
#if JVET_M0427_INLOOP_RESHAPER
  const CPelUnitBuf& org = sps.getUseReshaper() ? pcPic->getTrueOrigBuf() : pcPic->getOrigBuf();
#else
  const CPelUnitBuf& org = pcPic->getOrigBuf();
#endif
#if ENABLE_QPA
  const bool    useWPSNR = m_pcEncLib->getUseWPSNR();
#endif
  double  dPSNR[MAX_NUM_COMPONENT];
#if WCG_WPSNR
#if JVET_M0427_INLOOP_RESHAPER
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcCfg->getReshaper() && m_pcCfg->getReshapeSignalType() == RESHAPE_SIGNAL_PQ);
#else
  const bool    useLumaWPSNR = m_pcEncLib->getLumaLevelToDeltaQPMapping().isEnabled();
#endif
  double  dPSNRWeighted[MAX_NUM_COMPONENT];
  double  MSEyuvframeWeighted[MAX_NUM_COMPONENT];
#endif
  for(int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;
#if WCG_WPSNR
    dPSNRWeighted[i]=0.0;
    MSEyuvframeWeighted[i] = 0.0;
#endif
  }

  PelStorage interm;

  if (conversion != IPCOLOURSPACE_UNCHANGED)
  {
    interm.create(pic.chromaFormat, Area(Position(), pic.Y()));
    VideoIOYuv::ColourSpaceConvert(pic, interm, conversion, false);
  }

  const CPelUnitBuf& picC = (conversion == IPCOLOURSPACE_UNCHANGED) ? pic : interm;

  //===== calculate PSNR =====
  double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};
  const ChromaFormat formatD = pic.chromaFormat;
  const ChromaFormat format  = sps.getChromaFormatIdc();

  const bool bPicIsField     = pcPic->fieldPic;
  const Slice*  pcSlice      = pcPic->slices[0];

  for (int comp = 0; comp < ::getNumberValidComponents(formatD); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    const CPelBuf&    p = picC.get(compID);
    const CPelBuf&    o = org.get(compID);

    VTMCHECK(!( p.width  == o.width), "Unspecified error");
    VTMCHECK(!( p.height == o.height), "Unspecified error");

    const uint32_t   width  = p.width  - (m_pcEncLib->getPad(0) >> ::getComponentScaleX(compID, format));
    const uint32_t   height = p.height - (m_pcEncLib->getPad(1) >> (!!bPicIsField+::getComponentScaleY(compID,format)));

    // create new buffers with correct dimensions
    const CPelBuf recPB(p.bufAt(0, 0), p.stride, width, height);
    const CPelBuf orgPB(o.bufAt(0, 0), o.stride, width, height);
    const uint32_t    bitDepth = sps.getBitDepth(toChannelType(compID));
#if ENABLE_QPA
    const uint64_t uiSSDtemp = xFindDistortionPlane(recPB, orgPB, useWPSNR ? bitDepth : 0, ::getComponentScaleX(compID, format));
#else
    const uint64_t uiSSDtemp = xFindDistortionPlane(recPB, orgPB, 0);
#endif
    const uint32_t maxval = 255 << (bitDepth - 8);
    const uint32_t size   = width * height;
    const double fRefValue = (double)maxval * maxval * size;
    dPSNR[comp]       = uiSSDtemp ? 10.0 * log10(fRefValue / (double)uiSSDtemp) : 999.99;
    MSEyuvframe[comp] = (double)uiSSDtemp / size;
#if WCG_WPSNR
    const double uiSSDtempWeighted = xFindDistortionPlaneWPSNR(recPB, orgPB, 0, org.get(COMPONENT_Y), compID, format);
    if (useLumaWPSNR)
    {
      dPSNRWeighted[comp] = uiSSDtempWeighted ? 10.0 * log10(fRefValue / (double)uiSSDtempWeighted) : 999.99;
      MSEyuvframeWeighted[comp] = (double)uiSSDtempWeighted / size;
    }
#endif
  }

#if EXTENSION_360_VIDEO
  m_ext360.calculatePSNRs(pcPic);
#endif

  /* calculate the size of the access unit, excluding:
   *  - any AnnexB contributions (start_code_prefix, zero_byte, etc.,)
   *  - SEI NAL units
   */
  uint32_t numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
  {
    uint32_t numRBSPBytes_nal = uint32_t((*it)->m_nalUnitData.str().size());
    if (m_pcCfg->getSummaryVerboseness() > 0)
    {
      msg( NOTICE, "*** %6s numBytesInNALunit: %u\n", nalUnitTypeToString((*it)->m_nalUnitType), numRBSPBytes_nal);
    }
    if( ( *it )->m_nalUnitType != NAL_UNIT_PREFIX_SEI && ( *it )->m_nalUnitType != NAL_UNIT_SUFFIX_SEI )
    {
      numRBSPBytes += numRBSPBytes_nal;
#if HEVC_VPS
      if( it == accessUnit.begin() || ( *it )->m_nalUnitType == NAL_UNIT_VPS || ( *it )->m_nalUnitType == NAL_UNIT_SPS || ( *it )->m_nalUnitType == NAL_UNIT_PPS )
#else
      if (it == accessUnit.begin() || (*it)->m_nalUnitType == NAL_UNIT_SPS || (*it)->m_nalUnitType == NAL_UNIT_PPS)
#endif
      {
        numRBSPBytes += 4;
      }
      else
      {
        numRBSPBytes += 3;
      }
    }
  }

  uint32_t uibits = numRBSPBytes * 8;
  m_vRVM_RP.push_back( uibits );

  //===== add PSNR =====
  m_gcAnalyzeAll.addResult(dPSNR, (double)uibits, MSEyuvframe
    , isEncodeLtRef
  );
#if EXTENSION_360_VIDEO
  m_ext360.addResult(m_gcAnalyzeAll);
#endif
  if (pcSlice->isIntra())
  {
    m_gcAnalyzeI.addResult(dPSNR, (double)uibits, MSEyuvframe
      , isEncodeLtRef
    );
    *PSNR_Y = dPSNR[COMPONENT_Y];
#if EXTENSION_360_VIDEO
    m_ext360.addResult(m_gcAnalyzeI);
#endif
  }
  if (pcSlice->isInterP())
  {
    m_gcAnalyzeP.addResult(dPSNR, (double)uibits, MSEyuvframe
      , isEncodeLtRef
    );
    *PSNR_Y = dPSNR[COMPONENT_Y];
#if EXTENSION_360_VIDEO
    m_ext360.addResult(m_gcAnalyzeP);
#endif
  }
  if (pcSlice->isInterB())
  {
    m_gcAnalyzeB.addResult(dPSNR, (double)uibits, MSEyuvframe
      , isEncodeLtRef
    );
    *PSNR_Y = dPSNR[COMPONENT_Y];
#if EXTENSION_360_VIDEO
    m_ext360.addResult(m_gcAnalyzeB);
#endif
  }
#if WCG_WPSNR
  if (useLumaWPSNR)
  {
    m_gcAnalyzeWPSNR.addResult(dPSNRWeighted, (double)uibits, MSEyuvframeWeighted, isEncodeLtRef);
  }
#endif

  char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');
  if (! pcPic->referenced)
  {
    c += 32;
  }

  if( g_verbosity >= NOTICE )
  {
    msg( NOTICE, "POC %4d TId: %1d ( %c-SLICE, QP %d ) %10d bits",
         pcSlice->getPOC() - pcSlice->getLastIDR(),
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQp(),
         uibits );

    msg( NOTICE, " [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );

#if EXTENSION_360_VIDEO
    m_ext360.printPerPOCInfo(NOTICE);
#endif

    if (m_pcEncLib->getPrintHexPsnr())
    {
      uint64_t xPsnr[MAX_NUM_COMPONENT];
      for (int i = 0; i < MAX_NUM_COMPONENT; i++)
      {
        copy(reinterpret_cast<uint8_t *>(&dPSNR[i]),
             reinterpret_cast<uint8_t *>(&dPSNR[i]) + sizeof(dPSNR[i]),
             reinterpret_cast<uint8_t *>(&xPsnr[i]));
      }
      msg(NOTICE, " [xY %16" PRIx64 " xU %16" PRIx64 " xV %16" PRIx64 "]", xPsnr[COMPONENT_Y], xPsnr[COMPONENT_Cb], xPsnr[COMPONENT_Cr]);

#if EXTENSION_360_VIDEO
      m_ext360.printPerPOCInfo(NOTICE, true);
#endif
    }

    if( printFrameMSE )
    {
      msg( NOTICE, " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );
    }
#if WCG_WPSNR
    if (useLumaWPSNR)
    {
      msg(NOTICE, " [WY %6.4lf dB    WU %6.4lf dB    WV %6.4lf dB]", dPSNRWeighted[COMPONENT_Y], dPSNRWeighted[COMPONENT_Cb], dPSNRWeighted[COMPONENT_Cr]);
    }
#endif
    msg( NOTICE, " [ET %5.0f ]", dEncTime );

    // msg( SOME, " [WP %d]", pcSlice->getUseWeightedPrediction());

    for( int iRefList = 0; iRefList < 2; iRefList++ )
    {
      msg( NOTICE, " [L%d ", iRefList );
      for( int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx( RefPicList( iRefList ) ); iRefIndex++ )
      {
        msg( NOTICE, "%d ", pcSlice->getRefPOC( RefPicList( iRefList ), iRefIndex ) - pcSlice->getLastIDR() );
      }
      msg( NOTICE, "]" );
    }
  }
  else if( g_verbosity >= VTMINFO )
  {
    std::cout << "\r\t" << pcSlice->getPOC();
    std::cout.flush();
  }
}

void EncGOP::xCalculateInterlacedAddPSNR( Picture* pcPicOrgFirstField, Picture* pcPicOrgSecondField,
                                          PelUnitBuf cPicRecFirstField, PelUnitBuf cPicRecSecondField,
                                          const InputColourSpaceConversion conversion, const bool printFrameMSE, double* PSNR_Y
                                        , bool isEncodeLtRef
)
{
  const SPS &sps = *pcPicOrgFirstField->cs->sps;
  const ChromaFormat format = sps.getChromaFormatIdc();
  double  dPSNR[MAX_NUM_COMPONENT];
  Picture    *apcPicOrgFields[2] = {pcPicOrgFirstField, pcPicOrgSecondField};
  PelUnitBuf acPicRecFields[2]   = {cPicRecFirstField, cPicRecSecondField};
#if ENABLE_QPA
  const bool    useWPSNR = m_pcEncLib->getUseWPSNR();
#endif
  for(int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;
  }

  PelStorage cscd[2 /* first/second field */];
  if (conversion!=IPCOLOURSPACE_UNCHANGED)
  {
    for(uint32_t fieldNum=0; fieldNum<2; fieldNum++)
    {
      PelUnitBuf& reconField= (acPicRecFields[fieldNum]);
      cscd[fieldNum].create( reconField.chromaFormat, Area( Position(), reconField.Y()) );
      VideoIOYuv::ColourSpaceConvert(reconField, cscd[fieldNum], conversion, false);
      acPicRecFields[fieldNum]=cscd[fieldNum];
    }
  }

  //===== calculate PSNR =====
  double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};

  VTMCHECK(!(acPicRecFields[0].chromaFormat==acPicRecFields[1].chromaFormat), "Unspecified error");
  const uint32_t numValidComponents = ::getNumberValidComponents( acPicRecFields[0].chromaFormat );

  for (int chan = 0; chan < numValidComponents; chan++)
  {
    const ComponentID ch=ComponentID(chan);
    VTMCHECK(!(acPicRecFields[0].get(ch).width==acPicRecFields[1].get(ch).width), "Unspecified error");
    VTMCHECK(!(acPicRecFields[0].get(ch).height==acPicRecFields[0].get(ch).height), "Unspecified error");

    uint64_t uiSSDtemp=0;
    const uint32_t width    = acPicRecFields[0].get(ch).width - (m_pcEncLib->getPad(0) >> ::getComponentScaleX(ch, format));
    const uint32_t height   = acPicRecFields[0].get(ch).height - ((m_pcEncLib->getPad(1) >> 1) >> ::getComponentScaleY(ch, format));
    const uint32_t bitDepth = sps.getBitDepth(toChannelType(ch));

    for(uint32_t fieldNum=0; fieldNum<2; fieldNum++)
    {
      VTMCHECK(!(conversion == IPCOLOURSPACE_UNCHANGED), "Unspecified error");
#if ENABLE_QPA
      uiSSDtemp += xFindDistortionPlane( acPicRecFields[fieldNum].get(ch), apcPicOrgFields[fieldNum]->getOrigBuf().get(ch), useWPSNR ? bitDepth : 0, ::getComponentScaleX(ch, format) );
#else
      uiSSDtemp += xFindDistortionPlane( acPicRecFields[fieldNum].get(ch), apcPicOrgFields[fieldNum]->getOrigBuf().get(ch), 0 );
#endif
    }
    const uint32_t maxval = 255 << (bitDepth - 8);
    const uint32_t size   = width * height * 2;
    const double fRefValue = (double)maxval * maxval * size;
    dPSNR[ch]         = uiSSDtemp ? 10.0 * log10(fRefValue / (double)uiSSDtemp) : 999.99;
    MSEyuvframe[ch]   = (double)uiSSDtemp / size;
  }

  uint32_t uibits = 0; // the number of bits for the pair is not calculated here - instead the overall total is used elsewhere.

  //===== add PSNR =====
  m_gcAnalyzeAll_in.addResult (dPSNR, (double)uibits, MSEyuvframe
    , isEncodeLtRef
  );

  *PSNR_Y = dPSNR[COMPONENT_Y];

  msg( DETAILS, "\n                                      Interlaced frame %d: [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", pcPicOrgSecondField->getPOC()/2 , dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );
  if (printFrameMSE)
  {
    msg( DETAILS, " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );
  }

  for(uint32_t fieldNum=0; fieldNum<2; fieldNum++)
  {
    cscd[fieldNum].destroy();
  }
}

/** Function for deciding the nal_unit_type.
 * \param pocCurr POC of the current picture
 * \param lastIDR  POC of the last IDR picture
 * \param isField  true to indicate field coding
 * \returns the NAL unit type of the picture
 * This function checks the configuration and returns the appropriate nal_unit_type for the picture.
 */
NalUnitType EncGOP::getNalUnitType(int pocCurr, int lastIDR, bool isField)
{
  if (pocCurr == 0)
  {
    return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
  }

  if (m_pcCfg->getEfficientFieldIRAPEnabled() && isField && pocCurr == (m_pcCfg->getUseCompositeRef() ? 2: 1))
  {
    // to avoid the picture becoming an IRAP
    return NAL_UNIT_CODED_SLICE_TRAIL_R;
  }

  if (m_pcCfg->getDecodingRefreshType() != 3 && (pocCurr - isField) % (m_pcCfg->getIntraPeriod() * (m_pcCfg->getUseCompositeRef() ? 2 : 1)) == 0)
  {
    if (m_pcCfg->getDecodingRefreshType() == 1)
    {
      return NAL_UNIT_CODED_SLICE_CRA;
    }
    else if (m_pcCfg->getDecodingRefreshType() == 2)
    {
      return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
    }
  }
  if(m_pocCRA>0)
  {
    if(pocCurr<m_pocCRA)
    {
      // All leading pictures are being marked as TFD pictures here since current encoder uses all
      // reference pictures while encoding leading pictures. An encoder can ensure that a leading
      // picture can be still decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by
      // controlling the reference pictures used for encoding that leading picture. Such a leading
      // picture need not be marked as a TFD picture.
      return NAL_UNIT_CODED_SLICE_RASL_R;
    }
  }
  if (lastIDR>0)
  {
    if (pocCurr < lastIDR)
    {
      return NAL_UNIT_CODED_SLICE_RADL_R;
    }
  }
  return NAL_UNIT_CODED_SLICE_TRAIL_R;
}

void EncGOP::xUpdateRasInit(Slice* slice)
{
  slice->setPendingRasInit( false );
  if ( slice->getPOC() > m_lastRasPoc )
  {
    m_lastRasPoc = MAX_INT;
    slice->setPendingRasInit( true );
  }
  if ( slice->isIRAP() )
  {
    m_lastRasPoc = slice->getPOC();
  }
}

double EncGOP::xCalculateRVM()
{
  double dRVM = 0;

  if( m_pcCfg->getGOPSize() == 1 && m_pcCfg->getIntraPeriod() != 1 && m_pcCfg->getFramesToBeEncoded() > RVM_VCEGAM10_M * 2 )
  {
    // calculate RVM only for lowdelay configurations
    std::vector<double> vRL , vB;
    size_t N = m_vRVM_RP.size();
    vRL.resize( N );
    vB.resize( N );

    int i;
    double dRavg = 0 , dBavg = 0;
    vB[RVM_VCEGAM10_M] = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      vRL[i] = 0;
      for( int j = i - RVM_VCEGAM10_M ; j <= i + RVM_VCEGAM10_M - 1 ; j++ )
      {
        vRL[i] += m_vRVM_RP[j];
      }
      vRL[i] /= ( 2 * RVM_VCEGAM10_M );
      vB[i] = vB[i-1] + m_vRVM_RP[i] - vRL[i];
      dRavg += m_vRVM_RP[i];
      dBavg += vB[i];
    }

    dRavg /= ( N - 2 * RVM_VCEGAM10_M );
    dBavg /= ( N - 2 * RVM_VCEGAM10_M );

    double dSigamB = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      double tmp = vB[i] - dBavg;
      dSigamB += tmp * tmp;
    }
    dSigamB = sqrt( dSigamB / ( N - 2 * RVM_VCEGAM10_M ) );

    double f = sqrt( 12.0 * ( RVM_VCEGAM10_M - 1 ) / ( RVM_VCEGAM10_M + 1 ) );

    dRVM = dSigamB / dRavg * f;
  }

  return( dRVM );
}

/** Attaches the input bitstream to the stream in the output NAL unit
    Updates rNalu to contain concatenated bitstream. rpcBitstreamRedirect is cleared at the end of this function call.
 *  \param codedSliceData contains the coded slice data (bitstream) to be concatenated to rNalu
 *  \param rNalu          target NAL unit
 */
void EncGOP::xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, OutputBitstream* codedSliceData)
{
  // Byte-align
  rNalu.m_Bitstream.writeByteAlignment();   // Slice header byte-alignment

  // Perform bitstream concatenation
  if (codedSliceData->getNumberOfWrittenBits() > 0)
  {
    rNalu.m_Bitstream.addSubstream(codedSliceData);
  }
  codedSliceData->clear();
}

// Function will arrange the long-term pictures in the decreasing order of poc_lsb_lt,
// and among the pictures with the same lsb, it arranges them in increasing delta_poc_msb_cycle_lt value
void EncGOP::arrangeLongtermPicturesInRPS(Slice *pcSlice, PicList& rcListPic)
{
  if(pcSlice->getRPS()->getNumberOfLongtermPictures() == 0)
  {
    return;
  }
  // we can only modify the local RPS!
  VTMCHECK(!(pcSlice->getRPSidx()==-1), "Unspecified error");
  ReferencePictureSet *rps = pcSlice->getLocalRPS();

  // Arrange long-term reference pictures in the correct order of LSB and MSB,
  // and assign values for pocLSBLT and MSB present flag
  int longtermPicsPoc[MAX_NUM_REF_PICS], longtermPicsLSB[MAX_NUM_REF_PICS], indices[MAX_NUM_REF_PICS];
  int longtermPicsMSB[MAX_NUM_REF_PICS];
  bool mSBPresentFlag[MAX_NUM_REF_PICS];
  ::memset(longtermPicsPoc, 0, sizeof(longtermPicsPoc));    // Store POC values of LTRP
  ::memset(longtermPicsLSB, 0, sizeof(longtermPicsLSB));    // Store POC LSB values of LTRP
  ::memset(longtermPicsMSB, 0, sizeof(longtermPicsMSB));    // Store POC LSB values of LTRP
  ::memset(indices        , 0, sizeof(indices));            // Indices to aid in tracking sorted LTRPs
  ::memset(mSBPresentFlag , 0, sizeof(mSBPresentFlag));     // Indicate if MSB needs to be present

  // Get the long-term reference pictures
  int offset = rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures();
  int i, ctr = 0;
  int maxPicOrderCntLSB = 1 << pcSlice->getSPS()->getBitsForPOC();
  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)
  {
    longtermPicsPoc[ctr] = rps->getPOC(i);                                  // LTRP POC
    longtermPicsLSB[ctr] = getLSB(longtermPicsPoc[ctr], maxPicOrderCntLSB); // LTRP POC LSB
    indices[ctr]      = i;
    longtermPicsMSB[ctr] = longtermPicsPoc[ctr] - longtermPicsLSB[ctr];
  }
  int numLongPics = rps->getNumberOfLongtermPictures();
  VTMCHECK(!(ctr == numLongPics), "Unspecified error");

  // Arrange pictures in decreasing order of MSB;
  for(i = 0; i < numLongPics; i++)
  {
    for(int j = 0; j < numLongPics - 1; j++)
    {
      if(longtermPicsMSB[j] < longtermPicsMSB[j+1])
      {
        std::swap(longtermPicsPoc[j], longtermPicsPoc[j+1]);
        std::swap(longtermPicsLSB[j], longtermPicsLSB[j+1]);
        std::swap(longtermPicsMSB[j], longtermPicsMSB[j+1]);
        std::swap(indices[j]        , indices[j+1]        );
      }
    }
  }

  for(i = 0; i < numLongPics; i++)
  {
    // Check if MSB present flag should be enabled.
    // Check if the buffer contains any pictures that have the same LSB.
    PicList::iterator  iterPic = rcListPic.begin();
    Picture*                      pcPic;
    while ( iterPic != rcListPic.end() )
    {
      pcPic = *iterPic;
      if( (getLSB(pcPic->getPOC(), maxPicOrderCntLSB) == longtermPicsLSB[i])   &&     // Same LSB
                                      (pcPic->referenced)     &&    // Reference picture
                                        (pcPic->getPOC() != longtermPicsPoc[i])    )  // Not the LTRP itself
      {
        mSBPresentFlag[i] = true;
        break;
      }
      iterPic++;
    }
  }

  // tempArray for usedByCurr flag
  bool tempArray[MAX_NUM_REF_PICS]; ::memset(tempArray, 0, sizeof(tempArray));
  for(i = 0; i < numLongPics; i++)
  {
    tempArray[i] = rps->getUsed(indices[i]);
  }
  // Now write the final values;
  ctr = 0;
  int currMSB = 0, currLSB = 0;
  // currPicPoc = currMSB + currLSB
  currLSB = getLSB(pcSlice->getPOC(), maxPicOrderCntLSB);
  currMSB = pcSlice->getPOC() - currLSB;

  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)
  {
    rps->setPOC                   (i, longtermPicsPoc[ctr]);
    rps->setDeltaPOC              (i, - pcSlice->getPOC() + longtermPicsPoc[ctr]);
    rps->setUsed                  (i, tempArray[ctr]);
    rps->setPocLSBLT              (i, longtermPicsLSB[ctr]);
    rps->setDeltaPocMSBCycleLT    (i, (currMSB - (longtermPicsPoc[ctr] - longtermPicsLSB[ctr])) / maxPicOrderCntLSB);
    rps->setDeltaPocMSBPresentFlag(i, mSBPresentFlag[ctr]);

    VTMCHECK(!(rps->getDeltaPocMSBCycleLT(i) >= 0), "Unspecified error");   // Non-negative value
  }
  for(i = rps->getNumberOfPictures() - 1, ctr = 1; i >= offset; i--, ctr++)
  {
    for(int j = rps->getNumberOfPictures() - 1 - ctr; j >= offset; j--)
    {
      // Here at the encoder we know that we have set the full POC value for the LTRPs, hence we
      // don't have to check the MSB present flag values for this constraint.
      VTMCHECK(!( rps->getPOC(i) != rps->getPOC(j) ), "Unspecified error"); // If assert fails, LTRP entry repeated in RPS!!!
    }
  }
}

void EncGOP::arrangeCompositeReference(Slice* pcSlice, PicList& rcListPic, int pocCurr)
{
  Picture* curPic = NULL;
  PicList::iterator  iterPic = rcListPic.begin();
  const PreCalcValues *pcv = pcSlice->getPPS()->pcv;
  m_bgPOC = pocCurr + 1;
  if (m_picBg->getSpliceFull())
  {
    return;
  }
  while (iterPic != rcListPic.end())
  {
    curPic = *(iterPic++);
    if (curPic->getPOC() == pocCurr)
    {
      break;
    }
  }
  if (pcSlice->isIRAP())
  {
    return;
  }

  int width = pcv->lumaWidth;
  int height = pcv->lumaHeight;
  int stride = curPic->getOrigBuf().get(COMPONENT_Y).stride;
  int cStride = curPic->getOrigBuf().get(COMPONENT_Cb).stride;
  Pel* curLumaAddr = curPic->getOrigBuf().get(COMPONENT_Y).buf;
  Pel* curCbAddr = curPic->getOrigBuf().get(COMPONENT_Cb).buf;
  Pel* curCrAddr = curPic->getOrigBuf().get(COMPONENT_Cr).buf;
  Pel* bgOrgLumaAddr = m_picOrig->getOrigBuf().get(COMPONENT_Y).buf;
  Pel* bgOrgCbAddr = m_picOrig->getOrigBuf().get(COMPONENT_Cb).buf;
  Pel* bgOrgCrAddr = m_picOrig->getOrigBuf().get(COMPONENT_Cr).buf;
  int cuMaxWidth = pcv->maxCUWidth;
  int cuMaxHeight = pcv->maxCUHeight;
  int maxReplace = (pcv->sizeInCtus) / 2;
  maxReplace = maxReplace < 1 ? 1 : maxReplace;
  typedef struct tagCostStr
  {
    double cost;
    int ctuIdx;
  }CostStr;
  CostStr* minCtuCost = new CostStr[maxReplace];
  for (int i = 0; i < maxReplace; i++)
  {
    minCtuCost[i].cost = 1e10;
    minCtuCost[i].ctuIdx = -1;
  }
  int bitIncrementY = pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8;
  int bitIncrementUV = pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 8;
  for (int y = 0; y < height; y += cuMaxHeight)
  {
    for (int x = 0; x < width; x += cuMaxWidth)
    {
      double lcuDist = 0.0;
      double lcuDistCb = 0.0;
      double lcuDistCr = 0.0;
      int    realPixelCnt = 0;
      double lcuCost = 1e10;
      int largeDist = 0;

      for (int tmpy = 0; tmpy < cuMaxHeight; tmpy++)
      {
        if (y + tmpy >= height)
        {
          break;
        }
        for (int tmpx = 0; tmpx < cuMaxWidth; tmpx++)
        {
          if (x + tmpx >= width)
          {
            break;
          }

          realPixelCnt++;
          lcuDist += abs(curLumaAddr[(y + tmpy)*stride + x + tmpx] - bgOrgLumaAddr[(y + tmpy)*stride + x + tmpx]);
          if (abs(curLumaAddr[(y + tmpy)*stride + x + tmpx] - bgOrgLumaAddr[(y + tmpy)*stride + x + tmpx]) >(20 << bitIncrementY))
          {
            largeDist++;
          }

          if (tmpy % 2 == 0 && tmpx % 2 == 0)
          {
            lcuDistCb += abs(curCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] - bgOrgCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2]);
            lcuDistCr += abs(curCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] - bgOrgCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2]);
          }
        }
      }

      //Test the vertical or horizontal edge for background patches candidates
      int yInLCU = y / cuMaxHeight;
      int xInLCU = x / cuMaxWidth;
      int iLCUIdx = yInLCU * pcv->widthInCtus + xInLCU;
      if ((largeDist / (double)realPixelCnt < 0.01 &&lcuDist / realPixelCnt < (3.5 * (1 << bitIncrementY)) && lcuDistCb / realPixelCnt < (0.5 * (1 << bitIncrementUV)) && lcuDistCr / realPixelCnt < (0.5 * (1 << bitIncrementUV)) && m_picBg->getSpliceIdx(iLCUIdx) == 0))
      {
        lcuCost = lcuDist / realPixelCnt + lcuDistCb / realPixelCnt + lcuDistCr / realPixelCnt;
        //obtain the maxReplace smallest cost
        //1) find the largest cost in the maxReplace candidates
        for (int i = 0; i < maxReplace - 1; i++)
        {
          if (minCtuCost[i].cost > minCtuCost[i + 1].cost)
          {
            swap(minCtuCost[i].cost, minCtuCost[i + 1].cost);
            swap(minCtuCost[i].ctuIdx, minCtuCost[i + 1].ctuIdx);
          }
        }
        // 2) compare the current cost with the largest cost
        if (lcuCost < minCtuCost[maxReplace - 1].cost)
        {
          minCtuCost[maxReplace - 1].cost = lcuCost;
          minCtuCost[maxReplace - 1].ctuIdx = iLCUIdx;
        }
      }
    }
  }

  // modify QP for background CTU
  {
    for (int i = 0; i < maxReplace; i++)
    {
      if (minCtuCost[i].ctuIdx != -1)
      {
        m_picBg->setSpliceIdx(minCtuCost[i].ctuIdx, pocCurr);
      }
    }
  }
  delete[]minCtuCost;
}

void EncGOP::updateCompositeReference(Slice* pcSlice, PicList& rcListPic, int pocCurr)
{
  Picture* curPic = NULL;
  const PreCalcValues *pcv = pcSlice->getPPS()->pcv;
  PicList::iterator  iterPic = rcListPic.begin();
  iterPic = rcListPic.begin();
  while (iterPic != rcListPic.end())
  {
    curPic = *(iterPic++);
    if (curPic->getPOC() == pocCurr)
    {
      break;
    }
  }
  assert(curPic->getPOC() == pocCurr);

  int width = pcv->lumaWidth;
  int height = pcv->lumaHeight;
  int stride = curPic->getRecoBuf().get(COMPONENT_Y).stride;
  int cStride = curPic->getRecoBuf().get(COMPONENT_Cb).stride;

  Pel* bgLumaAddr = m_picBg->getRecoBuf().get(COMPONENT_Y).buf;
  Pel* bgCbAddr = m_picBg->getRecoBuf().get(COMPONENT_Cb).buf;
  Pel* bgCrAddr = m_picBg->getRecoBuf().get(COMPONENT_Cr).buf;
  Pel* curLumaAddr = curPic->getRecoBuf().get(COMPONENT_Y).buf;
  Pel* curCbAddr = curPic->getRecoBuf().get(COMPONENT_Cb).buf;
  Pel* curCrAddr = curPic->getRecoBuf().get(COMPONENT_Cr).buf;

  int maxCuWidth = pcv->maxCUWidth;
  int maxCuHeight = pcv->maxCUHeight;

  // Update background reference
  if (pcSlice->isIRAP())//(pocCurr == 0)
  {
    curPic->extendPicBorder();
    curPic->setBorderExtension(true);

    m_picBg->getRecoBuf().copyFrom(curPic->getRecoBuf());
    m_picOrig->getOrigBuf().copyFrom(curPic->getOrigBuf());
  }
  else
  {
    //cout << "update B" << pocCurr << endl;
    for (int y = 0; y < height; y += maxCuHeight)
    {
      for (int x = 0; x < width; x += maxCuWidth)
      {
        if (m_picBg->getSpliceIdx((y / maxCuHeight)*pcv->widthInCtus + x / maxCuWidth) == pocCurr)
        {
          for (int tmpy = 0; tmpy < maxCuHeight; tmpy++)
          {
            if (y + tmpy >= height)
            {
              break;
            }
            for (int tmpx = 0; tmpx < maxCuWidth; tmpx++)
            {
              if (x + tmpx >= width)
              {
                break;
              }
              bgLumaAddr[(y + tmpy)*stride + x + tmpx] = curLumaAddr[(y + tmpy)*stride + x + tmpx];
              if (tmpy % 2 == 0 && tmpx % 2 == 0)
              {
                bgCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] = curCbAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2];
                bgCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2] = curCrAddr[(y + tmpy) / 2 * cStride + (x + tmpx) / 2];
              }
            }
          }
        }
      }
    }
    m_picBg->setBorderExtension(false);
    m_picBg->extendPicBorder();
    m_picBg->setBorderExtension(true);

    curPic->extendPicBorder();
    curPic->setBorderExtension(true);
    m_picOrig->getOrigBuf().copyFrom(curPic->getOrigBuf());

    m_picBg->setBorderExtension(false);
    m_picBg->extendPicBorder();
    m_picBg->setBorderExtension(true);
  }
}

void EncGOP::applyDeblockingFilterMetric( Picture* pcPic, uint32_t uiNumSlices )
{
  PelBuf cPelBuf = pcPic->getRecoBuf().get( COMPONENT_Y );
  Pel* Rec    = cPelBuf.buf;
  const int  stride = cPelBuf.stride;
  const uint32_t picWidth = cPelBuf.width;
  const uint32_t picHeight = cPelBuf.height;

  Pel* tempRec = Rec;
  const Slice* pcSlice = pcPic->slices[0];
  uint32_t log2maxTB = pcSlice->getSPS()->getQuadtreeTULog2MaxSize();
  uint32_t maxTBsize = (1<<log2maxTB);
  const uint32_t minBlockArtSize = 8;
  const uint32_t noCol = (picWidth>>log2maxTB);
  const uint32_t noRows = (picHeight>>log2maxTB);
  VTMCHECK(!(noCol > 1), "Unspecified error");
  VTMCHECK(!(noRows > 1), "Unspecified error");
  std::vector<uint64_t> colSAD(noCol,  uint64_t(0));
  std::vector<uint64_t> rowSAD(noRows, uint64_t(0));
  uint32_t colIdx = 0;
  uint32_t rowIdx = 0;
  Pel p0, p1, p2, q0, q1, q2;

  int qp = pcSlice->getSliceQp();
  const int bitDepthLuma=pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  int bitdepthScale = 1 << (bitDepthLuma-8);
  int beta = LoopFilter::getBeta( qp ) * bitdepthScale;
  const int thr2 = (beta>>2);
  const int thr1 = 2*bitdepthScale;
  uint32_t a = 0;

  if (maxTBsize > minBlockArtSize)
  {
    // Analyze vertical artifact edges
    for(int c = maxTBsize; c < picWidth; c += maxTBsize)
    {
      for(int r = 0; r < picHeight; r++)
      {
        p2 = Rec[c-3];
        p1 = Rec[c-2];
        p0 = Rec[c-1];
        q0 = Rec[c];
        q1 = Rec[c+1];
        q2 = Rec[c+2];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if ( thr1 < a && a < thr2)
        {
          colSAD[colIdx] += abs(p0 - q0);
        }
        Rec += stride;
      }
      colIdx++;
      Rec = tempRec;
    }

    // Analyze horizontal artifact edges
    for(int r = maxTBsize; r < picHeight; r += maxTBsize)
    {
      for(int c = 0; c < picWidth; c++)
      {
        p2 = Rec[c + (r-3)*stride];
        p1 = Rec[c + (r-2)*stride];
        p0 = Rec[c + (r-1)*stride];
        q0 = Rec[c + r*stride];
        q1 = Rec[c + (r+1)*stride];
        q2 = Rec[c + (r+2)*stride];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if (thr1 < a && a < thr2)
        {
          rowSAD[rowIdx] += abs(p0 - q0);
        }
      }
      rowIdx++;
    }
  }

  uint64_t colSADsum = 0;
  uint64_t rowSADsum = 0;
  for(int c = 0; c < noCol-1; c++)
  {
    colSADsum += colSAD[c];
  }
  for(int r = 0; r < noRows-1; r++)
  {
    rowSADsum += rowSAD[r];
  }

  colSADsum <<= 10;
  rowSADsum <<= 10;
  colSADsum /= (noCol-1);
  colSADsum /= picHeight;
  rowSADsum /= (noRows-1);
  rowSADsum /= picWidth;

  uint64_t avgSAD = ((colSADsum + rowSADsum)>>1);
  avgSAD >>= (bitDepthLuma-8);

  if ( avgSAD > 2048 )
  {
    avgSAD >>= 9;
    int offset = Clip3(2,6,(int)avgSAD);
    for (int i=0; i<uiNumSlices; i++)
    {
      Slice* pcLocalSlice = pcPic->slices[i];
      pcLocalSlice->setDeblockingFilterOverrideFlag   ( true);
      pcLocalSlice->setDeblockingFilterDisable        ( false);
      pcLocalSlice->setDeblockingFilterBetaOffsetDiv2 ( offset );
      pcLocalSlice->setDeblockingFilterTcOffsetDiv2   ( offset );
    }
  }
  else
  {
    for (int i=0; i<uiNumSlices; i++)
    {
      Slice* pcLocalSlice = pcPic->slices[i];
      const PPS* pcPPS = pcSlice->getPPS();
      pcLocalSlice->setDeblockingFilterOverrideFlag  ( false);
      pcLocalSlice->setDeblockingFilterDisable       ( pcPPS->getPPSDeblockingFilterDisabledFlag() );
      pcLocalSlice->setDeblockingFilterBetaOffsetDiv2( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
      pcLocalSlice->setDeblockingFilterTcOffsetDiv2  ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
    }
  }
}

#if W0038_DB_OPT
void EncGOP::applyDeblockingFilterParameterSelection( Picture* pcPic, const uint32_t numSlices, const int gopID )
{
  enum DBFltParam
  {
    DBFLT_PARAM_AVAILABLE = 0,
    DBFLT_DISABLE_FLAG,
    DBFLT_BETA_OFFSETD2,
    DBFLT_TC_OFFSETD2,
    //NUM_DBFLT_PARAMS
  };
  const int MAX_BETA_OFFSET = 3;
  const int MIN_BETA_OFFSET = -3;
  const int MAX_TC_OFFSET = 3;
  const int MIN_TC_OFFSET = -3;

  PelUnitBuf reco = pcPic->getRecoBuf();

  const int currQualityLayer = (!pcPic->slices[0]->isIRAP()) ? m_pcCfg->getGOPEntry(gopID).m_temporalId+1 : 0;
  VTMCHECK(!(currQualityLayer <MAX_ENCODER_DEBLOCKING_QUALITY_LAYERS), "Unspecified error");

  CodingStructure& cs = *pcPic->cs;

  if(!m_pcDeblockingTempPicYuv)
  {
    m_pcDeblockingTempPicYuv = new PelStorage;
    m_pcDeblockingTempPicYuv->create( cs.area );
    memset(m_DBParam, 0, sizeof(m_DBParam));
  }

  //preserve current reconstruction
  m_pcDeblockingTempPicYuv->copyFrom ( reco );

  const bool bNoFiltering      = m_DBParam[currQualityLayer][DBFLT_PARAM_AVAILABLE] && m_DBParam[currQualityLayer][DBFLT_DISABLE_FLAG]==false /*&& pcPic->getTLayer()==0*/;
  const int  maxBetaOffsetDiv2 = bNoFiltering? Clip3(MIN_BETA_OFFSET, MAX_BETA_OFFSET, m_DBParam[currQualityLayer][DBFLT_BETA_OFFSETD2]+1) : MAX_BETA_OFFSET;
  const int  minBetaOffsetDiv2 = bNoFiltering? Clip3(MIN_BETA_OFFSET, MAX_BETA_OFFSET, m_DBParam[currQualityLayer][DBFLT_BETA_OFFSETD2]-1) : MIN_BETA_OFFSET;
  const int  maxTcOffsetDiv2   = bNoFiltering? Clip3(MIN_TC_OFFSET, MAX_TC_OFFSET, m_DBParam[currQualityLayer][DBFLT_TC_OFFSETD2]+2)       : MAX_TC_OFFSET;
  const int  minTcOffsetDiv2   = bNoFiltering? Clip3(MIN_TC_OFFSET, MAX_TC_OFFSET, m_DBParam[currQualityLayer][DBFLT_TC_OFFSETD2]-2)       : MIN_TC_OFFSET;

  uint64_t distBetaPrevious      = std::numeric_limits<uint64_t>::max();
  uint64_t distMin               = std::numeric_limits<uint64_t>::max();
  bool   bDBFilterDisabledBest = true;
  int    betaOffsetDiv2Best    = 0;
  int    tcOffsetDiv2Best      = 0;

  for(int betaOffsetDiv2=maxBetaOffsetDiv2; betaOffsetDiv2>=minBetaOffsetDiv2; betaOffsetDiv2--)
  {
    uint64_t distTcMin = std::numeric_limits<uint64_t>::max();
    for(int tcOffsetDiv2=maxTcOffsetDiv2; tcOffsetDiv2 >= minTcOffsetDiv2; tcOffsetDiv2--)
    {
      for (int i=0; i<numSlices; i++)
      {
        Slice* pcSlice = pcPic->slices[i];
        pcSlice->setDeblockingFilterOverrideFlag  ( true);
        pcSlice->setDeblockingFilterDisable       ( false);
        pcSlice->setDeblockingFilterBetaOffsetDiv2( betaOffsetDiv2 );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( tcOffsetDiv2 );
      }

      // restore reconstruction
      reco.copyFrom( *m_pcDeblockingTempPicYuv );

      const uint64_t dist = preLoopFilterPicAndCalcDist( pcPic );

      if(dist < distMin)
      {
        distMin = dist;
        bDBFilterDisabledBest = false;
        betaOffsetDiv2Best  = betaOffsetDiv2;
        tcOffsetDiv2Best = tcOffsetDiv2;
      }
      if(dist < distTcMin)
      {
        distTcMin = dist;
      }
      else if(tcOffsetDiv2 <-2)
      {
        break;
      }
    }
    if(betaOffsetDiv2<-1 && distTcMin >= distBetaPrevious)
    {
      break;
    }
    distBetaPrevious = distTcMin;
  }

  //update:
  m_DBParam[currQualityLayer][DBFLT_PARAM_AVAILABLE] = 1;
  m_DBParam[currQualityLayer][DBFLT_DISABLE_FLAG]    = bDBFilterDisabledBest;
  m_DBParam[currQualityLayer][DBFLT_BETA_OFFSETD2]   = betaOffsetDiv2Best;
  m_DBParam[currQualityLayer][DBFLT_TC_OFFSETD2]     = tcOffsetDiv2Best;

  // restore reconstruction
  reco.copyFrom( *m_pcDeblockingTempPicYuv );

  const PPS* pcPPS = pcPic->slices[0]->getPPS();
  if(bDBFilterDisabledBest)
  {
    for (int i=0; i<numSlices; i++)
    {
      Slice* pcSlice = pcPic->slices[i];
      pcSlice->setDeblockingFilterOverrideFlag( true);
      pcSlice->setDeblockingFilterDisable     ( true);
    }
  }
  else if(betaOffsetDiv2Best == pcPPS->getDeblockingFilterBetaOffsetDiv2() &&  tcOffsetDiv2Best == pcPPS->getDeblockingFilterTcOffsetDiv2())
  {
    for (int i=0; i<numSlices; i++)
    {
      Slice*      pcSlice = pcPic->slices[i];
      pcSlice->setDeblockingFilterOverrideFlag   ( false);
      pcSlice->setDeblockingFilterDisable        ( pcPPS->getPPSDeblockingFilterDisabledFlag() );
      pcSlice->setDeblockingFilterBetaOffsetDiv2 ( pcPPS->getDeblockingFilterBetaOffsetDiv2() );
      pcSlice->setDeblockingFilterTcOffsetDiv2   ( pcPPS->getDeblockingFilterTcOffsetDiv2()   );
    }
  }
  else
  {
    for (int i=0; i<numSlices; i++)
    {
      Slice* pcSlice = pcPic->slices[i];
      pcSlice->setDeblockingFilterOverrideFlag   ( true);
      pcSlice->setDeblockingFilterDisable        ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2 ( betaOffsetDiv2Best);
      pcSlice->setDeblockingFilterTcOffsetDiv2   ( tcOffsetDiv2Best);
    }
  }
}
#endif
//! \}
