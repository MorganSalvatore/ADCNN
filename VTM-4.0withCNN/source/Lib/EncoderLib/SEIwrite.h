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

#pragma once

#ifndef __SEIWRITE__
#define __SEIWRITE__

#include "VLCWriter.h"
#include "CommonLib/SEI.h"

class OutputBitstream;

//! \ingroup EncoderLib
//! \{
class SEIWriter : public VLCWriter
{
public:
  SEIWriter() {};
  virtual ~SEIWriter() {};

  void writeSEImessages(OutputBitstream& bs, const SEIMessages &seiList, const SPS *sps, bool isNested);

protected:
  void xWriteSEIuserDataUnregistered(const SEIuserDataUnregistered &sei);
  void xWriteSEIActiveParameterSets(const SEIActiveParameterSets& sei);
  void xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const SPS *sps);
  void xWriteSEIDecodedPictureHash(const SEIDecodedPictureHash& sei);
  void xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei, const SPS *sps);
  void xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SPS *sps);
  void xWriteSEIRecoveryPoint(const SEIRecoveryPoint& sei);
  void xWriteSEIFramePacking(const SEIFramePacking& sei);
  void xWriteSEISegmentedRectFramePacking(const SEISegmentedRectFramePacking& sei);
  void xWriteSEIDisplayOrientation(const SEIDisplayOrientation &sei);
  void xWriteSEITemporalLevel0Index(const SEITemporalLevel0Index &sei);
  void xWriteSEIGradualDecodingRefreshInfo(const SEIGradualDecodingRefreshInfo &sei);
  void xWriteSEINoDisplay(const SEINoDisplay &sei);
  void xWriteSEIToneMappingInfo(const SEIToneMappingInfo& sei);
  void xWriteSEISOPDescription(const SEISOPDescription& sei);
  void xWriteSEIScalableNesting(OutputBitstream& bs, const SEIScalableNesting& sei, const SPS *sps);
#if HEVC_TILES_WPP
  void xWriteSEITempMotionConstrainedTileSets(const SEITempMotionConstrainedTileSets& sei);
#endif
  void xWriteSEITimeCode(const SEITimeCode& sei);
  void xWriteSEIChromaResamplingFilterHint(const SEIChromaResamplingFilterHint& sei);
  void xWriteSEIKneeFunctionInfo(const SEIKneeFunctionInfo &sei);
  void xWriteSEIColourRemappingInfo(const SEIColourRemappingInfo& sei);
  void xWriteSEIMasteringDisplayColourVolume( const SEIMasteringDisplayColourVolume& sei);
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  void xWriteSEIAlternativeTransferCharacteristics(const SEIAlternativeTransferCharacteristics& sei);
#endif
  void xWriteSEIGreenMetadataInfo(const SEIGreenMetadataInfo &sei);

  void xWriteSEIpayloadData(OutputBitstream& bs, const SEI& sei, const SPS *sps);
  void xWriteByteAlign();
};

//! \}

#endif
