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

/** \file     DecAppCfg.h
    \brief    Decoder configuration class (header)
*/

#ifndef __DECAPPCFG__
#define __DECAPPCFG__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "CommonLib/CommonDef.h"
#include <vector>

//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// Decoder configuration class
class DecAppCfg
{
protected:
  std::string   m_bitstreamFileName;                    ///< input bitstream file name
  std::string   m_reconFileName;                        ///< output reconstruction file name
  int           m_iSkipFrame;                           ///< counter for frames prior to the random access point to skip
  int           m_outputBitDepth[MAX_NUM_CHANNEL_TYPE]; ///< bit depth used for writing output
  InputColourSpaceConversion m_outputColourSpaceConvert;

  int           m_iMaxTemporalLayer;                  ///< maximum temporal layer to be decoded
  int           m_decodedPictureHashSEIEnabled;       ///< Checksum(3)/CRC(2)/MD5(1)/disable(0) acting on decoded picture hash SEI message
  bool          m_decodedNoDisplaySEIEnabled;         ///< Enable(true)/disable(false) writing only pictures that get displayed based on the no display SEI message
  std::string   m_colourRemapSEIFileName;             ///< output Colour Remapping file name
  std::vector<int> m_targetDecLayerIdSet;             ///< set of LayerIds to be included in the sub-bitstream extraction process.
  int           m_respectDefDispWindow;               ///< Only output content inside the default display window
  std::string   m_outputDecodedSEIMessagesFilename;   ///< filename to output decoded SEI messages to. If '-', then use stdout. If empty, do not output details.
  bool          m_bClipOutputVideoToRec709Range;      ///< If true, clip the output video to the Rec 709 range on saving.
  bool          m_packedYUVMode;                      ///< If true, output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data
  std::string   m_cacheCfgFile;                       ///< Config file of cache model
  int           m_statMode;                           ///< Config statistic mode (0 - bit stat, 1 - tool stat, 3 - both)
#if ADCNN
  std::string		m_pbpath;
  std::string		m_sWorkingMode;
  std::string		m_iGPUid;
#endif

public:
  DecAppCfg();
  virtual ~DecAppCfg();

  bool  parseCfg        ( int argc, char* argv[] );   ///< initialize option class from configuration
};

//! \}

#endif  // __DECAPPCFG__


