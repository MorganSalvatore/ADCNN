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

#ifndef __SEI__
#define __SEI__

#pragma once
#include <list>
#include <vector>
#include <cstring>

#include "CommonDef.h"
#include "libmd5/MD5.h"

//! \ingroup CommonLib
//! \{
class SPS;

/**
 * Abstract class representing an SEI message with lightweight RTTI.
 */
class SEI
{
public:
  enum PayloadType
  {
    BUFFERING_PERIOD                     = 0,
    PICTURE_TIMING                       = 1,
    PAN_SCAN_RECT                        = 2,
    FILLER_PAYLOAD                       = 3,
    USER_DATA_REGISTERED_ITU_T_T35       = 4,
    USER_DATA_UNREGISTERED               = 5,
    RECOVERY_POINT                       = 6,
    SCENE_INFO                           = 9,
    FULL_FRAME_SNAPSHOT                  = 15,
    PROGRESSIVE_REFINEMENT_SEGMENT_START = 16,
    PROGRESSIVE_REFINEMENT_SEGMENT_END   = 17,
    FILM_GRAIN_CHARACTERISTICS           = 19,
    POST_FILTER_HINT                     = 22,
    TONE_MAPPING_INFO                    = 23,
    FRAME_PACKING                        = 45,
    DISPLAY_ORIENTATION                  = 47,
    GREEN_METADATA                       = 56,
    SOP_DESCRIPTION                      = 128,
    ACTIVE_PARAMETER_SETS                = 129,
    DECODING_UNIT_INFO                   = 130,
    TEMPORAL_LEVEL0_INDEX                = 131,
    DECODED_PICTURE_HASH                 = 132,
    SCALABLE_NESTING                     = 133,
    REGION_REFRESH_INFO                  = 134,
    NO_DISPLAY                           = 135,
    TIME_CODE                            = 136,
    MASTERING_DISPLAY_COLOUR_VOLUME      = 137,
    SEGM_RECT_FRAME_PACKING              = 138,
#if HEVC_TILES_WPP
    TEMP_MOTION_CONSTRAINED_TILE_SETS    = 139,
#endif
    CHROMA_RESAMPLING_FILTER_HINT        = 140,
    KNEE_FUNCTION_INFO                   = 141,
    COLOUR_REMAPPING_INFO                = 142,
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
    ALTERNATIVE_TRANSFER_CHARACTERISTICS = 182,
#endif
  };

  SEI() {}
  virtual ~SEI() {}

  static const char *getSEIMessageString(SEI::PayloadType payloadType);

  virtual PayloadType payloadType() const = 0;
};

static const uint32_t ISO_IEC_11578_LEN=16;

class SEIuserDataUnregistered : public SEI
{
public:
  PayloadType payloadType() const { return USER_DATA_UNREGISTERED; }

  SEIuserDataUnregistered()
    : userData(0)
    {}

  virtual ~SEIuserDataUnregistered()
  {
    delete userData;
  }

  uint8_t uuid_iso_iec_11578[ISO_IEC_11578_LEN];
  uint32_t  userDataLength;
  uint8_t *userData;
};

class SEIDecodedPictureHash : public SEI
{
public:
  PayloadType payloadType() const { return DECODED_PICTURE_HASH; }

  SEIDecodedPictureHash() {}
  virtual ~SEIDecodedPictureHash() {}

  HashType method;

  PictureHash m_pictureHash;
};

class SEIActiveParameterSets : public SEI
{
public:
  PayloadType payloadType() const { return ACTIVE_PARAMETER_SETS; }

  SEIActiveParameterSets()
#if HEVC_VPS
    : activeVPSId            (0)
    , m_selfContainedCvsFlag (false)
#else
    : m_selfContainedCvsFlag(false)
#endif
    , m_noParameterSetUpdateFlag (false)
    , numSpsIdsMinus1        (0)
  {}
  virtual ~SEIActiveParameterSets() {}

#if HEVC_VPS
  int activeVPSId;
#endif
  bool m_selfContainedCvsFlag;
  bool m_noParameterSetUpdateFlag;
  int numSpsIdsMinus1;
  std::vector<int> activeSeqParameterSetId;
};

class SEIBufferingPeriod : public SEI
{
public:
  PayloadType payloadType() const { return BUFFERING_PERIOD; }
  void copyTo (SEIBufferingPeriod& target);

  SEIBufferingPeriod()
  : m_bpSeqParameterSetId (0)
  , m_rapCpbParamsPresentFlag (false)
  , m_cpbDelayOffset      (0)
  , m_dpbDelayOffset      (0)
  {
    ::memset(m_initialCpbRemovalDelay, 0, sizeof(m_initialCpbRemovalDelay));
    ::memset(m_initialCpbRemovalDelayOffset, 0, sizeof(m_initialCpbRemovalDelayOffset));
    ::memset(m_initialAltCpbRemovalDelay, 0, sizeof(m_initialAltCpbRemovalDelay));
    ::memset(m_initialAltCpbRemovalDelayOffset, 0, sizeof(m_initialAltCpbRemovalDelayOffset));
  }
  virtual ~SEIBufferingPeriod() {}

  uint32_t m_bpSeqParameterSetId;
  bool m_rapCpbParamsPresentFlag;
  uint32_t m_cpbDelayOffset;
  uint32_t m_dpbDelayOffset;
  uint32_t m_initialCpbRemovalDelay         [MAX_CPB_CNT][2];
  uint32_t m_initialCpbRemovalDelayOffset   [MAX_CPB_CNT][2];
  uint32_t m_initialAltCpbRemovalDelay      [MAX_CPB_CNT][2];
  uint32_t m_initialAltCpbRemovalDelayOffset[MAX_CPB_CNT][2];
  bool m_concatenationFlag;
  uint32_t m_auCpbRemovalDelayDelta;
};
class SEIPictureTiming : public SEI
{
public:
  PayloadType payloadType() const { return PICTURE_TIMING; }
  void copyTo (SEIPictureTiming& target);

  SEIPictureTiming()
  : m_picStruct               (0)
  , m_sourceScanType          (0)
  , m_duplicateFlag           (false)
  , m_picDpbOutputDuDelay     (0)
  {}
  virtual ~SEIPictureTiming()
  {
  }

  uint32_t  m_picStruct;
  uint32_t  m_sourceScanType;
  bool  m_duplicateFlag;

  uint32_t  m_auCpbRemovalDelay;
  uint32_t  m_picDpbOutputDelay;
  uint32_t  m_picDpbOutputDuDelay;
  uint32_t  m_numDecodingUnitsMinus1;
  bool  m_duCommonCpbRemovalDelayFlag;
  uint32_t  m_duCommonCpbRemovalDelayMinus1;
  std::vector<uint32_t> m_numNalusInDuMinus1;
  std::vector<uint32_t> m_duCpbRemovalDelayMinus1;
};

class SEIDecodingUnitInfo : public SEI
{
public:
  PayloadType payloadType() const { return DECODING_UNIT_INFO; }

  SEIDecodingUnitInfo()
    : m_decodingUnitIdx(0)
    , m_duSptCpbRemovalDelay(0)
    , m_dpbOutputDuDelayPresentFlag(false)
    , m_picSptDpbOutputDuDelay(0)
  {}
  virtual ~SEIDecodingUnitInfo() {}
  int m_decodingUnitIdx;
  int m_duSptCpbRemovalDelay;
  bool m_dpbOutputDuDelayPresentFlag;
  int m_picSptDpbOutputDuDelay;
};

class SEIRecoveryPoint : public SEI
{
public:
  PayloadType payloadType() const { return RECOVERY_POINT; }

  SEIRecoveryPoint() {}
  virtual ~SEIRecoveryPoint() {}

  int  m_recoveryPocCnt;
  bool m_exactMatchingFlag;
  bool m_brokenLinkFlag;
};

class SEIFramePacking : public SEI
{
public:
  PayloadType payloadType() const { return FRAME_PACKING; }

  SEIFramePacking() {}
  virtual ~SEIFramePacking() {}

  int  m_arrangementId;
  bool m_arrangementCancelFlag;
  int  m_arrangementType;
  bool m_quincunxSamplingFlag;
  int  m_contentInterpretationType;
  bool m_spatialFlippingFlag;
  bool m_frame0FlippedFlag;
  bool m_fieldViewsFlag;
  bool m_currentFrameIsFrame0Flag;
  bool m_frame0SelfContainedFlag;
  bool m_frame1SelfContainedFlag;
  int  m_frame0GridPositionX;
  int  m_frame0GridPositionY;
  int  m_frame1GridPositionX;
  int  m_frame1GridPositionY;
  int  m_arrangementReservedByte;
  bool m_arrangementPersistenceFlag;
  bool m_upsampledAspectRatio;
};

class SEISegmentedRectFramePacking : public SEI
{
public:
  PayloadType payloadType() const { return SEGM_RECT_FRAME_PACKING; }

  SEISegmentedRectFramePacking() {}
  virtual ~SEISegmentedRectFramePacking() {}

  bool m_arrangementCancelFlag;
  int  m_contentInterpretationType;
  bool m_arrangementPersistenceFlag;
};

class SEIDisplayOrientation : public SEI
{
public:
  PayloadType payloadType() const { return DISPLAY_ORIENTATION; }

  SEIDisplayOrientation()
    : cancelFlag(true)
    , persistenceFlag(0)
    , extensionFlag(false)
    {}
  virtual ~SEIDisplayOrientation() {}

  bool cancelFlag;
  bool horFlip;
  bool verFlip;

  uint32_t anticlockwiseRotation;
  bool persistenceFlag;
  bool extensionFlag;
};

class SEITemporalLevel0Index : public SEI
{
public:
  PayloadType payloadType() const { return TEMPORAL_LEVEL0_INDEX; }

  SEITemporalLevel0Index()
    : tl0Idx(0)
    , rapIdx(0)
    {}
  virtual ~SEITemporalLevel0Index() {}

  uint32_t tl0Idx;
  uint32_t rapIdx;
};

class SEIGradualDecodingRefreshInfo : public SEI
{
public:
  PayloadType payloadType() const { return REGION_REFRESH_INFO; }

  SEIGradualDecodingRefreshInfo()
    : m_gdrForegroundFlag(0)
  {}
  virtual ~SEIGradualDecodingRefreshInfo() {}

  bool m_gdrForegroundFlag;
};

class SEINoDisplay : public SEI
{
public:
  PayloadType payloadType() const { return NO_DISPLAY; }

  SEINoDisplay()
    : m_noDisplay(false)
  {}
  virtual ~SEINoDisplay() {}

  bool m_noDisplay;
};

class SEISOPDescription : public SEI
{
public:
  PayloadType payloadType() const { return SOP_DESCRIPTION; }

  SEISOPDescription() {}
  virtual ~SEISOPDescription() {}

  uint32_t m_sopSeqParameterSetId;
  uint32_t m_numPicsInSopMinus1;

  uint32_t m_sopDescVclNaluType[MAX_NUM_PICS_IN_SOP];
  uint32_t m_sopDescTemporalId[MAX_NUM_PICS_IN_SOP];
  uint32_t m_sopDescStRpsIdx[MAX_NUM_PICS_IN_SOP];
  int m_sopDescPocDelta[MAX_NUM_PICS_IN_SOP];
};

class SEIToneMappingInfo : public SEI
{
public:
  PayloadType payloadType() const { return TONE_MAPPING_INFO; }
  SEIToneMappingInfo() {}
  virtual ~SEIToneMappingInfo() {}

  int    m_toneMapId;
  bool   m_toneMapCancelFlag;
  bool   m_toneMapPersistenceFlag;
  int    m_codedDataBitDepth;
  int    m_targetBitDepth;
  int    m_modelId;
  int    m_minValue;
  int    m_maxValue;
  int    m_sigmoidMidpoint;
  int    m_sigmoidWidth;
  std::vector<int> m_startOfCodedInterval;
  int    m_numPivots;
  std::vector<int> m_codedPivotValue;
  std::vector<int> m_targetPivotValue;
  int    m_cameraIsoSpeedIdc;
  int    m_cameraIsoSpeedValue;
  int    m_exposureIndexIdc;
  int    m_exposureIndexValue;
  bool   m_exposureCompensationValueSignFlag;
  int    m_exposureCompensationValueNumerator;
  int    m_exposureCompensationValueDenomIdc;
  int    m_refScreenLuminanceWhite;
  int    m_extendedRangeWhiteLevel;
  int    m_nominalBlackLevelLumaCodeValue;
  int    m_nominalWhiteLevelLumaCodeValue;
  int    m_extendedWhiteLevelLumaCodeValue;
};

class SEIKneeFunctionInfo : public SEI
{
public:
  PayloadType payloadType() const { return KNEE_FUNCTION_INFO; }
  SEIKneeFunctionInfo() {}
  virtual ~SEIKneeFunctionInfo() {}

  int   m_kneeId;
  bool  m_kneeCancelFlag;
  bool  m_kneePersistenceFlag;
  int   m_kneeInputDrange;
  int   m_kneeInputDispLuminance;
  int   m_kneeOutputDrange;
  int   m_kneeOutputDispLuminance;
  int   m_kneeNumKneePointsMinus1;
  std::vector<int> m_kneeInputKneePoint;
  std::vector<int> m_kneeOutputKneePoint;
};

class SEIColourRemappingInfo : public SEI
{
public:

  struct CRIlut
  {
    int codedValue;
    int targetValue;
    bool operator < (const CRIlut& a) const
    {
      return codedValue < a.codedValue;
    }
  };

  PayloadType payloadType() const { return COLOUR_REMAPPING_INFO; }
  SEIColourRemappingInfo() {}
  ~SEIColourRemappingInfo() {}

  void copyFrom( const SEIColourRemappingInfo &seiCriInput)
  {
    (*this) = seiCriInput;
  }

  uint32_t                m_colourRemapId;
  bool                m_colourRemapCancelFlag;
  bool                m_colourRemapPersistenceFlag;
  bool                m_colourRemapVideoSignalInfoPresentFlag;
  bool                m_colourRemapFullRangeFlag;
  int                 m_colourRemapPrimaries;
  int                 m_colourRemapTransferFunction;
  int                 m_colourRemapMatrixCoefficients;
  int                 m_colourRemapInputBitDepth;
  int                 m_colourRemapBitDepth;
  int                 m_preLutNumValMinus1[3];
  std::vector<CRIlut> m_preLut[3];
  bool                m_colourRemapMatrixPresentFlag;
  int                 m_log2MatrixDenom;
  int                 m_colourRemapCoeffs[3][3];
  int                 m_postLutNumValMinus1[3];
  std::vector<CRIlut> m_postLut[3];
};

class SEIChromaResamplingFilterHint : public SEI
{
public:
  PayloadType payloadType() const {return CHROMA_RESAMPLING_FILTER_HINT;}
  SEIChromaResamplingFilterHint() {}
  virtual ~SEIChromaResamplingFilterHint() {}

  int                            m_verChromaFilterIdc;
  int                            m_horChromaFilterIdc;
  bool                           m_verFilteringFieldProcessingFlag;
  int                            m_targetFormatIdc;
  bool                           m_perfectReconstructionFlag;
  std::vector<std::vector<int> > m_verFilterCoeff;
  std::vector<std::vector<int> > m_horFilterCoeff;
};

class SEIMasteringDisplayColourVolume : public SEI
{
public:
    PayloadType payloadType() const { return MASTERING_DISPLAY_COLOUR_VOLUME; }
    SEIMasteringDisplayColourVolume() {}
    virtual ~SEIMasteringDisplayColourVolume(){}

    SEIMasteringDisplay values;
};

typedef std::list<SEI*> SEIMessages;

/// output a selection of SEI messages by payload type. Ownership stays in original message list.
SEIMessages getSeisByType(SEIMessages &seiList, SEI::PayloadType seiType);

/// remove a selection of SEI messages by payload type from the original list and return them in a new list.
SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType);

/// delete list of SEI messages (freeing the referenced objects)
void deleteSEIs (SEIMessages &seiList);

class SEIScalableNesting : public SEI
{
public:
  PayloadType payloadType() const { return SCALABLE_NESTING; }

  SEIScalableNesting() {}

  virtual ~SEIScalableNesting()
  {
    deleteSEIs(m_nestedSEIs);
  }

  bool  m_bitStreamSubsetFlag;
  bool  m_nestingOpFlag;
  bool  m_defaultOpFlag;                             //value valid if m_nestingOpFlag != 0
  uint32_t  m_nestingNumOpsMinus1;                       // -"-
  uint32_t  m_nestingMaxTemporalIdPlus1[MAX_TLAYER];     // -"-
  uint32_t  m_nestingOpIdx[MAX_NESTING_NUM_OPS];         // -"-

  bool  m_allLayersFlag;                             //value valid if m_nestingOpFlag == 0
  uint32_t  m_nestingNoOpMaxTemporalIdPlus1;             //value valid if m_nestingOpFlag == 0 and m_allLayersFlag == 0
  uint32_t  m_nestingNumLayersMinus1;                    //value valid if m_nestingOpFlag == 0 and m_allLayersFlag == 0
  uint8_t m_nestingLayerId[MAX_NESTING_NUM_LAYER];     //value valid if m_nestingOpFlag == 0 and m_allLayersFlag == 0. This can e.g. be a static array of 64 uint8_t values

  SEIMessages m_nestedSEIs;
};

class SEITimeCode : public SEI
{
public:
  PayloadType payloadType() const { return TIME_CODE; }
  SEITimeCode() {}
  virtual ~SEITimeCode(){}

  uint32_t numClockTs;
  SEITimeSet timeSetArray[MAX_TIMECODE_SEI_SETS];
};

#if HEVC_TILES_WPP
//definition according to P1005_v1;
class SEITempMotionConstrainedTileSets: public SEI
{
  struct TileSetData
  {
    protected:
      std::vector<int> m_top_left_tile_index;  //[tileSetIdx][tileIdx];
      std::vector<int> m_bottom_right_tile_index;

    public:
      int     m_mcts_id;
      bool    m_display_tile_set_flag;
      int     m_num_tile_rects_in_set; //_minus1;
      bool    m_exact_sample_value_match_flag;
      bool    m_mcts_tier_level_idc_present_flag;
      bool    m_mcts_tier_flag;
      int     m_mcts_level_idc;

      void setNumberOfTileRects(const int number)
      {
        m_top_left_tile_index    .resize(number);
        m_bottom_right_tile_index.resize(number);
      }

      int  getNumberOfTileRects() const
      {
        VTMCHECK(m_top_left_tile_index.size() != m_bottom_right_tile_index.size(), "Inconsistent tile arrangement");
        return int(m_top_left_tile_index.size());
      }

            int &topLeftTileIndex    (const int tileRectIndex)       { return m_top_left_tile_index    [tileRectIndex]; }
            int &bottomRightTileIndex(const int tileRectIndex)       { return m_bottom_right_tile_index[tileRectIndex]; }
      const int &topLeftTileIndex    (const int tileRectIndex) const { return m_top_left_tile_index    [tileRectIndex]; }
      const int &bottomRightTileIndex(const int tileRectIndex) const { return m_bottom_right_tile_index[tileRectIndex]; }
  };

protected:
  std::vector<TileSetData> m_tile_set_data;

public:

  bool    m_mc_all_tiles_exact_sample_value_match_flag;
  bool    m_each_tile_one_tile_set_flag;
  bool    m_limited_tile_set_display_flag;
  bool    m_max_mcs_tier_level_idc_present_flag;
  bool    m_max_mcts_tier_flag;
  int     m_max_mcts_level_idc;

  PayloadType payloadType() const { return TEMP_MOTION_CONSTRAINED_TILE_SETS; }

  void setNumberOfTileSets(const int number)       { m_tile_set_data.resize(number);     }
  int  getNumberOfTileSets()                 const { return int(m_tile_set_data.size()); }

        TileSetData &tileSetData (const int index)       { return m_tile_set_data[index]; }
  const TileSetData &tileSetData (const int index) const { return m_tile_set_data[index]; }

};
#endif

#if ENABLE_TRACING
void xTraceSEIHeader();
void xTraceSEIMessageType( SEI::PayloadType payloadType );
#endif

#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
class SEIAlternativeTransferCharacteristics : public SEI
{
public:
  PayloadType payloadType() const { return ALTERNATIVE_TRANSFER_CHARACTERISTICS; }

  SEIAlternativeTransferCharacteristics() : m_preferredTransferCharacteristics(18)
  { }

  virtual ~SEIAlternativeTransferCharacteristics() {}

  uint32_t m_preferredTransferCharacteristics;
};
#endif

class SEIGreenMetadataInfo : public SEI
{
public:
    PayloadType payloadType() const { return GREEN_METADATA; }
    SEIGreenMetadataInfo() {}

    virtual ~SEIGreenMetadataInfo() {}

    uint32_t m_greenMetadataType;
    uint32_t m_xsdMetricType;
    uint32_t m_xsdMetricValue;
};

#endif

//! \}
