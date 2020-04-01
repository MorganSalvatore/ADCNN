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

/** \file     VLCWReader.cpp
 *  \brief    Reader for high level syntax
 */

//! \ingroup DecoderLib
//! \{

#include "VLCReader.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/dtrace_next.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif
#include "CommonLib/AdaptiveLoopFilter.h"
#if ADCNN
#include "CommonLib/CNNLoopFilter.h"
#endif

#if ENABLE_TRACING

void  VLCReader::xReadCodeTr(uint32_t length, uint32_t& rValue, const char *pSymbolName)
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadCode (length, rValue, pSymbolName);
#else
  xReadCode (length, rValue);
#endif
  if (length < 10)
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %u\n", pSymbolName, length, rValue );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %u\n", pSymbolName, length, rValue );
  }
}

void  VLCReader::xReadUvlcTr(uint32_t& rValue, const char *pSymbolName)
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadUvlc (rValue, pSymbolName);
#else
  xReadUvlc (rValue);
#endif
  DTRACE( g_trace_ctx, D_HEADER, "%-50s ue(v) : %u\n", pSymbolName, rValue );
}

void  VLCReader::xReadSvlcTr(int& rValue, const char *pSymbolName)
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadSvlc (rValue, pSymbolName);
#else
  xReadSvlc (rValue);
#endif
  DTRACE( g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", pSymbolName, rValue );
}

void  VLCReader::xReadFlagTr(uint32_t& rValue, const char *pSymbolName)
{
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadFlag (rValue, pSymbolName);
#else
  xReadFlag (rValue);
#endif
  DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, rValue );
}

void xTraceFillerData ()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Filler Data ===========\n");
}

#endif


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
#if RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadCode (uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName)
#else
void VLCReader::xReadCode (uint32_t uiLength, uint32_t& ruiCode)
#endif
{
  VTMCHECK( uiLength == 0, "Reading a code of lenght '0'" );
  m_pcBitstream->read (uiLength, ruiCode);
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(pSymbolName, uiLength, ruiCode);
#endif
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadUvlc( uint32_t& ruiVal, const char *pSymbolName)
#else
void VLCReader::xReadUvlc( uint32_t& ruiVal)
#endif
{
  uint32_t uiVal = 0;
  uint32_t uiCode = 0;
  uint32_t uiLength;
  m_pcBitstream->read( 1, uiCode );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  uint32_t totalLen=1;
#endif

  if( 0 == uiCode )
  {
    uiLength = 0;

    while( ! ( uiCode & 1 ))
    {
      m_pcBitstream->read( 1, uiCode );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiVal );

    uiVal += (1 << uiLength)-1;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    totalLen+=uiLength+uiLength;
#endif
  }

  ruiVal = uiVal;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(pSymbolName, int(totalLen), ruiVal);
#endif
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadSvlc( int& riVal, const char *pSymbolName)
#else
void VLCReader::xReadSvlc( int& riVal)
#endif
{
  uint32_t uiBits = 0;
  m_pcBitstream->read( 1, uiBits );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  uint32_t totalLen=1;
#endif
  if( 0 == uiBits )
  {
    uint32_t uiLength = 0;

    while( ! ( uiBits & 1 ))
    {
      m_pcBitstream->read( 1, uiBits );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiBits );

    uiBits += (1 << uiLength);
    riVal = ( uiBits & 1) ? -(int)(uiBits>>1) : (int)(uiBits>>1);
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    totalLen+=uiLength+uiLength;
#endif
  }
  else
  {
    riVal = 0;
  }
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(pSymbolName, int(totalLen), uiBits);
#endif
}

#if RExt__DECODER_DEBUG_BIT_STATISTICS
void VLCReader::xReadFlag (uint32_t& ruiCode, const char *pSymbolName)
#else
void VLCReader::xReadFlag (uint32_t& ruiCode)
#endif
{
  m_pcBitstream->read( 1, ruiCode );
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(pSymbolName, 1, int(/*ruiCode*/0));
#endif
}

void VLCReader::xReadRbspTrailingBits()
{
  uint32_t bit;
  READ_FLAG( bit, "rbsp_stop_one_bit");
  VTMCHECK(bit!=1, "Trailing bit not '1'");
  int cnt = 0;
  while (m_pcBitstream->getNumBitsUntilByteAligned())
  {
    READ_FLAG( bit, "rbsp_alignment_zero_bit");
    VTMCHECK(bit!=0, "Alignment bit is not '0'");
    cnt++;
  }
  VTMCHECK(cnt >= 8, "Read more than '8' trailing bits");
}

void AUDReader::parseAccessUnitDelimiter(InputBitstream* bs, uint32_t &picType)
{
  setBitstream(bs);

#if ENABLE_TRACING
  xTraceAccessUnitDelimiter();
#endif

  READ_CODE (3, picType, "pic_type");
  xReadRbspTrailingBits();
}

void FDReader::parseFillerData(InputBitstream* bs, uint32_t &fdSize)
{
  setBitstream(bs);
#if ENABLE_TRACING
  xTraceFillerData();
#endif
  uint32_t ffByte;
  fdSize = 0;
  while( m_pcBitstream->getNumBitsLeft() >8 )
  {
    READ_CODE (8, ffByte, "ff_byte");
    VTMCHECK(ffByte!=0xff, "Invalid fillter data not '0xff'");
    fdSize++;
  }
  xReadRbspTrailingBits();
}

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

HLSyntaxReader::HLSyntaxReader()
{
}

HLSyntaxReader::~HLSyntaxReader()
{

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void HLSyntaxReader::parseShortTermRefPicSet( SPS* sps, ReferencePictureSet* rps, int idx )
{
  uint32_t code;
  uint32_t interRPSPred;
  if (idx > 0)
  {
    READ_FLAG(interRPSPred, "inter_ref_pic_set_prediction_flag");  rps->setInterRPSPrediction(interRPSPred);
  }
  else
  {
    interRPSPred = false;
    rps->setInterRPSPrediction(false);
  }

  if (interRPSPred)
  {
    uint32_t bit;
    if(idx == sps->getRPSList()->getNumberOfReferencePictureSets())
    {
      READ_UVLC(code, "delta_idx_minus1" ); // delta index of the Reference Picture Set used for prediction minus 1
    }
    else
    {
      code = 0;
    }
    rps->setDeltaRIdxMinus1(code); // th we need that for proper transcoding
    VTMCHECK(code > idx-1, "Code exceeds boundary"); // delta_idx_minus1 shall not be larger than idx-1, otherwise we will predict from a negative row position that does not exist. When idx equals 0 there is no legal value and interRPSPred must be zero. See J0185-r2
    int rIdx =  idx - 1 - code;
    VTMCHECK(rIdx > idx-1 || rIdx < 0, "Invalid index"); // Made assert tighter; if rIdx = idx then prediction is done from itself. rIdx must belong to range 0, idx-1, inclusive, see J0185-r2
    ReferencePictureSet*   rpsRef = sps->getRPSList()->getReferencePictureSet(rIdx);
    int k = 0, k0 = 0, k1 = 0;
    READ_CODE(1, bit, "delta_rps_sign"); // delta_RPS_sign
    READ_UVLC(code, "abs_delta_rps_minus1");  // absolute delta RPS minus 1
    int deltaRPS = (1 - 2 * bit) * (code + 1); // delta_RPS

    rps->setDeltaRPS( deltaRPS ); // th we need that for proper transcoding

    for(int j=0 ; j <= rpsRef->getNumberOfPictures(); j++)
    {
      READ_CODE(1, bit, "used_by_curr_pic_flag" ); //first bit is "1" if Idc is 1
      int refIdc = bit;
      if (refIdc == 0)
      {
        READ_CODE(1, bit, "use_delta_flag" ); //second bit is "1" if Idc is 2, "0" otherwise.
        refIdc = bit<<1; //second bit is "1" if refIdc is 2, "0" if refIdc = 0.
      }
      if (refIdc == 1 || refIdc == 2)
      {
        int deltaPOC = deltaRPS + ((j < rpsRef->getNumberOfPictures())? rpsRef->getDeltaPOC(j) : 0);
        rps->setDeltaPOC(k, deltaPOC);
        rps->setUsed(k, (refIdc == 1));

        if (deltaPOC < 0)
        {
          k0++;
        }
        else
        {
          k1++;
        }
        k++;
      }
      rps->setRefIdc(j,refIdc);
    }
    rps->setNumRefIdc(rpsRef->getNumberOfPictures()+1);
    rps->setNumberOfPictures(k);
    rps->setNumberOfNegativePictures(k0);
    rps->setNumberOfPositivePictures(k1);
    rps->sortDeltaPOC();
  }
  else
  {
    READ_UVLC(code, "num_negative_pics");           rps->setNumberOfNegativePictures(code);
    READ_UVLC(code, "num_positive_pics");           rps->setNumberOfPositivePictures(code);
    int prev = 0;
    int poc;
    for(int j=0 ; j < rps->getNumberOfNegativePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s0_minus1");
      poc = prev-code-1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s0_flag");  rps->setUsed(j,code);
    }
    prev = 0;
    for(int j=rps->getNumberOfNegativePictures(); j < rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s1_minus1");
      poc = prev+code+1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s1_flag");  rps->setUsed(j,code);
    }
    rps->setNumberOfPictures(rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures());
  }

  rps->printDeltaPOC();
}

void HLSyntaxReader::parsePPS( PPS* pcPPS )
{
#if ENABLE_TRACING
  xTracePPSHeader ();
#endif
  uint32_t  uiCode;

  int   iCode;

  READ_UVLC( uiCode, "pps_pic_parameter_set_id");
  VTMCHECK(uiCode > 63, "PPS id exceeds boundary (63)");
  pcPPS->setPPSId (uiCode);

  READ_UVLC( uiCode, "pps_seq_parameter_set_id");
  VTMCHECK(uiCode > 15, "SPS id exceeds boundary (15)");
  pcPPS->setSPSId (uiCode);

#if HEVC_DEPENDENT_SLICES
  READ_FLAG( uiCode, "dependent_slice_segments_enabled_flag"    );    pcPPS->setDependentSliceSegmentsEnabledFlag   ( uiCode == 1 );
#endif

  READ_FLAG( uiCode, "output_flag_present_flag" );                    pcPPS->setOutputFlagPresentFlag( uiCode==1 );

  READ_CODE(3, uiCode, "num_extra_slice_header_bits");                pcPPS->setNumExtraSliceHeaderBits(uiCode);


  READ_FLAG( uiCode,   "cabac_init_present_flag" );            pcPPS->setCabacInitPresentFlag( uiCode ? true : false );

  READ_UVLC(uiCode, "num_ref_idx_l0_default_active_minus1");
  VTMCHECK(uiCode > 14, "Invalid code read");
  pcPPS->setNumRefIdxL0DefaultActive(uiCode+1);

  READ_UVLC(uiCode, "num_ref_idx_l1_default_active_minus1");
  VTMCHECK(uiCode > 14, "Invalid code read");
  pcPPS->setNumRefIdxL1DefaultActive(uiCode+1);

  READ_SVLC(iCode, "init_qp_minus26" );                            pcPPS->setPicInitQPMinus26(iCode);
  READ_FLAG( uiCode, "constrained_intra_pred_flag" );              pcPPS->setConstrainedIntraPred( uiCode ? true : false );
  READ_FLAG( uiCode, "transform_skip_enabled_flag" );
  pcPPS->setUseTransformSkip ( uiCode ? true : false );

  READ_FLAG( uiCode, "cu_qp_delta_enabled_flag" );            pcPPS->setUseDQP( uiCode ? true : false );
  if( pcPPS->getUseDQP() )
  {
    READ_UVLC( uiCode, "diff_cu_qp_delta_depth" );
    pcPPS->setMaxCuDQPDepth( uiCode );
  }
  else
  {
    pcPPS->setMaxCuDQPDepth( 0 );
  }
  READ_SVLC( iCode, "pps_cb_qp_offset");
  pcPPS->setQpOffset(COMPONENT_Cb, iCode);
  VTMCHECK( pcPPS->getQpOffset(COMPONENT_Cb) < -12, "Invalid Cb QP offset" );
  VTMCHECK( pcPPS->getQpOffset(COMPONENT_Cb) >  12, "Invalid Cb QP offset" );

  READ_SVLC( iCode, "pps_cr_qp_offset");
  pcPPS->setQpOffset(COMPONENT_Cr, iCode);
  VTMCHECK( pcPPS->getQpOffset(COMPONENT_Cr) < -12, "Invalid Cr QP offset" );
  VTMCHECK( pcPPS->getQpOffset(COMPONENT_Cr) >  12, "Invalid Cr QP offset" );

  VTMCHECK(MAX_NUM_COMPONENT>3, "Invalid maximal number of components");

  READ_FLAG( uiCode, "pps_slice_chroma_qp_offsets_present_flag" );
  pcPPS->setSliceChromaQpFlag( uiCode ? true : false );

  READ_FLAG( uiCode, "weighted_pred_flag" );          // Use of Weighting Prediction (P_SLICE)
  pcPPS->setUseWP( uiCode==1 );
  READ_FLAG( uiCode, "weighted_bipred_flag" );         // Use of Bi-Directional Weighting Prediction (B_SLICE)
  pcPPS->setWPBiPred( uiCode==1 );

  READ_FLAG( uiCode, "transquant_bypass_enabled_flag");
  pcPPS->setTransquantBypassEnabledFlag(uiCode ? true : false);
#if HEVC_TILES_WPP
  READ_FLAG( uiCode, "tiles_enabled_flag" );    pcPPS->setTilesEnabledFlag( uiCode == 1 );
#endif
#if HEVC_TILES_WPP
  READ_FLAG( uiCode, "entropy_coding_sync_enabled_flag" );    pcPPS->setEntropyCodingSyncEnabledFlag( uiCode == 1 );

  if( pcPPS->getTilesEnabledFlag() )
  {
    READ_UVLC ( uiCode, "num_tile_columns_minus1" );                pcPPS->setNumTileColumnsMinus1( uiCode );
    READ_UVLC ( uiCode, "num_tile_rows_minus1" );                   pcPPS->setNumTileRowsMinus1( uiCode );
    READ_FLAG ( uiCode, "uniform_spacing_flag" );                   pcPPS->setTileUniformSpacingFlag( uiCode == 1 );

    const uint32_t tileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
    const uint32_t tileRowsMinus1    = pcPPS->getNumTileRowsMinus1();

    if ( !pcPPS->getTileUniformSpacingFlag())
    {
      if (tileColumnsMinus1 > 0)
      {
        std::vector<int> columnWidth(tileColumnsMinus1);
        for(uint32_t i = 0; i < tileColumnsMinus1; i++)
        {
          READ_UVLC( uiCode, "column_width_minus1" );
          columnWidth[i] = uiCode+1;
        }
        pcPPS->setTileColumnWidth(columnWidth);
      }

      if (tileRowsMinus1 > 0)
      {
        std::vector<int> rowHeight (tileRowsMinus1);
        for(uint32_t i = 0; i < tileRowsMinus1; i++)
        {
          READ_UVLC( uiCode, "row_height_minus1" );
          rowHeight[i] = uiCode + 1;
        }
        pcPPS->setTileRowHeight(rowHeight);
      }
    }
    VTMCHECK((tileColumnsMinus1 + tileRowsMinus1) == 0, "Invalid tile configuration");
    READ_FLAG ( uiCode, "loop_filter_across_tiles_enabled_flag" );     pcPPS->setLoopFilterAcrossTilesEnabledFlag( uiCode ? true : false );
  }
#endif
  READ_FLAG( uiCode, "pps_loop_filter_across_slices_enabled_flag" );   pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode ? true : false );
  READ_FLAG( uiCode, "deblocking_filter_control_present_flag" );       pcPPS->setDeblockingFilterControlPresentFlag( uiCode ? true : false );
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    READ_FLAG( uiCode, "deblocking_filter_override_enabled_flag" );    pcPPS->setDeblockingFilterOverrideEnabledFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_deblocking_filter_disabled_flag" );        pcPPS->setPPSDeblockingFilterDisabledFlag(uiCode ? true : false );
    if(!pcPPS->getPPSDeblockingFilterDisabledFlag())
    {
      READ_SVLC ( iCode, "pps_beta_offset_div2" );                     pcPPS->setDeblockingFilterBetaOffsetDiv2( iCode );
      READ_SVLC ( iCode, "pps_tc_offset_div2" );                       pcPPS->setDeblockingFilterTcOffsetDiv2( iCode );
    }
  }
#if HEVC_USE_SCALING_LISTS
  READ_FLAG( uiCode, "pps_scaling_list_data_present_flag" );           pcPPS->setScalingListPresentFlag( uiCode ? true : false );
  if(pcPPS->getScalingListPresentFlag ())
  {
    parseScalingList( &(pcPPS->getScalingList()) );
  }
#endif

  READ_FLAG( uiCode, "lists_modification_present_flag");
  pcPPS->setListsModificationPresentFlag(uiCode);

  READ_UVLC( uiCode, "log2_parallel_merge_level_minus2");
  pcPPS->setLog2ParallelMergeLevelMinus2 (uiCode);

  READ_FLAG( uiCode, "slice_segment_header_extension_present_flag");
  pcPPS->setSliceHeaderExtensionPresentFlag(uiCode);


  READ_FLAG( uiCode, "pps_extension_present_flag");
  if (uiCode)
  {
#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const char *syntaxStrings[]={ "pps_range_extension_flag",
      "pps_multilayer_extension_flag",
      "pps_extension_6bits[0]",
      "pps_extension_6bits[1]",
      "pps_extension_6bits[2]",
      "pps_extension_6bits[3]",
      "pps_extension_6bits[4]",
      "pps_extension_6bits[5]" };
#endif

    bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS];
    for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      pps_extension_flags[i] = uiCode!=0;
    }

    bool bSkipTrailingExtensionBits=false;
    for(int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (pps_extension_flags[i])
      {
        switch (PPSExtensionFlagIndex(i))
        {
        case PPS_EXT__REXT:
        {
          PPSRExt &ppsRangeExtension = pcPPS->getPpsRangeExtension();
          VTMCHECK(bSkipTrailingExtensionBits, "Invalid state");

          if (pcPPS->getUseTransformSkip())
          {
            READ_UVLC( uiCode, "log2_max_transform_skip_block_size_minus2");
            ppsRangeExtension.setLog2MaxTransformSkipBlockSize(uiCode+2);
          }

          READ_FLAG( uiCode, "cross_component_prediction_enabled_flag");
          ppsRangeExtension.setCrossComponentPredictionEnabledFlag(uiCode != 0);

          READ_FLAG( uiCode, "chroma_qp_offset_list_enabled_flag");
          if (uiCode == 0)
          {
            ppsRangeExtension.clearChromaQpOffsetList();
            ppsRangeExtension.setDiffCuChromaQpOffsetDepth(0);
          }
          else
          {
            READ_UVLC(uiCode, "diff_cu_chroma_qp_offset_depth"); ppsRangeExtension.setDiffCuChromaQpOffsetDepth(uiCode);
            uint32_t tableSizeMinus1 = 0;
            READ_UVLC(tableSizeMinus1, "chroma_qp_offset_list_len_minus1");
            VTMCHECK(tableSizeMinus1 >= MAX_QP_OFFSET_LIST_SIZE, "Table size exceeds maximum");

            for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= (tableSizeMinus1); cuChromaQpOffsetIdx++)
            {
              int cbOffset;
              int crOffset;
              READ_SVLC(cbOffset, "cb_qp_offset_list[i]");
              VTMCHECK(cbOffset < -12 || cbOffset > 12, "Invalid chroma QP offset");
              READ_SVLC(crOffset, "cr_qp_offset_list[i]");
              VTMCHECK(crOffset < -12 || crOffset > 12, "Invalid chroma QP offset");
              // table uses +1 for index (see comment inside the function)
              ppsRangeExtension.setChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1, cbOffset, crOffset);
            }
            VTMCHECK(ppsRangeExtension.getChromaQpOffsetListLen() != tableSizeMinus1 + 1, "Invalid chroma QP offset list lenght");
          }

          READ_UVLC( uiCode, "log2_sao_offset_scale_luma");
          ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_LUMA, uiCode);
          READ_UVLC( uiCode, "log2_sao_offset_scale_chroma");
          ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA, uiCode);
        }
        break;
        default:
          bSkipTrailingExtensionBits=true;
          break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "pps_extension_data_flag");
      }
    }
  }
  xReadRbspTrailingBits();
}

void  HLSyntaxReader::parseVUI(VUI* pcVUI, SPS *pcSPS)
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif
  uint32_t  uiCode;

  READ_FLAG(     uiCode, "aspect_ratio_info_present_flag");           pcVUI->setAspectRatioInfoPresentFlag(uiCode);
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    READ_CODE(8, uiCode, "aspect_ratio_idc");                         pcVUI->setAspectRatioIdc(uiCode);
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      READ_CODE(16, uiCode, "sar_width");                             pcVUI->setSarWidth(uiCode);
      READ_CODE(16, uiCode, "sar_height");                            pcVUI->setSarHeight(uiCode);
    }
  }

  READ_FLAG(     uiCode, "overscan_info_present_flag");               pcVUI->setOverscanInfoPresentFlag(uiCode);
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    READ_FLAG(   uiCode, "overscan_appropriate_flag");                pcVUI->setOverscanAppropriateFlag(uiCode);
  }

  READ_FLAG(     uiCode, "video_signal_type_present_flag");           pcVUI->setVideoSignalTypePresentFlag(uiCode);
  if (pcVUI->getVideoSignalTypePresentFlag())
  {
    READ_CODE(3, uiCode, "video_format");                             pcVUI->setVideoFormat(uiCode);
    READ_FLAG(   uiCode, "video_full_range_flag");                    pcVUI->setVideoFullRangeFlag(uiCode);
    READ_FLAG(   uiCode, "colour_description_present_flag");          pcVUI->setColourDescriptionPresentFlag(uiCode);
    if (pcVUI->getColourDescriptionPresentFlag())
    {
      READ_CODE(8, uiCode, "colour_primaries");                       pcVUI->setColourPrimaries(uiCode);
      READ_CODE(8, uiCode, "transfer_characteristics");               pcVUI->setTransferCharacteristics(uiCode);
      READ_CODE(8, uiCode, "matrix_coeffs");                          pcVUI->setMatrixCoefficients(uiCode);
    }
  }

  READ_FLAG(     uiCode, "chroma_loc_info_present_flag");             pcVUI->setChromaLocInfoPresentFlag(uiCode);
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    READ_UVLC(   uiCode, "chroma_sample_loc_type_top_field" );        pcVUI->setChromaSampleLocTypeTopField(uiCode);
    READ_UVLC(   uiCode, "chroma_sample_loc_type_bottom_field" );     pcVUI->setChromaSampleLocTypeBottomField(uiCode);
  }

  READ_FLAG(     uiCode, "neutral_chroma_indication_flag");           pcVUI->setNeutralChromaIndicationFlag(uiCode);

  READ_FLAG(     uiCode, "field_seq_flag");                           pcVUI->setFieldSeqFlag(uiCode);

  READ_FLAG(uiCode, "frame_field_info_present_flag");                 pcVUI->setFrameFieldInfoPresentFlag(uiCode);

  READ_FLAG(     uiCode, "default_display_window_flag");
  if (uiCode != 0)
  {
    Window &defDisp = pcVUI->getDefaultDisplayWindow();
    READ_UVLC(   uiCode, "def_disp_win_left_offset" );                defDisp.setWindowLeftOffset  ( uiCode * SPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_right_offset" );               defDisp.setWindowRightOffset ( uiCode * SPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_top_offset" );                 defDisp.setWindowTopOffset   ( uiCode * SPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_bottom_offset" );              defDisp.setWindowBottomOffset( uiCode * SPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
  }

  TimingInfo *timingInfo = pcVUI->getTimingInfo();
  READ_FLAG(       uiCode, "vui_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vui_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vui_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vui_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vui_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }

    READ_FLAG(     uiCode, "vui_hrd_parameters_present_flag");        pcVUI->setHrdParametersPresentFlag(uiCode);
    if( pcVUI->getHrdParametersPresentFlag() )
    {
      parseHrdParameters( pcVUI->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
  }

  READ_FLAG(     uiCode, "bitstream_restriction_flag");               pcVUI->setBitstreamRestrictionFlag(uiCode);
  if (pcVUI->getBitstreamRestrictionFlag())
  {
#if HEVC_TILES_WPP
    READ_FLAG(   uiCode, "tiles_fixed_structure_flag");               pcVUI->setTilesFixedStructureFlag(uiCode);
#endif
    READ_FLAG(   uiCode, "motion_vectors_over_pic_boundaries_flag");  pcVUI->setMotionVectorsOverPicBoundariesFlag(uiCode);
    READ_FLAG(   uiCode, "restricted_ref_pic_lists_flag");            pcVUI->setRestrictedRefPicListsFlag(uiCode);
    READ_UVLC(   uiCode, "min_spatial_segmentation_idc");             pcVUI->setMinSpatialSegmentationIdc(uiCode);
    VTMCHECK(uiCode >= 4096, "Invalid code signalled");
    READ_UVLC(   uiCode, "max_bytes_per_pic_denom" );                 pcVUI->setMaxBytesPerPicDenom(uiCode);
    READ_UVLC(   uiCode, "max_bits_per_min_cu_denom" );               pcVUI->setMaxBitsPerMinCuDenom(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_horizontal" );           pcVUI->setLog2MaxMvLengthHorizontal(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_vertical" );             pcVUI->setLog2MaxMvLengthVertical(uiCode);
  }
}

void HLSyntaxReader::parseHrdParameters(HRD *hrd, bool commonInfPresentFlag, uint32_t maxNumSubLayersMinus1)
{
  uint32_t  uiCode;
  if( commonInfPresentFlag )
  {
    READ_FLAG( uiCode, "nal_hrd_parameters_present_flag" );           hrd->setNalHrdParametersPresentFlag( uiCode == 1 ? true : false );
    READ_FLAG( uiCode, "vcl_hrd_parameters_present_flag" );           hrd->setVclHrdParametersPresentFlag( uiCode == 1 ? true : false );
    if( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() )
    {
      READ_FLAG( uiCode, "sub_pic_hrd_params_present_flag" );         hrd->setSubPicCpbParamsPresentFlag( uiCode == 1 ? true : false );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 8, uiCode, "tick_divisor_minus2" );                hrd->setTickDivisorMinus2( uiCode );
        READ_CODE( 5, uiCode, "du_cpb_removal_delay_increment_length_minus1" ); hrd->setDuCpbRemovalDelayLengthMinus1( uiCode );
        READ_FLAG( uiCode, "sub_pic_cpb_params_in_pic_timing_sei_flag" ); hrd->setSubPicCpbParamsInPicTimingSEIFlag( uiCode == 1 ? true : false );
        READ_CODE( 5, uiCode, "dpb_output_delay_du_length_minus1"  ); hrd->setDpbOutputDelayDuLengthMinus1( uiCode );
      }
      READ_CODE( 4, uiCode, "bit_rate_scale" );                       hrd->setBitRateScale( uiCode );
      READ_CODE( 4, uiCode, "cpb_size_scale" );                       hrd->setCpbSizeScale( uiCode );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 4, uiCode, "cpb_size_du_scale" );                  hrd->setDuCpbSizeScale( uiCode );
      }
      READ_CODE( 5, uiCode, "initial_cpb_removal_delay_length_minus1" ); hrd->setInitialCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "au_cpb_removal_delay_length_minus1" );      hrd->setCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "dpb_output_delay_length_minus1" );       hrd->setDpbOutputDelayLengthMinus1( uiCode );
    }
  }
  int i, j, nalOrVcl;
  for( i = 0; i <= maxNumSubLayersMinus1; i ++ )
  {
    READ_FLAG( uiCode, "fixed_pic_rate_general_flag" );                     hrd->setFixedPicRateFlag( i, uiCode == 1 ? true : false  );
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      READ_FLAG( uiCode, "fixed_pic_rate_within_cvs_flag" );                hrd->setFixedPicRateWithinCvsFlag( i, uiCode == 1 ? true : false  );
    }
    else
    {
      hrd->setFixedPicRateWithinCvsFlag( i, true );
    }

    hrd->setLowDelayHrdFlag( i, 0 ); // Infered to be 0 when not present
    hrd->setCpbCntMinus1   ( i, 0 ); // Infered to be 0 when not present

    if( hrd->getFixedPicRateWithinCvsFlag( i ) )
    {
      READ_UVLC( uiCode, "elemental_duration_in_tc_minus1" );             hrd->setPicDurationInTcMinus1( i, uiCode );
    }
    else
    {
      READ_FLAG( uiCode, "low_delay_hrd_flag" );                      hrd->setLowDelayHrdFlag( i, uiCode == 1 ? true : false  );
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      READ_UVLC( uiCode, "cpb_cnt_minus1" );                          hrd->setCpbCntMinus1( i, uiCode );
    }

    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
          ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          READ_UVLC( uiCode, "bit_rate_value_minus1" );             hrd->setBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          READ_UVLC( uiCode, "cpb_size_value_minus1" );             hrd->setCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
          if( hrd->getSubPicCpbParamsPresentFlag() )
          {
            READ_UVLC( uiCode, "cpb_size_du_value_minus1" );        hrd->setDuCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
            READ_UVLC( uiCode, "bit_rate_du_value_minus1" );        hrd->setDuBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          }
          READ_FLAG( uiCode, "cbr_flag" );                          hrd->setCbrFlag( i, j, nalOrVcl, uiCode == 1 ? true : false  );
        }
      }
    }
  }
}

#if JVET_M0427_INLOOP_RESHAPER
void HLSyntaxReader::parseReshaper(SliceReshapeInfo& info, const SPS* pcSPS, const bool isIntra)
{
  unsigned  symbol = 0;
  READ_FLAG(symbol, "tile_group_reshaper_model_present_flag");                 info.setSliceReshapeModelPresentFlag(symbol == 1);
  if (info.getSliceReshapeModelPresentFlag())
  {
    memset(info.reshaperModelBinCWDelta, 0, PIC_CODE_CW_BINS * sizeof(int));
    READ_UVLC(symbol, "reshaper_model_min_bin_idx");                             info.reshaperModelMinBinIdx = symbol;
    READ_UVLC(symbol, "reshaper_model_delta_max_bin_idx");                       info.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1 - symbol;
    READ_UVLC(symbol, "reshaper_model_bin_delta_abs_cw_prec_minus1");            info.maxNbitsNeededDeltaCW = symbol + 1;
    assert(info.maxNbitsNeededDeltaCW > 0);
    for (uint32_t i = info.reshaperModelMinBinIdx; i <= info.reshaperModelMaxBinIdx; i++)
    {
      READ_CODE(info.maxNbitsNeededDeltaCW, symbol, "reshaper_model_bin_delta_abs_CW");
      int absCW = symbol;
      if (absCW > 0)
      {
        READ_CODE(1, symbol, "reshaper_model_bin_delta_sign_CW_flag");
      }
      int signCW = symbol;
      info.reshaperModelBinCWDelta[i] = (1 - 2 * signCW) * absCW;
    }
  }
  READ_FLAG(symbol, "tile_group_reshaper_enable_flag");           info.setUseSliceReshaper(symbol == 1);
  if (info.getUseSliceReshaper())
  {
    if (!(pcSPS->getUseDualITree() && isIntra))
    {
      READ_FLAG(symbol, "slice_reshaper_ChromaAdj");                info.setSliceReshapeChromaAdj(symbol);
    }
    else
    {
      info.setSliceReshapeChromaAdj(0);
    }
  }
}
#endif
void HLSyntaxReader::parseSPS(SPS* pcSPS)
{
  uint32_t  uiCode;

#if ENABLE_TRACING
  xTraceSPSHeader ();
#endif
#if HEVC_VPS
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id");          pcSPS->setVPSId        ( uiCode );
#endif
  READ_UVLC(     uiCode, "sps_seq_parameter_set_id" );           pcSPS->setSPSId( uiCode );
  VTMCHECK(uiCode > 15, "Invalid SPS id signalled");

  READ_FLAG(uiCode, "intra_only_constraint_flag");               pcSPS->setIntraOnlyConstraintFlag(uiCode > 0 ? true : false);
  READ_CODE(4, uiCode, "max_bitdepth_constraint_idc");           pcSPS->setMaxBitDepthConstraintIdc(uiCode);
  READ_CODE(2, uiCode, "max_chroma_format_constraint_idc");      pcSPS->setMaxChromaFormatConstraintIdc(uiCode);
  READ_FLAG(uiCode, "frame_only_constraint_flag");               pcSPS->setFrameConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_qtbtt_dual_tree_intra_constraint_flag"); pcSPS->setNoQtbttDualTreeIntraConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_cclm_constraint_flag");                  pcSPS->setNoCclmConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_sao_constraint_flag");                   pcSPS->setNoSaoConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_alf_constraint_flag");                   pcSPS->setNoAlfConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_pcm_constraint_flag");                   pcSPS->setNoPcmConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_temporal_mvp_constraint_flag");          pcSPS->setNoTemporalMvpConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_sbtmvp_constraint_flag");                pcSPS->setNoSbtmvpConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_amvr_constraint_flag");                  pcSPS->setNoAmvrConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_affine_motion_constraint_flag");         pcSPS->setNoAffineMotionConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_mts_constraint_flag");                   pcSPS->setNoMtsConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_ladf_constraint_flag");                  pcSPS->setNoLadfConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_dep_quant_constraint_flag");             pcSPS->setNoDepQuantConstraintFlag(uiCode > 0 ? true : false);
  READ_FLAG(uiCode, "no_sign_data_hiding_constraint_flag");      pcSPS->setNoSignDataHidingConstraintFlag(uiCode > 0 ? true : false);

  // KJS: Marakech decision: sub-layers added back
  READ_CODE( 3,  uiCode, "sps_max_sub_layers_minus1" );          pcSPS->setMaxTLayers   ( uiCode+1 );
  VTMCHECK(uiCode > 6, "Invalid maximum number of T-layer signalled");
  READ_FLAG( uiCode, "sps_temporal_id_nesting_flag" );           pcSPS->setTemporalIdNestingFlag ( uiCode > 0 ? true : false );
  if ( pcSPS->getMaxTLayers() == 1 )
  {
    // sps_temporal_id_nesting_flag must be 1 when sps_max_sub_layers_minus1 is 0
    VTMCHECK( uiCode != 1, "Invalid maximum number of T-layers" );
  }
  parsePTL(pcSPS->getPTL(), 1, pcSPS->getMaxTLayers() - 1);

  READ_UVLC(     uiCode, "chroma_format_idc" );                  pcSPS->setChromaFormatIdc( ChromaFormat(uiCode) );
  VTMCHECK(uiCode > 3, "Invalid chroma format signalled");

  // KJS: ENABLE_CHROMA_422 does not exist anymore o.O
  if( pcSPS->getChromaFormatIdc() == CHROMA_422 )
  {
    EXIT( "Error:  4:2:2 chroma sampling format not supported with current compiler setting."
          "\n        Set compiler flag \"ENABLE_CHROMA_422\" equal to 1 for enabling 4:2:2.\n" );
  }

  if( pcSPS->getChromaFormatIdc() == CHROMA_444 )
  {
    READ_FLAG(     uiCode, "separate_colour_plane_flag");        VTMCHECK(uiCode != 0, "Invalid code");
  }

  READ_UVLC (    uiCode, "pic_width_in_luma_samples" );          pcSPS->setPicWidthInLumaSamples ( uiCode    );
  READ_UVLC (    uiCode, "pic_height_in_luma_samples" );         pcSPS->setPicHeightInLumaSamples( uiCode    );

  // KJS: not removing yet
  READ_FLAG(     uiCode, "conformance_window_flag");
  if (uiCode != 0)
  {
    Window &conf = pcSPS->getConformanceWindow();
    READ_UVLC(   uiCode, "conf_win_left_offset" );               conf.setWindowLeftOffset  ( uiCode * SPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC(   uiCode, "conf_win_right_offset" );              conf.setWindowRightOffset ( uiCode * SPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC(   uiCode, "conf_win_top_offset" );                conf.setWindowTopOffset   ( uiCode * SPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC(   uiCode, "conf_win_bottom_offset" );             conf.setWindowBottomOffset( uiCode * SPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
  }

  READ_UVLC(     uiCode, "bit_depth_luma_minus8" );
  VTMCHECK(uiCode > 8, "Invalid luma bit depth signalled");
  pcSPS->setBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);

  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (int) (6*uiCode) );

  READ_UVLC( uiCode,    "bit_depth_chroma_minus8" );
  VTMCHECK(uiCode > 8, "Invalid chroma bit depth signalled");
  pcSPS->setBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (int) (6*uiCode) );

  READ_UVLC( uiCode,    "log2_max_pic_order_cnt_lsb_minus4" );   pcSPS->setBitsForPOC( 4 + uiCode );
  VTMCHECK(uiCode > 12, "Invalid code");

  // KJS: Marakech decision: sub-layers added back
  uint32_t subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "sps_sub_layer_ordering_info_present_flag");

  for(uint32_t i=0; i <= pcSPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC ( uiCode, "sps_max_dec_pic_buffering_minus1[i]");
    pcSPS->setMaxDecPicBuffering( uiCode + 1, i);
    READ_UVLC ( uiCode, "sps_max_num_reorder_pics[i]" );
    pcSPS->setNumReorderPics(uiCode, i);
    READ_UVLC ( uiCode, "sps_max_latency_increase_plus1[i]");
    pcSPS->setMaxLatencyIncreasePlus1( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcSPS->getMaxTLayers()-1; i++)
      {
        pcSPS->setMaxDecPicBuffering(pcSPS->getMaxDecPicBuffering(0), i);
        pcSPS->setNumReorderPics(pcSPS->getNumReorderPics(0), i);
        pcSPS->setMaxLatencyIncreasePlus1(pcSPS->getMaxLatencyIncreasePlus1(0), i);
      }
      break;
    }
  }

  unsigned  minQT[3] = { 0, 0, 0 };
  unsigned  maxBTD[3] = { 0, 0, 0 };

  unsigned  maxBTSize[3] = { 0, 0, 0 };
  unsigned  maxTTSize[3] = { 0, 0, 0 };
  READ_FLAG(uiCode, "qtbtt_dual_tree_intra_flag");             pcSPS->setUseDualITree(uiCode);
  READ_UVLC(uiCode, "log2_ctu_size_minus2");                   pcSPS->setCTUSize(1 << (uiCode + 2));
  pcSPS->setMaxCodingDepth(uiCode);
  pcSPS->setLog2DiffMaxMinCodingBlockSize(uiCode);
  pcSPS->setMaxCUWidth(pcSPS->getCTUSize());
  pcSPS->setMaxCUHeight(pcSPS->getCTUSize());

  READ_UVLC(uiCode, "log2_min_luma_coding_block_size_minus2");
  int log2MinCUSize = uiCode + 2;
  pcSPS->setLog2MinCodingBlockSize(log2MinCUSize);
  READ_FLAG(uiCode, "partition_constraints_override_enabled_flag"); pcSPS->setSplitConsOverrideEnabledFlag(uiCode);
  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_intra_tile_group_luma");      minQT[0] = 1 << (uiCode + pcSPS->getLog2MinCodingBlockSize());
  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_inter_tile_group");      minQT[1] = 1 << (uiCode + pcSPS->getLog2MinCodingBlockSize());
  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_inter_tile_group");     maxBTD[1] = uiCode;
  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_intra_tile_group_luma");     maxBTD[0] = uiCode;

  maxTTSize[0] = maxBTSize[0] = minQT[0];
  if (maxBTD[0] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_intra_tile_group_luma");     maxBTSize[0] <<= uiCode;
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_intra_tile_group_luma");     maxTTSize[0] <<= uiCode;
  }
  maxTTSize[1] = maxBTSize[1] = minQT[1];
  if (maxBTD[1] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_inter_tile_group");     maxBTSize[1] <<= uiCode;
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_inter_tile_group");     maxTTSize[1] <<= uiCode;
  }
  if (pcSPS->getUseDualITree())
  {
    READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_intra_tile_group_chroma"); minQT[2] = 1 << (uiCode + pcSPS->getLog2MinCodingBlockSize());
    READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_intra_tile_group_chroma"); maxBTD[2] = uiCode;
    maxTTSize[2] = maxBTSize[2] = minQT[2];
    if (maxBTD[2] != 0)
    {
      READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_intra_tile_group_chroma");       maxBTSize[2] <<= uiCode;
      READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_intra_tile_group_chroma");       maxTTSize[2] <<= uiCode;
    }
}

  pcSPS->setMinQTSizes(minQT);
  pcSPS->setMaxBTDepth(maxBTD[1], maxBTD[0], maxBTD[2]);
  pcSPS->setMaxBTSize(maxBTSize[1], maxBTSize[0], maxBTSize[2]);
  pcSPS->setMaxTTSize(maxTTSize[1], maxTTSize[0], maxTTSize[2]);

  if (pcSPS->getPTL()->getGeneralPTL()->getLevelIdc() >= Level::LEVEL5)
  {
    VTMCHECK(log2MinCUSize + pcSPS->getLog2DiffMaxMinCodingBlockSize() < 5, "Invalid code");
  }

  // KJS: does not exist anymore -> remove!
  READ_UVLC( uiCode, "log2_min_luma_transform_block_size_minus2" );   pcSPS->setQuadtreeTULog2MinSize( uiCode + 2 );
  READ_UVLC( uiCode, "log2_diff_max_min_luma_transform_block_size" ); pcSPS->setQuadtreeTULog2MaxSize( uiCode + pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxTrSize( 1<<(uiCode + pcSPS->getQuadtreeTULog2MinSize()) );

  READ_FLAG( uiCode, "sps_sao_enabled_flag" );                      pcSPS->setSAOEnabledFlag ( uiCode ? true : false );
  READ_FLAG( uiCode, "sps_alf_enabled_flag" );                      pcSPS->setALFEnabledFlag ( uiCode ? true : false );
#if ADCNN
  pcSPS->setCNNLFEnabledFlag(true);        // always enable CNNLF
#endif
  READ_FLAG( uiCode, "sps_pcm_enabled_flag" );                          pcSPS->setPCMEnabledFlag( uiCode ? true : false );
  if( pcSPS->getPCMEnabledFlag() )
  {
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_luma_minus1" );          pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_LUMA, 1 + uiCode );
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_chroma_minus1" );        pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_CHROMA, 1 + uiCode );
    READ_UVLC( uiCode, "log2_min_pcm_luma_coding_block_size_minus3" );   pcSPS->setPCMLog2MinSize ( uiCode+3 );
    READ_UVLC( uiCode, "log2_diff_max_min_pcm_luma_coding_block_size" ); pcSPS->setPCMLog2MaxSize ( uiCode+pcSPS->getPCMLog2MinSize() );
    READ_FLAG( uiCode, "pcm_loop_filter_disable_flag" );                 pcSPS->setPCMFilterDisableFlag ( uiCode ? true : false );
  }

  READ_FLAG(uiCode, "sps_ref_wraparound_enabled_flag");                  pcSPS->setWrapAroundEnabledFlag( uiCode ? true : false );
  if (pcSPS->getWrapAroundEnabledFlag())
  {
    READ_UVLC(uiCode, "sps_ref_wraparound_offset");                      pcSPS->setWrapAroundOffset( uiCode );
  }

  READ_FLAG( uiCode, "sps_temporal_mvp_enabled_flag" );                  pcSPS->setSPSTemporalMVPEnabledFlag(uiCode);

  if ( pcSPS->getSPSTemporalMVPEnabledFlag() )
  {
    READ_FLAG( uiCode,    "sps_sbtmvp_enabled_flag" );                   pcSPS->setSBTMVPEnabledFlag      ( uiCode != 0 );
  }
  else
  {
    pcSPS->setSBTMVPEnabledFlag(false);
  }

  READ_FLAG( uiCode,  "sps_amvr_enabled_flag" );                     pcSPS->setAMVREnabledFlag ( uiCode != 0 );

  READ_FLAG( uiCode, "sps_bdof_enabled_flag" );                      pcSPS->setBDOFEnabledFlag ( uiCode != 0 );

#if JVET_M0246_AFFINE_AMVR
  READ_FLAG( uiCode,  "sps_affine_amvr_enabled_flag" );             pcSPS->setAffineAmvrEnabledFlag ( uiCode != 0 );
#endif

#if JVET_M0147_DMVR
  READ_FLAG(uiCode, "sps_dmvr_enable_flag");                        pcSPS->setUseDMVR(uiCode != 0);
#endif

  // KJS: sps_cclm_enabled_flag
  READ_FLAG( uiCode,    "lm_chroma_enabled_flag" );                 pcSPS->setUseLMChroma            ( uiCode != 0 );
#if JVET_M0142_CCLM_COLLOCATED_CHROMA
  if ( pcSPS->getUseLMChroma() && pcSPS->getChromaFormatIdc() == CHROMA_420 )
  {
    READ_FLAG( uiCode,  "sps_cclm_collocated_chroma_flag" );        pcSPS->setCclmCollocatedChromaFlag( uiCode != 0 );
  }
#endif

#if JVET_M0303_IMPLICIT_MTS
  READ_FLAG( uiCode,    "mts_enabled_flag" );                       pcSPS->setUseMTS                 ( uiCode != 0 );
  if ( pcSPS->getUseMTS() )
  {
#endif
#if JVET_M0464_UNI_MTS
    READ_FLAG( uiCode,    "mts_intra_enabled_flag" );               pcSPS->setUseIntraMTS            ( uiCode != 0 );
    READ_FLAG( uiCode,    "mts_inter_enabled_flag" );               pcSPS->setUseInterMTS            ( uiCode != 0 );
#else
    READ_FLAG( uiCode,    "emt_intra_enabled_flag" );               pcSPS->setUseIntraEMT            ( uiCode != 0 );
    READ_FLAG( uiCode,    "emt_inter_enabled_flag" );               pcSPS->setUseInterEMT            ( uiCode != 0 );
#endif
#if JVET_M0303_IMPLICIT_MTS
  }
#endif
  // KJS: sps_affine_enabled_flag
  READ_FLAG( uiCode,    "affine_flag" );                            pcSPS->setUseAffine              ( uiCode != 0 );
  if ( pcSPS->getUseAffine() )
  {
    READ_FLAG( uiCode,  "affine_type_flag" );                       pcSPS->setUseAffineType          ( uiCode != 0 );
  }
  READ_FLAG( uiCode,    "gbi_flag" );                               pcSPS->setUseGBi                 ( uiCode != 0 );
#if JVET_M0483_IBC
  READ_FLAG(uiCode, "ibc_flag");                                    pcSPS->setIBCFlag(uiCode);
#else
  READ_FLAG( uiCode, "ibc_flag");                                   pcSPS->setIBCMode                ( uiCode != 0 );
#endif
  // KJS: sps_ciip_enabled_flag
  READ_FLAG( uiCode,     "mhintra_flag" );                           pcSPS->setUseMHIntra             ( uiCode != 0 );

  READ_FLAG( uiCode,    "triangle_flag" );                          pcSPS->setUseTriangle            ( uiCode != 0 );

  // KJS: not in draft yet
#if JVET_M0255_FRACMMVD_SWITCH
  READ_FLAG( uiCode,  "sps_fracmmvd_disabled_flag" );               pcSPS->setDisFracMmvdEnabledFlag ( uiCode != 0 );
#endif
  // KJS: not in draft yet
#if JVET_M0140_SBT
  READ_FLAG(uiCode, "sbt_enable_flag");                             pcSPS->setUseSBT(uiCode != 0);
  if( pcSPS->getUseSBT() )
  {
    READ_FLAG(uiCode, "max_sbt_size_64_flag");                      pcSPS->setMaxSbtSize(uiCode != 0 ? 64 : 32);
  }
#endif
  // KJS: not in draft yet
#if JVET_M0427_INLOOP_RESHAPER
  READ_FLAG(uiCode, "sps_reshaper_enable_flag");                   pcSPS->setUseReshaper(uiCode == 1);
#endif
  
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  READ_FLAG( uiCode, "sps_ladf_enabled_flag" );                     pcSPS->setLadfEnabled( uiCode != 0 );
  if ( pcSPS->getLadfEnabled() )
  {
    int signedSymbol = 0;
    READ_CODE( 2, uiCode, "sps_num_ladf_intervals_minus2");         pcSPS->setLadfNumIntervals( uiCode + 2 );
    READ_SVLC(signedSymbol, "sps_ladf_lowest_interval_qp_offset" );      pcSPS->setLadfQpOffset( signedSymbol, 0 );
    for ( int k = 1; k < pcSPS->getLadfNumIntervals(); k++ )
    {
      READ_SVLC(signedSymbol, "sps_ladf_qp_offset" );                    pcSPS->setLadfQpOffset( signedSymbol, k );
      READ_UVLC( uiCode, "sps_ladf_delta_threshold_minus1");
      pcSPS->setLadfIntervalLowerBound(uiCode + pcSPS->getLadfIntervalLowerBound(k - 1) + 1, k);
    }
  }
#endif
 
  // KJS: reference picture sets to be replaced
  READ_UVLC( uiCode, "num_short_term_ref_pic_sets" );
  VTMCHECK(uiCode > 64, "Invalid code");
  pcSPS->createRPSList(uiCode);

  RPSList* rpsList = pcSPS->getRPSList();
  ReferencePictureSet* rps;

  for(uint32_t i=0; i< rpsList->getNumberOfReferencePictureSets(); i++)
  {
    rps = rpsList->getReferencePictureSet(i);
    parseShortTermRefPicSet(pcSPS,rps,i);
  }
  READ_FLAG( uiCode, "long_term_ref_pics_present_flag" );          pcSPS->setLongTermRefsPresent(uiCode);
  if (pcSPS->getLongTermRefsPresent())
  {
    READ_UVLC( uiCode, "num_long_term_ref_pics_sps" );
    pcSPS->setNumLongTermRefPicSPS(uiCode);
    for (uint32_t k = 0; k < pcSPS->getNumLongTermRefPicSPS(); k++)
    {
      READ_CODE( pcSPS->getBitsForPOC(), uiCode, "lt_ref_pic_poc_lsb_sps" );
      pcSPS->setLtRefPicPocLsbSps(k, uiCode);
      READ_FLAG( uiCode,  "used_by_curr_pic_lt_sps_flag[i]");
      pcSPS->setUsedByCurrPicLtSPSFlag(k, uiCode?1:0);
    }
  }

  // KJS: not found in draft -> does not exist
#if HEVC_USE_INTRA_SMOOTHING_T32 || HEVC_USE_INTRA_SMOOTHING_T64
  READ_FLAG( uiCode, "strong_intra_smoothing_enable_flag" );      pcSPS->setUseStrongIntraSmoothing(uiCode);
#endif

  // KJS: remove scaling lists?
#if HEVC_USE_SCALING_LISTS
  READ_FLAG( uiCode, "scaling_list_enabled_flag" );                 pcSPS->setScalingListFlag ( uiCode );
  if(pcSPS->getScalingListFlag())
  {
    READ_FLAG( uiCode, "sps_scaling_list_data_present_flag" );                 pcSPS->setScalingListPresentFlag ( uiCode );
    if(pcSPS->getScalingListPresentFlag ())
    {
      parseScalingList( &(pcSPS->getScalingList()) );
    }
  }
#endif

  // KJS: no VUI defined yet
  READ_FLAG( uiCode, "vui_parameters_present_flag" );             pcSPS->setVuiParametersPresentFlag(uiCode);

  if (pcSPS->getVuiParametersPresentFlag())
  {
    parseVUI(pcSPS->getVuiParameters(), pcSPS);
  }

  // KJS: no SPS extensions defined yet

  READ_FLAG( uiCode, "sps_extension_present_flag");
  if (uiCode)
  {
#if ENABLE_TRACING || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const char *syntaxStrings[]={ "sps_range_extension_flag",
      "sps_multilayer_extension_flag",
      "sps_extension_6bits[0]",
      "sps_extension_6bits[1]",
      "sps_extension_6bits[2]",
      "sps_extension_6bits[3]",
      "sps_extension_6bits[4]",
      "sps_extension_6bits[5]" };
#endif
    bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS];

    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      sps_extension_flags[i] = uiCode!=0;
    }

    bool bSkipTrailingExtensionBits=false;
    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
        case SPS_EXT__REXT:
          VTMCHECK(bSkipTrailingExtensionBits, "Skipping trailing extension bits not supported");
          {
            SPSRExt &spsRangeExtension = pcSPS->getSpsRangeExtension();
            READ_FLAG( uiCode, "transform_skip_rotation_enabled_flag");     spsRangeExtension.setTransformSkipRotationEnabledFlag(uiCode != 0);
            READ_FLAG( uiCode, "transform_skip_context_enabled_flag");      spsRangeExtension.setTransformSkipContextEnabledFlag (uiCode != 0);
            READ_FLAG( uiCode, "implicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT, (uiCode != 0));
            READ_FLAG( uiCode, "explicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT, (uiCode != 0));
            READ_FLAG( uiCode, "extended_precision_processing_flag");       spsRangeExtension.setExtendedPrecisionProcessingFlag (uiCode != 0);
            READ_FLAG( uiCode, "intra_smoothing_disabled_flag");            spsRangeExtension.setIntraSmoothingDisabledFlag      (uiCode != 0);
            READ_FLAG( uiCode, "high_precision_offsets_enabled_flag");      spsRangeExtension.setHighPrecisionOffsetsEnabledFlag (uiCode != 0);
            READ_FLAG( uiCode, "persistent_rice_adaptation_enabled_flag");  spsRangeExtension.setPersistentRiceAdaptationEnabledFlag (uiCode != 0);
            READ_FLAG( uiCode, "cabac_bypass_alignment_enabled_flag");      spsRangeExtension.setCabacBypassAlignmentEnabledFlag  (uiCode != 0);
          }
          break;
        default:
          bSkipTrailingExtensionBits=true;
          break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "sps_extension_data_flag");
      }
    }
  }
  xReadRbspTrailingBits();
}

#if HEVC_VPS
void HLSyntaxReader::parseVPS(VPS* pcVPS)
{
#if ENABLE_TRACING
  xTraceVPSHeader ();
#endif
  uint32_t  uiCode;

  READ_CODE( 4,  uiCode,  "vps_video_parameter_set_id" );         pcVPS->setVPSId( uiCode );
  READ_FLAG( uiCode,      "vps_base_layer_internal_flag" );       VTMCHECK(uiCode != 1, "Invalid code");
  READ_FLAG( uiCode,      "vps_base_layer_available_flag" );      VTMCHECK(uiCode != 1, "Invalid code");
  READ_CODE( 6,  uiCode,  "vps_max_layers_minus1" );
  READ_CODE( 3,  uiCode,  "vps_max_sub_layers_minus1" );          pcVPS->setMaxTLayers( uiCode + 1 );    VTMCHECK(uiCode+1 > MAX_TLAYER, "Invalid code");
  READ_FLAG(     uiCode,  "vps_temporal_id_nesting_flag" );       pcVPS->setTemporalNestingFlag( uiCode ? true:false );
  VTMCHECK (pcVPS->getMaxTLayers()<=1&&!pcVPS->getTemporalNestingFlag(), "Invalid VPS state");
  READ_CODE( 16, uiCode,  "vps_reserved_0xffff_16bits" );         VTMCHECK(uiCode != 0xffff, "Invalid value for reserved bits");
  parsePTL ( pcVPS->getPTL(), true, pcVPS->getMaxTLayers()-1);
  uint32_t subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "vps_sub_layer_ordering_info_present_flag");
  for(uint32_t i = 0; i <= pcVPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC( uiCode,  "vps_max_dec_pic_buffering_minus1[i]" );    pcVPS->setMaxDecPicBuffering( uiCode + 1, i );
    READ_UVLC( uiCode,  "vps_max_num_reorder_pics[i]" );            pcVPS->setNumReorderPics( uiCode, i );
    READ_UVLC( uiCode,  "vps_max_latency_increase_plus1[i]" );      pcVPS->setMaxLatencyIncrease( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcVPS->getMaxTLayers()-1; i++)
      {
        pcVPS->setMaxDecPicBuffering(pcVPS->getMaxDecPicBuffering(0), i);
        pcVPS->setNumReorderPics(pcVPS->getNumReorderPics(0), i);
        pcVPS->setMaxLatencyIncrease(pcVPS->getMaxLatencyIncrease(0), i);
      }
      break;
    }
  }

  VTMCHECK( pcVPS->getNumHrdParameters() >= MAX_VPS_OP_SETS_PLUS1, "Too many HDR parameters" );
  VTMCHECK( pcVPS->getMaxNuhReservedZeroLayerId() >= MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1, "Reserved zero layer id too big" );
  READ_CODE( 6, uiCode, "vps_max_layer_id" );                        pcVPS->setMaxNuhReservedZeroLayerId( uiCode );
  READ_UVLC(    uiCode, "vps_num_layer_sets_minus1" );               pcVPS->setMaxOpSets( uiCode + 1 );
  for( uint32_t opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx ++ )
  {
    // Operation point set
    for( uint32_t i = 0; i <= pcVPS->getMaxNuhReservedZeroLayerId(); i ++ )
    {
      READ_FLAG( uiCode, "layer_id_included_flag[opsIdx][i]" );   pcVPS->setLayerIdIncludedFlag( uiCode == 1 ? true : false, opsIdx, i );
    }
  }

  TimingInfo *timingInfo = pcVPS->getTimingInfo();
  READ_FLAG(       uiCode, "vps_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vps_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vps_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vps_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vps_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }

    READ_UVLC( uiCode, "vps_num_hrd_parameters" );                  pcVPS->setNumHrdParameters( uiCode );

    if( pcVPS->getNumHrdParameters() > 0 )
    {
      pcVPS->createHrdParamBuffer();
    }
    for( uint32_t i = 0; i < pcVPS->getNumHrdParameters(); i ++ )
    {
      READ_UVLC( uiCode, "hrd_layer_set_idx[i]" );                  pcVPS->setHrdOpSetIdx( uiCode, i );
      if( i > 0 )
      {
        READ_FLAG( uiCode, "cprms_present_flag[i]" );               pcVPS->setCprmsPresentFlag( uiCode == 1 ? true : false, i );
      }
      else
      {
        pcVPS->setCprmsPresentFlag( true, i );
      }

      parseHrdParameters(pcVPS->getHrdParameters(i), pcVPS->getCprmsPresentFlag( i ), pcVPS->getMaxTLayers() - 1);
    }
  }

  READ_FLAG( uiCode,  "vps_extension_flag" );
  if (uiCode)
  {
    while ( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "vps_extension_data_flag");
    }
  }

  xReadRbspTrailingBits();
}
#endif

void HLSyntaxReader::parseSliceHeader (Slice* pcSlice, ParameterSetManager *parameterSetManager, const int prevTid0POC)
{
  uint32_t  uiCode;
  int   iCode;

#if ENABLE_TRACING
  xTraceSliceHeader();
#endif
  PPS* pps = NULL;
  SPS* sps = NULL;

  uint32_t firstSliceSegmentInPic;
  READ_FLAG( firstSliceSegmentInPic, "first_slice_segment_in_pic_flag" );
  if( pcSlice->getRapPicFlag())
  {
    READ_FLAG( uiCode, "no_output_of_prior_pics_flag" );  //ignored -- updated already
    pcSlice->setNoOutputPriorPicsFlag(uiCode ? true : false);
  }
  READ_UVLC (    uiCode, "slice_pic_parameter_set_id" );  pcSlice->setPPSId(uiCode);
  pps = parameterSetManager->getPPS(uiCode);
  //!KS: need to add error handling code here, if PPS is not available
  VTMCHECK(pps==0, "Invalid PPS");
  sps = parameterSetManager->getSPS(pps->getSPSId());
  //!KS: need to add error handling code here, if SPS is not available
  VTMCHECK(sps==0, "Invalid SPS");

  const ChromaFormat chFmt = sps->getChromaFormatIdc();
  const uint32_t numValidComp=getNumberValidComponents(chFmt);
  const bool bChroma=(chFmt!=CHROMA_400);

#if HEVC_DEPENDENT_SLICES
  if( pps->getDependentSliceSegmentsEnabledFlag() && ( !firstSliceSegmentInPic ))
  {
    READ_FLAG( uiCode, "dependent_slice_segment_flag" );       pcSlice->setDependentSliceSegmentFlag(uiCode ? true : false);
  }
  else
  {
    pcSlice->setDependentSliceSegmentFlag(false);
  }
#endif
  int numCTUs = ((sps->getPicWidthInLumaSamples()+sps->getMaxCUWidth()-1)/sps->getMaxCUWidth())*((sps->getPicHeightInLumaSamples()+sps->getMaxCUHeight()-1)/sps->getMaxCUHeight());
  uint32_t sliceSegmentAddress = 0;
  int bitsSliceSegmentAddress = 0;
  while(numCTUs>(1<<bitsSliceSegmentAddress))
  {
    bitsSliceSegmentAddress++;
  }

  if(!firstSliceSegmentInPic)
  {
    READ_CODE( bitsSliceSegmentAddress, sliceSegmentAddress, "slice_segment_address" );
  }
  //set uiCode to equal slice start address (or dependent slice start address)
#if HEVC_DEPENDENT_SLICES
  pcSlice->setSliceSegmentCurStartCtuTsAddr( sliceSegmentAddress );// this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion table defined yet.
  pcSlice->setSliceSegmentCurEndCtuTsAddr(numCTUs);                // Set end as the last CTU of the picture.

  if (!pcSlice->getDependentSliceSegmentFlag())
  {
#endif
    pcSlice->setSliceCurStartCtuTsAddr(sliceSegmentAddress); // this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion table defined yet.
    pcSlice->setSliceCurEndCtuTsAddr(numCTUs);
#if HEVC_DEPENDENT_SLICES
  }

  if(!pcSlice->getDependentSliceSegmentFlag())
  {
#endif
    for (int i = 0; i < pps->getNumExtraSliceHeaderBits(); i++)
    {
      READ_FLAG(uiCode, "slice_reserved_flag[]"); // ignored
    }

    READ_UVLC (    uiCode, "slice_type" );            pcSlice->setSliceType((SliceType)uiCode);
    if( pps->getOutputFlagPresentFlag() )
    {
      READ_FLAG( uiCode, "pic_output_flag" );    pcSlice->setPicOutputFlag( uiCode ? true : false );
    }
    else
    {
      pcSlice->setPicOutputFlag( true );
    }

    // if (separate_colour_plane_flag == 1)
    //   read colour_plane_id
    //   (separate_colour_plane_flag == 1) is not supported in this version of the standard.

    if( pcSlice->getIdrPicFlag() )
    {
      READ_CODE(sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb");
      pcSlice->setPOC(uiCode);
      ReferencePictureSet* rps = pcSlice->getLocalRPS();
      (*rps)=ReferencePictureSet();
      pcSlice->setRPS(rps);
    }
    else
    {
      READ_CODE(sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb");
      int iPOClsb = uiCode;
      int iPrevPOC = prevTid0POC;
      int iMaxPOClsb = 1<< sps->getBitsForPOC();
      int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
      int iPrevPOCmsb = iPrevPOC-iPrevPOClsb;
      int iPOCmsb;
      if( ( iPOClsb  <  iPrevPOClsb ) && ( ( iPrevPOClsb - iPOClsb )  >=  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if( (iPOClsb  >  iPrevPOClsb )  && ( (iPOClsb - iPrevPOClsb )  >  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      }
      else
      {
        iPOCmsb = iPrevPOCmsb;
      }
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
           || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
           || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // For BLA picture types, POCmsb is set to 0.
        iPOCmsb = 0;
      }
      pcSlice->setPOC              (iPOCmsb+iPOClsb);

      ReferencePictureSet* rps;
      rps = pcSlice->getLocalRPS();
      (*rps)=ReferencePictureSet();

      pcSlice->setRPS(rps);
      READ_FLAG( uiCode, "short_term_ref_pic_set_sps_flag" );
      if(uiCode == 0) // use short-term reference picture set explicitly signalled in slice header
      {
        parseShortTermRefPicSet(sps,rps, sps->getRPSList()->getNumberOfReferencePictureSets());
      }
      else // use reference to short-term reference picture set in PPS
      {
        int numBits = 0;
        while ((1 << numBits) < sps->getRPSList()->getNumberOfReferencePictureSets())
        {
          numBits++;
        }
        if (numBits > 0)
        {
          READ_CODE( numBits, uiCode, "short_term_ref_pic_set_idx");
        }
        else
        {
          uiCode = 0;

        }
        *rps = *(sps->getRPSList()->getReferencePictureSet(uiCode));
      }
      if(sps->getLongTermRefsPresent())
      {
        int offset = rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures();
        uint32_t numOfLtrp = 0;
        uint32_t numLtrpInSPS = 0;
        if (sps->getNumLongTermRefPicSPS() > 0)
        {
          READ_UVLC( uiCode, "num_long_term_sps");
          numLtrpInSPS = uiCode;
          numOfLtrp += numLtrpInSPS;
          rps->setNumberOfLongtermPictures(numOfLtrp);
        }
        int bitsForLtrpInSPS = 0;
        while (sps->getNumLongTermRefPicSPS() > (1 << bitsForLtrpInSPS))
        {
          bitsForLtrpInSPS++;
        }
        READ_UVLC( uiCode, "num_long_term_pics");             rps->setNumberOfLongtermPictures(uiCode);
        numOfLtrp += uiCode;
        rps->setNumberOfLongtermPictures(numOfLtrp);
        int maxPicOrderCntLSB = 1 << sps->getBitsForPOC();
        int prevDeltaMSB = 0, deltaPocMSBCycleLT = 0;
        for(int j=offset+rps->getNumberOfLongtermPictures()-1, k = 0; k < numOfLtrp; j--, k++)
        {
          int pocLsbLt;
          if (k < numLtrpInSPS)
          {
            uiCode = 0;
            if (bitsForLtrpInSPS > 0)
            {
              READ_CODE(bitsForLtrpInSPS, uiCode, "lt_idx_sps[i]");
            }
            bool usedByCurrFromSPS=sps->getUsedByCurrPicLtSPSFlag(uiCode);

            pocLsbLt = sps->getLtRefPicPocLsbSps(uiCode);
            rps->setUsed(j,usedByCurrFromSPS);
          }
          else
          {
            READ_CODE(sps->getBitsForPOC(), uiCode, "poc_lsb_lt"); pocLsbLt= uiCode;
            READ_FLAG( uiCode, "used_by_curr_pic_lt_flag");     rps->setUsed(j,uiCode);
          }
          READ_FLAG(uiCode,"delta_poc_msb_present_flag");
          bool mSBPresentFlag = uiCode ? true : false;
          if(mSBPresentFlag)
          {
            READ_UVLC( uiCode, "delta_poc_msb_cycle_lt[i]" );
            bool deltaFlag = false;
            //            First LTRP                               || First LTRP from SH
            if( (j == offset+rps->getNumberOfLongtermPictures()-1) || (j == offset+(numOfLtrp-numLtrpInSPS)-1) )
            {
              deltaFlag = true;
            }
            if(deltaFlag)
            {
              deltaPocMSBCycleLT = uiCode;
            }
            else
            {
              deltaPocMSBCycleLT = uiCode + prevDeltaMSB;
            }

            int pocLTCurr = pcSlice->getPOC() - deltaPocMSBCycleLT * maxPicOrderCntLSB
              - iPOClsb + pocLsbLt;
            rps->setPOC     (j, pocLTCurr);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLTCurr);
            rps->setCheckLTMSBPresent(j,true);
          }
          else
          {
            rps->setPOC     (j, pocLsbLt);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLsbLt);
            rps->setCheckLTMSBPresent(j,false);

            // reset deltaPocMSBCycleLT for first LTRP from slice header if MSB not present
            if( j == offset+(numOfLtrp-numLtrpInSPS)-1 )
            {
              deltaPocMSBCycleLT = 0;
            }
          }
          prevDeltaMSB = deltaPocMSBCycleLT;
        }
        offset += rps->getNumberOfLongtermPictures();
        rps->setNumberOfPictures(offset);
      }
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
           || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
           || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // In the case of BLA picture types, rps data is read from slice header but ignored
        rps = pcSlice->getLocalRPS();
        (*rps)=ReferencePictureSet();
        pcSlice->setRPS(rps);
      }
      if (sps->getSPSTemporalMVPEnabledFlag())
      {
        READ_FLAG( uiCode, "slice_temporal_mvp_enabled_flag" );
        pcSlice->setEnableTMVPFlag( uiCode == 1 ? true : false );
      }
      else
      {
        pcSlice->setEnableTMVPFlag(false);
      }
    }



    if(sps->getSAOEnabledFlag())
    {
      READ_FLAG(uiCode, "slice_sao_luma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, (bool)uiCode);

      if (bChroma)
      {
        READ_FLAG(uiCode, "slice_sao_chroma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, (bool)uiCode);
      }
    }

    if( sps->getALFEnabledFlag() )
    {
      alf( pcSlice->getAlfSliceParam() );
    }

#if ADCNN
	if (sps->getCNNLFEnabledFlag())
	{
		cnnlf(pcSlice->getCnnlfSliceParam());
	}
#endif

    if (pcSlice->getIdrPicFlag())
    {
      pcSlice->setEnableTMVPFlag(false);
    }
    if (!pcSlice->isIntra())
    {

      READ_FLAG( uiCode, "num_ref_idx_active_override_flag");
      if (uiCode)
      {
        READ_UVLC (uiCode, "num_ref_idx_l0_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_0, uiCode + 1 );
        if (pcSlice->isInterB())
        {
          READ_UVLC (uiCode, "num_ref_idx_l1_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_1, uiCode + 1 );
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
        }
      }
      else
      {
        pcSlice->setNumRefIdx(REF_PIC_LIST_0, pps->getNumRefIdxL0DefaultActive());
        if (pcSlice->isInterB())
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, pps->getNumRefIdxL1DefaultActive());
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1,0);
        }
      }
    }
    // }
    RefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    if(!pcSlice->isIntra())
    {
      if( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 )
      {
        refPicListModification->setRefPicListModificationFlagL0( 0 );
      }
      else
      {
        READ_FLAG( uiCode, "ref_pic_list_modification_flag_l0" ); refPicListModification->setRefPicListModificationFlagL0( uiCode ? 1 : 0 );
      }

      if(refPicListModification->getRefPicListModificationFlagL0())
      {
        uiCode = 0;
        int i = 0;
        int numRpsCurrTempList0 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList0 > 1 )
        {
          int length = 1;
          numRpsCurrTempList0 --;
          while ( numRpsCurrTempList0 >>= 1)
          {
            length ++;
          }
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
          {
            READ_CODE( length, uiCode, "list_entry_l0" );
            refPicListModification->setRefPicSetIdxL0(i, uiCode );
          }
        }
        else
        {
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
          {
            refPicListModification->setRefPicSetIdxL0(i, 0 );
          }
        }
      }
    }
    else
    {
      refPicListModification->setRefPicListModificationFlagL0(0);
    }
    if(pcSlice->isInterB())
    {
      if( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 )
      {
        refPicListModification->setRefPicListModificationFlagL1( 0 );
      }
      else
      {
        READ_FLAG( uiCode, "ref_pic_list_modification_flag_l1" ); refPicListModification->setRefPicListModificationFlagL1( uiCode ? 1 : 0 );
      }
      if(refPicListModification->getRefPicListModificationFlagL1())
      {
        uiCode = 0;
        int i = 0;
        int numRpsCurrTempList1 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList1 > 1 )
        {
          int length = 1;
          numRpsCurrTempList1 --;
          while ( numRpsCurrTempList1 >>= 1)
          {
            length ++;
          }
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
          {
            READ_CODE( length, uiCode, "list_entry_l1" );
            refPicListModification->setRefPicSetIdxL1(i, uiCode );
          }
        }
        else
        {
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
          {
            refPicListModification->setRefPicSetIdxL1(i, 0 );
          }
        }
      }
    }
    else
    {
      refPicListModification->setRefPicListModificationFlagL1(0);
    }
    if (pcSlice->isInterB())
    {
      READ_FLAG( uiCode, "mvd_l1_zero_flag" );       pcSlice->setMvdL1ZeroFlag( (uiCode ? true : false) );
    }

    pcSlice->setCabacInitFlag( false ); // default
    if(pps->getCabacInitPresentFlag() && !pcSlice->isIntra())
    {
      READ_FLAG(uiCode, "cabac_init_flag");
      pcSlice->setCabacInitFlag( uiCode ? true : false );
      pcSlice->setEncCABACTableIdx( pcSlice->getSliceType() == B_SLICE ? ( uiCode ? P_SLICE : B_SLICE ) : ( uiCode ? B_SLICE : P_SLICE ) );
    }

    if ( pcSlice->getEnableTMVPFlag() )
    {
      if ( pcSlice->getSliceType() == B_SLICE )
      {
        READ_FLAG( uiCode, "collocated_from_l0_flag" );
        pcSlice->setColFromL0Flag(uiCode);
      }
      else
      {
        pcSlice->setColFromL0Flag( 1 );
      }

      if ( pcSlice->getSliceType() != I_SLICE &&
           ((pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx(REF_PIC_LIST_0) > 1)||
           (pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx(REF_PIC_LIST_1) > 1)))
      {
        READ_UVLC( uiCode, "collocated_ref_idx" );
        pcSlice->setColRefIdx(uiCode);
      }
      else
      {
        pcSlice->setColRefIdx(0);
      }
    }
    if ( (pps->getUseWP() && pcSlice->getSliceType()==P_SLICE) || (pps->getWPBiPred() && pcSlice->getSliceType()==B_SLICE) )
    {
      parsePredWeightTable(pcSlice, sps);
      pcSlice->initWpScaling(sps);
    }
    READ_FLAG( uiCode, "dep_quant_enable_flag" );
    pcSlice->setDepQuantEnabledFlag( uiCode != 0 );
#if HEVC_USE_SIGN_HIDING
    if( !pcSlice->getDepQuantEnabledFlag() )
    {
      READ_FLAG( uiCode, "sign_data_hiding_enable_flag" );
      pcSlice->setSignDataHidingEnabledFlag( uiCode != 0 );
    }
#endif
    if (
      sps->getSplitConsOverrideEnabledFlag()
      )
    {
      READ_FLAG(uiCode, "partition_constrainst_override_flag");        pcSlice->setSplitConsOverrideFlag(uiCode ? true : false);
      if (pcSlice->getSplitConsOverrideFlag())
      {
        READ_UVLC(uiCode, "log2_diff_min_qt_min_cb");                 pcSlice->setMinQTSize(1 << (uiCode + sps->getLog2MinCodingBlockSize()));
        READ_UVLC(uiCode, "max_mtt_hierarchy_depth");                 pcSlice->setMaxBTDepth(uiCode);
        if (pcSlice->getMaxBTDepth() != 0)
        {
          READ_UVLC(uiCode, "log2_diff_max_bt_min_qt");             pcSlice->setMaxBTSize(pcSlice->getMinQTSize() << uiCode);
          READ_UVLC(uiCode, "log2_diff_max_tt_min_qt");             pcSlice->setMaxTTSize(pcSlice->getMinQTSize() << uiCode);
        }
        else
        {
          pcSlice->setMaxBTSize(pcSlice->getMinQTSize());
          pcSlice->setMaxTTSize(pcSlice->getMinQTSize());
        }
        if (
          pcSlice->isIntra() && sps->getUseDualITree()
          )
        {
          READ_UVLC(uiCode, "log2_diff_min_qt_min_cb_chroma");                 pcSlice->setMinQTSizeIChroma(1 << (uiCode + sps->getLog2MinCodingBlockSize()));
          READ_UVLC(uiCode, "max_mtt_hierarchy_depth_chroma");                            pcSlice->setMaxBTDepthIChroma(uiCode);
          if (pcSlice->getMaxBTDepthIChroma() != 0)
          {
            READ_UVLC(uiCode, "log2_diff_max_bt_min_qt_chroma");             pcSlice->setMaxBTSizeIChroma(pcSlice->getMinQTSizeIChroma() << uiCode);
            READ_UVLC(uiCode, "log2_diff_max_tt_min_qt_chroma");             pcSlice->setMaxTTSizeIChroma(pcSlice->getMinQTSizeIChroma() << uiCode);
          }
          else
          {
            pcSlice->setMaxBTSizeIChroma(pcSlice->getMinQTSizeIChroma());
            pcSlice->setMaxTTSizeIChroma(pcSlice->getMinQTSizeIChroma());
          }
        }
      }
    }

#if JVET_M0483_IBC
    if (!pcSlice->isIntra() || sps->getIBCFlag())
    {
      READ_UVLC(uiCode, "six_minus_max_num_merge_cand");
      pcSlice->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - uiCode);
    }
#endif

    if (!pcSlice->isIntra())
    {
#if JVET_M0483_IBC==0
      READ_UVLC(uiCode, "six_minus_max_num_merge_cand");
      pcSlice->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - uiCode);
#endif

      if ( sps->getSBTMVPEnabledFlag() && !sps->getUseAffine() ) // ATMVP only
      {
        pcSlice->setMaxNumAffineMergeCand( 1 );
      }
      else if ( !sps->getSBTMVPEnabledFlag() && !sps->getUseAffine() ) // both off
      {
        pcSlice->setMaxNumAffineMergeCand( 0 );
      }
      else
      if ( sps->getUseAffine() )
      {
        READ_UVLC( uiCode, "five_minus_max_num_affine_merge_cand" );
        pcSlice->setMaxNumAffineMergeCand( AFFINE_MRG_MAX_NUM_CANDS - uiCode );
      }
#if JVET_M0255_FRACMMVD_SWITCH
      if ( sps->getDisFracMmvdEnabledFlag() )
      {
        READ_FLAG( uiCode, "tile_group_fracmmvd_disabled_flag" );
        pcSlice->setDisFracMMVD( uiCode ? true : false );
      }
#endif
    }

    READ_SVLC( iCode, "slice_qp_delta" );
    pcSlice->setSliceQp (26 + pps->getPicInitQPMinus26() + iCode);
    pcSlice->setSliceQpBase( pcSlice->getSliceQp() );

    VTMCHECK( pcSlice->getSliceQp() < -sps->getQpBDOffset(CHANNEL_TYPE_LUMA), "Invalid slice QP delta" );
    VTMCHECK( pcSlice->getSliceQp() > MAX_QP, "Invalid slice QP" );

    if (pps->getSliceChromaQpFlag())
    {
      if (numValidComp>COMPONENT_Cb)
      {
        READ_SVLC( iCode, "slice_cb_qp_offset" );
        pcSlice->setSliceChromaQpDelta(COMPONENT_Cb, iCode );
        VTMCHECK( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb) < -12, "Invalid chroma QP offset" );
        VTMCHECK( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb) >  12, "Invalid chroma QP offset" );
        VTMCHECK( (pps->getQpOffset(COMPONENT_Cb) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cb)) < -12, "Invalid chroma QP offset" );
        VTMCHECK( (pps->getQpOffset(COMPONENT_Cb) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cb)) >  12, "Invalid chroma QP offset" );
      }

      if (numValidComp>COMPONENT_Cr)
      {
        READ_SVLC( iCode, "slice_cr_qp_offset" );
        pcSlice->setSliceChromaQpDelta(COMPONENT_Cr, iCode );
        VTMCHECK( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr) < -12, "Invalid chroma QP offset" );
        VTMCHECK( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr) >  12, "Invalid chroma QP offset" );
        VTMCHECK( (pps->getQpOffset(COMPONENT_Cr) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cr)) < -12, "Invalid chroma QP offset" );
        VTMCHECK( (pps->getQpOffset(COMPONENT_Cr) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cr)) >  12, "Invalid chroma QP offset" );
      }
    }

    if (pps->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag())
    {
      READ_FLAG(uiCode, "cu_chroma_qp_offset_enabled_flag"); pcSlice->setUseChromaQpAdj(uiCode != 0);
    }
    else
    {
      pcSlice->setUseChromaQpAdj(false);
    }

    if (pps->getDeblockingFilterControlPresentFlag())
    {
      if(pps->getDeblockingFilterOverrideEnabledFlag())
      {
        READ_FLAG ( uiCode, "deblocking_filter_override_flag" );        pcSlice->setDeblockingFilterOverrideFlag(uiCode ? true : false);
      }
      else
      {
        pcSlice->setDeblockingFilterOverrideFlag(0);
      }
      if(pcSlice->getDeblockingFilterOverrideFlag())
      {
        READ_FLAG ( uiCode, "slice_deblocking_filter_disabled_flag" );   pcSlice->setDeblockingFilterDisable(uiCode ? 1 : 0);
        if(!pcSlice->getDeblockingFilterDisable())
        {
          READ_SVLC( iCode, "slice_beta_offset_div2" );                       pcSlice->setDeblockingFilterBetaOffsetDiv2(iCode);
          VTMCHECK(  pcSlice->getDeblockingFilterBetaOffsetDiv2() < -6 &&
                  pcSlice->getDeblockingFilterBetaOffsetDiv2() >  6, "Invalid deblocking filter configuration");
          READ_SVLC( iCode, "slice_tc_offset_div2" );                         pcSlice->setDeblockingFilterTcOffsetDiv2(iCode);
          VTMCHECK  (pcSlice->getDeblockingFilterTcOffsetDiv2() < -6 &&
                  pcSlice->getDeblockingFilterTcOffsetDiv2() >  6, "Invalid deblocking filter configuration");
        }
      }
      else
      {
        pcSlice->setDeblockingFilterDisable       ( pps->getPPSDeblockingFilterDisabledFlag() );
        pcSlice->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( pps->getDeblockingFilterTcOffsetDiv2() );
      }
    }
    else
    {
      pcSlice->setDeblockingFilterDisable       ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterTcOffsetDiv2  ( 0 );
    }

    bool isSAOEnabled = sps->getSAOEnabledFlag() && (pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_LUMA) || (bChroma && pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA)));
    bool isDBFEnabled = (!pcSlice->getDeblockingFilterDisable());

    if(pps->getLoopFilterAcrossSlicesEnabledFlag() && ( isSAOEnabled || isDBFEnabled ))
    {
      READ_FLAG( uiCode, "slice_loop_filter_across_slices_enabled_flag");
    }
    else
    {
      uiCode = pps->getLoopFilterAcrossSlicesEnabledFlag()?1:0;
    }
    pcSlice->setLFCrossSliceBoundaryFlag( (uiCode==1)?true:false);

#if JVET_M0427_INLOOP_RESHAPER
    if (sps->getUseReshaper())
    {
      parseReshaper(pcSlice->getReshapeInfo(), sps, pcSlice->isIntra());
    }
#endif
#if HEVC_DEPENDENT_SLICES
  }
#endif

  if( firstSliceSegmentInPic )
  {
    pcSlice->setDefaultClpRng( *sps );

  }

  if(pps->getSliceHeaderExtensionPresentFlag())
  {
    READ_UVLC(uiCode,"slice_segment_header_extension_length");
    for(int i=0; i<uiCode; i++)
    {
      uint32_t ignore_;
      READ_CODE(8,ignore_,"slice_segment_header_extension_data_byte");
    }
  }


#if HEVC_TILES_WPP
  std::vector<uint32_t> entryPointOffset;
  if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
  {
    uint32_t numEntryPointOffsets;
    uint32_t offsetLenMinus1;
    READ_UVLC( numEntryPointOffsets, "num_entry_point_offsets" );
    if( numEntryPointOffsets > 0 )
    {
      READ_UVLC( offsetLenMinus1, "offset_len_minus1" );
      entryPointOffset.resize( numEntryPointOffsets );
      for( uint32_t idx = 0; idx < numEntryPointOffsets; idx++ )
      {
        READ_CODE( offsetLenMinus1 + 1, uiCode, "entry_point_offset_minus1" );
        entryPointOffset[idx] = uiCode + 1;
      }
    }
  }
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,m_pcBitstream->readByteAlignment(),0);
#else
  m_pcBitstream->readByteAlignment();
#endif

  pcSlice->clearSubstreamSizes();

#if HEVC_TILES_WPP
  if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
  {
    int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

    // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
    for ( uint32_t curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
    {
      if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) < endOfSliceHeaderLocation )
      {
        endOfSliceHeaderLocation++;
      }
    }

    int  curEntryPointOffset     = 0;
    int  prevEntryPointOffset    = 0;
    for (uint32_t idx=0; idx<entryPointOffset.size(); idx++)
    {
      curEntryPointOffset += entryPointOffset[ idx ];

      int emulationPreventionByteCount = 0;
      for ( uint32_t curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
      {
        if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) >= ( prevEntryPointOffset + endOfSliceHeaderLocation ) &&
             m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) <  ( curEntryPointOffset  + endOfSliceHeaderLocation ) )
        {
          emulationPreventionByteCount++;
        }
      }

      entryPointOffset[ idx ] -= emulationPreventionByteCount;
      prevEntryPointOffset = curEntryPointOffset;
      pcSlice->addSubstreamSize(entryPointOffset [ idx ] );
    }
  }
#endif
  return;
}



void HLSyntaxReader::parsePTL( PTL *rpcPTL, bool profilePresentFlag, int maxNumSubLayersMinus1 )
{
  uint32_t uiCode;
  if(profilePresentFlag)
  {
    parseProfileTier(rpcPTL->getGeneralPTL(), false);
  }
  READ_CODE( 8, uiCode, "general_level_idc" );    rpcPTL->getGeneralPTL()->setLevelIdc(Level::Name(uiCode));

  for (int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    READ_FLAG( uiCode, "sub_layer_profile_present_flag[i]" ); rpcPTL->setSubLayerProfilePresentFlag(i, uiCode);
    READ_FLAG( uiCode, "sub_layer_level_present_flag[i]"   ); rpcPTL->setSubLayerLevelPresentFlag  (i, uiCode);
  }

  if (maxNumSubLayersMinus1 > 0)
  {
    for (int i = maxNumSubLayersMinus1; i < 8; i++)
    {
      READ_CODE(2, uiCode, "reserved_zero_2bits");
      VTMCHECK(uiCode != 0, "Invalid code");
    }
  }

  for(int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    if( rpcPTL->getSubLayerProfilePresentFlag(i) )
    {
      parseProfileTier(rpcPTL->getSubLayerPTL(i), true);
    }
    if(rpcPTL->getSubLayerLevelPresentFlag(i))
    {
      READ_CODE( 8, uiCode, "sub_layer_level_idc[i]" );   rpcPTL->getSubLayerPTL(i)->setLevelIdc(Level::Name(uiCode));
    }
  }
}

#if ENABLE_TRACING|| RExt__DECODER_DEBUG_BIT_STATISTICS
void HLSyntaxReader::parseProfileTier(ProfileTierLevel *ptl, const bool bIsSubLayer)
#define PTL_TRACE_TEXT(txt) bIsSubLayer?("sub_layer_" txt) : ("general_" txt)
#else
void HLSyntaxReader::parseProfileTier(ProfileTierLevel *ptl, const bool /*bIsSubLayer*/)
#define PTL_TRACE_TEXT(txt) txt
#endif
{
  uint32_t uiCode;
  READ_CODE(2 , uiCode,   PTL_TRACE_TEXT("profile_space"                   )); ptl->setProfileSpace(uiCode);
  READ_FLAG(    uiCode,   PTL_TRACE_TEXT("tier_flag"                       )); ptl->setTierFlag    (uiCode ? Level::HIGH : Level::MAIN);
  READ_CODE(5 , uiCode,   PTL_TRACE_TEXT("profile_idc"                     )); ptl->setProfileIdc  (Profile::Name(uiCode));
  for(int j = 0; j < 32; j++)
  {
    READ_FLAG(  uiCode,   PTL_TRACE_TEXT("profile_compatibility_flag[][j]" )); ptl->setProfileCompatibilityFlag(j, uiCode ? 1 : 0);
  }
  READ_FLAG(uiCode,       PTL_TRACE_TEXT("progressive_source_flag"         )); ptl->setProgressiveSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("interlaced_source_flag"          )); ptl->setInterlacedSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("non_packed_constraint_flag"      )); ptl->setNonPackedConstraintFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("frame_only_constraint_flag"      )); ptl->setFrameOnlyConstraintFlag(uiCode ? true : false);

  if (ptl->getProfileIdc() == Profile::MAINREXT           || ptl->getProfileCompatibilityFlag(Profile::MAINREXT) ||
      ptl->getProfileIdc() == Profile::HIGHTHROUGHPUTREXT || ptl->getProfileCompatibilityFlag(Profile::HIGHTHROUGHPUTREXT))
  {
    uint32_t maxBitDepth=16;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_12bit_constraint_flag"       )); if (uiCode) maxBitDepth=12;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_10bit_constraint_flag"       )); if (uiCode) maxBitDepth=10;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_8bit_constraint_flag"        )); if (uiCode) maxBitDepth=8;
    ptl->setBitDepthConstraint(maxBitDepth);
    ChromaFormat chromaFmtConstraint=CHROMA_444;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_422chroma_constraint_flag"   )); if (uiCode) chromaFmtConstraint=CHROMA_422;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_420chroma_constraint_flag"   )); if (uiCode) chromaFmtConstraint=CHROMA_420;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_monochrome_constraint_flag"  )); if (uiCode) chromaFmtConstraint=CHROMA_400;
    ptl->setChromaFormatConstraint(chromaFmtConstraint);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("intra_constraint_flag"           )); ptl->setIntraConstraintFlag(uiCode != 0);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("one_picture_only_constraint_flag")); ptl->setOnePictureOnlyConstraintFlag(uiCode != 0);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("lower_bit_rate_constraint_flag"  )); ptl->setLowerBitRateConstraintFlag(uiCode != 0);
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[0..15]"     ));
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[16..31]"    ));
    READ_CODE(2,  uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[32..33]"    ));
  }
  else
  {
    ptl->setBitDepthConstraint( ( ptl->getProfileIdc() == Profile::MAIN10 || ptl->getProfileIdc() == Profile::NEXT ) ? 10 : 8 );
    ptl->setChromaFormatConstraint(CHROMA_420);
    ptl->setIntraConstraintFlag(false);
    ptl->setLowerBitRateConstraintFlag(true);
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[0..15]"     ));
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[16..31]"    ));
    READ_CODE(11, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[32..42]"    ));
  }

  if ((ptl->getProfileIdc() >= Profile::MAIN && ptl->getProfileIdc() <= Profile::HIGHTHROUGHPUTREXT) ||
       ptl->getProfileCompatibilityFlag(Profile::MAIN) ||
       ptl->getProfileCompatibilityFlag(Profile::MAIN10) ||
       ptl->getProfileCompatibilityFlag(Profile::MAINSTILLPICTURE) ||
       ptl->getProfileCompatibilityFlag(Profile::MAINREXT) ||
       ptl->getProfileCompatibilityFlag(Profile::HIGHTHROUGHPUTREXT) )
  {
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("inbld_flag"                      )); VTMCHECK(uiCode != 0, "Invalid code");
  }
  else
  {
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("reserved_zero_bit"               ));
  }
#undef PTL_TRACE_TEXT
}

void HLSyntaxReader::parseTerminatingBit( uint32_t& ruiBit )
{
  ruiBit = false;
  int iBitsLeft = m_pcBitstream->getNumBitsLeft();
  if(iBitsLeft <= 8)
  {
    uint32_t uiPeekValue = m_pcBitstream->peekBits(iBitsLeft);
    if (uiPeekValue == (1<<(iBitsLeft-1)))
    {
      ruiBit = true;
    }
  }
}

void HLSyntaxReader::parseRemainingBytes( bool noTrailingBytesExpected )
{
  if (noTrailingBytesExpected)
  {
    VTMCHECK( 0 != m_pcBitstream->getNumBitsLeft(), "Bits left although no bits expected" );
  }
  else
  {
    while (m_pcBitstream->getNumBitsLeft())
    {
      uint32_t trailingNullByte=m_pcBitstream->readByte();
      if (trailingNullByte!=0)
      {
        msg( VTMERROR, "Trailing byte should be 0, but has value %02x\n", trailingNullByte);
        THROW("Invalid trailing '0' byte");
      }
    }
  }
}




// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! parse explicit wp tables
void HLSyntaxReader::parsePredWeightTable( Slice* pcSlice, const SPS *sps )
{
  WPScalingParam *wp;
  const ChromaFormat    chFmt        = sps->getChromaFormatIdc();
  const int             numValidComp = int(getNumberValidComponents(chFmt));
  const bool            bChroma      = (chFmt!=CHROMA_400);
  const SliceType       eSliceType   = pcSlice->getSliceType();
  const int             iNbRef       = (eSliceType == B_SLICE ) ? (2) : (1);
  uint32_t            uiLog2WeightDenomLuma=0, uiLog2WeightDenomChroma=0;
  uint32_t            uiTotalSignalledWeightFlags = 0;

  int iDeltaDenom;
  // decode delta_luma_log2_weight_denom :
  READ_UVLC( uiLog2WeightDenomLuma, "luma_log2_weight_denom" );
  VTMCHECK( uiLog2WeightDenomLuma > 7, "Invalid code" );
  if( bChroma )
  {
    READ_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
    VTMCHECK((iDeltaDenom + (int)uiLog2WeightDenomLuma)<0, "Invalid code");
    VTMCHECK((iDeltaDenom + (int)uiLog2WeightDenomLuma)>7, "Invalid code");
    uiLog2WeightDenomChroma = (uint32_t)(iDeltaDenom + uiLog2WeightDenomLuma);
  }

  for ( int iNumRef=0 ; iNumRef<iNbRef ; iNumRef++ ) // loop over l0 and l1 syntax elements
  {
    RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);

      wp[COMPONENT_Y].uiLog2WeightDenom = uiLog2WeightDenomLuma;
      for(int j=1; j<numValidComp; j++)
      {
        wp[j].uiLog2WeightDenom = uiLog2WeightDenomChroma;
      }

      uint32_t  uiCode;
      READ_FLAG( uiCode, iNumRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
      wp[COMPONENT_Y].bPresentFlag = ( uiCode == 1 );
      uiTotalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
    }
    if ( bChroma )
    {
      uint32_t  uiCode;
      for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        READ_FLAG( uiCode, iNumRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
        for(int j=1; j<numValidComp; j++)
        {
          wp[j].bPresentFlag = ( uiCode == 1 );
        }
        uiTotalSignalledWeightFlags += 2*wp[COMPONENT_Cb].bPresentFlag;
      }
    }
    for ( int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
      if ( wp[COMPONENT_Y].bPresentFlag )
      {
        int iDeltaWeight;
        READ_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
        VTMCHECK( iDeltaWeight < -128, "Invalid code" );
        VTMCHECK( iDeltaWeight >  127, "Invalid code" );
        wp[COMPONENT_Y].iWeight = (iDeltaWeight + (1<<wp[COMPONENT_Y].uiLog2WeightDenom));
        READ_SVLC( wp[COMPONENT_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
        const int range=sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<sps->getBitDepth(CHANNEL_TYPE_LUMA))/2 : 128;
        if( wp[0].iOffset < -range ) { THROW("Offset out of range"); }
        if( wp[0].iOffset >= range ) { THROW("Offset out of range"); }
      }
      else
      {
        wp[COMPONENT_Y].iWeight = (1 << wp[COMPONENT_Y].uiLog2WeightDenom);
        wp[COMPONENT_Y].iOffset = 0;
      }
      if ( bChroma )
      {
        if ( wp[COMPONENT_Cb].bPresentFlag )
        {
          int range=sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<sps->getBitDepth(CHANNEL_TYPE_CHROMA))/2 : 128;
          for ( int j=1 ; j<numValidComp ; j++ )
          {
            int iDeltaWeight;
            READ_SVLC( iDeltaWeight, iNumRef==0?"delta_chroma_weight_l0[i]":"delta_chroma_weight_l1[i]" );
            VTMCHECK( iDeltaWeight < -128, "Invalid code" );
            VTMCHECK( iDeltaWeight >  127, "Invalid code" );
            wp[j].iWeight = (iDeltaWeight + (1<<wp[j].uiLog2WeightDenom));

            int iDeltaChroma;
            READ_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
            VTMCHECK( iDeltaChroma <  -4*range, "Invalid code" );
            VTMCHECK( iDeltaChroma >=  4*range, "Invalid code" );
            int pred = ( range - ( ( range*wp[j].iWeight)>>(wp[j].uiLog2WeightDenom) ) );
            wp[j].iOffset = Clip3(-range, range-1, (iDeltaChroma + pred) );
          }
        }
        else
        {
          for ( int j=1 ; j<numValidComp ; j++ )
          {
            wp[j].iWeight = (1 << wp[j].uiLog2WeightDenom);
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for ( int iRefIdx=pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx<MAX_NUM_REF ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);

      wp[0].bPresentFlag = false;
      wp[1].bPresentFlag = false;
      wp[2].bPresentFlag = false;
    }
  }
  VTMCHECK(uiTotalSignalledWeightFlags>24, "Too many weight flag signalled");
}

#if HEVC_USE_SCALING_LISTS
/** decode quantization matrix
* \param scalingList quantization matrix information
*/
void HLSyntaxReader::parseScalingList(ScalingList* scalingList)
{
  uint32_t  code, sizeId, listId;
  bool scalingListPredModeFlag;
  //for each size
  for(sizeId = SCALING_LIST_FIRST_CODED; sizeId <= SCALING_LIST_LAST_CODED; sizeId++)
  {
    for(listId = 0; listId <  SCALING_LIST_NUM; listId++)
    {
      if ((sizeId==SCALING_LIST_32x32) && (listId%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) != 0))
      {
        int *src = scalingList->getScalingListAddress(sizeId, listId);
        const int size = std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId]);
        const int *srcNextSmallerSize = scalingList->getScalingListAddress(sizeId-1, listId);
        for(int i=0; i<size; i++)
        {
          src[i] = srcNextSmallerSize[i];
        }
        scalingList->setScalingListDC(sizeId,listId,(sizeId > SCALING_LIST_8x8) ? scalingList->getScalingListDC(sizeId-1, listId) : src[0]);
      }
      else
      {
        READ_FLAG( code, "scaling_list_pred_mode_flag");
        scalingListPredModeFlag = (code) ? true : false;
        scalingList->setScalingListPredModeFlag(sizeId, listId, scalingListPredModeFlag);
        if(!scalingListPredModeFlag) //Copy Mode
        {
          READ_UVLC( code, "scaling_list_pred_matrix_id_delta");

          if (sizeId==SCALING_LIST_32x32)
          {
            code*=(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES); // Adjust the decoded code for this size, to cope with the missing 32x32 chroma entries.
          }

          scalingList->setRefMatrixId (sizeId,listId,(uint32_t)((int)(listId)-(code)));
          if( sizeId > SCALING_LIST_8x8 )
          {
            scalingList->setScalingListDC(sizeId,listId,((listId == scalingList->getRefMatrixId (sizeId,listId))? 16 :scalingList->getScalingListDC(sizeId, scalingList->getRefMatrixId (sizeId,listId))));
          }
          scalingList->processRefMatrix( sizeId, listId, scalingList->getRefMatrixId (sizeId,listId));

        }
        else //DPCM Mode
        {
          decodeScalingList(scalingList, sizeId, listId);
        }
      }
    }
  }

  return;
}

/** decode DPCM
* \param scalingList  quantization matrix information
* \param sizeId size index
* \param listId list index
*/
void HLSyntaxReader::decodeScalingList(ScalingList *scalingList, uint32_t sizeId, uint32_t listId)
{
  int i,coefNum = std::min(MAX_MATRIX_COEF_NUM,(int)g_scalingListSize[sizeId]);
  int data;
  int scalingListDcCoefMinus8 = 0;
  int nextCoef = SCALING_LIST_START_VALUE;
  uint32_t* scan = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][gp_sizeIdxInfo->idxFrom( 1 << ( sizeId == SCALING_LIST_FIRST_CODED ? 2 : 3 ) )][gp_sizeIdxInfo->idxFrom( 1 << ( sizeId == SCALING_LIST_FIRST_CODED ? 2 : 3 ) )];
  int *dst = scalingList->getScalingListAddress(sizeId, listId);

  if( sizeId > SCALING_LIST_8x8 )
  {
    READ_SVLC( scalingListDcCoefMinus8, "scaling_list_dc_coef_minus8");
    scalingList->setScalingListDC(sizeId,listId,scalingListDcCoefMinus8 + 8);
    nextCoef = scalingList->getScalingListDC(sizeId,listId);
  }

  for(i = 0; i < coefNum; i++)
  {
    READ_SVLC( data, "scaling_list_delta_coef");
    nextCoef = (nextCoef + data + 256 ) % 256;
    dst[scan[i]] = nextCoef;
  }
}
#endif

bool HLSyntaxReader::xMoreRbspData()
{
  int bitsLeft = m_pcBitstream->getNumBitsLeft();

  // if there are more than 8 bits, it cannot be rbsp_trailing_bits
  if (bitsLeft > 8)
  {
    return true;
  }

  uint8_t lastByte = m_pcBitstream->peekBits(bitsLeft);
  int cnt = bitsLeft;

  // remove trailing bits equal to zero
  while ((cnt>0) && ((lastByte & 1) == 0))
  {
    lastByte >>= 1;
    cnt--;
  }
  // remove bit equal to one
  cnt--;

  // we should not have a negative number of bits
  VTMCHECK (cnt<0, "Negative number of bits");

  // we have more data, if cnt is not zero
  return (cnt>0);
}

void HLSyntaxReader::alf( AlfSliceParam& alfSliceParam )
{
  uint32_t code;
  READ_FLAG( code, "tile_group_alf_enabled_flag" );
  alfSliceParam.enabledFlag[COMPONENT_Y] = code ? true : false;

  if( !code )
  {
    alfSliceParam.enabledFlag[COMPONENT_Cb] = alfSliceParam.enabledFlag[COMPONENT_Cr] = false;
    return;
  }

  int alfChromaIdc = truncatedUnaryEqProb( 3 );        //alf_chroma_idc
  alfSliceParam.enabledFlag[COMPONENT_Cb] = alfChromaIdc >> 1;
  alfSliceParam.enabledFlag[COMPONENT_Cr] = alfChromaIdc & 1;

  xReadTruncBinCode( code, MAX_NUM_ALF_CLASSES );  //number_of_filters_minus1
  alfSliceParam.numLumaFilters = code + 1;
  if( alfSliceParam.numLumaFilters > 1 )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      xReadTruncBinCode( code, alfSliceParam.numLumaFilters );
      alfSliceParam.filterCoeffDeltaIdx[i] = code;
    }
  }
  else
  {
    memset( alfSliceParam.filterCoeffDeltaIdx, 0, sizeof( alfSliceParam.filterCoeffDeltaIdx ) );
  }

  alfFilter( alfSliceParam, false );

  if( alfChromaIdc )
  {
    alfFilter( alfSliceParam, true );
  }
}

int HLSyntaxReader::alfGolombDecode( const int k )
{
  uint32_t uiSymbol;
  int q = -1;
  int nr = 0;
  int m = (int)pow( 2.0, k );
  int a;

  uiSymbol = 1;
  while( uiSymbol )
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( uiSymbol, "" );
#else
    xReadFlag( uiSymbol );
#endif
    q++;
  }

  for( a = 0; a < k; ++a )          // read out the sequential log2(M) bits
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( uiSymbol, "" );
#else
    xReadFlag( uiSymbol );
#endif
    if( uiSymbol )
    {
      nr += 1 << a;
    }
  }
  nr += q * m;                    // add the bits and the multiple of M
  if( nr != 0 )
  {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( uiSymbol, "" );
#else
    xReadFlag( uiSymbol );
#endif
    nr = ( uiSymbol ) ? nr : -nr;
  }
  return nr;
}

void HLSyntaxReader::alfFilter( AlfSliceParam& alfSliceParam, const bool isChroma )
{
  uint32_t code;
  if( !isChroma )
  {
    READ_FLAG( code, "alf_luma_coeff_delta_flag" );
    alfSliceParam.alfLumaCoeffDeltaFlag = code;

    if( !alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      std::memset( alfSliceParam.alfLumaCoeffFlag, true, sizeof( alfSliceParam.alfLumaCoeffFlag ) );

      if( alfSliceParam.numLumaFilters > 1 )
      {
        READ_FLAG( code, "alf_luma_coeff_delta_prediction_flag" );
        alfSliceParam.alfLumaCoeffDeltaPredictionFlag = code;
      }
      else
      {
        alfSliceParam.alfLumaCoeffDeltaPredictionFlag = 0;
      }
    }
    else
    {
      alfSliceParam.alfLumaCoeffDeltaPredictionFlag = 0;
    }
  }

  // derive maxGolombIdx
  AlfFilterShape alfShape( isChroma ? 5 : 7 );
  const int maxGolombIdx = AdaptiveLoopFilter::getMaxGolombIdx( alfShape.filterType );
  READ_UVLC( code, isChroma ? "alf_chroma_min_eg_order_minus1" : "alf_luma_min_eg_order_minus1" );

  int kMin = code + 1;
  static int kMinTab[MAX_NUM_ALF_COEFF];
  const int numFilters = isChroma ? 1 : alfSliceParam.numLumaFilters;
  short* coeff = isChroma ? alfSliceParam.chromaCoeff : alfSliceParam.lumaCoeff;

  for( int idx = 0; idx < maxGolombIdx; idx++ )
  {
    READ_FLAG( code, isChroma ? "alf_chroma_eg_order_increase_flag"  : "alf_luma_eg_order_increase_flag" );
    VTMCHECK( code > 1, "Wrong golomb_order_increase_flag" );
    kMinTab[idx] = kMin + code;
    kMin = kMinTab[idx];
  }

  if( !isChroma )
  {
    if( alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      for( int ind = 0; ind < alfSliceParam.numLumaFilters; ++ind )
      {
        READ_FLAG( code, "alf_luma_coeff_flag[i]" );
        alfSliceParam.alfLumaCoeffFlag[ind] = code;
      }
    }
  }

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( !isChroma && !alfSliceParam.alfLumaCoeffFlag[ind] && alfSliceParam.alfLumaCoeffDeltaFlag )
    {
      memset( coeff + ind * MAX_NUM_ALF_LUMA_COEFF, 0, sizeof( *coeff ) * alfShape.numCoeff );
      continue;
    }

    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = alfGolombDecode( kMinTab[alfShape.golombIdx[i]] );
    }
  }
}

int HLSyntaxReader::truncatedUnaryEqProb( const int maxSymbol )
{
  for( int k = 0; k < maxSymbol; k++ )
  {
    uint32_t symbol;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( symbol, "" );
#else
    xReadFlag( symbol );
#endif

    if( !symbol )
    {
      return k;
    }
  }
  return maxSymbol;
}

void HLSyntaxReader::xReadTruncBinCode( uint32_t& ruiSymbol, const int uiMaxSymbol )
{
  int uiThresh;
  if( uiMaxSymbol > 256 )
  {
    int uiThreshVal = 1 << 8;
    uiThresh = 8;
    while( uiThreshVal <= uiMaxSymbol )
    {
      uiThresh++;
      uiThreshVal <<= 1;
    }
    uiThresh--;
  }
  else
  {
    uiThresh = g_tbMax[uiMaxSymbol];
  }

  int uiVal = 1 << uiThresh;
  int b = uiMaxSymbol - uiVal;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  xReadCode( uiThresh, ruiSymbol, "" );
#else
  xReadCode( uiThresh, ruiSymbol );
#endif
  if( ruiSymbol >= uiVal - b )
  {
    uint32_t uiSymbol;
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    xReadFlag( uiSymbol, "" );
#else
    xReadFlag( uiSymbol );
#endif
    ruiSymbol <<= 1;
    ruiSymbol += uiSymbol;
    ruiSymbol -= ( uiVal - b );
  }
}

#if ADCNN
void HLSyntaxReader::cnnlf(CnnlfSliceParam& cnnlfSliceParam)
{
	uint32_t code;
	READ_FLAG(code, "cnnlf_slice_enable_flag");
	cnnlfSliceParam.enabledFlag[COMPONENT_Y] = code ? true : false;

	if (!code)
	{
		cnnlfSliceParam.enabledFlag[COMPONENT_Cb] = cnnlfSliceParam.enabledFlag[COMPONENT_Cr] = false;
		return;
	}

	int cnnlfChromaIdc = truncatedUnaryEqProb(3);
	cnnlfSliceParam.enabledFlag[COMPONENT_Cb] = cnnlfChromaIdc >> 1;
	cnnlfSliceParam.enabledFlag[COMPONENT_Cr] = cnnlfChromaIdc & 1;
}
#endif

//! \}

