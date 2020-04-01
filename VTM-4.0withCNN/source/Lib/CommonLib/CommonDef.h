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

/** \file     CommonDef.h
    \brief    Defines version information, constants and small in-line functions
*/

#ifndef __COMMONDEF__
#define __COMMONDEF__

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <limits>


#if _MSC_VER > 1000
// disable "signed and unsigned mismatch"
#pragma warning( disable : 4018 )
// disable bool coercion "performance warning"
#pragma warning( disable : 4800 )
#endif // _MSC_VER > 1000
#include "TypeDef.h"
#include "version.h"

// MS Visual Studio before 2014 does not support required C++11 features
#ifdef _MSC_VER
#if _MSC_VER < 1900
#error "MS Visual Studio version not supported. Please upgrade to Visual Studio 2015 or higher (or use other compilers)"
#endif
#endif

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Platform information
// ====================================================================================================================

#ifdef __clang__
#define NVM_COMPILEDBY  "[clang %d.%d.%d]", __clang_major__, __clang_minor__, __clang_patchlevel__
#ifdef __IA64__
#define NVM_ONARCH    "[on 64-bit] "
#else
#define NVM_ONARCH    "[on 32-bit] "
#endif
#elif __GNUC__
#define NVM_COMPILEDBY  "[GCC %d.%d.%d]", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__
#ifdef __IA64__
#define NVM_ONARCH    "[on 64-bit] "
#else
#define NVM_ONARCH    "[on 32-bit] "
#endif
#endif

#ifdef __INTEL_COMPILER
#define NVM_COMPILEDBY  "[ICC %d]", __INTEL_COMPILER
#elif  _MSC_VER
#define NVM_COMPILEDBY  "[VS %d]", _MSC_VER
#endif

#ifndef NVM_COMPILEDBY
#define NVM_COMPILEDBY "[Unk-CXX]"
#endif

#ifdef _WIN32
#define NVM_ONOS        "[Windows]"
#elif  __linux
#define NVM_ONOS        "[Linux]"
#elif  __CYGWIN__
#define NVM_ONOS        "[Cygwin]"
#elif __APPLE__
#define NVM_ONOS        "[Mac OS X]"
#else
#define NVM_ONOS "[Unk-OS]"
#endif

#define NVM_BITS          "[%d bit] ", (sizeof(void*) == 8 ? 64 : 32) ///< used for checking 64-bit O/S

#ifndef NULL
#define NULL              0
#endif

typedef enum
{
  AFFINEMODEL_4PARAM,
  AFFINEMODEL_6PARAM,
  AFFINE_MODEL_NUM
} EAffineModel;

static const int AFFINE_ME_LIST_SIZE =                             4;
static const int AFFINE_ME_LIST_SIZE_LD =                          3;
static const double AFFINE_ME_LIST_MVP_TH =                        1.0;

// ====================================================================================================================
// Common constants
// ====================================================================================================================
static const uint64_t   MAX_UINT64 =                  0xFFFFFFFFFFFFFFFFU;
static const uint32_t   MAX_UINT =                            0xFFFFFFFFU; ///< max. value of unsigned 32-bit integer
static const int    MAX_INT =                              2147483647; ///< max. value of signed 32-bit integer
static const uint8_t  MAX_UCHAR =                                   255;
static const uint8_t  MAX_SCHAR =                                   127;
static const double MAX_DOUBLE =                             1.7e+308; ///< max. value of double-type value

// ====================================================================================================================
// Coding tool configuration
// ====================================================================================================================
// Most of these should not be changed - they resolve the meaning of otherwise magic numbers.

static const int MAX_GOP =                                         64; ///< max. value of hierarchical GOP size
static const int MAX_NUM_REF_PICS =                                16; ///< max. number of pictures used for reference
static const int MAX_NUM_REF =                                     16; ///< max. number of entries in picture reference list
static const int MAX_QP =                                          63;
static const int NOT_VALID =                                       -1;

static const int AMVP_MAX_NUM_CANDS =                               2; ///< AMVP: advanced motion vector prediction - max number of final candidates
static const int AMVP_MAX_NUM_CANDS_MEM =                           3; ///< AMVP: advanced motion vector prediction - max number of candidates
static const int AMVP_DECIMATION_FACTOR =                           2;
static const int MRG_MAX_NUM_CANDS =                                7; ///< MERGE
static const int AFFINE_MRG_MAX_NUM_CANDS =                         5; ///< AFFINE MERGE

static const int MAX_TLAYER =                                       7; ///< Explicit temporal layer QP offset - max number of temporal layer

static const int ADAPT_SR_SCALE =                                   1; ///< division factor for adaptive search range

static const int MAX_NUM_PICS_IN_SOP =                           1024;

static const int MAX_NESTING_NUM_OPS =                           1024;
static const int MAX_NESTING_NUM_LAYER =                           64;

#if HEVC_VPS
static const int MAX_VPS_NUM_HRD_PARAMETERS =                       1;
static const int MAX_VPS_OP_SETS_PLUS1 =                         1024;
static const int MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 =         1;
#endif

static const int MAXIMUM_INTRA_FILTERED_WIDTH =                    16;
static const int MAXIMUM_INTRA_FILTERED_HEIGHT =                   16;


static const int MAX_CPB_CNT =                                     32; ///< Upper bound of (cpb_cnt_minus1 + 1)
static const int MAX_NUM_LAYER_IDS =                               64;
#if JVET_M0470
static const int COEF_REMAIN_BIN_REDUCTION =                        5; ///< indicates the level at which the VLC transitions from Golomb-Rice to TU+EG(k)
#else
static const int COEF_REMAIN_BIN_REDUCTION =                        3; ///< indicates the level at which the VLC transitions from Golomb-Rice to TU+EG(k)
#endif
static const int CU_DQP_TU_CMAX =                                   5; ///< max number bins for truncated unary
static const int CU_DQP_EG_k =                                      0; ///< expgolomb order

static const int SBH_THRESHOLD =                                    4; ///< value of the fixed SBH controlling threshold

static const int C1FLAG_NUMBER =                                    8; ///< maximum number of largerThan1 flag coded in one chunk: 16 in HM5
static const int C2FLAG_NUMBER =                                    1; ///< maximum number of largerThan2 flag coded in one chunk: 16 in HM5

static const int MAX_NUM_VPS =                                     16;
static const int MAX_NUM_SPS =                                     16;
static const int MAX_NUM_PPS =                                     64;

static const int MLS_GRP_NUM =                                   1024; ///< Max number of coefficient groups, max(16, 256)

static const int MLS_CG_SIZE =                                      4; ///< Coefficient group size of 4x4; = MLS_CG_LOG2_WIDTH + MLS_CG_LOG2_HEIGHT

static const int ADJ_QUANT_SHIFT =                                  7;
static const int ADJ_DEQUANT_SHIFT =            ( ADJ_QUANT_SHIFT + 1 );

static const int RVM_VCEGAM10_M =                                   4;

static const int MAX_REF_LINE_IDX =                                 3; //highest refLine offset in the list
static const int MRL_NUM_REF_LINES =                                3; //number of candidates in the array
static const int MULTI_REF_LINE_IDX[4] =               { 0, 1, 3, 0 };

static const int NUM_LUMA_MODE =                                   67; ///< Planar + DC + 65 directional mode (4*16 + 1)
static const int NUM_LMC_MODE =                                    1 + 2; ///< LMC + MDLM_T + MDLM_L
static const int NUM_INTRA_MODE = (NUM_LUMA_MODE + NUM_LMC_MODE);

static const int NUM_DIR =           (((NUM_LUMA_MODE - 3) >> 2) + 1);
static const int PLANAR_IDX =                                       0; ///< index for intra PLANAR mode
static const int DC_IDX =                                           1; ///< index for intra DC     mode
static const int HOR_IDX =                    (1 * (NUM_DIR - 1) + 2); ///< index for intra HORIZONTAL mode
static const int DIA_IDX =                    (2 * (NUM_DIR - 1) + 2); ///< index for intra DIAGONAL   mode
static const int VER_IDX =                    (3 * (NUM_DIR - 1) + 2); ///< index for intra VERTICAL   mode
static const int VDIA_IDX =                   (4 * (NUM_DIR - 1) + 2); ///< index for intra VDIAGONAL  mode
static const int NOMODE_IDX =                               MAX_UCHAR; ///< indicating uninitialized elements

static const int NUM_CHROMA_MODE = (5 + NUM_LMC_MODE); ///< total number of chroma modes
static const int LM_CHROMA_IDX = NUM_LUMA_MODE; ///< chroma mode index for derived from LM mode
static const int MDLM_L_IDX =                          LM_CHROMA_IDX + 1; ///< MDLM_L
static const int MDLM_T_IDX =                          LM_CHROMA_IDX + 2; ///< MDLM_T
static const int DM_CHROMA_IDX =                       NUM_INTRA_MODE; ///< chroma mode index for derived from luma intra mode

static const uint8_t INTER_MODE_IDX =                               255; ///< index for inter modes

#if JVET_M0464_UNI_MTS
static const uint32_t  NUM_TRAFO_MODES_MTS =                            6; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128
static const uint32_t  MTS_INTRA_MAX_CU_SIZE =                         32; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128
static const uint32_t  MTS_INTER_MAX_CU_SIZE =                         32; ///< Max Inter CU size applying EMT, supported values: 8, 16, 32, 64, 128
#else
static const uint32_t  EMT_INTRA_MAX_CU_WITH_QTBT =                    32; ///< Max Intra CU size applying EMT, supported values: 8, 16, 32, 64, 128
static const uint32_t  EMT_INTER_MAX_CU_WITH_QTBT =                    32; ///< Max Inter CU size applying EMT, supported values: 8, 16, 32, 64, 128
#endif
static const int NUM_MOST_PROBABLE_MODES = 6;
static const int LM_SYMBOL_NUM = (1 + NUM_LMC_MODE);

static const int FAST_UDI_MAX_RDMODE_NUM =              NUM_LUMA_MODE; ///< maximum number of RD comparison in fast-UDI estimation loop

static const int MDCS_ANGLE_LIMIT =                                 9; ///< 0 = Horizontal/vertical only, 1 = Horizontal/vertical +/- 1, 2 = Horizontal/vertical +/- 2 etc...

static const int MDCS_MAXIMUM_WIDTH =                               8; ///< (measured in pixels) TUs with width greater than this can only use diagonal scan
static const int MDCS_MAXIMUM_HEIGHT =                              8; ///< (measured in pixels) TUs with height greater than this can only use diagonal scan


static const int LOG2_MAX_NUM_COLUMNS_MINUS1 =                      7;
static const int LOG2_MAX_NUM_ROWS_MINUS1 =                         7;

static const int CABAC_INIT_PRESENT_FLAG =                          1;

static const int MV_FRACTIONAL_BITS_INTERNAL                      = 4;
static const int MV_FRACTIONAL_BITS_SIGNAL                        = 2;
static const int MV_FRACTIONAL_BITS_DIFF = MV_FRACTIONAL_BITS_INTERNAL - MV_FRACTIONAL_BITS_SIGNAL;
static const int LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL = 1 << MV_FRACTIONAL_BITS_SIGNAL;
static const int LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS = 1 << MV_FRACTIONAL_BITS_INTERNAL;
static const int CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS = 1 << (MV_FRACTIONAL_BITS_INTERNAL + 1);

static const int MAX_NUM_LONG_TERM_REF_PICS =                      33;
static const int NUM_LONG_TERM_REF_PIC_SPS =                        0;


static const int MAX_QP_OFFSET_LIST_SIZE =                          6; ///< Maximum size of QP offset list is 6 entries

// Cost mode support
static const int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP =      0; ///< QP to use for lossless coding.
static const int LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME =4; ///< QP' to use for mixed_lossy_lossless coding.

static const int CR_FROM_CB_REG_COST_SHIFT                        = 9;

static const int RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS =     4;

static const int RExt__PREDICTION_WEIGHTING_ANALYSIS_DC_PRECISION = 0; ///< Additional fixed bit precision used during encoder-side weighting prediction analysis. Currently only used when high_precision_prediction_weighting_flag is set, for backwards compatibility reasons.

static const int MAX_TIMECODE_SEI_SETS =                            3; ///< Maximum number of time sets

static const int MAX_CU_DEPTH =                                     7; ///< log2(CTUSize)
static const int MAX_CU_SIZE =                        1<<MAX_CU_DEPTH;
static const int MIN_CU_LOG2 =                                      2;
static const int MIN_PU_SIZE =                                      4;
static const int MIN_TU_SIZE =                                      4;
static const int MAX_TU_SIZE =                                    128;
static const int MAX_LOG2_TU_SIZE_PLUS_ONE =                        8; ///< log2(MAX_TU_SIZE) + 1
static const int MAX_NUM_PARTS_IN_CTU =                         ( ( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 ) );
static const int MAX_TR_SIZE =                            MAX_CU_SIZE;
static const int MAX_TU_SIZE_FOR_PROFILE =                         64;
static const int MAX_LOG2_DIFF_CU_TR_SIZE =                         2;
static const int MAX_CU_TILING_PARTITIONS = 1 << ( MAX_LOG2_DIFF_CU_TR_SIZE << 1 );

static const int JVET_C0024_ZERO_OUT_TH =                          32;

static const int MAX_NUM_PART_IDXS_IN_CTU_WIDTH = MAX_CU_SIZE/MIN_PU_SIZE; ///< maximum number of partition indices across the width of a CTU (or height of a CTU)
static const int SCALING_LIST_REM_NUM =                             6;

static const int QUANT_SHIFT =                                     14; ///< Q(4) = 2^14
static const int IQUANT_SHIFT =                                     6;
static const int SCALE_BITS =                                      15; ///< Precision for fractional bit estimates

static const int SCALING_LIST_NUM = MAX_NUM_COMPONENT * NUMBER_OF_PREDICTION_MODES; ///< list number for quantization matrix

static const int SCALING_LIST_START_VALUE =                         8; ///< start value for dpcm mode
static const int MAX_MATRIX_COEF_NUM =                             64; ///< max coefficient number for quantization matrix
static const int MAX_MATRIX_SIZE_NUM =                              8; ///< max size number for quantization matrix
static const int SCALING_LIST_BITS =                                8; ///< bit depth of scaling list entries
static const int LOG2_SCALING_LIST_NEUTRAL_VALUE =                  4; ///< log2 of the value that, when used in a scaling list, has no effect on quantisation
static const int SCALING_LIST_DC =                                 16; ///< default DC value

static const int CONTEXT_STATE_BITS =                               6;
static const int LAST_SIGNIFICANT_GROUPS =                         14;
static const int MAX_GR_ORDER_RESIDUAL =                           10;

static const int AFFINE_MIN_BLOCK_SIZE =                            4; ///< Minimum affine MC block size


static const int MMVD_REFINE_STEP =                                 8; ///< max number of distance step
static const int MMVD_MAX_REFINE_NUM =                              (MMVD_REFINE_STEP * 4); ///< max number of candidate from a base candidate
static const int MMVD_BASE_MV_NUM =                                 2; ///< max number of base candidate
static const int MMVD_ADD_NUM =                                     (MMVD_MAX_REFINE_NUM * MMVD_BASE_MV_NUM);///< total number of mmvd candidate
static const int MMVD_MRG_MAX_RD_NUM =                              MRG_MAX_NUM_CANDS;
static const int MMVD_MRG_MAX_RD_BUF_NUM =                          (MMVD_MRG_MAX_RD_NUM + 1);///< increase buffer size by 1

static const int MAX_NUM_REG_BINS_4x4SUBBLOCK =                    32; ///< max number of context-coded bins (incl. gt2 bins) per 4x4 subblock
static const int MAX_NUM_GT2_BINS_4x4SUBBLOCK =                     4; ///< max number of gt2 bins per 4x4 subblock
static const int MAX_NUM_REG_BINS_2x2SUBBLOCK =                     8; ///< max number of context-coded bins (incl. gt2 bins) per 2x2 subblock (chroma)
static const int MAX_NUM_GT2_BINS_2x2SUBBLOCK =                     2; ///< max number of gt2 bins per 2x2 subblock (chroma)

static const int BIO_EXTEND_SIZE              =                     1;
static const int BIO_TEMP_BUFFER_SIZE         =                     (MAX_CU_SIZE + 2 * BIO_EXTEND_SIZE) * (MAX_CU_SIZE + 2 * BIO_EXTEND_SIZE);

static const int GBI_NUM =                                          5; ///< the number of weight options
static const int GBI_DEFAULT =                                      ((uint8_t)(GBI_NUM >> 1)); ///< Default weighting index representing for w=0.5
static const int GBI_SIZE_CONSTRAINT =                            256; ///< disabling GBi if cu size is smaller than 256
static const int MAX_NUM_HMVP_CANDS =                              5; ///< maximum number of HMVP candidates to be stored and used in merge list
static const int MAX_NUM_HMVP_AVMPCANDS =                          4; ///< maximum number of HMVP candidates to be used in AMVP list

#if W0038_DB_OPT
static const int MAX_ENCODER_DEBLOCKING_QUALITY_LAYERS =           8 ;
#endif

#if SHARP_LUMA_DELTA_QP
static const uint32_t LUMA_LEVEL_TO_DQP_LUT_MAXSIZE =                1024; ///< max LUT size for QP offset based on luma

#endif
#if !JVET_M0464_UNI_MTS
static const int NUM_EMT_CU_FLAG_CTX =                              6;      ///< number of context models for EMT CU-level flag
#endif
#if JVET_M0147_DMVR
static const int DMVR_SUBCU_WIDTH = 16;
static const int DMVR_SUBCU_HEIGHT = 16;
static const int DMVR_SUBCU_WIDTH_LOG2 = 4;
static const int DMVR_SUBCU_HEIGHT_LOG2 = 4;
static const int MAX_NUM_SUBCU_DMVR = ((MAX_CU_SIZE * MAX_CU_SIZE) >> (DMVR_SUBCU_WIDTH_LOG2 + DMVR_SUBCU_HEIGHT_LOG2));
static const int DMVR_NUM_ITERATION = 2;
#endif

//QTBT high level parameters
//for I slice luma CTB configuration para.
static const int    MAX_BT_DEPTH  =                                 4;      ///<  <=7
static const int    MAX_BT_SIZE   =                                32;      ///<  [1<<MIN_QT_SIZE, 1<<CTU_LOG2]
static const int    MIN_BT_SIZE   =                                 4;      ///<  can be set down to 1<<MIN_CU_LOG2

static const int    MAX_TT_SIZE   =                                32;      ///<  [1<<MIN_QT_SIZE, 1<<CTU_LOG2]
static const int    MAX_TT_SIZE_C =                                32;      ///<  [1<<MIN_QT_SIZE, 1<<CTU_LOG2]
static const int    MIN_TT_SIZE   =                                 4;      ///<  can be set down to 1<<MIN_CU_LOG2
static const int    MIN_TT_SIZE_C =                                 4;      ///<  can be set down to 1<<MIN_CU_LOG2
                                                                            //for P/B slice CTU config. para.
static const int    MAX_BT_DEPTH_INTER =                            4;      ///< <=7
static const int    MAX_BT_SIZE_INTER  =                          128;      ///< for initialization, [1<<MIN_BT_SIZE_INTER, 1<<CTU_LOG2]
static const int    MIN_BT_SIZE_INTER  =                            4;      ///<

                                                                            //for I slice chroma CTB configuration para. (in luma samples)
static const int    MAX_BT_DEPTH_C      =                           0;      ///< <=7
static const int    MAX_BT_SIZE_C       =                          64;      ///< [1<<MIN_QT_SIZE_C, 1<<CTU_LOG2], in luma samples
static const int    MIN_BT_SIZE_C       =                           4;      ///< can be set down to 4, in luma samples

static const int    MAX_TT_SIZE_INTER  =                           64;      ///< for initialization, [1<<MIN_CU_LOG2, 64]
static const int    MIN_TT_SIZE_INTER  =                            4;      ///<

static const SplitSeries SPLIT_BITS         =                       5;
static const SplitSeries SPLIT_DMULT        =                       5;
static const SplitSeries SPLIT_MASK         =                      31;      ///< = (1 << SPLIT_BITS) - 1

static const int    SKIP_DEPTH =                                    3;
static const int    PICTURE_DISTANCE_TH =                           1;
static const int    FAST_SKIP_DEPTH =                               2;

static const double PBINTRA_RATIO     =                             1.1;
static const int    NUM_MRG_SATD_CAND =                             4;
static const double MRG_FAST_RATIO    =                             1.25;
static const int    NUM_AFF_MRG_SATD_CAND =                         2;

static const double AMAXBT_TH32 =                                  15.0;
static const double AMAXBT_TH64 =                                  30.0;

// need to know for static memory allocation
static const int MAX_DELTA_QP   =                                   7;      ///< maximum supported delta QP value
static const int MAX_TESTED_QPs =   ( 1 + 1 + ( MAX_DELTA_QP << 1 ) );      ///< dqp=0 +- max_delta_qp + lossless mode

static const int COM16_C806_TRANS_PREC =                            0;

static const int NUM_MERGE_IDX_EXT_CTX =                            5;
static const unsigned E0104_ALF_MAX_TEMPLAYERID =                  5;       // define to zero to switch of  code
static const unsigned C806_ALF_TEMPPRED_NUM =                      6;


static const int NTAPS_LUMA               =                         8; ///< Number of taps for luma
static const int NTAPS_CHROMA             =                         4; ///< Number of taps for chroma
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
static const int MAX_LADF_INTERVALS       =                         5; /// max number of luma adaptive deblocking filter qp offset intervals
#endif

static const int NTAPS_BILINEAR           =                         2; ///< Number of taps for bilinear filter

static const int ATMVP_SUB_BLOCK_SIZE =                             3; ///< sub-block size for ATMVP
static const int TRIANGLE_MAX_NUM_UNI_CANDS =                       5;
static const int TRIANGLE_MAX_NUM_CANDS_MEM =                       7;
static const int TRIANGLE_MAX_NUM_CANDS =                          40;
static const int TRIANGLE_MAX_NUM_SATD_CANDS =                      3;
static const int TRIANGLE_MIN_SIZE =                            8 * 8;

#if JVET_M0140_SBT
static const int SBT_MAX_SIZE =                                    64; ///< maximum CU size for using SBT
static const int SBT_NUM_SL =                                      10; ///< maximum number of historical PU decision saved for a CU
static const int SBT_NUM_RDO =                                      2; ///< maximum number of SBT mode tried for a PU
#endif

static const int IBC_MAX_CAND_SIZE = 16; // max block size for ibc search
static const int IBC_NUM_CANDIDATES = 64; ///< Maximum number of candidates to store/test
static const int CHROMA_REFINEMENT_CANDIDATES = 8; /// 8 candidates BV to choose from
static const int IBC_FAST_METHOD_NOINTRA_IBCCBF0 = 0x01;
static const int IBC_FAST_METHOD_BUFFERBV = 0X02;
static const int IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE = 0X04;
#if JVET_M0512_MOTION_BUFFER_COMPRESSION
static constexpr int MV_EXPONENT_BITCOUNT    = 4;
static constexpr int MV_MANTISSA_BITCOUNT    = 6;
static constexpr int MV_MANTISSA_UPPER_LIMIT = ((1 << (MV_MANTISSA_BITCOUNT - 1)) - 1);
static constexpr int MV_MANTISSA_LIMIT       = (1 << (MV_MANTISSA_BITCOUNT - 1));
static constexpr int MV_EXPONENT_MASK        = ((1 << MV_EXPONENT_BITCOUNT) - 1);
#endif
#if JVET_M0427_INLOOP_RESHAPER
static const int PIC_ANALYZE_CW_BINS =                           32;
static const int PIC_CODE_CW_BINS =                              16;
static const int FP_PREC =                                       14;
static const int CSCALE_FP_PREC =                                11;
#endif
// ====================================================================================================================
// Macro functions
// ====================================================================================================================

struct ClpRng
{
  int min;
  int max;
  int bd;
  int n;
};

struct ClpRngs
{
  ClpRng comp[MAX_NUM_COMPONENT]; ///< the bit depth as indicated in the SPS
  bool used;
  bool chroma;
};

template <typename T> inline T Clip3 (const T minVal, const T maxVal, const T a) { return std::min<T> (std::max<T> (minVal, a) , maxVal); }  ///< general min/max clip
template <typename T> inline T ClipBD( const T x, const int bitDepth ) { return Clip3( T( 0 ), T( ( 1 << bitDepth ) - 1 ), x ); }
template <typename T> inline T ClipPel (const T a, const ClpRng& clpRng)         { return std::min<T> (std::max<T> (clpRng.min, a) , clpRng.max); }  ///< clip reconstruction

template <typename T> inline void Check3( T minVal, T maxVal, T a)
{
  VTMCHECK( ( a > maxVal ) || ( a < minVal ), "VTMERROR: Range check " << minVal << " >= " << a << " <= " << maxVal << " failed" );
}  ///< general min/max clip

extern MsgLevel g_verbosity;

#include <stdarg.h>
inline void msg( MsgLevel level, const char* fmt, ... )
{
  if( g_verbosity >= level )
  {
    va_list args;
    va_start( args, fmt );
    vfprintf( level == VTMERROR ? stderr : stdout, fmt, args );
    va_end( args );
  }
}

template<typename T> bool isPowerOf2( const T val ) { return ( val & ( val - 1 ) ) == 0; }

#define MEMORY_ALIGN_DEF_SIZE       32  // for use with avx2 (256 bit)
#define CACHE_MEM_ALIGN_SIZE      1024

#define ALIGNED_MALLOC              1   ///< use 32-bit aligned malloc/free

#if ALIGNED_MALLOC
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
void *cache_mem_align_malloc(int size, int align_size);
void cache_mem_align_free(void *ptr);
#define xMalloc(type, len)          cache_mem_align_malloc(sizeof(type) * len, CACHE_MEM_ALIGN_SIZE)
#define xFree(ptr)                  cache_mem_align_free(ptr)
#elif     ( _WIN32 && ( _MSC_VER > 1300 ) ) || defined (__MINGW64_VERSION_MAJOR)
#define xMalloc( type, len )        _aligned_malloc( sizeof(type)*(len), MEMORY_ALIGN_DEF_SIZE )
#define xFree( ptr )                _aligned_free  ( ptr )
#elif defined (__MINGW32__)
#define xMalloc( type, len )        __mingw_aligned_malloc( sizeof(type)*(len), MEMORY_ALIGN_DEF_SIZE )
#define xFree( ptr )                __mingw_aligned_free( ptr )
#else
namespace detail {
template<typename T>
T* aligned_malloc(size_t len, size_t alignement) {
  T* p = NULL;
  if( posix_memalign( (void**)&p, alignement, sizeof(T)*(len) ) )
  {
    THROW("posix_memalign failed");
  }
  return p;
}
}
#define xMalloc( type, len )        detail::aligned_malloc<type>( len, MEMORY_ALIGN_DEF_SIZE )
#define xFree( ptr )                free( ptr )
#endif

#else
#define xMalloc( type, len )        malloc   ( sizeof(type)*(len) )
#define xFree( ptr )                free     ( ptr )
#endif //#if ALIGNED_MALLOC

#if defined _MSC_VER
#define ALIGN_DATA(nBytes,v) __declspec(align(nBytes)) v
#else
//#elif defined linux
#define ALIGN_DATA(nBytes,v) v __attribute__ ((aligned (nBytes)))
//#else
//#error unknown platform
#endif

#if defined(__GNUC__) && !defined(__clang__)
#    define GCC_VERSION_AT_LEAST(x,y) (__GNUC__ > x || __GNUC__ == x && __GNUC_MINOR__ >= y)
#else
#    define GCC_VERSION_AT_LEAST(x,y) 0
#endif

#ifdef __clang__
#    define CLANG_VERSION_AT_LEAST(x,y) (__clang_major__ > x || __clang_major__ == x && __clang_minor__ >= y)
#else
#    define CLANG_VERSION_AT_LEAST(x,y) 0
#endif

#ifdef __GNUC__
#    define ALWAYS_INLINE __attribute__((always_inline)) inline
#elif defined _MSC_VER
#    define ALWAYS_INLINE __forceinline
#else
#    define ALWAYS_INLINE
#endif

#if ENABLE_SIMD_OPT

#if defined(__i386__) || defined(i386) || defined(__x86_64__) || defined(_M_X64) || defined (_WIN32) || defined (_MSC_VER)
#define TARGET_SIMD_X86
typedef enum{
  SCALAR = 0,
  SSE41,
  SSE42,
  AVX,
  AVX2,
  AVX512
} X86_VEXT;
#elif defined (__ARM_NEON__)
#define TARGET_SIMD_ARM 1
#else
#error no simd target
#endif

#ifdef TARGET_SIMD_X86
X86_VEXT read_x86_extension_flags(const std::string &extStrId = std::string());
const char* read_x86_extension(const std::string &extStrId);
#endif

#endif //ENABLE_SIMD_OPT

template <typename ValueType> inline ValueType leftShift       (const ValueType value, const int shift) { return (shift >= 0) ? ( value                                  << shift) : ( value                                   >> -shift); }
template <typename ValueType> inline ValueType rightShift      (const ValueType value, const int shift) { return (shift >= 0) ? ( value                                  >> shift) : ( value                                   << -shift); }
template <typename ValueType> inline ValueType leftShift_round (const ValueType value, const int shift) { return (shift >= 0) ? ( value                                  << shift) : ((value + (ValueType(1) << (-shift - 1))) >> -shift); }
template <typename ValueType> inline ValueType rightShift_round(const ValueType value, const int shift) { return (shift >= 0) ? ((value + (ValueType(1) << (shift - 1))) >> shift) : ( value                                   << -shift); }

static inline int floorLog2(uint32_t x)
{
  if (x == 0)
  {
    return -1;
  }
#ifdef __GNUC__
  return 31 - __builtin_clz(x);
#else
  int result = 0;
  if (x & 0xffff0000)
  {
    x >>= 16;
    result += 16;
  }
  if (x & 0xff00)
  {
    x >>= 8;
    result += 8;
  }
  if (x & 0xf0)
  {
    x >>= 4;
    result += 4;
  }
  if (x & 0xc)
  {
    x >>= 2;
    result += 2;
  }
  if (x & 0x2)
  {
    x >>= 1;
    result += 1;
  }
  return result;
#endif
}

//CASE-BREAK for breakpoints
#if defined ( _MSC_VER ) && defined ( _DEBUG )
#define _CASE(_x) if(_x)
#define _BREAK while(0);
#define _AREA_AT(_a,_x,_y,_w,_h)  (_a.x==_x && _a.y==_y && _a.width==_w && _a.height==_h)
#define _AREA_CONTAINS(_a,_x,_y)  (_a.contains( Position{ _x, _y} ))
#define _UNIT_AREA_AT(_a,_x,_y,_w,_h) (_a.Y().x==_x && _a.Y().y==_y && _a.Y().width==_w && _a.Y().height==_h)
#else
#define _CASE(...)
#define _BREAK
#define _AREA_AT(...)
#define _AREA_CONTAINS(_a,_x,_y)
#define _UNIT_AREA_AT(_a,_x,_y,_w,_h)
#endif

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
#include <omp.h>

#define PARL_PARAM(DEF) , DEF
#define PARL_PARAM0(DEF) DEF
#else
#define PARL_PARAM(DEF)
#define PARL_PARAM0(DEF)
#endif

//! \}

#endif // end of #ifndef  __COMMONDEF__

