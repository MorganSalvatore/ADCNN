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

/** \file     TypeDef.h
    \brief    Define macros, basic types, new types and enumerations
*/

#ifndef __TYPEDEF__
#define __TYPEDEF__

#ifndef __COMMONDEF__
#error Include CommonDef.h not TypeDef.h
#endif

#include <vector>
#include <utility>
#include <sstream>
#include <cstddef>
#include <cstring>
#include <assert.h>
#include <cassert>

#define ADCNN   1

#define SUMMARY_OUT 1 //WJL

#define JVET_M0055_DEBUG_CTU                              1 // DebugCTU encoder debug option

#define JVET_M0297_32PT_MTS_ZERO_OUT                      1 // 32 point MTS based on skipping high frequency coefficients

#define JVET_M0126_HMVP_MRG_PRUNING                       1 // HMVP merge candidates pruning simplification

#define JVET_M0483_IBC                                    1 // Block level flag signaling and independent IBC mode

#define JVET_M0102_INTRA_SUBPARTITIONS                    1

#define JVET_M0303_IMPLICIT_MTS                           1 // Implicit transform selection (can be enabled with MTSImplicit encoder config parameter)

#define FIX_DB_MAX_TRANSFORM_SIZE                         1
#define JVET_M0908_CIIP_DB                                1
#define JVET_M0471_LONG_DEBLOCKING_FILTERS                1
#define JVET_M0427_INLOOP_RESHAPER                        1
#define JVET_M0470                                        1 // Fixed GR/TU+EG-k transition point, use limited prefix length for escape codes

#define JVET_M0253_HASH_ME                                1

#define JVET_M0257                                        1 // Scan only non zero-out regions of large TUs
#define JVET_M0193_PAIR_AVG_REDUCTION                     1 //Use only one pairwise average candidate

#define JVET_M0281_AMVP_ROUNDING                          1 // Perform all AMVP rounding before pruning even AMVR is off

#define JVET_M0117_AMVP_LIST_GEN                          1 // AMVP candidate list generation simplification

#define JVET_M0192_AFF_CHROMA_SIMPL                       1 // Affine chroma MV derivation simplification and rounding unification

#define JVET_M0116_ATMVP_LEFT_NB_FOR_OFFSET               1 // Only use left neighbor for ATMVP offset derivation, from M0273, M0240, M0116, M0338, M0204

#define JVET_M0063_BDOF_FIX                               1 // BDOF bitdepth bugfix

#define JVET_M0265_MV_ROUNDING_CLEANUP                    1 // Unify MV roundings and make SW/WD allignment
#define JVET_M0883_TRIANGLE_SIGNALING                     1 // Using regular merge index signaling for triangle mode

#define JVET_M0228_REMOVE_CPMV_COMPARE                    1 // Remove CPMV comparisons for construnted affine merge candidates from JVET-M0228, M0166, M0477

#define JVET_M0145_AFFINE_MV_CLIP                         1 // Missing clipping for MV storage in affine

#define JVET_M0264_HMVP_WITH_GBIIDX                       1 // Harmonization between HMVP and GBi

#define JVET_M0381_ONE_CTX_FOR_SUBBLOCK_MRG_IDX           1 // CE2.2.2 a: one context for subblock Merge index

#define JVET_M0118_M0185_TRIANGLE_FLAG_FIX                1 // Avoid signaling triangle flag if a CU uses MMVD or CIIP

#define JVET_M0487_INT_EXTEND                             1  // CE9.1.1 b: integer reference samples as 1 extended samples

#define JVET_M0444_SMVD                                   1 // SMVD mode

#define JVET_M0170_MRG_SHARELIST                          1
#if JVET_M0170_MRG_SHARELIST
#define MRG_SHARELIST_SHARSIZE                            32
#endif

#define JVET_M0064_CCLM_SIMPLIFICATION                    1

#define JVET_M0142_CCLM_COLLOCATED_CHROMA                 1 // Adding support for chroma sample location type 2 in CCLM

#define JVET_M0328_KEEP_ONE_WEIGHT_GROUP                  1

#define JVET_M0479_18BITS_MV_CLIP                         1

#define JVET_M0497_FAST_DST7                              1
#if JVET_M0497_FAST_DST7
#define JVET_M0497_MATRIX_MULT                            0 // 0: Fast method; 1: Matrix multiplication
#endif
#define JVET_M0502_PRED_MODE_CTX                          1

#define JVET_M0140_SBT                                    1 // Sub-Block transform for Inter blocks
#if JVET_M0140_SBT
#define APPLY_SBT_SL_ON_MTS                               1 // apply save & load fast algorithm on inter MTS when SBT is on
#endif

#define JVET_M0407_IBC_RANGE                              1 // extend IBC search range to some part of left CTU

#define JVET_M0464_UNI_MTS                                1
#define JVET_M0068_M0171_MMVD_CLEANUP                     1 // MMVD cleanup with 1) flip removal, 2) L1 zero vector fix, 3) bi-pred restriction after merge/MMVD
#define JVET_M0255_FRACMMVD_SWITCH                        1 // disable fractional MVD in MMVD adaptively
#define JVET_M0854_FRACMMVD_SWITCH_FOR_UHD                1 // disable fractional MVD for UHD pictures
#define JVET_M0823_MMVD_ENCOPT                            1 // encoder optimization for MMVD

#define JVET_M0147_DMVR                                   1 //Decoder side Motion Vector Refinement

#if JVET_M0464_UNI_MTS
typedef std::pair<int, bool> TrMode;
typedef std::pair<int, int>  TrCost;
#endif

#define JVET_M0246_AFFINE_AMVR                            1
#if JVET_M0246_AFFINE_AMVR
#define JVET_M0247_AFFINE_AMVR_ENCOPT                     1
#endif
#define JVET_M0421_SPLIT_SIG                              1

#define JVET_M0173_MOVE_GT2_TO_FIRST_PASS                 1 // Moving the gtr2 flag to the first coding pass

#define REMOVE_BIN_DECISION_TREE                          1

#define JVET_M0446_M0888_M0905_VPDU_AT_PIC_BOUNDARY       1

// clang-format off
#define JVET_M0453_CABAC_ENGINE                           1
#define JVET_M0512_MOTION_BUFFER_COMPRESSION              1
#define JVET_M0238_PDPC_NO_INTERPOLATION                  1

#define JVET_M0409_ATMVP_FIX                              1

#define ENABLE_JVET_L0283_MRL                             1 // 1: Enable MRL, 0: Disable MRL
#define JVET_L0090_PAIR_AVG                               1 // Add pairwise average candidates, replace HEVC combined candidates
#define REUSE_CU_RESULTS                                  1
#if REUSE_CU_RESULTS && JVET_M0102_INTRA_SUBPARTITIONS
#define REUSE_CU_RESULTS_WITH_MULTIPLE_TUS                1
#define MAX_NUM_TUS                                       4
#endif
// clang-format on

#ifndef JVET_B0051_NON_MPM_MODE
#define JVET_B0051_NON_MPM_MODE                         ( 1 && JEM_TOOLS )
#endif
#ifndef QTBT_AS_IN_JEM
#define QTBT_AS_IN_JEM                                    1
#endif
#ifndef HEVC_TOOLS
#define HEVC_TOOLS                                        0
#endif

#ifndef JVET_J0090_MEMORY_BANDWITH_MEASURE
#define JVET_J0090_MEMORY_BANDWITH_MEASURE                0
#endif

#ifndef EXTENSION_360_VIDEO
#define EXTENSION_360_VIDEO                               0   ///< extension for 360/spherical video coding support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#ifndef ENABLE_WPP_PARALLELISM
#define ENABLE_WPP_PARALLELISM                            0
#endif
#if ENABLE_WPP_PARALLELISM
#ifndef ENABLE_WPP_STATIC_LINK
#define ENABLE_WPP_STATIC_LINK                            0 // bug fix static link
#endif
#define PARL_WPP_MAX_NUM_THREADS                         16

#endif
#ifndef ENABLE_SPLIT_PARALLELISM
#define ENABLE_SPLIT_PARALLELISM                          0
#endif
#if ENABLE_SPLIT_PARALLELISM
#define PARL_SPLIT_MAX_NUM_JOBS                           6                             // number of parallel jobs that can be defined and need memory allocated
#define NUM_RESERVERD_SPLIT_JOBS                        ( PARL_SPLIT_MAX_NUM_JOBS + 1 )  // number of all data structures including the merge thread (0)
#define PARL_SPLIT_MAX_NUM_THREADS                        PARL_SPLIT_MAX_NUM_JOBS
#define NUM_SPLIT_THREADS_IF_MSVC                         4

#endif


// ====================================================================================================================
// NEXT software switches
// ====================================================================================================================
#define K0238_SAO_GREEDY_MERGE_ENCODING                   1

#ifndef ENABLE_TRACING
#define ENABLE_TRACING                                    0 // DISABLE by default (enable only when debugging, requires 15% run-time in decoding) -- see documentation in 'doc/DTrace for NextSoftware.pdf'
#endif

#if ENABLE_TRACING
#define K0149_BLOCK_STATISTICS                            1 // enables block statistics, which can be analysed with YUView (https://github.com/IENT/YUView)
#if K0149_BLOCK_STATISTICS
#define BLOCK_STATS_AS_CSV                                0 // statistics will be written in a comma separated value format. this is not supported by YUView
#endif
#endif

#if JVET_M0427_INLOOP_RESHAPER
#define WCG_EXT                                           1
#else
#define WCG_EXT                                           0 // part of JEM sharp Luma qp
#endif
#define WCG_WPSNR                                         WCG_EXT

#if HEVC_TOOLS
#define HEVC_USE_INTRA_SMOOTHING_T32                      1
#define HEVC_USE_INTRA_SMOOTHING_T64                      1
#define HEVC_USE_DC_PREDFILTERING                         1
#define HEVC_USE_HOR_VER_PREDFILTERING                    1
#define HEVC_USE_MDCS                                     1
#define HEVC_USE_SIGN_HIDING                              1
#define HEVC_USE_SCALING_LISTS                            1
#define HEVC_VPS                                          1
#define HEVC_DEPENDENT_SLICES                             1
#define HEVC_TILES_WPP                                    1
#else
#define HEVC_USE_SIGN_HIDING                              1
#define HEVC_TILES_WPP                                    1
#endif

#define KEEP_PRED_AND_RESI_SIGNALS                        0


#if QTBT_AS_IN_JEM // macros which will cause changes in the decoder behavior ara marked with *** - keep them on to retain compatibility with JEM-toolcheck
#define HM_QTBT_AS_IN_JEM                                 1   // ***
#if     HM_QTBT_AS_IN_JEM
#define HM_QTBT_AS_IN_JEM_QUANT                           1   // ***
#define HM_QTBT_REPRODUCE_FAST_LCTU_BUG                   1
#endif
#define HM_CODED_CU_INFO                                  1   // like in JEM, when related CU is skipped, it stays like this even if a non skip mode wins...
#define HM_4TAPIF_AS_IN_JEM                               1   // *** - PM: condition not well suited for 4-tap interpolation filters
#define HM_MDIS_AS_IN_JEM                                 1   // *** - PM: not filtering ref. samples for 64xn case and using Planar MDIS condition at encoder
#define HM_JEM_CLIP_PEL                                   1   // ***
#define HM_JEM_MERGE_CANDS                                0   // ***

#define JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT 1 ///< When 1, do not scale transform skip blocks by sqrt(2) between (I)Transform and (I)Quantizer

#endif//JEM_COMP

// ====================================================================================================================
// Debugging
// ====================================================================================================================

// most debugging tools are now bundled within the ENABLE_TRACING macro -- see documentation to see how to use

#define PRINT_MACRO_VALUES                                1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their values on startup

#define INTRA_FULL_SEARCH                                 0 ///< enables full mode search for intra estimation

// TODO: rename this macro to DECODER_DEBUG_BIT_STATISTICS (may currently cause merge issues with other branches)
// This can be enabled by the makefile
#ifndef RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS                0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces bit usage statistics (will impact decoder run time by up to ~10%)
#endif

#ifndef RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_TOOL_STATISTICS               0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces tool usage statistics
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS || RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_STATISTICS                    1
#endif

// ====================================================================================================================
// Tool Switches - transitory (these macros are likely to be removed in future revisions)
// ====================================================================================================================

#define DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES  1 ///< TODO: integrate this macro into a broader conformance checking system.
#define T0196_SELECTIVE_RDOQ                              1 ///< selective RDOQ
#define U0040_MODIFIED_WEIGHTEDPREDICTION_WITH_BIPRED_AND_CLIPPING 1
#define U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI    1 ///< Alternative transfer characteristics SEI message (JCTVC-U0033, with syntax naming from V1005)
#define X0038_LAMBDA_FROM_QP_CAPABILITY                   1 ///< This approach derives lambda from QP+QPoffset+QPoffset2. QPoffset2 is derived from QP+QPoffset using a linear model that is clipped between 0 and 3.
                                                            // To use this capability enable config parameter LambdaFromQpEnable

// ====================================================================================================================
// Tool Switches
// ====================================================================================================================


// This can be enabled by the makefile
#ifndef RExt__HIGH_BIT_DEPTH_SUPPORT
#define RExt__HIGH_BIT_DEPTH_SUPPORT                      0 ///< 0 (default) use data type definitions for 8-10 bit video, 1 = use larger data types to allow for up to 16-bit video (originally developed as part of N0188)
#endif

// SIMD optimizations
#define SIMD_ENABLE                                       1
#define ENABLE_SIMD_OPT                                 ( SIMD_ENABLE && !RExt__HIGH_BIT_DEPTH_SUPPORT )    ///< SIMD optimizations, no impact on RD performance
#define ENABLE_SIMD_OPT_MCIF                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the interpolation filter, no impact on RD performance
#define ENABLE_SIMD_OPT_BUFFER                          ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the buffer operations, no impact on RD performance
#define ENABLE_SIMD_OPT_DIST                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the distortion calculations(SAD,SSE,HADAMARD), no impact on RD performance
#define ENABLE_SIMD_OPT_AFFINE_ME                       ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for affine ME, no impact on RD performance
#define ENABLE_SIMD_OPT_ALF                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for ALF
#if ENABLE_SIMD_OPT_BUFFER
#define ENABLE_SIMD_OPT_GBI                               1                                                 ///< SIMD optimization for GBi
#endif

// End of SIMD optimizations


#define ME_ENABLE_ROUNDING_OF_MVS                         1 ///< 0 (default) = disables rounding of motion vectors when right shifted,  1 = enables rounding

#define RDOQ_CHROMA_LAMBDA                                1 ///< F386: weighting of chroma for RDOQ

#define U0132_TARGET_BITS_SATURATION                      1 ///< Rate control with target bits saturation method
#ifdef  U0132_TARGET_BITS_SATURATION
#define V0078_ADAPTIVE_LOWER_BOUND                        1 ///< Target bits saturation with adaptive lower bound
#endif
#define W0038_DB_OPT                                      1 ///< adaptive DB parameter selection, LoopFilterOffsetInPPS and LoopFilterDisable are set to 0 and DeblockingFilterMetric=2;
#define W0038_CQP_ADJ                                     1 ///< chroma QP adjustment based on TL, CQPTLAdjustEnabled is set to 1;

#define SHARP_LUMA_DELTA_QP                               1 ///< include non-normative LCU deltaQP and normative chromaQP change
#define ER_CHROMA_QP_WCG_PPS                              1 ///< Chroma QP model for WCG used in Anchor 3.2
#define ENABLE_QPA                                        1 ///< Non-normative perceptual QP adaptation according to JVET-H0047 and JVET-K0206. Deactivated by default, activated using encoder arguments --PerceptQPA=1 --SliceChromaQPOffsetPeriodicity=1



#define RDOQ_CHROMA                                       1 ///< use of RDOQ in chroma

#define QP_SWITCHING_FOR_PARALLEL                         1 ///< Replace floating point QP with a source-file frame number. After switching POC, increase base QP instead of frame level QP.

#define LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET         1 /// JVET-L0414 (CE11.2.2) with explicit signalling of num interval, threshold and qpOffset
// ====================================================================================================================
// Derived macros
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            1 ///< 0 use original 6-bit transform matrices for both forward and inverse transform, 1 (default) = use original matrices for inverse transform and high precision matrices for forward transform
#else
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            0 ///< 0 (default) use original 6-bit transform matrices for both forward and inverse transform, 1 = use original matrices for inverse transform and high precision matrices for forward transform
#endif

#if FULL_NBIT
#define DISTORTION_PRECISION_ADJUSTMENT(x)                0
#else
#define DISTORTION_ESTIMATION_BITS                        8
#define DISTORTION_PRECISION_ADJUSTMENT(x)                ((x>DISTORTION_ESTIMATION_BITS)? ((x)-DISTORTION_ESTIMATION_BITS) : 0)
#endif

// ====================================================================================================================
// Error checks
// ====================================================================================================================

#if ((RExt__HIGH_PRECISION_FORWARD_TRANSFORM != 0) && (RExt__HIGH_BIT_DEPTH_SUPPORT == 0))
#error VTMERROR: cannot enable RExt__HIGH_PRECISION_FORWARD_TRANSFORM without RExt__HIGH_BIT_DEPTH_SUPPORT
#endif

// ====================================================================================================================
// Named numerical types
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
typedef       int             Pel;               ///< pixel type
typedef       int64_t           TCoeff;            ///< transform coefficient
typedef       int             TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t           TFilterCoeff;      ///< filter coefficient
typedef       int64_t           Intermediate_Int;  ///< used as intermediate value in calculations
typedef       uint64_t          Intermediate_UInt; ///< used as intermediate value in calculations
#else
typedef       int16_t           Pel;               ///< pixel type
typedef       int             TCoeff;            ///< transform coefficient
typedef       int16_t           TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t           TFilterCoeff;      ///< filter coefficient
typedef       int             Intermediate_Int;  ///< used as intermediate value in calculations
typedef       uint32_t            Intermediate_UInt; ///< used as intermediate value in calculations
#endif

typedef       uint64_t          SplitSeries;       ///< used to encoded the splits that caused a particular CU size

typedef       uint64_t        Distortion;        ///< distortion measurement

// ====================================================================================================================
// Enumeration
// ====================================================================================================================
enum QuantFlags
{
  Q_INIT           = 0x0,
  Q_USE_RDOQ       = 0x1,
  Q_RDOQTS         = 0x2,
  Q_SELECTIVE_RDOQ = 0x4,
};

//EMT transform tags
enum TransType
{
  DCT2 = 0,
  DCT8 = 1,
  DST7 = 2,
  NUM_TRANS_TYPE = 3,
  DCT2_EMT = 4
};

#if JVET_M0102_INTRA_SUBPARTITIONS
enum ISPType
{
  NOT_INTRA_SUBPARTITIONS       = 0,
  HOR_INTRA_SUBPARTITIONS       = 1,
  VER_INTRA_SUBPARTITIONS       = 2,
  NUM_INTRA_SUBPARTITIONS_MODES = 3,
  CAN_USE_VER_AND_HORL_SPLITS   = 4
};
#endif

#if JVET_M0140_SBT
enum SbtIdx
{
  SBT_OFF_DCT  = 0,
  SBT_VER_HALF = 1,
  SBT_HOR_HALF = 2,
  SBT_VER_QUAD = 3,
  SBT_HOR_QUAD = 4,
  NUMBER_SBT_IDX,
  SBT_OFF_MTS, //note: must be after all SBT modes, only used in fast algorithm to discern the best mode is inter EMT
};

enum SbtPos
{
  SBT_POS0 = 0,
  SBT_POS1 = 1,
  NUMBER_SBT_POS
};

enum SbtMode
{
  SBT_VER_H0 = 0,
  SBT_VER_H1 = 1,
  SBT_HOR_H0 = 2,
  SBT_HOR_H1 = 3,
  SBT_VER_Q0 = 4,
  SBT_VER_Q1 = 5,
  SBT_HOR_Q0 = 6,
  SBT_HOR_Q1 = 7,
  NUMBER_SBT_MODE
};
#endif

enum RDPCMMode
{
  RDPCM_OFF             = 0,
  RDPCM_HOR             = 1,
  RDPCM_VER             = 2,
  NUMBER_OF_RDPCM_MODES = 3
};

enum RDPCMSignallingMode
{
  RDPCM_SIGNAL_IMPLICIT            = 0,
  RDPCM_SIGNAL_EXPLICIT            = 1,
  NUMBER_OF_RDPCM_SIGNALLING_MODES = 2
};

/// supported slice type
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  NUMBER_OF_SLICE_TYPES = 3
};

/// chroma formats (according to semantics of chroma_format_idc)
enum ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
};

enum ChannelType
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE = 2
};

#define CH_L CHANNEL_TYPE_LUMA
#define CH_C CHANNEL_TYPE_CHROMA

enum ComponentID
{
  COMPONENT_Y         = 0,
  COMPONENT_Cb        = 1,
  COMPONENT_Cr        = 2,
  MAX_NUM_COMPONENT   = 3,
  MAX_NUM_TBLOCKS     = MAX_NUM_COMPONENT
};

#define MAP_CHROMA(c) (ComponentID(c))

enum InputColourSpaceConversion // defined in terms of conversion prior to input of encoder.
{
  IPCOLOURSPACE_UNCHANGED               = 0,
  IPCOLOURSPACE_YCbCrtoYCrCb            = 1, // Mainly used for debug!
  IPCOLOURSPACE_YCbCrtoYYY              = 2, // Mainly used for debug!
  IPCOLOURSPACE_RGBtoGBR                = 3,
  NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS = 4
};

enum MATRIX_COEFFICIENTS // Table E.5 (Matrix coefficients)
{
  MATRIX_COEFFICIENTS_RGB                           = 0,
  MATRIX_COEFFICIENTS_BT709                         = 1,
  MATRIX_COEFFICIENTS_UNSPECIFIED                   = 2,
  MATRIX_COEFFICIENTS_RESERVED_BY_ITUISOIEC         = 3,
  MATRIX_COEFFICIENTS_USFCCT47                      = 4,
  MATRIX_COEFFICIENTS_BT601_625                     = 5,
  MATRIX_COEFFICIENTS_BT601_525                     = 6,
  MATRIX_COEFFICIENTS_SMPTE240                      = 7,
  MATRIX_COEFFICIENTS_YCGCO                         = 8,
  MATRIX_COEFFICIENTS_BT2020_NON_CONSTANT_LUMINANCE = 9,
  MATRIX_COEFFICIENTS_BT2020_CONSTANT_LUMINANCE     = 10,
};

enum DeblockEdgeDir
{
  EDGE_VER     = 0,
  EDGE_HOR     = 1,
  NUM_EDGE_DIR = 2
};

/// supported prediction type
enum PredMode
{
  MODE_INTER                 = 0,     ///< inter-prediction mode
  MODE_INTRA                 = 1,     ///< intra-prediction mode
#if JVET_M0483_IBC
  MODE_IBC                   = 2,     ///< ibc-prediction mode
  NUMBER_OF_PREDICTION_MODES = 3,
#else
  NUMBER_OF_PREDICTION_MODES = 2,
#endif
};

/// reference list index
enum RefPicList
{
  REF_PIC_LIST_0               = 0,   ///< reference list 0
  REF_PIC_LIST_1               = 1,   ///< reference list 1
  NUM_REF_PIC_LIST_01          = 2,
  REF_PIC_LIST_X               = 100  ///< special mark
};

#define L0 REF_PIC_LIST_0
#define L1 REF_PIC_LIST_1

/// distortion function index
enum DFunc
{
  DF_SSE             = 0,             ///< general size SSE
  DF_SSE2            = DF_SSE+1,      ///<   2xM SSE
  DF_SSE4            = DF_SSE+2,      ///<   4xM SSE
  DF_SSE8            = DF_SSE+3,      ///<   8xM SSE
  DF_SSE16           = DF_SSE+4,      ///<  16xM SSE
  DF_SSE32           = DF_SSE+5,      ///<  32xM SSE
  DF_SSE64           = DF_SSE+6,      ///<  64xM SSE
  DF_SSE16N          = DF_SSE+7,      ///< 16NxM SSE

  DF_SAD             = 8,             ///< general size SAD
  DF_SAD2            = DF_SAD+1,      ///<   2xM SAD
  DF_SAD4            = DF_SAD+2,      ///<   4xM SAD
  DF_SAD8            = DF_SAD+3,      ///<   8xM SAD
  DF_SAD16           = DF_SAD+4,      ///<  16xM SAD
  DF_SAD32           = DF_SAD+5,      ///<  32xM SAD
  DF_SAD64           = DF_SAD+6,      ///<  64xM SAD
  DF_SAD16N          = DF_SAD+7,      ///< 16NxM SAD

  DF_HAD             = 16,            ///< general size Hadamard
  DF_HAD2            = DF_HAD+1,      ///<   2xM HAD
  DF_HAD4            = DF_HAD+2,      ///<   4xM HAD
  DF_HAD8            = DF_HAD+3,      ///<   8xM HAD
  DF_HAD16           = DF_HAD+4,      ///<  16xM HAD
  DF_HAD32           = DF_HAD+5,      ///<  32xM HAD
  DF_HAD64           = DF_HAD+6,      ///<  64xM HAD
  DF_HAD16N          = DF_HAD+7,      ///< 16NxM HAD

  DF_SAD12           = 24,
  DF_SAD24           = 25,
  DF_SAD48           = 26,

  DF_MRSAD           = 27,            ///< general size MR SAD
  DF_MRSAD2          = DF_MRSAD+1,    ///<   2xM MR SAD
  DF_MRSAD4          = DF_MRSAD+2,    ///<   4xM MR SAD
  DF_MRSAD8          = DF_MRSAD+3,    ///<   8xM MR SAD
  DF_MRSAD16         = DF_MRSAD+4,    ///<  16xM MR SAD
  DF_MRSAD32         = DF_MRSAD+5,    ///<  32xM MR SAD
  DF_MRSAD64         = DF_MRSAD+6,    ///<  64xM MR SAD
  DF_MRSAD16N        = DF_MRSAD+7,    ///< 16NxM MR SAD

  DF_MRHAD           = 35,            ///< general size MR Hadamard
  DF_MRHAD2          = DF_MRHAD+1,    ///<   2xM MR HAD
  DF_MRHAD4          = DF_MRHAD+2,    ///<   4xM MR HAD
  DF_MRHAD8          = DF_MRHAD+3,    ///<   8xM MR HAD
  DF_MRHAD16         = DF_MRHAD+4,    ///<  16xM MR HAD
  DF_MRHAD32         = DF_MRHAD+5,    ///<  32xM MR HAD
  DF_MRHAD64         = DF_MRHAD+6,    ///<  64xM MR HAD
  DF_MRHAD16N        = DF_MRHAD+7,    ///< 16NxM MR HAD

  DF_MRSAD12         = 43,
  DF_MRSAD24         = 44,
  DF_MRSAD48         = 45,

  DF_SAD_FULL_NBIT    = 46,
  DF_SAD_FULL_NBIT2   = DF_SAD_FULL_NBIT+1,    ///<   2xM SAD with full bit usage
  DF_SAD_FULL_NBIT4   = DF_SAD_FULL_NBIT+2,    ///<   4xM SAD with full bit usage
  DF_SAD_FULL_NBIT8   = DF_SAD_FULL_NBIT+3,    ///<   8xM SAD with full bit usage
  DF_SAD_FULL_NBIT16  = DF_SAD_FULL_NBIT+4,    ///<  16xM SAD with full bit usage
  DF_SAD_FULL_NBIT32  = DF_SAD_FULL_NBIT+5,    ///<  32xM SAD with full bit usage
  DF_SAD_FULL_NBIT64  = DF_SAD_FULL_NBIT+6,    ///<  64xM SAD with full bit usage
  DF_SAD_FULL_NBIT16N = DF_SAD_FULL_NBIT+7,    ///< 16NxM SAD with full bit usage

#if WCG_EXT
  DF_SSE_WTD          = 54,                ///< general size SSE
  DF_SSE2_WTD         = DF_SSE_WTD+1,      ///<   4xM SSE
  DF_SSE4_WTD         = DF_SSE_WTD+2,      ///<   4xM SSE
  DF_SSE8_WTD         = DF_SSE_WTD+3,      ///<   8xM SSE
  DF_SSE16_WTD        = DF_SSE_WTD+4,      ///<  16xM SSE
  DF_SSE32_WTD        = DF_SSE_WTD+5,      ///<  32xM SSE
  DF_SSE64_WTD        = DF_SSE_WTD+6,      ///<  64xM SSE
  DF_SSE16N_WTD       = DF_SSE_WTD+7,      ///< 16NxM SSE
  DF_DEFAULT_ORI      = DF_SSE_WTD+8,
#endif

  DF_SAD_INTERMEDIATE_BITDEPTH = 63,

  DF_TOTAL_FUNCTIONS = 64
};

/// motion vector predictor direction used in AMVP
enum MvpDir
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

enum StoredResidualType
{
  RESIDUAL_RECONSTRUCTED          = 0,
  RESIDUAL_ENCODER_SIDE           = 1,
  NUMBER_OF_STORED_RESIDUAL_TYPES = 2
};

enum TransformDirection
{
  TRANSFORM_FORWARD              = 0,
  TRANSFORM_INVERSE              = 1,
  TRANSFORM_NUMBER_OF_DIRECTIONS = 2
};

/// supported ME search methods
enum MESearchMethod
{
  MESEARCH_FULL              = 0,
  MESEARCH_DIAMOND           = 1,
  MESEARCH_SELECTIVE         = 2,
  MESEARCH_DIAMOND_ENHANCED  = 3,
  MESEARCH_NUMBER_OF_METHODS = 4
};

/// coefficient scanning type used in ACS
enum CoeffScanType
{
  SCAN_DIAG = 0,        ///< up-right diagonal scan
#if HEVC_USE_MDCS
  SCAN_HOR  = 1,        ///< horizontal first scan
  SCAN_VER  = 2,        ///< vertical first scan
#endif
  SCAN_NUMBER_OF_TYPES
};

enum CoeffScanGroupType
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum SignificanceMapContextType
{
  CONTEXT_TYPE_4x4    = 0,
  CONTEXT_TYPE_8x8    = 1,
  CONTEXT_TYPE_NxN    = 2,
  CONTEXT_TYPE_SINGLE = 3,
  CONTEXT_NUMBER_OF_TYPES = 4
};

#if HEVC_USE_SCALING_LISTS
enum ScalingListMode
{
  SCALING_LIST_OFF,
  SCALING_LIST_DEFAULT,
  SCALING_LIST_FILE_READ
};

enum ScalingListSize
{
  SCALING_LIST_2x2 = 0,
  SCALING_LIST_4x4,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_64x64,
  SCALING_LIST_128x128,
  SCALING_LIST_SIZE_NUM,
  SCALING_LIST_FIRST_CODED = SCALING_LIST_4x4, // smallest scaling coded as High Level Parameter
  SCALING_LIST_LAST_CODED  = SCALING_LIST_32x32
};
#endif

// Slice / Slice segment encoding modes
enum SliceConstraint
{
  NO_SLICES              = 0,          ///< don't use slices / slice segments
  FIXED_NUMBER_OF_CTU    = 1,          ///< Limit maximum number of largest coding tree units in a slice / slice segments
  FIXED_NUMBER_OF_BYTES  = 2,          ///< Limit maximum number of bytes in a slice / slice segment
#if HEVC_TILES_WPP
  FIXED_NUMBER_OF_TILES  = 3,          ///< slices / slice segments span an integer number of tiles
  NUMBER_OF_SLICE_CONSTRAINT_MODES = 4
#else
  NUMBER_OF_SLICE_CONSTRAINT_MODES = 3
#endif
};

// For use with decoded picture hash SEI messages, generated by encoder.
enum HashType
{
  HASHTYPE_MD5             = 0,
  HASHTYPE_CRC             = 1,
  HASHTYPE_CHECKSUM        = 2,
  HASHTYPE_NONE            = 3,
  NUMBER_OF_HASHTYPES      = 4
};

enum SAOMode //mode
{
  SAO_MODE_OFF = 0,
  SAO_MODE_NEW,
  SAO_MODE_MERGE,
  NUM_SAO_MODES
};

enum SAOModeMergeTypes
{
  SAO_MERGE_LEFT =0,
  SAO_MERGE_ABOVE,
  NUM_SAO_MERGE_TYPES
};


enum SAOModeNewTypes
{
  SAO_TYPE_START_EO =0,
  SAO_TYPE_EO_0 = SAO_TYPE_START_EO,
  SAO_TYPE_EO_90,
  SAO_TYPE_EO_135,
  SAO_TYPE_EO_45,

  SAO_TYPE_START_BO,
  SAO_TYPE_BO = SAO_TYPE_START_BO,

  NUM_SAO_NEW_TYPES
};
#define NUM_SAO_EO_TYPES_LOG2 2

enum SAOEOClasses
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};

#define NUM_SAO_BO_CLASSES_LOG2  5
#define NUM_SAO_BO_CLASSES       (1<<NUM_SAO_BO_CLASSES_LOG2)

namespace Profile
{
  enum Name
  {
    NONE = 0,
    MAIN = 1,
    MAIN10 = 2,
    MAINSTILLPICTURE = 3,
    MAINREXT = 4,
    HIGHTHROUGHPUTREXT = 5,
    NEXT = 6
  };
}

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
  };

  enum Name
  {
    // code = (level * 30)
    NONE     = 0,
    LEVEL1   = 30,
    LEVEL2   = 60,
    LEVEL2_1 = 63,
    LEVEL3   = 90,
    LEVEL3_1 = 93,
    LEVEL4   = 120,
    LEVEL4_1 = 123,
    LEVEL5   = 150,
    LEVEL5_1 = 153,
    LEVEL5_2 = 156,
    LEVEL6   = 180,
    LEVEL6_1 = 183,
    LEVEL6_2 = 186,
    LEVEL8_5 = 255,
  };
}

enum CostMode
{
  COST_STANDARD_LOSSY              = 0,
  COST_SEQUENCE_LEVEL_LOSSLESS     = 1,
  COST_LOSSLESS_CODING             = 2,
  COST_MIXED_LOSSLESS_LOSSY_CODING = 3
};

enum WeightedPredictionMethod
{
  WP_PER_PICTURE_WITH_SIMPLE_DC_COMBINED_COMPONENT                          =0,
  WP_PER_PICTURE_WITH_SIMPLE_DC_PER_COMPONENT                               =1,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT                           =2,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING              =3,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING_AND_EXTENSION=4
};

enum FastInterSearchMode
{
  FASTINTERSEARCH_DISABLED = 0,
  FASTINTERSEARCH_MODE1    = 1, // TODO: assign better names to these.
  FASTINTERSEARCH_MODE2    = 2,
  FASTINTERSEARCH_MODE3    = 3
};

enum SPSExtensionFlagIndex
{
  SPS_EXT__REXT           = 0,
//SPS_EXT__MVHEVC         = 1, //for use in future versions
//SPS_EXT__SHVC           = 2, //for use in future versions
  SPS_EXT__NEXT           = 3,
  NUM_SPS_EXTENSION_FLAGS = 8
};

enum PPSExtensionFlagIndex
{
  PPS_EXT__REXT           = 0,
//PPS_EXT__MVHEVC         = 1, //for use in future versions
//PPS_EXT__SHVC           = 2, //for use in future versions
  NUM_PPS_EXTENSION_FLAGS = 8
};

// TODO: Existing names used for the different NAL unit types can be altered to better reflect the names in the spec.
//       However, the names in the spec are not yet stable at this point. Once the names are stable, a cleanup
//       effort can be done without use of macros to alter the names used to indicate the different NAL unit types.
enum NalUnitType
{
  NAL_UNIT_CODED_SLICE_TRAIL_N = 0, // 0
  NAL_UNIT_CODED_SLICE_TRAIL_R,     // 1

  NAL_UNIT_CODED_SLICE_TSA_N,       // 2
  NAL_UNIT_CODED_SLICE_TSA_R,       // 3

  NAL_UNIT_CODED_SLICE_STSA_N,      // 4
  NAL_UNIT_CODED_SLICE_STSA_R,      // 5

  NAL_UNIT_CODED_SLICE_RADL_N,      // 6
  NAL_UNIT_CODED_SLICE_RADL_R,      // 7

  NAL_UNIT_CODED_SLICE_RASL_N,      // 8
  NAL_UNIT_CODED_SLICE_RASL_R,      // 9

  NAL_UNIT_RESERVED_VCL_N10,
  NAL_UNIT_RESERVED_VCL_R11,
  NAL_UNIT_RESERVED_VCL_N12,
  NAL_UNIT_RESERVED_VCL_R13,
  NAL_UNIT_RESERVED_VCL_N14,
  NAL_UNIT_RESERVED_VCL_R15,

  NAL_UNIT_CODED_SLICE_BLA_W_LP,    // 16
  NAL_UNIT_CODED_SLICE_BLA_W_RADL,  // 17
  NAL_UNIT_CODED_SLICE_BLA_N_LP,    // 18
  NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 19
  NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 20
  NAL_UNIT_CODED_SLICE_CRA,         // 21
  NAL_UNIT_RESERVED_IRAP_VCL22,
  NAL_UNIT_RESERVED_IRAP_VCL23,

  NAL_UNIT_RESERVED_VCL24,
  NAL_UNIT_RESERVED_VCL25,
  NAL_UNIT_RESERVED_VCL26,
  NAL_UNIT_RESERVED_VCL27,
  NAL_UNIT_RESERVED_VCL28,
  NAL_UNIT_RESERVED_VCL29,
  NAL_UNIT_RESERVED_VCL30,
  NAL_UNIT_RESERVED_VCL31,

#if HEVC_VPS
  NAL_UNIT_VPS,                     // 32
#else
  NAL_UNIT_RESERVED_32,
#endif
  NAL_UNIT_SPS,                     // 33
  NAL_UNIT_PPS,                     // 34
  NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 35
  NAL_UNIT_EOS,                     // 36
  NAL_UNIT_EOB,                     // 37
  NAL_UNIT_FILLER_DATA,             // 38
  NAL_UNIT_PREFIX_SEI,              // 39
  NAL_UNIT_SUFFIX_SEI,              // 40

  NAL_UNIT_RESERVED_NVCL41,
  NAL_UNIT_RESERVED_NVCL42,
  NAL_UNIT_RESERVED_NVCL43,
  NAL_UNIT_RESERVED_NVCL44,
  NAL_UNIT_RESERVED_NVCL45,
  NAL_UNIT_RESERVED_NVCL46,
  NAL_UNIT_RESERVED_NVCL47,
  NAL_UNIT_UNSPECIFIED_48,
  NAL_UNIT_UNSPECIFIED_49,
  NAL_UNIT_UNSPECIFIED_50,
  NAL_UNIT_UNSPECIFIED_51,
  NAL_UNIT_UNSPECIFIED_52,
  NAL_UNIT_UNSPECIFIED_53,
  NAL_UNIT_UNSPECIFIED_54,
  NAL_UNIT_UNSPECIFIED_55,
  NAL_UNIT_UNSPECIFIED_56,
  NAL_UNIT_UNSPECIFIED_57,
  NAL_UNIT_UNSPECIFIED_58,
  NAL_UNIT_UNSPECIFIED_59,
  NAL_UNIT_UNSPECIFIED_60,
  NAL_UNIT_UNSPECIFIED_61,
  NAL_UNIT_UNSPECIFIED_62,
  NAL_UNIT_UNSPECIFIED_63,
  NAL_UNIT_INVALID,
};

#if SHARP_LUMA_DELTA_QP
enum LumaLevelToDQPMode
{
  LUMALVL_TO_DQP_DISABLED   = 0,
  LUMALVL_TO_DQP_AVG_METHOD = 1, // use average of CTU to determine luma level
#if !WCG_EXT
  LUMALVL_TO_DQP_MAX_METHOD = 2,  // use maximum value of CTU to determine luma level
  LUMALVL_TO_DQP_NUM_MODES  = 3
#else
  LUMALVL_TO_DQP_NUM_MODES  = 2
#endif
};
#endif

enum SaveLoadTag
{
  SAVE_LOAD_INIT = 0,
  SAVE_ENC_INFO  = 1,
  LOAD_ENC_INFO  = 2
};

enum MergeType
{
  MRG_TYPE_DEFAULT_N        = 0, // 0
  MRG_TYPE_SUBPU_ATMVP,
  MRG_TYPE_IBC,
  NUM_MRG_TYPE                   // 5
};

enum TriangleSplit
{
  TRIANGLE_DIR_135 = 0,
  TRIANGLE_DIR_45,
  TRIANGLE_DIR_NUM
};

#if JVET_M0170_MRG_SHARELIST
enum SharedMrgState
{
  NO_SHARE            = 0,
  GEN_ON_SHARED_BOUND = 1,
  SHARING             = 2
};
#endif
//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////

enum EncModeFeature
{
  ENC_FT_FRAC_BITS = 0,
  ENC_FT_DISTORTION,
  ENC_FT_RD_COST,
  ENC_FT_ENC_MODE_TYPE,
  ENC_FT_ENC_MODE_OPTS,
  ENC_FT_ENC_MODE_PART,
  NUM_ENC_FEATURES
};

enum ImvMode
{
  IMV_OFF = 0,
  IMV_DEFAULT,
  IMV_4PEL,
  NUM_IMV_MODES
};


// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for adaptive loop filter
class PicSym;

#define MAX_NUM_SAO_CLASSES  32  //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  SAOMode modeIdc; // NEW, MERGE, OFF
  int typeIdc;     // union of SAOModeMergeTypes and SAOModeNewTypes, depending on modeIdc.
  int typeAuxInfo; // BO: starting band index
  int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset();
  ~SAOOffset();
  void reset();

  const SAOOffset& operator= (const SAOOffset& src);
};

struct SAOBlkParam
{

  SAOBlkParam();
  ~SAOBlkParam();
  void reset();
  const SAOBlkParam& operator= (const SAOBlkParam& src);
  SAOOffset& operator[](int compIdx){ return offsetParam[compIdx];}
  const SAOOffset& operator[](int compIdx) const { return offsetParam[compIdx];}
private:
  SAOOffset offsetParam[MAX_NUM_COMPONENT];

};



struct BitDepths
{
  int recon[MAX_NUM_CHANNEL_TYPE]; ///< the bit depth as indicated in the SPS
};

/// parameters for deblocking filter
struct LFCUParam
{
  bool internalEdge;                     ///< indicates internal edge
  bool leftEdge;                         ///< indicates left edge
  bool topEdge;                          ///< indicates top edge
};



struct PictureHash
{
  std::vector<uint8_t> hash;

  bool operator==(const PictureHash &other) const
  {
    if (other.hash.size() != hash.size())
    {
      return false;
    }
    for(uint32_t i=0; i<uint32_t(hash.size()); i++)
    {
      if (other.hash[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const PictureHash &other) const
  {
    return !(*this == other);
  }
};

struct SEITimeSet
{
  SEITimeSet() : clockTimeStampFlag(false),
                     numUnitFieldBasedFlag(false),
                     countingType(0),
                     fullTimeStampFlag(false),
                     discontinuityFlag(false),
                     cntDroppedFlag(false),
                     numberOfFrames(0),
                     secondsValue(0),
                     minutesValue(0),
                     hoursValue(0),
                     secondsFlag(false),
                     minutesFlag(false),
                     hoursFlag(false),
                     timeOffsetLength(0),
                     timeOffsetValue(0)
  { }
  bool clockTimeStampFlag;
  bool numUnitFieldBasedFlag;
  int  countingType;
  bool fullTimeStampFlag;
  bool discontinuityFlag;
  bool cntDroppedFlag;
  int  numberOfFrames;
  int  secondsValue;
  int  minutesValue;
  int  hoursValue;
  bool secondsFlag;
  bool minutesFlag;
  bool hoursFlag;
  int  timeOffsetLength;
  int  timeOffsetValue;
};

struct SEIMasteringDisplay
{
  bool      colourVolumeSEIEnabled;
  uint32_t      maxLuminance;
  uint32_t      minLuminance;
  uint16_t    primaries[3][2];
  uint16_t    whitePoint[2];
};

#if SHARP_LUMA_DELTA_QP
struct LumaLevelToDeltaQPMapping
{
  LumaLevelToDQPMode                 mode;             ///< use deltaQP determined by block luma level
  double                             maxMethodWeight;  ///< weight of max luma value when mode = 2
  std::vector< std::pair<int, int> > mapping;          ///< first=luma level, second=delta QP.
#if ENABLE_QPA
  bool isEnabled() const { return (mode != LUMALVL_TO_DQP_DISABLED && mode != LUMALVL_TO_DQP_NUM_MODES); }
#else
  bool isEnabled() const { return mode!=LUMALVL_TO_DQP_DISABLED; }
#endif
};
#endif

#if ER_CHROMA_QP_WCG_PPS
struct WCGChromaQPControl
{
  bool isEnabled() const { return enabled; }
  bool   enabled;         ///< Enabled flag (0:default)
  double chromaCbQpScale; ///< Chroma Cb QP Scale (1.0:default)
  double chromaCrQpScale; ///< Chroma Cr QP Scale (1.0:default)
  double chromaQpScale;   ///< Chroma QP Scale (0.0:default)
  double chromaQpOffset;  ///< Chroma QP Offset (0.0:default)
};
#endif

class ChromaCbfs
{
public:
  ChromaCbfs()
    : Cb(true), Cr(true)
  {}
  ChromaCbfs( bool _cbf )
    : Cb( _cbf ), Cr( _cbf )
  {}
public:
  bool sigChroma( ChromaFormat chromaFormat ) const
  {
    if( chromaFormat == CHROMA_400 )
    {
      return false;
    }
    return   ( Cb || Cr );
  }
  bool& cbf( ComponentID compID )
  {
    bool *cbfs[MAX_NUM_TBLOCKS] = { nullptr, &Cb, &Cr };

    return *cbfs[compID];
  }
public:
  bool Cb;
  bool Cr;
};


enum MsgLevel
{
  SILENT  = 0,
  VTMERROR   = 1,
  VTMWARNING = 2,
  VTMINFO    = 3,
  NOTICE  = 4,
  VERBOSE = 5,
  DETAILS = 6
};
#if JVET_M0427_INLOOP_RESHAPER
enum RESHAPE_SIGNAL_TYPE
{
  RESHAPE_SIGNAL_SDR = 0,
  RESHAPE_SIGNAL_PQ  = 1,
  RESHAPE_SIGNAL_HLG = 2,
  RESHAPE_SIGNAL_NULL = 100,
};
#endif


// ---------------------------------------------------------------------------
// exception class
// ---------------------------------------------------------------------------

class Exception : public std::exception
{
public:
  Exception( const std::string& _s ) : m_str( _s ) { }
  Exception( const Exception& _e ) : std::exception( _e ), m_str( _e.m_str ) { }
  virtual ~Exception() noexcept { };
  virtual const char* what() const noexcept { return m_str.c_str(); }
  Exception& operator=( const Exception& _e ) { std::exception::operator=( _e ); m_str = _e.m_str; return *this; }
  template<typename T> Exception& operator<<( T t ) { std::ostringstream oss; oss << t; m_str += oss.str(); return *this; }
private:
  std::string m_str;
};

// if a check fails with THROW or VTMCHECK, please check if ported correctly from assert in revision 1196)
#define THROW(x)            throw( Exception( "\nERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
#define VTMCHECK(c,x)          if(c){ THROW(x); }
#define EXIT(x)             throw( Exception( "\n" ) << x << "\n" )
#define CHECK_NULLPTR(_ptr) VTMCHECK( !( _ptr ), "Accessing an empty pointer pointer!" )

#if !NDEBUG  // for non MSVC compiler, define _DEBUG if in debug mode to have same behavior between MSVC and others in debug
#ifndef _DEBUG
#define _DEBUG 1
#endif
#endif

#if defined( _DEBUG )
#define CHECKD(c,x)         if(c){ THROW(x); }
#else
#define CHECKD(c,x)
#endif // _DEBUG

// ---------------------------------------------------------------------------
// static vector
// ---------------------------------------------------------------------------

template<typename T, size_t N>
class static_vector
{
  T _arr[ N ];
  size_t _size;

public:

  typedef T         value_type;
  typedef size_t    size_type;
  typedef ptrdiff_t difference_type;
  typedef T&        reference;
  typedef T const&  const_reference;
  typedef T*        pointer;
  typedef T const*  const_pointer;
  typedef T*        iterator;
  typedef T const*  const_iterator;

  static const size_type max_num_elements = N;

  static_vector() : _size( 0 )                                 { }
  static_vector( size_t N_ ) : _size( N_ )                     { }
  static_vector( size_t N_, const T& _val ) : _size( 0 )       { resize( N_, _val ); }
  template<typename It>
  static_vector( It _it1, It _it2 ) : _size( 0 )               { while( _it1 < _it2 ) _arr[ _size++ ] = *_it1++; }
  static_vector( std::initializer_list<T> _il ) : _size( 0 )
  {
    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }
  static_vector& operator=( std::initializer_list<T> _il )
  {
    _size = 0;

    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }

  void resize( size_t N_ )                      { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = T() ; _size = N_; }
  void resize( size_t N_, const T& _val )       { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = _val; _size = N_; }
  void reserve( size_t N_ )                     { CHECKD( N_ > N, "capacity exceeded" ); }
  void push_back( const T& _val )               { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = _val; }
  void push_back( T&& val )                     { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = std::forward<T>( val ); }
  void pop_back()                               { CHECKD( _size == 0, "calling pop_back on an empty vector" ); _size--; }
  void pop_front()                              { CHECKD( _size == 0, "calling pop_front on an empty vector" ); _size--; for( int i = 0; i < _size; i++ ) _arr[i] = _arr[i + 1]; }
  void clear()                                  { _size = 0; }
  reference       at( size_t _i )               { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference at( size_t _i ) const         { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       operator[]( size_t _i )       { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference operator[]( size_t _i ) const { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       front()                       { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  const_reference front() const                 { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  reference       back()                        { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  const_reference back() const                  { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  pointer         data()                        { return _arr; }
  const_pointer   data() const                  { return _arr; }
  iterator        begin()                       { return _arr; }
  const_iterator  begin() const                 { return _arr; }
  const_iterator  cbegin() const                { return _arr; }
  iterator        end()                         { return _arr + _size; }
  const_iterator  end() const                   { return _arr + _size; };
  const_iterator  cend() const                  { return _arr + _size; };
  size_type       size() const                  { return _size; };
  size_type       byte_size() const             { return _size * sizeof( T ); }
  bool            empty() const                 { return _size == 0; }

  size_type       capacity() const              { return N; }
  size_type       max_size() const              { return N; }
  size_type       byte_capacity() const         { return sizeof(_arr); }

  iterator        insert( const_iterator _pos, const T& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *const_cast<iterator>( _pos ) = _val;
                                                  _size++;
                                                  return const_cast<iterator>( _pos ); }

  iterator        insert( const_iterator _pos, T&& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *const_cast<iterator>( _pos ) = std::forward<T>( _val );
                                                  _size++; return const_cast<iterator>( _pos ); }
  template<class InputIt>
  iterator        insert( const_iterator _pos, InputIt first, InputIt last )
                                                { const difference_type numEl = last - first;
                                                  CHECKD( _size + numEl >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + numEl] = _arr[i];
                                                  iterator it = const_cast<iterator>( _pos ); _size += numEl;
                                                  while( first != last ) *it++ = *first++;
                                                  return const_cast<iterator>( _pos ); }
};


// ---------------------------------------------------------------------------
// dynamic cache
// ---------------------------------------------------------------------------

template<typename T>
class dynamic_cache
{
  std::vector<T*> m_cache;
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  int64_t         m_cacheId;
#endif

public:

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
  dynamic_cache()
  {
    static int cacheId = 0;
    m_cacheId = cacheId++;
  }

#endif
  ~dynamic_cache()
  {
    deleteEntries();
  }

  void deleteEntries()
  {
    for( auto &p : m_cache )
    {
      delete p;
      p = nullptr;
    }

    m_cache.clear();
  }

  T* get()
  {
    T* ret;

    if( !m_cache.empty() )
    {
      ret = m_cache.back();
      m_cache.pop_back();
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
      VTMCHECK( ret->cacheId != m_cacheId, "Putting item into wrong cache!" );
      VTMCHECK( !ret->cacheUsed,           "Fetched an element that should've been in cache!!" );
#endif
    }
    else
    {
      ret = new T;
    }

#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    ret->cacheId   = m_cacheId;
    ret->cacheUsed = false;

#endif
    return ret;
  }

  void cache( T* el )
  {
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    VTMCHECK( el->cacheId != m_cacheId, "Putting item into wrong cache!" );
    VTMCHECK( el->cacheUsed,            "Putting cached item back into cache!" );

    el->cacheUsed = true;

#endif
    m_cache.push_back( el );
  }

  void cache( std::vector<T*>& vel )
  {
#if ENABLE_SPLIT_PARALLELISM || ENABLE_WPP_PARALLELISM
    for( auto el : vel )
    {
      VTMCHECK( el->cacheId != m_cacheId, "Putting item into wrong cache!" );
      VTMCHECK( el->cacheUsed,            "Putting cached item back into cache!" );

      el->cacheUsed = true;
    }

#endif
    m_cache.insert( m_cache.end(), vel.begin(), vel.end() );
    vel.clear();
  }
};

typedef dynamic_cache<struct CodingUnit    > CUCache;
typedef dynamic_cache<struct PredictionUnit> PUCache;
typedef dynamic_cache<struct TransformUnit > TUCache;

struct XUCache
{
  CUCache cuCache;
  PUCache puCache;
  TUCache tuCache;
};

#define SIGN(x) ( (x) >= 0 ? 1 : -1 )

#define MAX_NUM_ALF_CLASSES             25
#define MAX_NUM_ALF_LUMA_COEFF          13
#define MAX_NUM_ALF_CHROMA_COEFF        7
#define MAX_ALF_FILTER_LENGTH           7
#define MAX_NUM_ALF_COEFF               (MAX_ALF_FILTER_LENGTH * MAX_ALF_FILTER_LENGTH / 2 + 1)

enum AlfFilterType
{
  ALF_FILTER_5,
  ALF_FILTER_7,
  ALF_NUM_OF_FILTER_TYPES
};

struct AlfFilterShape
{
  AlfFilterShape( int size )
    : filterLength( size ),
    numCoeff( size * size / 4 + 1 ),
    filterSize( size * size / 2 + 1 )
  {
    if( size == 5 )
    {
      pattern = {
                 0,
             1,  2,  3,
         4,  5,  6,  5,  4,
             3,  2,  1,
                 0
      };

      weights = {
                 2,
              2, 2, 2,
           2, 2, 1, 1
      };

      golombIdx = {
                 0,
              0, 1, 0,
           0, 1, 2, 2
      };

      filterType = ALF_FILTER_5;
    }
    else if( size == 7 )
    {
      pattern = {
                     0,
                 1,  2,  3,
             4,  5,  6,  7,  8,
         9, 10, 11, 12, 11, 10, 9,
             8,  7,  6,  5,  4,
                 3,  2,  1,
                     0
      };

      weights = {
                    2,
                2,  2,  2,
            2,  2,  2,  2,  2,
        2,  2,  2,  1,  1
      };

      golombIdx = {
                    0,
                 0, 1, 0,
              0, 1, 2, 1, 0,
           0, 1, 2, 3, 3
      };

      filterType = ALF_FILTER_7;
    }
    else
    {
      filterType = ALF_NUM_OF_FILTER_TYPES;
      VTMCHECK( 0, "Wrong ALF filter shape" );
    }
  }

  AlfFilterType filterType;
  int filterLength;
  int numCoeff;      //TO DO: check whether we need both numCoeff and filterSize
  int filterSize;
  std::vector<int> pattern;
  std::vector<int> weights;
  std::vector<int> golombIdx;
};

struct AlfSliceParam
{
  bool                         enabledFlag[MAX_NUM_COMPONENT];                          // alf_slice_enable_flag, alf_chroma_idc
  short                        lumaCoeff[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF]; // alf_coeff_luma_delta[i][j]
  short                        chromaCoeff[MAX_NUM_ALF_CHROMA_COEFF];                   // alf_coeff_chroma[i]
  short                        filterCoeffDeltaIdx[MAX_NUM_ALF_CLASSES];                // filter_coeff_delta[i]
  bool                         alfLumaCoeffFlag[MAX_NUM_ALF_CLASSES];                   // alf_luma_coeff_flag[i]
  int                          numLumaFilters;                                          // number_of_filters_minus1 + 1
  bool                         alfLumaCoeffDeltaFlag;                                   // alf_luma_coeff_delta_flag
  bool                         alfLumaCoeffDeltaPredictionFlag;                         // alf_luma_coeff_delta_prediction_flag
  std::vector<AlfFilterShape>* filterShapes;

  void reset()
  {
    std::memset( enabledFlag, false, sizeof( enabledFlag ) );
    std::memset( lumaCoeff, 0, sizeof( lumaCoeff ) );
    std::memset( chromaCoeff, 0, sizeof( chromaCoeff ) );
    std::memset( filterCoeffDeltaIdx, 0, sizeof( filterCoeffDeltaIdx ) );
    std::memset( alfLumaCoeffFlag, true, sizeof( alfLumaCoeffFlag ) );
    numLumaFilters = 1;
    alfLumaCoeffDeltaFlag = false;
    alfLumaCoeffDeltaPredictionFlag = false;
  }

  const AlfSliceParam& operator = ( const AlfSliceParam& src )
  {
    std::memcpy( enabledFlag, src.enabledFlag, sizeof( enabledFlag ) );
    std::memcpy( lumaCoeff, src.lumaCoeff, sizeof( lumaCoeff ) );
    std::memcpy( chromaCoeff, src.chromaCoeff, sizeof( chromaCoeff ) );
    std::memcpy( filterCoeffDeltaIdx, src.filterCoeffDeltaIdx, sizeof( filterCoeffDeltaIdx ) );
    std::memcpy( alfLumaCoeffFlag, src.alfLumaCoeffFlag, sizeof( alfLumaCoeffFlag ) );
    numLumaFilters = src.numLumaFilters;
    alfLumaCoeffDeltaFlag = src.alfLumaCoeffDeltaFlag;
    alfLumaCoeffDeltaPredictionFlag = src.alfLumaCoeffDeltaPredictionFlag;
    filterShapes = src.filterShapes;
    return *this;
  }
};

#if ADCNN
struct CnnlfSliceParam
{
	bool enabledFlag[MAX_NUM_COMPONENT];

	void reset()
	{
		std::memset(enabledFlag, false, sizeof(enabledFlag));

	}

	const CnnlfSliceParam& operator = (const CnnlfSliceParam& src)
	{
		std::memcpy(enabledFlag, src.enabledFlag, sizeof(enabledFlag));
		return *this;
	}
};
#endif

//! \}

#endif


