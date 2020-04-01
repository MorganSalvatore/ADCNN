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

/** \file     dtrace_blockstatistics.h
 *  \brief    DTrace block statistcis support for next software
 */

#ifndef _DTRACE_BLOCKSTATISTICS_H_
#define _DTRACE_BLOCKSTATISTICS_H_

#include <map>
#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"

#if K0149_BLOCK_STATISTICS
#define DTRACE_HEADER(ctx,...) ctx->dtrace_header( __VA_ARGS__ )
#define DTRACE_BLOCK_SCALAR(ctx,channel,cs_cu_pu,stat_type,val)                         ctx->dtrace_block_scalar( channel, cs_cu_pu, stat_type, val )
#define DTRACE_BLOCK_SCALAR_CHROMA(ctx,channel,cs_cu_pu,stat_type,val)                  ctx->dtrace_block_scalar( channel, cs_cu_pu, stat_type, val, true)
#define DTRACE_BLOCK_VECTOR(ctx,channel,cu_pu,stat_type,v_x,v_y)                        ctx->dtrace_block_vector( channel, cu_pu, stat_type, v_x, v_y )
#define DTRACE_BLOCK_VECTOR_CHROMA(ctx,channel,pu,stat_type,v_x,v_y)                    ctx->dtrace_block_vector( channel, pu, stat_type, v_x, v_y, true )
#define DTRACE_BLOCK_AFFINETF(ctx,channel,pu,stat_type,v_x0,v_y0,v_x1,v_y1,v_x2,v_y2)   ctx->dtrace_block_affinetf( channel, pu, stat_type, v_x0, v_y0, v_x1, v_y1, v_x2, v_y2 )
#define DTRACE_LINE(ctx,channel,cs_cu_pu,stat_type, x0, y0, x1, y1)                     ctx->dtrace_block_line( channel, cs_cu_pu, stat_type, x0, y0, x1, y1)
#define DTRACE_POLYGON_SCALAR(ctx,channel, poc, polygon,stat_type,val)                  ctx->dtrace_polygon_scalar( channel, poc, polygon, stat_type, val )
#define DTRACE_POLYGON_VECTOR(ctx,channel, poc, polygon,stat_type,v_x,v_y)              ctx->dtrace_polygon_vector( channel, poc, polygon, stat_type, v_x, v_y )

enum class BlockStatistic {
  // general
  PredMode,
  Depth,
  QT_Depth,
  BT_Depth,
  MT_Depth,
  ChromaQPAdj,
  QP,
  SplitSeries,
  TransQuantBypassFlag,
#if JVET_M0464_UNI_MTS
  MTSIdx,
#else
  EMTFlag,
  TransformSkipFlag_Y,
  TransformSkipFlag_Cb,
  TransformSkipFlag_Cr,
#endif

  // intra
  IPCM,
  Luma_IntraMode,
  Chroma_IntraMode,
  MultiRefIdx,
  // inter
  SkipFlag,
  RootCbf,
  Cbf_Y,
  Cbf_Cb,
  Cbf_Cr,
  IMVMode,
  InterDir,
  MergeFlag,
  MergeIdx,
  MergeType,
  MVPIdxL0,
  MVPIdxL1,
  MVL0,
  MVL1,
  MVDL0,
  MVDL1,
  MotionBufL0,
  MotionBufL1,
  RefIdxL0,
  RefIdxL1,
  AffineFlag,
  AffineMVL0,
  AffineMVL1,
  AffineType,
  MMVDSkipFlag,
  MMVDMergeFlag,
  MMVDMergeIdx,
  MHIntraFlag,
  TriangleFlag,
  TrianglePartitioning,
  TriangleMVL0, //<< currently only uni-prediction enabled
  TriangleMVL1, //<< currently only uni-prediction enabled
  GBIIndex,
  IBCFlag,
// for dual tree
  // general
  Depth_Chroma,
  QT_Depth_Chroma,
  BT_Depth_Chroma,
  MT_Depth_Chroma,
  ChromaQPAdj_Chroma,
  QP_Chroma,
  SplitSeries_Chroma,
  TransQuantBypassFlag_Chroma,
#if !JVET_M0464_UNI_MTS
  EMTFlag_Chroma, // this is called flag, though the type is UChar ?!
#endif

  // intra
  IPCM_Chroma,

  NumBlockStatistics,
};

enum class BlockStatisticType {
  Flag,
  Vector,
  Integer,
  AffineTFVectors,
  Line,
  FlagPolygon,
  VectorPolygon,
  IntegerPolygon,
};

static const std::map<BlockStatistic, std::tuple<std::string, BlockStatisticType, std::string>> blockstatistic2description =
{
  // Statistics enum                                                                                Statistics name string         Statistic Type                              Type specific information:
  //                                                                                                                                                                           Value range, vector scale
  { BlockStatistic::PredMode,               std::tuple<std::string, BlockStatisticType, std::string>{"PredMode",                    BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::MergeFlag,              std::tuple<std::string, BlockStatisticType, std::string>{"MergeFlag",                   BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::MVL0,                   std::tuple<std::string, BlockStatisticType, std::string>{"MVL0",                        BlockStatisticType::Vector,                 "Scale: 4"}},
  { BlockStatistic::MVL1,                   std::tuple<std::string, BlockStatisticType, std::string>{"MVL1",                        BlockStatisticType::Vector,                 "Scale: 4"}},
  { BlockStatistic::IPCM,                   std::tuple<std::string, BlockStatisticType, std::string>{"IPCM",                        BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::Luma_IntraMode,         std::tuple<std::string, BlockStatisticType, std::string>{"Luma_IntraMode",              BlockStatisticType::Integer,                "[0, " + std::to_string(NUM_INTRA_MODE) + "]"}},
  { BlockStatistic::Chroma_IntraMode,       std::tuple<std::string, BlockStatisticType, std::string>{"Chroma_IntraMode",            BlockStatisticType::Integer,                "[0, " + std::to_string(NUM_INTRA_MODE) + "]"}},
  { BlockStatistic::SkipFlag,               std::tuple<std::string, BlockStatisticType, std::string>{"SkipFlag",                    BlockStatisticType::Flag,                   ""}},
#if JVET_M0464_UNI_MTS
  { BlockStatistic::MTSIdx,                 std::tuple<std::string, BlockStatisticType, std::string>{"TransformSkipFlag_Y",         BlockStatisticType::Integer,                ""}},
#else
  { BlockStatistic::TransformSkipFlag_Y,    std::tuple<std::string, BlockStatisticType, std::string>{"TransformSkipFlag_Y",         BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::TransformSkipFlag_Cb,   std::tuple<std::string, BlockStatisticType, std::string>{"TransformSkipFlag_Cb",        BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::TransformSkipFlag_Cr,   std::tuple<std::string, BlockStatisticType, std::string>{"TransformSkipFlag_Cr",        BlockStatisticType::Flag,                   ""}},
#endif
  { BlockStatistic::Depth,                  std::tuple<std::string, BlockStatisticType, std::string>{"Depth",                       BlockStatisticType::Integer,                "[0, 7]"}},
  { BlockStatistic::QT_Depth,               std::tuple<std::string, BlockStatisticType, std::string>{"QT_Depth",                    BlockStatisticType::Integer,                "[0, 7]"}},
  { BlockStatistic::BT_Depth,               std::tuple<std::string, BlockStatisticType, std::string>{"BT_Depth",                    BlockStatisticType::Integer,                "[0, 7]"}},
  { BlockStatistic::MT_Depth,               std::tuple<std::string, BlockStatisticType, std::string>{"MT_Depth",                    BlockStatisticType::Integer,                "[0, 7]"}},
  { BlockStatistic::ChromaQPAdj,            std::tuple<std::string, BlockStatisticType, std::string>{"ChromaQPAdj",                 BlockStatisticType::Integer,                "[-10, 10]"}},
  { BlockStatistic::QP,                     std::tuple<std::string, BlockStatisticType, std::string>{"QP",                          BlockStatisticType::Integer,                "[0, 51]"}},
  { BlockStatistic::SplitSeries,            std::tuple<std::string, BlockStatisticType, std::string>{"SplitSeries",                 BlockStatisticType::Integer,                "[0, " + std::to_string(std::numeric_limits<SplitSeries>::max()) + "]"}},
  { BlockStatistic::RootCbf,                std::tuple<std::string, BlockStatisticType, std::string>{"RootCbf",                     BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::Cbf_Y,                  std::tuple<std::string, BlockStatisticType, std::string>{"Cbf_Y",                       BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::Cbf_Cb,                 std::tuple<std::string, BlockStatisticType, std::string>{"Cbf_Cb",                      BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::Cbf_Cr,                 std::tuple<std::string, BlockStatisticType, std::string>{"Cbf_Cr",                      BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::TransQuantBypassFlag,   std::tuple<std::string, BlockStatisticType, std::string>{"TransQuantBypassFlag",        BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::MergeIdx,               std::tuple<std::string, BlockStatisticType, std::string>{"MergeIdx",                    BlockStatisticType::Integer,                "[0, 7]"}},
  { BlockStatistic::InterDir,               std::tuple<std::string, BlockStatisticType, std::string>{"InterDir",                    BlockStatisticType::Integer,                "[1, 3]"}},
  { BlockStatistic::MergeType,              std::tuple<std::string, BlockStatisticType, std::string>{"MergeType",                   BlockStatisticType::Integer,                "[0, 2]"}},
  { BlockStatistic::MVPIdxL0,               std::tuple<std::string, BlockStatisticType, std::string>{"MVPIdxL0",                    BlockStatisticType::Integer,                "[0, 1]"}},
  { BlockStatistic::MVDL0,                  std::tuple<std::string, BlockStatisticType, std::string>{"MVDL0",                       BlockStatisticType::Vector,                 "Scale: 4"}},
  { BlockStatistic::RefIdxL0,               std::tuple<std::string, BlockStatisticType, std::string>{"RefIdxL0",                    BlockStatisticType::Integer,                "[0, 4]"}},
  { BlockStatistic::MVPIdxL1,               std::tuple<std::string, BlockStatisticType, std::string>{"MVPIdxL1",                    BlockStatisticType::Integer,                "[0, 1]"}},
  { BlockStatistic::MVDL1,                  std::tuple<std::string, BlockStatisticType, std::string>{"MVDL1",                       BlockStatisticType::Vector,                 "Scale: 4"}},
  { BlockStatistic::RefIdxL1,               std::tuple<std::string, BlockStatisticType, std::string>{"RefIdxL1",                    BlockStatisticType::Integer,                "[0, 4]"}},
  { BlockStatistic::IMVMode,                std::tuple<std::string, BlockStatisticType, std::string>{"IMVMode",                     BlockStatisticType::Integer,                "[0, 2]"}},
  { BlockStatistic::AffineFlag,             std::tuple<std::string, BlockStatisticType, std::string>{"AffineFlag",                  BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::AffineMVL0,             std::tuple<std::string, BlockStatisticType, std::string>{"AffineMVL0",                  BlockStatisticType::AffineTFVectors,        "Scale: 4"}},
  { BlockStatistic::AffineMVL1,             std::tuple<std::string, BlockStatisticType, std::string>{"AffineMVL1",                  BlockStatisticType::AffineTFVectors,        "Scale: 4"}},
  { BlockStatistic::AffineType,             std::tuple<std::string, BlockStatisticType, std::string>{"AffineType",                  BlockStatisticType::Flag,                   ""} },
#if !JVET_M0464_UNI_MTS
  { BlockStatistic::EMTFlag,                std::tuple<std::string, BlockStatisticType, std::string>{"EMTFlag",                     BlockStatisticType::Flag,                   ""}},
#endif
  { BlockStatistic::MotionBufL0,            std::tuple<std::string, BlockStatisticType, std::string>{"MotionBufL0",                 BlockStatisticType::Vector,                 "Scale: 16"}},
  { BlockStatistic::MotionBufL1,            std::tuple<std::string, BlockStatisticType, std::string>{"MotionBufL1",                 BlockStatisticType::Vector,                 "Scale: 16"}},
  { BlockStatistic::MultiRefIdx,            std::tuple<std::string, BlockStatisticType, std::string>{"MultiRefIdx",                 BlockStatisticType::Integer,                "[0, 1]"}},
  { BlockStatistic::MMVDSkipFlag,           std::tuple<std::string, BlockStatisticType, std::string>{"MMVDSkipFlag",                BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::MMVDMergeFlag,          std::tuple<std::string, BlockStatisticType, std::string>{"MMVDMergeFlag",               BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::MMVDMergeIdx,           std::tuple<std::string, BlockStatisticType, std::string>{"MMVDMergeIdx",                BlockStatisticType::Integer,                "[0, 1]"}},
  { BlockStatistic::MHIntraFlag,            std::tuple<std::string, BlockStatisticType, std::string>{"MHIntraFlag",                 BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::TriangleFlag,           std::tuple<std::string, BlockStatisticType, std::string>{"TriangleFlag",                BlockStatisticType::Flag,                   ""}},
  { BlockStatistic::TrianglePartitioning,   std::tuple<std::string, BlockStatisticType, std::string>{"TrianglePartitioning",        BlockStatisticType::Line,                   ""}},
  { BlockStatistic::TriangleMVL0,           std::tuple<std::string, BlockStatisticType, std::string>{"TriangleMVL0",                BlockStatisticType::VectorPolygon,          "Scale: 4"}},
  { BlockStatistic::TriangleMVL1,           std::tuple<std::string, BlockStatisticType, std::string>{"TriangleMVL1",                BlockStatisticType::VectorPolygon,          "Scale: 4"}},
  { BlockStatistic::GBIIndex,               std::tuple<std::string, BlockStatisticType, std::string>{"GBIIndex",                    BlockStatisticType::Integer,                "[0, 4]"}},
  { BlockStatistic::IBCFlag,                std::tuple<std::string, BlockStatisticType, std::string>{"IBCFlag",                     BlockStatisticType::Flag,                   ""}},
  // for dual tree
  { BlockStatistic::Depth_Chroma,                  std::tuple<std::string, BlockStatisticType, std::string>{"Depth_Chroma",                       BlockStatisticType::Integer,                "[0, 10]"}}, // todo: actual limits?
  { BlockStatistic::QT_Depth_Chroma,               std::tuple<std::string, BlockStatisticType, std::string>{"QT_Depth_Chroma",                    BlockStatisticType::Integer,                "[0, 10]"}}, // todo: actual limits?
  { BlockStatistic::BT_Depth_Chroma,               std::tuple<std::string, BlockStatisticType, std::string>{"BT_Depth_Chroma",                    BlockStatisticType::Integer,                "[0, 10]"}}, // todo: actual limits?
  { BlockStatistic::MT_Depth_Chroma,               std::tuple<std::string, BlockStatisticType, std::string>{"MT_Depth_Chroma",                    BlockStatisticType::Integer,                "[0, 10]"}}, // todo: actual limits?
  { BlockStatistic::ChromaQPAdj_Chroma,            std::tuple<std::string, BlockStatisticType, std::string>{"ChromaQPAdj_Chroma",                 BlockStatisticType::Integer,                "[-10, 10]"}}, // todo: actual limits?
  { BlockStatistic::QP_Chroma,                     std::tuple<std::string, BlockStatisticType, std::string>{"QP_Chroma",                          BlockStatisticType::Integer,                "[0, 51]"}},
  { BlockStatistic::SplitSeries_Chroma,            std::tuple<std::string, BlockStatisticType, std::string>{"SplitSeries_Chroma",                 BlockStatisticType::Integer,                "[0, " + std::to_string(std::numeric_limits<SplitSeries>::max()) + "]"}},
  { BlockStatistic::TransQuantBypassFlag_Chroma,   std::tuple<std::string, BlockStatisticType, std::string>{"TransQuantBypassFlag_Chroma",        BlockStatisticType::Flag,                   ""}},
#if !JVET_M0464_UNI_MTS
  { BlockStatistic::EMTFlag_Chroma,                std::tuple<std::string, BlockStatisticType, std::string>{"EMTFlag_Chroma",                     BlockStatisticType::Integer,                "[0, 10]"}}, // todo: actual limits?
#endif
  { BlockStatistic::IPCM_Chroma,                   std::tuple<std::string, BlockStatisticType, std::string>{"IPCM_Chroma",                        BlockStatisticType::Flag,                   ""}},

};


std::string GetBlockStatisticName(BlockStatistic statistic);
std::string GetBlockStatisticTypeString(BlockStatistic statistic);
std::string GetBlockStatisticTypeSpecificInfo(BlockStatistic statistic);

void writeBlockStatisticsHeader(const SPS *sps);
void getAndStoreBlockStatistics(const CodingStructure& cs, const UnitArea& ctuArea);
void writeAllData(const CodingStructure& cs, const UnitArea& ctuArea);
void writeAllCodedData(const CodingStructure& cs, const UnitArea& ctuArea);
#endif

#endif // _DTRACE_BLOCKSTATISTICS_H_
