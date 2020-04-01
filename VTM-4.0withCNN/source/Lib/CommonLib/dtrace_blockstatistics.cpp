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

/** \file     dtrace_blockstatistics.cpp
 *  \brief    DTrace block statistcis support for next software
 */

#include "dtrace_blockstatistics.h"
#include "dtrace.h"
#include "dtrace_next.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
//#include "CommonLib/CodingStructure.h"
#define BLOCK_STATS_POLYGON_MIN_POINTS                    3
#define BLOCK_STATS_POLYGON_MAX_POINTS                    5

#if K0149_BLOCK_STATISTICS
std::string GetBlockStatisticName(BlockStatistic statistic)
{
  auto statisticIterator = blockstatistic2description.find(statistic);
  // enforces that all delcared statistic enum items are also part of the map
  assert(statisticIterator != blockstatistic2description.end() && "A block statistics declared in the enum is missing in the map for statistic description.");

  return std::get<0>(statisticIterator->second);
}

std::string GetBlockStatisticTypeString(BlockStatistic statistic)
{
  auto statisticIterator = blockstatistic2description.find(statistic);
  // enforces that all delcared statistic enum items are also part of the map
  assert(statisticIterator != blockstatistic2description.end() && "A block statistics declared in the enum is missing in the map for statistic description.");

  BlockStatisticType statisticType = std::get<1>(statisticIterator->second);
  switch (statisticType) {
  case BlockStatisticType::Flag:
    return std::string("Flag");
    break;
  case BlockStatisticType::Vector:
    return std::string("Vector");
    break;
  case BlockStatisticType::Integer:
    return std::string("Integer");
    break;
  case BlockStatisticType::AffineTFVectors:
    return std::string("AffineTFVectors");
    break;
  case BlockStatisticType::Line:
    return std::string("Line");
    break;
  case BlockStatisticType::FlagPolygon:
    return std::string("FlagPolygon");
    break;
  case BlockStatisticType::VectorPolygon:
    return std::string("VectorPolygon");
    break;
  case BlockStatisticType::IntegerPolygon:
    return std::string("IntegerPolygon");
    break;
  default:
    assert(0);
    break;
  }
  return std::string();
}

std::string GetBlockStatisticTypeSpecificInfo(BlockStatistic statistic)
{
  auto statisticIterator = blockstatistic2description.find(statistic);
  // enforces that all delcared statistic enum items are also part of the map
  assert(statisticIterator != blockstatistic2description.end() && "A block statistics declared in the enum is missing in the map for statistic description.");

  return std::get<2>(statisticIterator->second);
}

void CDTrace::dtrace_block_scalar( int k, const CodingStructure &cs, std::string stat_type, signed value )
{
#if BLOCK_STATS_AS_CSV
  dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, cs.area.lx(), cs.area.ly(), cs.area.lwidth(), cs.area.lheight(), stat_type.c_str(), value );
#else
  dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, cs.area.lx(), cs.area.ly(), cs.area.lwidth(), cs.area.lheight(), stat_type.c_str(), value );
#endif
}

void CDTrace::dtrace_block_scalar( int k, const CodingUnit &cu, std::string stat_type, signed value,  bool isChroma /*= false*/  )
{
  const CodingStructure& cs = *cu.cs;
#if BLOCK_STATS_AS_CSV
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, cu.Cb().x*2, cu.Cb().y*2, cu.Cb().width*2, cu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), value );
  }
#else
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, cu.Cb().x*2, cu.Cb().y*2, cu.Cb().width*2, cu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), value );
  }
#endif
}

void CDTrace::dtrace_block_vector( int k, const CodingUnit &cu, std::string stat_type, signed val_x, signed val_y )
{
  const CodingStructure& cs = *cu.cs;
#if BLOCK_STATS_AS_CSV
  dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), val_x, val_y );
#else
  dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->poc, cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), val_x, val_y );
#endif
}

void CDTrace::dtrace_block_scalar( int k, const PredictionUnit &pu, std::string stat_type, signed value, bool isChroma /*= false*/  )
{
  const CodingStructure& cs = *pu.cs;
#if BLOCK_STATS_AS_CSV
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, pu.Cb().x*2, pu.Cb().y*2, pu.Cb().width*2, pu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(), value );
  }
#else
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, pu.Cb().x*2, pu.Cb().y*2, pu.Cb().width*2, pu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(), value );
  }
#endif
}

void CDTrace::dtrace_block_vector( int k, const PredictionUnit &pu, std::string stat_type, signed val_x, signed val_y, bool isChroma /*= false*/  )
{
  const CodingStructure& cs = *pu.cs;
#if BLOCK_STATS_AS_CSV
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc,  pu.Cb().x*2, pu.Cb().y*2, pu.Cb().width*2, pu.Cb().height*2, stat_type.c_str(), val_x*2, val_y*2 );
  }
  else
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(), val_x, val_y );
  }
#else
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->poc, pu.Cb().x*2, pu.Cb().y*2, pu.Cb().width*2, pu.Cb().height*2, stat_type.c_str(), val_x*2, val_y*2 );
  }
  else
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(), val_x, val_y );
  }
#endif
}

void CDTrace::dtrace_block_scalar(int k, const TransformUnit &tu, std::string stat_type, signed value, bool isChroma /*= false*/  )
{
  const CodingStructure& cs = *tu.cs;
#if BLOCK_STATS_AS_CSV
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, tu.Cb().x*2, tu.Cb().y*2, tu.Cb().width*2, tu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%d\n", cs.picture->poc, tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), stat_type.c_str(), value );
  }
#else
  if(isChroma)
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, tu.Cb().x*2, tu.Cb().y*2, tu.Cb().width*2, tu.Cb().height*2, stat_type.c_str(), value );
  }
  else
  {
    dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s=%d\n", cs.picture->poc, tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), stat_type.c_str(), value );
  }
#endif
}

void CDTrace::dtrace_block_vector(int k, const TransformUnit &tu, std::string stat_type, signed val_x, signed val_y)
{
  const CodingStructure& cs = *tu.cs;
#if BLOCK_STATS_AS_CSV
  dtrace<false>(k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d\n", cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(), val_x, val_y);
#else
  dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n", cs.picture->poc, tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), stat_type.c_str(), val_x, val_y);
#endif
}

void CDTrace::dtrace_block_affinetf( int k, const PredictionUnit &pu, std::string stat_type, signed val_x0, signed val_y0, signed val_x1, signed val_y1, signed val_x2, signed val_y2 )
{
  const CodingStructure& cs = *pu.cs;
#if BLOCK_STATS_AS_CSV
  dtrace<false>( k, "BlockStat;%d;%4d;%4d;%2d;%2d;%s;%4d;%4d;%4d;%4d;%4d;%4d\n",
                 cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(),
                 val_x0, val_y0, val_x1, val_y1 , val_x2, val_y2  );
#else
  dtrace<false>( k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d,%4d,%4d,%4d,%4d}\n",
                 cs.picture->poc, pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), stat_type.c_str(),
                 val_x0, val_y0, val_x1, val_y1 , val_x2, val_y2  );
#endif
}

void CDTrace::dtrace_block_line(int k, const CodingUnit &cu, std::string stat_type, int x0, int y0, int x1, int y1)
{
  dtrace<false>(k, "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d,%4d,%4d}\n", cu.slice->getPOC(), cu.lx(), cu.ly(), cu.lwidth(), cu.lheight(), stat_type.c_str(), x0, y0, x1, y1);
}

void CDTrace::dtrace_polygon_scalar(int k, int poc, const std::vector<Position> &polygon, std::string stat_type, signed value)
{
  assert(polygon.size() >= BLOCK_STATS_POLYGON_MIN_POINTS && "Not enough points to from polygon!");
  assert(polygon.size() <= BLOCK_STATS_POLYGON_MAX_POINTS && "Too many points. Unsupported polygon!");
  std::string polygonDescription;
  for (auto position : polygon)
  {
    polygonDescription += "(" + std::to_string(position.x) + ", " + std::to_string(position.y) + ")--";
  }
  dtrace<false>(k, "BlockStat: POC %d @[%s] %s=%d\n", poc, polygonDescription.c_str(), stat_type.c_str(), value);
}

void CDTrace::dtrace_polygon_vector(int k, int poc, const std::vector<Position> &polygon, std::string stat_type, signed val_x, signed val_y)
{
  assert(polygon.size() >= BLOCK_STATS_POLYGON_MIN_POINTS && "Not enough points to from polygon!");
  assert(polygon.size() <= BLOCK_STATS_POLYGON_MAX_POINTS && "Too many points. Unsupported polygon!");
  std::string polygonDescription;
  for (auto position : polygon)
  {
    polygonDescription += "(" + std::to_string(position.x) + ", " + std::to_string(position.y) + ")--";
  }
  dtrace<false>(k, "BlockStat: POC %d @[%s] %s={%4d,%4d}\n", poc, polygonDescription.c_str(), stat_type.c_str(), val_x, val_y);
}

void retrieveTriangularMvInfo(const PredictionUnit& pu, MotionInfo& mi0, MotionInfo& mi1)
{
#if JVET_M0883_TRIANGLE_SIGNALING
  int triangleDir = pu.triangleSplitDir;
#else
  int triangleDir = g_triangleCombination[pu.mergeIdx][0];
#endif
  CMotionBuf mb = pu.getMotionBuf();
  bool foundMv[2] = { false, false };
  bool foundBi = false;
  int32_t idxW  = (int32_t)(g_aucLog2[pu.lwidth() ] - MIN_CU_LOG2);
  int32_t idxH  = (int32_t)(g_aucLog2[pu.lheight()] - MIN_CU_LOG2);
  for (int32_t y = 0; y < mb.height; y++)
  {
    for (int32_t x = 0; x < mb.width; x++)
    {
      if (g_triangleMvStorage[triangleDir][idxH][idxW][y][x] == 0 && foundMv[0] == false)
      {
        mi0.mv[0]     = mb.at(x, y).mv[0];
        mi0.mv[1]     = mb.at(x, y).mv[1];
        mi0.refIdx[0] = mb.at(x, y).refIdx[0];
        mi0.refIdx[1] = mb.at(x, y).refIdx[1];
        foundMv[0] = true;
      }
      if (g_triangleMvStorage[triangleDir][idxH][idxW][y][x] == 1 && foundMv[1] == false)
      {
        mi1.mv[0]     = mb.at(x, y).mv[0];
        mi1.mv[1]     = mb.at(x, y).mv[1];
        mi1.refIdx[0] = mb.at(x, y).refIdx[0];
        mi1.refIdx[1] = mb.at(x, y).refIdx[1];
        foundMv[1] = true;
      }
      if (g_triangleMvStorage[triangleDir][idxH][idxW][y][x] == 2 && foundMv[0] == false && foundMv[1] == false)
      {
        mi0.mv[0] = Mv(0, 0);
        mi0.mv[1] = Mv(0, 0);
        mi1.mv[0] = Mv(0, 0);
        mi1.mv[1] = Mv(0, 0);
        mi0.refIdx[0] = -1;
        mi0.refIdx[1] = -1;
        mi1.refIdx[0] = -1;
        mi1.refIdx[1] = -1;
        if (mb.at(x, y).interDir == 3)
        {
          mi0.mv[0] = mb.at(x, y).mv[0];
          mi1.mv[0] = mb.at(x, y).mv[1];
          mi0.refIdx[0] = mb.at(x, y).refIdx[0];
          mi1.refIdx[0] = mb.at(x, y).refIdx[1];
          foundBi = true;
        }
      }
      if ((foundMv[0] == true && foundMv[1] == true) || foundBi == true)
        return;
    }
  }
}
void retrieveTrianglePolygon(const PredictionUnit& pu, std::vector<Position>& triangle0, std::vector<Position>& triangle1, Position& S, Position& E)
{
#if JVET_M0883_TRIANGLE_SIGNALING
  TriangleSplit triangleDir = TriangleSplit(pu.triangleSplitDir);
#else
  TriangleSplit triangleDir = TriangleSplit(g_triangleCombination[pu.mergeIdx][0]);
#endif
  Position TL = pu.Y().topLeft();
  Position TR = pu.Y().topRight();    TR = TR.offset(1, 0);
  Position BL = pu.Y().bottomLeft();  BL = BL.offset(0, 1);
  Position BR = pu.Y().bottomRight(); BR = BR.offset(1, 1);

  if (triangleDir == TRIANGLE_DIR_135)
  {
    S = Position(0, 0);
    E = Position(pu.Y().width, pu.Y().height);
    triangle0.push_back(TL);
    triangle0.push_back(TR);
    triangle0.push_back(BR);
    triangle1.push_back(TL);
    triangle1.push_back(BL);
    triangle1.push_back(BR);
  }
  else if (triangleDir == TRIANGLE_DIR_45)
  {
    S = Position(0,  pu.Y().height);
    E = Position(pu.Y().width, 0);
    triangle0.push_back(TL);
    triangle0.push_back(TR);
    triangle0.push_back(BL);
    triangle1.push_back(TR);
    triangle1.push_back(BL);
    triangle1.push_back(BR);
  }
  else
  {
    VTMCHECK(triangleDir != TRIANGLE_DIR_45 && triangleDir != TRIANGLE_DIR_135, "Unknown triangle type");
  }
}

void writeBlockStatisticsHeader(const SPS *sps)
{
  static bool has_header_been_written = false;
  if (has_header_been_written)
  {
    return;
  }

  // only write header when block statistics are used
  bool write_blockstatistics =   g_trace_ctx->isChannelActive( D_BLOCK_STATISTICS_ALL) || g_trace_ctx->isChannelActive( D_BLOCK_STATISTICS_CODED);
  if(!write_blockstatistics)
  {
    return;
  }

  DTRACE_HEADER( g_trace_ctx, "# VTMBMS Block Statistics\n");
  // sequence info
  DTRACE_HEADER( g_trace_ctx, "# Sequence size: [%dx %d]\n", sps->getPicWidthInLumaSamples(), sps->getPicHeightInLumaSamples());
  // list statistics
  for( auto i = static_cast<int>(BlockStatistic::PredMode); i < static_cast<int>(BlockStatistic::NumBlockStatistics); i++)
  {
    BlockStatistic statistic = BlockStatistic(i);
    std::string statitic_name = GetBlockStatisticName(statistic);
    std::string statitic_type = GetBlockStatisticTypeString(statistic);
    std::string statitic_type_specific_info = GetBlockStatisticTypeSpecificInfo(statistic);
    DTRACE_HEADER( g_trace_ctx, "# Block Statistic Type: %s; %s; %s\n", statitic_name.c_str(), statitic_type.c_str(), statitic_type_specific_info.c_str());
  }

  has_header_been_written = true;
}

void getAndStoreBlockStatistics(const CodingStructure& cs, const UnitArea& ctuArea)
{
  // two differemt behaviors, depending on which information is needed
  bool writeAll =   g_trace_ctx->isChannelActive( D_BLOCK_STATISTICS_ALL);
  bool writeCoded =   g_trace_ctx->isChannelActive( D_BLOCK_STATISTICS_CODED);

  VTMCHECK(writeAll && writeCoded, "Either used D_BLOCK_STATISTICS_ALL or D_BLOCK_STATISTICS_CODED. Not both at once!")

  if (writeCoded)
    writeAllCodedData(cs, ctuArea);    // this will write out important cu-based data, only if it is actually decoded and used
  else if (writeAll)
    writeAllData(cs, ctuArea);         // this will write out all inter- or intra-prediction related data
}

void writeAllData(const CodingStructure& cs, const UnitArea& ctuArea)
{
  const int maxNumChannelType = cs.pcv->chrFormat != CHROMA_400 && CS::isDualITree( cs ) ? 2 : 1;
  const int nShift = MV_FRACTIONAL_BITS_DIFF;
  const int nOffset = 1 << (nShift - 1);
  for( int ch = 0; ch < maxNumChannelType; ch++ )
  {
    const ChannelType chType = ChannelType( ch );

    for( const CodingUnit &cu : cs.traverseCUs( CS::getArea( cs, ctuArea, chType ), chType ) )
    {
      if( chType == CHANNEL_TYPE_LUMA )
      {
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::PredMode), cu.predMode);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::Depth), cu.depth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QT_Depth), cu.qtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BT_Depth), cu.btDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::MT_Depth), cu.mtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::ChromaQPAdj), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QP), cu.qp);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SplitSeries), (int)cu.splitSeries);

        if (cs.pps->getTransquantBypassEnabledFlag())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::TransQuantBypassFlag), cu.transQuantBypass);
        }

        // skip flag
        if (!cs.slice->isIntra())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SkipFlag), cu.skip);
        }

#if !JVET_M0464_UNI_MTS
        if (!(!((cs.sps->getUseIntraEMT() && CU::isIntra(cu)) || (cs.sps->getUseInterEMT() && CU::isInter(cu))) || isChroma(cu.chType)))
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::EMTFlag), cu.emtFlag);
        }
#endif
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::MMVDSkipFlag), cu.mmvdSkip);
      }
      else if( chType == CHANNEL_TYPE_CHROMA )
      {
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::Depth_Chroma), cu.depth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QT_Depth_Chroma), cu.qtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::BT_Depth_Chroma), cu.btDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::MT_Depth_Chroma), cu.mtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::ChromaQPAdj_Chroma), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::QP_Chroma), cu.qp);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::SplitSeries_Chroma), (int)cu.splitSeries);

        if (cs.pps->getTransquantBypassEnabledFlag())
        {
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::TransQuantBypassFlag_Chroma), cu.transQuantBypass);
        }

#if !JVET_M0464_UNI_MTS
        if (!(!((cs.sps->getUseIntraEMT() && CU::isIntra(cu)) || (cs.sps->getUseInterEMT() && CU::isInter(cu))) || isChroma(cu.chType)))
        {
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::EMTFlag_Chroma), cu.emtFlag);
        }
#endif
      }


      switch( cu.predMode )
      {
      case MODE_INTER:
        {
          for( const PredictionUnit &pu : CU::traversePUs( cu ) )
          {
            if (!pu.cu->skip)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MergeFlag), pu.mergeFlag);
            }
            if( pu.mergeFlag )
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MergeIdx),  pu.mergeIdx);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MergeType), pu.mergeType);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MMVDMergeFlag),  pu.mmvdMergeFlag);
              if (cu.mmvdSkip || pu.mmvdMergeFlag)
              {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MMVDMergeIdx),  pu.mmvdMergeIdx);
              }
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MHIntraFlag),  pu.mhIntraFlag);
              if (pu.mhIntraFlag)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::Luma_IntraMode),  pu.intraDir[COMPONENT_Y]);
              }
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::TriangleFlag), pu.cu->triangle);
            }
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::AffineFlag), pu.cu->affine);
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::AffineType), pu.cu->affineType);
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::InterDir), pu.interDir);

            if (pu.interDir != 2 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVPIdxL0), pu.mvpIdx[REF_PIC_LIST_0]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::RefIdxL0), pu.refIdx[REF_PIC_LIST_0]);
            }
            if (pu.interDir != 1 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVPIdxL1), pu.mvpIdx[REF_PIC_LIST_1]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::RefIdxL1), pu.refIdx[REF_PIC_LIST_1]);
            }
            if (!pu.cu->affine && !pu.cu->triangle)
            {
              if (pu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv = pu.mv[REF_PIC_LIST_0];
                Mv mvd = pu.mvd[REF_PIC_LIST_0];
                mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVDL0), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVL0), mv.hor, mv.ver);
              }
              if (pu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv = pu.mv[REF_PIC_LIST_1];
                Mv mvd = pu.mvd[REF_PIC_LIST_1];
                mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVDL1), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MVL1), mv.hor, mv.ver);
              }
            }
            else if (pu.cu->triangle)
            {
              MotionInfo mi[2];
              std::vector<Position> triangleCorners[2];
              Position S, E;
              retrieveTrianglePolygon(pu, triangleCorners[0], triangleCorners[1], S, E);
              DTRACE_LINE(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::TrianglePartitioning), S.x, S.y, E.x, E.y);
              retrieveTriangularMvInfo(pu, mi[0], mi[1]);
              for (int triangleIdx = 0; triangleIdx < 2; triangleIdx++)
              {
                for (int refIdx = 0; refIdx < 2; refIdx++)
                {
                  mi[triangleIdx].mv[refIdx].hor = mi[triangleIdx].mv[refIdx].hor >= 0 ? (mi[triangleIdx].mv[refIdx].hor + nOffset) >> nShift : -((-mi[triangleIdx].mv[refIdx].hor + nOffset) >> nShift);
                  mi[triangleIdx].mv[refIdx].ver = mi[triangleIdx].mv[refIdx].ver >= 0 ? (mi[triangleIdx].mv[refIdx].ver + nOffset) >> nShift : -((-mi[triangleIdx].mv[refIdx].ver + nOffset) >> nShift);
                  if (mi[triangleIdx].refIdx[refIdx] != -1)
                  {
                    DTRACE_POLYGON_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu.cu->slice->getPOC(), triangleCorners[triangleIdx], GetBlockStatisticName(refIdx==0?BlockStatistic::TriangleMVL0:BlockStatistic::TriangleMVL1), mi[triangleIdx].mv[refIdx].hor, mi[triangleIdx].mv[refIdx].ver);
                  }
                }
              }
            }
            else
            {
              if (pu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv[3];
                const CMotionBuf &mb = pu.getMotionBuf();
                mv[0] = mb.at(0, 0).mv[REF_PIC_LIST_0];
                mv[1] = mb.at(mb.width - 1, 0).mv[REF_PIC_LIST_0];
                mv[2] = mb.at(0, mb.height - 1).mv[REF_PIC_LIST_0];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::AffineMVL0), mv[0].hor, mv[0].ver, mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
              if (pu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv[3];
                const CMotionBuf &mb = pu.getMotionBuf();
                mv[0] = mb.at(0, 0).mv[REF_PIC_LIST_1];
                mv[1] = mb.at(mb.width - 1, 0).mv[REF_PIC_LIST_1];
                mv[2] = mb.at(0, mb.height - 1).mv[REF_PIC_LIST_1];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::AffineMVL1), mv[0].hor, mv[0].ver, mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
            }

            // tracing Motion buffers
            CMotionBuf mb = pu.getMotionBuf();
            // todo: assuming granulatiry == 4. can it be derived?
            for( int y = 0; y < mb.height; y++ )
            {
              for( int x = 0; x < mb.width; x++ )
              {
                const MotionInfo &pixMi = mb.at( x, y );

                if( pixMi.interDir == 1)
                {
                  const Mv mv = pixMi.mv[REF_PIC_LIST_0];
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(),
                     mv.hor,
                     mv.ver);
                }
                else if( pixMi.interDir == 2)
                {
                  const Mv mv = pixMi.mv[REF_PIC_LIST_1];
                  g_trace_ctx->dtrace<false>(
                    D_BLOCK_STATISTICS_ALL,
                    "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                     cs.picture->poc,
                     pu.lx() + 4*x,
                     pu.ly() + 4*y,
                     4,
                     4,
                     GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(),
                     mv.hor,
                     mv.ver);
                }
                else if( pixMi.interDir == 3)
                {
                  {
                    const Mv mv = pixMi.mv[REF_PIC_LIST_0];
                    g_trace_ctx->dtrace<false>(
                      D_BLOCK_STATISTICS_ALL,
                      "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                      cs.picture->poc,
                      pu.lx() + 4*x,
                      pu.ly() + 4*y,
                      4,
                      4,
                      GetBlockStatisticName(BlockStatistic::MotionBufL0).c_str(),
                      mv.hor,
                      mv.ver);
                  }
                  {
                    const Mv mv = pixMi.mv[REF_PIC_LIST_1];
                    g_trace_ctx->dtrace<false>(
                      D_BLOCK_STATISTICS_ALL,
                      "BlockStat: POC %d @(%4d,%4d) [%2dx%2d] %s={%4d,%4d}\n",
                      cs.picture->poc,
                      pu.lx() + 4*x,
                      pu.ly() + 4*y,
                      4,
                      4,
                      GetBlockStatisticName(BlockStatistic::MotionBufL1).c_str(),
                      mv.hor,
                      mv.ver);
                  }
                }

              }
            }




          }
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::IMVMode), cu.imv);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::RootCbf), cu.rootCbf);
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::GBIIndex), cu.GBiIdx);
#if !JVET_M0483_IBC
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::IBCFlag), cu.ibc);
#else
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::IBCFlag), cu.predMode == MODE_IBC);
#endif
        }
        break;
      case MODE_INTRA:
        {

          if(chType == CHANNEL_TYPE_LUMA)
          {
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::IPCM), cu.ipcm);
          }
          else if(chType == CHANNEL_TYPE_CHROMA)
          {
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::IPCM_Chroma), cu.ipcm);
          }

          const uint32_t numChType = ::getNumberValidChannels( cu.chromaFormat );

          for( uint32_t chType = CHANNEL_TYPE_LUMA; chType < numChType; chType++ )
          {
            if( cu.blocks[chType].valid() )
            {
              for( const PredictionUnit &pu : CU::traversePUs( cu ) )
              {
                if( isLuma( ChannelType( chType ) ) )
                {
                  const uint32_t uiChFinalMode  = PU::getFinalIntraMode( pu, ChannelType( chType ) );
                  DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::Luma_IntraMode), uiChFinalMode);
                  DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::MultiRefIdx), pu.multiRefIdx);
                }
                else
                {
                  const uint32_t uiChFinalMode  = PU::getFinalIntraMode( pu, ChannelType( chType ) );
                  DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, pu, GetBlockStatisticName(BlockStatistic::Chroma_IntraMode), uiChFinalMode);
#if ENABLE_CHROMA_422
                    assert(0);
#endif
                }
              }
            }
          }
        }
        break;
      default:
        THROW( "Invalid prediction mode" );
        break;
      }

      for (const TransformUnit &tu : CU::traverseTUs(cu))
      {
        if (tu.Y().valid())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::Cbf_Y), tu.cbf[COMPONENT_Y]);
#if JVET_M0464_UNI_MTS
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::MTSIdx), tu.mtsIdx);
#else
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::TransformSkipFlag_Y), tu.transformSkip[COMPONENT_Y]);
#endif
        }
        if (!(cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_LUMA)))
        {
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::Cbf_Cb), tu.cbf[COMPONENT_Cb]);
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::Cbf_Cr), tu.cbf[COMPONENT_Cr]);
#if !JVET_M0464_UNI_MTS
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::TransformSkipFlag_Cb), tu.transformSkip[COMPONENT_Cb]);
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_ALL, tu, GetBlockStatisticName(BlockStatistic::TransformSkipFlag_Cr), tu.transformSkip[COMPONENT_Cr]);
#endif
        }
      }
    }
  }
}

void writeAllCodedData(const CodingStructure & cs, const UnitArea & ctuArea)
{
  const int nShift = MV_FRACTIONAL_BITS_DIFF;
  const int nOffset = 1 << (nShift - 1);
  const int maxNumChannelType = cs.pcv->chrFormat != CHROMA_400 && CS::isDualITree(cs) ? 2 : 1;

  for (int ch = 0; ch < maxNumChannelType; ch++)
  {
    const ChannelType chType = ChannelType(ch);
    const SPS& sps = *cs.sps;

    for (const CodingUnit &cu : cs.traverseCUs(CS::getArea(cs, ctuArea, chType), chType))
    {
      if( chType == CHANNEL_TYPE_LUMA )
      {
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::Depth), cu.depth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QT_Depth), cu.qtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::BT_Depth), cu.btDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::MT_Depth), cu.mtDepth);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::ChromaQPAdj), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QP), cu.qp);
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::SplitSeries), (int)cu.splitSeries);
        // transquant bypass flag
        if (cs.pps->getTransquantBypassEnabledFlag())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::TransQuantBypassFlag), cu.transQuantBypass);
        }
        // skip flag
        if (!cs.slice->isIntra() && cu.Y().valid())
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::SkipFlag), cu.skip);
          if (cu.skip)
          {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::MMVDSkipFlag), cu.mmvdSkip);
          }
        }

        // prediction mode and partitioning data
        DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::PredMode), cu.predMode);

        if (CU::isIntra(cu))
        {
          if (!(!sps.getPCMEnabledFlag() || cu.lumaSize().width > (1 << sps.getPCMLog2MaxSize()) || cu.lumaSize().width < (1 << sps.getPCMLog2MinSize())))
          {
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::IPCM), cu.ipcm);
          }
        }
      }
      else if (chType == CHANNEL_TYPE_CHROMA )
      {
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::Depth_Chroma), cu.depth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QT_Depth_Chroma), cu.qtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::BT_Depth_Chroma), cu.btDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::MT_Depth_Chroma), cu.mtDepth);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::ChromaQPAdj_Chroma), cu.chromaQpAdj);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::QP_Chroma), cu.qp);
        DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::SplitSeries_Chroma), (int)cu.splitSeries);
        // transquant bypass flag
        if (cs.pps->getTransquantBypassEnabledFlag())
        {
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::TransQuantBypassFlag_Chroma), cu.transQuantBypass);
        }

        if (CU::isIntra(cu))
        {
          if (!(!sps.getPCMEnabledFlag() || cu.lumaSize().width > (1 << sps.getPCMLog2MaxSize()) || cu.lumaSize().width < (1 << sps.getPCMLog2MinSize())))
          {
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::IPCM_Chroma), cu.ipcm);
          }
        }
      }

      for (const PredictionUnit &pu : CU::traversePUs(cu))
      {
        switch (pu.cu->predMode)
        {
          case MODE_INTRA:
          {
            if (pu.Y().valid())
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::Luma_IntraMode), PU::getFinalIntraMode(pu, ChannelType(chType)));
            }
            if (!(pu.chromaFormat == CHROMA_400 || (CS::isDualITree(*pu.cs) && pu.chType == CHANNEL_TYPE_LUMA)))
            {
              DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::Chroma_IntraMode), PU::getFinalIntraMode(pu, CHANNEL_TYPE_CHROMA));
            }
            if (cu.Y().valid() && isLuma(cu.chType))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MultiRefIdx), pu.multiRefIdx);
            }
            break;
          }
          case MODE_INTER:
          {
            if (!pu.cu->skip)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MergeFlag), pu.mergeFlag);
            }
            if (pu.mergeFlag)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MergeIdx),  pu.mergeIdx);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MergeType), pu.mergeType);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MMVDMergeFlag), pu.mmvdMergeFlag);
              if (pu.mmvdMergeFlag)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MMVDMergeIdx), pu.mmvdMergeIdx);
              }
              if (!cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8
                && !pu.mmvdMergeFlag && !cu.mmvdSkip
                )
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::AffineFlag), pu.cu->affine);
              }
              if (pu.cs->sps->getUseMHIntra() && !pu.cu->skip && !pu.cu->affine && !(pu.cu->lwidth() * pu.cu->lheight() < 64 || pu.cu->lwidth() >= MAX_CU_SIZE || pu.cu->lheight() >= MAX_CU_SIZE)
                && !pu.mmvdMergeFlag
                )
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MHIntraFlag), pu.mhIntraFlag);
                if (pu.mhIntraFlag)
                {
                  if (cu.Y().valid())
                  {
                    DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::Luma_IntraMode), pu.intraDir[0]);
                    DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::Chroma_IntraMode), pu.intraDir[1]);
                  }
                }
              }
              if (cu.cs->slice->getSPS()->getUseTriangle() && cu.cs->slice->isInterB() && cu.lwidth() * cu.lheight() >= TRIANGLE_MIN_SIZE && !cu.affine)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::TriangleFlag), cu.triangle);

              }
            }
            else
            {
              if (!pu.cs->slice->isInterP())
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::InterDir), pu.interDir);
              }
              if (!cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8)
              {
                DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::AffineFlag), pu.cu->affine);
                if (cu.affine && !cu.firstPU->mergeFlag && cu.cs->sps->getUseAffineType())
                {
                  DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::AffineType), pu.cu->affineType);
                }
              }
            }
            if (pu.interDir != 2 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVPIdxL0), pu.mvpIdx[REF_PIC_LIST_0]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::RefIdxL0), pu.refIdx[REF_PIC_LIST_0]);
            }
            if (pu.interDir != 1 /* PRED_L1 */)
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVPIdxL1), pu.mvpIdx[REF_PIC_LIST_1]);
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::RefIdxL1), pu.refIdx[REF_PIC_LIST_1]);
            }
            if (!pu.cu->affine && !pu.cu->triangle)
            {
              if (pu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv = pu.mv[REF_PIC_LIST_0];
                Mv mvd = pu.mvd[REF_PIC_LIST_0];
                mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVDL0), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVL0), mv.hor, mv.ver);
              }
              if (pu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv = pu.mv[REF_PIC_LIST_1];
                Mv mvd = pu.mvd[REF_PIC_LIST_1];
                mv.hor = mv.hor >= 0 ? (mv.hor + nOffset) >> nShift : -((-mv.hor + nOffset) >> nShift);
                mv.ver = mv.ver >= 0 ? (mv.ver + nOffset) >> nShift : -((-mv.ver + nOffset) >> nShift);
                mvd.hor = mvd.hor >= 0 ? (mvd.hor + nOffset) >> nShift : -((-mvd.hor + nOffset) >> nShift);
                mvd.ver = mvd.ver >= 0 ? (mvd.ver + nOffset) >> nShift : -((-mvd.ver + nOffset) >> nShift);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVDL1), mvd.hor, mvd.ver);
                DTRACE_BLOCK_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::MVL1), mv.hor, mv.ver);
              }
            }
            else if (pu.cu->triangle)
            {
              MotionInfo mi[2];
              std::vector<Position> triangleCorners[2];
              Position S, E;
              retrieveTrianglePolygon(pu, triangleCorners[0], triangleCorners[1], S, E);
              DTRACE_LINE(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::TrianglePartitioning), S.x, S.y, E.x, E.y);
              retrieveTriangularMvInfo(pu, mi[0], mi[1]);
              for (int triangleIdx = 0; triangleIdx < 2; triangleIdx++)
              {
                for (int refIdx = 0; refIdx < 2; refIdx++)
                {
                  mi[triangleIdx].mv[refIdx].hor = mi[triangleIdx].mv[refIdx].hor >= 0 ? (mi[triangleIdx].mv[refIdx].hor + nOffset) >> nShift : -((-mi[triangleIdx].mv[refIdx].hor + nOffset) >> nShift);
                  mi[triangleIdx].mv[refIdx].ver = mi[triangleIdx].mv[refIdx].ver >= 0 ? (mi[triangleIdx].mv[refIdx].ver + nOffset) >> nShift : -((-mi[triangleIdx].mv[refIdx].ver + nOffset) >> nShift);
                  if (mi[triangleIdx].refIdx[refIdx] != -1)
                  {
                    DTRACE_POLYGON_VECTOR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu.cu->slice->getPOC(), triangleCorners[triangleIdx], GetBlockStatisticName(refIdx==0?BlockStatistic::TriangleMVL0:BlockStatistic::TriangleMVL1), mi[triangleIdx].mv[refIdx].hor, mi[triangleIdx].mv[refIdx].ver);
                  }
                }
              }
            }
            else
            {
              if (pu.interDir != 2 /* PRED_L1 */)
              {
                Mv mv[3];
                const CMotionBuf &mb = pu.getMotionBuf();
                mv[0] = mb.at(0, 0).mv[REF_PIC_LIST_0];
                mv[1] = mb.at(mb.width - 1, 0).mv[REF_PIC_LIST_0];
                mv[2] = mb.at(0, mb.height - 1).mv[REF_PIC_LIST_0];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::AffineMVL0), mv[0].hor, mv[0].ver, mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
              if (pu.interDir != 1 /* PRED_L1 */)
              {
                Mv mv[3];
                const CMotionBuf &mb = pu.getMotionBuf();
                mv[0] = mb.at(0, 0).mv[REF_PIC_LIST_1];
                mv[1] = mb.at(mb.width - 1, 0).mv[REF_PIC_LIST_1];
                mv[2] = mb.at(0, mb.height - 1).mv[REF_PIC_LIST_1];
                // motion vectors should use low precision or they will appear to large
                mv[0].hor = mv[0].hor >= 0 ? (mv[0].hor + nOffset) >> nShift : -((-mv[0].hor + nOffset) >> nShift);
                mv[0].ver = mv[0].ver >= 0 ? (mv[0].ver + nOffset) >> nShift : -((-mv[0].ver + nOffset) >> nShift);
                mv[1].hor = mv[1].hor >= 0 ? (mv[1].hor + nOffset) >> nShift : -((-mv[1].hor + nOffset) >> nShift);
                mv[1].ver = mv[1].ver >= 0 ? (mv[1].ver + nOffset) >> nShift : -((-mv[1].ver + nOffset) >> nShift);
                mv[2].hor = mv[2].hor >= 0 ? (mv[2].hor + nOffset) >> nShift : -((-mv[2].hor + nOffset) >> nShift);
                mv[2].ver = mv[2].ver >= 0 ? (mv[2].ver + nOffset) >> nShift : -((-mv[2].ver + nOffset) >> nShift);
                DTRACE_BLOCK_AFFINETF(g_trace_ctx, D_BLOCK_STATISTICS_CODED, pu, GetBlockStatisticName(BlockStatistic::AffineMVL1), mv[0].hor, mv[0].ver, mv[1].hor, mv[1].ver, mv[2].hor, mv[2].ver);
              }
            }
            if (cu.cs->sps->getAMVREnabledFlag() && CU::hasSubCUNonZeroMVd(cu))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::IMVMode), cu.imv);
            }
            if (CU::isGBiIdxCoded(cu))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_ALL, cu, GetBlockStatisticName(BlockStatistic::GBIIndex), cu.GBiIdx);
            }
            break;
          }
          default:
          {
            VTMCHECK(1, "Invalid prediction mode");
            break;
          }
        }
      } // end pu
      if (CU::isInter(cu))
      {
        const PredictionUnit &pu = *cu.firstPU;
        if ( !pu.mergeFlag )
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::RootCbf), cu.rootCbf);
        }
      }
      if (cu.rootCbf || CU::isIntra(cu))
      {
        for (const TransformUnit &tu : CU::traverseTUs(cu))
        {
          if (tu.Y().valid())
          {
            DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::Cbf_Y), tu.cbf[COMPONENT_Y]);
#if JVET_M0464_UNI_MTS
            DTRACE_BLOCK_SCALAR( g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName( BlockStatistic::MTSIdx ), tu.mtsIdx );
#else
            if (!(!tu.cu->cs->pps->getUseTransformSkip() || tu.cu->transQuantBypass || !TU::hasTransformSkipFlag(*tu.cs, tu.blocks[COMPONENT_Y]) || (isLuma(COMPONENT_Y) && tu.cu->emtFlag)))
            {
              DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::TransformSkipFlag_Y), tu.transformSkip[COMPONENT_Y]);
            }
#endif
          }
          if (!(cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_LUMA)))
          {
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::Cbf_Cb), tu.cbf[COMPONENT_Cb]);
            DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::Cbf_Cr), tu.cbf[COMPONENT_Cr]);
#if !JVET_M0464_UNI_MTS
            if (!(!tu.cu->cs->pps->getUseTransformSkip() || tu.cu->transQuantBypass || !TU::hasTransformSkipFlag(*tu.cs, tu.blocks[COMPONENT_Cb]) || (isLuma(COMPONENT_Cb) && tu.cu->emtFlag)))
            {
              DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::TransformSkipFlag_Cb), tu.transformSkip[COMPONENT_Cb]);
            }
            if (!(!tu.cu->cs->pps->getUseTransformSkip() || tu.cu->transQuantBypass || !TU::hasTransformSkipFlag(*tu.cs, tu.blocks[COMPONENT_Cr]) || (isLuma(COMPONENT_Cr) && tu.cu->emtFlag)))
            {
              DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, tu, GetBlockStatisticName(BlockStatistic::TransformSkipFlag_Cr), tu.transformSkip[COMPONENT_Cr]);
            }
#endif
          }
        }
      }
#if !JVET_M0464_UNI_MTS
      if (!(!((cs.sps->getUseIntraEMT() && CU::isIntra(cu)) || (cs.sps->getUseInterEMT() && CU::isInter(cu))) || isChroma(cu.chType)))
      {
        if( isLuma( ChannelType( chType ) ) )
        {
          DTRACE_BLOCK_SCALAR(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::EMTFlag), cu.emtFlag);
        }
        else
        {
          DTRACE_BLOCK_SCALAR_CHROMA(g_trace_ctx, D_BLOCK_STATISTICS_CODED, cu, GetBlockStatisticName(BlockStatistic::EMTFlag_Chroma), cu.emtFlag);
        }
      }
#endif
    }
  }
}
#endif
