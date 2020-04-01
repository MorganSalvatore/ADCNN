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

/** \file     Rom.cpp
    \brief    global variables & functions
*/

#include "Rom.h"
#include "UnitTools.h"

#include <memory.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iomanip>

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

#if ENABLE_TRACING
CDTrace *g_trace_ctx = NULL;
#endif


//! \ingroup CommonLib
//! \{

MsgLevel g_verbosity = VERBOSE;

const char* nalUnitTypeToString(NalUnitType type)
{
  switch (type)
  {
  case NAL_UNIT_CODED_SLICE_TRAIL_R:    return "TRAIL_R";
  case NAL_UNIT_CODED_SLICE_TRAIL_N:    return "TRAIL_N";
  case NAL_UNIT_CODED_SLICE_TSA_R:      return "TSA_R";
  case NAL_UNIT_CODED_SLICE_TSA_N:      return "TSA_N";
  case NAL_UNIT_CODED_SLICE_STSA_R:     return "STSA_R";
  case NAL_UNIT_CODED_SLICE_STSA_N:     return "STSA_N";
  case NAL_UNIT_CODED_SLICE_BLA_W_LP:   return "BLA_W_LP";
  case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_W_RADL";
  case NAL_UNIT_CODED_SLICE_BLA_N_LP:   return "BLA_N_LP";
  case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_W_RADL";
  case NAL_UNIT_CODED_SLICE_IDR_N_LP:   return "IDR_N_LP";
  case NAL_UNIT_CODED_SLICE_CRA:        return "CRA";
  case NAL_UNIT_CODED_SLICE_RADL_R:     return "RADL_R";
  case NAL_UNIT_CODED_SLICE_RADL_N:     return "RADL_N";
  case NAL_UNIT_CODED_SLICE_RASL_R:     return "RASL_R";
  case NAL_UNIT_CODED_SLICE_RASL_N:     return "RASL_N";
#if HEVC_VPS
  case NAL_UNIT_VPS:                    return "VPS";
#endif
  case NAL_UNIT_SPS:                    return "SPS";
  case NAL_UNIT_PPS:                    return "PPS";
  case NAL_UNIT_ACCESS_UNIT_DELIMITER:  return "AUD";
  case NAL_UNIT_EOS:                    return "EOS";
  case NAL_UNIT_EOB:                    return "EOB";
  case NAL_UNIT_FILLER_DATA:            return "FILLER";
  case NAL_UNIT_PREFIX_SEI:             return "Prefix SEI";
  case NAL_UNIT_SUFFIX_SEI:             return "Suffix SEI";
  default:                              return "UNK";
  }
}

class ScanGenerator
{
private:
  uint32_t m_line, m_column;
  const uint32_t m_blockWidth, m_blockHeight;
  const uint32_t m_stride;
  const CoeffScanType m_scanType;

public:
  ScanGenerator(uint32_t blockWidth, uint32_t blockHeight, uint32_t stride, CoeffScanType scanType)
    : m_line(0), m_column(0), m_blockWidth(blockWidth), m_blockHeight(blockHeight), m_stride(stride), m_scanType(scanType)
  { }

  uint32_t GetCurrentX() const { return m_column; }
  uint32_t GetCurrentY() const { return m_line; }

  uint32_t GetNextIndex(uint32_t blockOffsetX, uint32_t blockOffsetY)
  {
    const uint32_t rtn = ((m_line + blockOffsetY) * m_stride) + m_column + blockOffsetX;

    //advance line and column to the next position
    switch (m_scanType)
    {
      //------------------------------------------------

      case SCAN_DIAG:

        if ((m_column == m_blockWidth - 1) || (m_line == 0)) //if we reach the end of a rank, go diagonally down to the next one
        {
          m_line += m_column + 1;
          m_column = 0;

          if (m_line >= m_blockHeight) //if that takes us outside the block, adjust so that we are back on the bottom row
          {
            m_column += m_line - (m_blockHeight - 1);
            m_line = m_blockHeight - 1;
          }
        }
        else
        {
          m_column++;
          m_line--;
        }
        break;

#if HEVC_USE_MDCS
      //------------------------------------------------
      case SCAN_HOR:

        if (m_column == m_blockWidth - 1)
        {
          m_line++;
          m_column = 0;
        }
        else
        {
          m_column++;
        }
        break;

      //------------------------------------------------

      case SCAN_VER:

        if (m_line == m_blockHeight - 1)
        {
          m_column++;
          m_line = 0;
        }
        else
        {
          m_line++;
        }
        break;

#endif
      //------------------------------------------------

      default:

        THROW("VTMERROR: Unknown scan type \"" << m_scanType << "\"in ScanGenerator::GetNextIndex");
        break;
    }

    return rtn;
  }
};
#if !JVET_M0064_CCLM_SIMPLIFICATION
int g_aiLMDivTableLow[] = {
  0,     0,     21845, 0,     13107, 43690, 18724, 0,     50972, 39321, 53620, 21845, 15123, 9362,  4369,  0,     3855,
  58254, 17246, 52428, 49932, 59578, 25644, 43690, 28835, 40329, 16990, 37449, 56496, 34952, 4228,  0,     61564, 34695,
  29959, 29127, 15941, 41391, 26886, 26214, 28771, 24966, 6096,  29789, 23301, 45590, 25098, 21845, 30761, 47185, 1285,
  20164, 34622, 41263, 36938, 18724, 49439, 61016, 51095, 17476, 23635, 2114,  16644, 0,     16131, 63550, 9781,  50115,
  52238, 14979, 2769,  14563, 49376, 40738, 53302, 20695, 7660,  13443, 37330, 13107, 5663,  14385, 38689, 12483, 771,
  3048,  18832, 47662, 23563, 11650, 11522, 22795, 45100, 12549, 55878, 43690, 41213, 48148, 64212, 23592, 57100, 33410,
  17815, 10082, 9986,  17311, 31849, 53399, 16233, 51237, 27159, 9362,  63216, 57487, 57557, 63276, 8962,  25547, 47362,
  8738,  40621, 11817, 53281, 33825, 18874, 8322,  2064,  0,     2032,  8065,  18009, 31775, 49275, 4890,  29612, 57825,
  23918, 58887, 31589, 7489,  52056, 34152, 19248, 7281,  63728, 57456, 53944, 53137, 54979, 59419, 868,   10347, 22273,
  36598, 53274, 6721,  27967, 51433, 11540, 39321, 3663,  35599, 4020,  39960, 12312, 52112, 28255, 6241,  51575, 33153,
  16479, 1524,  53792, 42184, 32206, 23831, 17031, 11781, 8054,  5825,  5069,  5761,  7878,  11397, 16295, 22550, 30139,
  39042, 49238, 60707, 7891,  21845, 37012, 53374, 5377,  24074, 43912, 64874, 21406, 44564, 3260,  28550, 54882, 16705,
  45075, 8907,  39258, 5041,  37314, 4993,  39135, 8655,  44613, 15924, 53648, 26699, 604,   40884, 16458, 58386, 35585,
  13579, 57895, 37449, 17767, 64376, 46192, 28743, 12019, 61546, 46244, 31638, 17720, 4481,  57448, 45541, 34288, 23681,
  13710, 4369,  61185, 53078, 45578, 38676, 32366, 26640, 21491, 16912, 12896, 9437,  6527,  4161,  2331,  1032,  257,
  0,     255,   1016,  2277,  4032,  6277,  9004,  12210, 15887, 20031, 24637, 29699, 35213, 41173, 47574, 54411, 61680,
  3840,  11959, 20494, 29443, 38801, 48562, 58724, 3744,  14693, 26028, 37746, 49844, 62316, 9624,  22834, 36408, 50342,
  64632, 13737, 28728, 44063, 59740, 10219, 26568, 43249, 60257, 12055, 29709, 47682, 434,   19033, 37941, 57155, 11136,
  30953, 51067, 5938,  26637, 47624, 3360,  24916, 46751, 3328,  25716, 48376, 5770,  28967, 52428, 10616, 34599, 58840,
  17799, 42547, 2010,  27256, 52748, 12947, 38924, 65140, 26056, 52743, 14127, 41277, 3120,  30726, 58555, 21072, 49344,
  12300, 41007, 4394,  33530, 62876, 26896, 56659, 21092, 51264, 16103, 46678, 11915, 42886, 8515,  39875, 5890,  37632,
  4027,  36145, 2912,  35400, 2534,  35385, 2880,  36089, 3939,  37500, 5698,  39605, 8147,  42395, 11275, 45857, 15069,
  49982, 19521, 54758, 24619, 60175, 30353, 688,   36713, 7357,  43690, 14639, 51274, 22522, 59455, 30999, 2688,  40059,
  12037, 49693, 21956, 59894, 32437, 5117,  43471, 16425, 55050, 28273, 1630,  40655, 14275, 53561, 27441, 1449,  41120,
  15382, 55305, 29818, 4453,  44748, 19629, 60166, 35288, 10529, 51425, 26902, 2496,  43742, 19567, 61042, 37095, 13261,
  55074, 31463, 7962,  50106, 26824, 3649,  46117, 23157, 302,   43088, 20442, 63436, 40997, 18660, 61961, 39826, 17792,
  61393, 39557, 17819, 61715, 40171, 18724, 62908, 41651, 20489, 64956, 43980, 23096, 2304,  47139, 26529, 6009,  51115,
  30773, 10519, 55890, 35811, 15819, 61448, 41628, 21892, 2240,  48208, 28724, 9322,  55538, 36301, 17144, 63604, 44608,
  25692, 6855,  53632, 34952, 16349, 63360, 44911, 26539, 8242,  55557, 37410, 19338, 1340,  48951, 31099, 13320, 61149,
  43513, 25949, 8456,  56569, 39216, 21932, 4718,  53109, 36031, 19022, 2080,  50741, 33933, 17191, 516,   49441, 32896,
  16416, 0,
};
int g_aiLMDivTableHigh[] = {
  65536, 32768, 21845, 16384, 13107, 10922, 9362, 8192, 7281, 6553, 5957, 5461, 5041, 4681, 4369, 4096, 3855, 3640,
  3449,  3276,  3120,  2978,  2849,  2730,  2621, 2520, 2427, 2340, 2259, 2184, 2114, 2048, 1985, 1927, 1872, 1820,
  1771,  1724,  1680,  1638,  1598,  1560,  1524, 1489, 1456, 1424, 1394, 1365, 1337, 1310, 1285, 1260, 1236, 1213,
  1191,  1170,  1149,  1129,  1110,  1092,  1074, 1057, 1040, 1024, 1008, 992,  978,  963,  949,  936,  923,  910,
  897,   885,   873,   862,   851,   840,   829,  819,  809,  799,  789,  780,  771,  762,  753,  744,  736,  728,
  720,   712,   704,   697,   689,   682,   675,  668,  661,  655,  648,  642,  636,  630,  624,  618,  612,  606,
  601,   595,   590,   585,   579,   574,   569,  564,  560,  555,  550,  546,  541,  537,  532,  528,  524,  520,
  516,   512,   508,   504,   500,   496,   492,  489,  485,  481,  478,  474,  471,  468,  464,  461,  458,  455,
  451,   448,   445,   442,   439,   436,   434,  431,  428,  425,  422,  420,  417,  414,  412,  409,  407,  404,
  402,   399,   397,   394,   392,   390,   387,  385,  383,  381,  378,  376,  374,  372,  370,  368,  366,  364,
  362,   360,   358,   356,   354,   352,   350,  348,  346,  344,  343,  341,  339,  337,  336,  334,  332,  330,
  329,   327,   326,   324,   322,   321,   319,  318,  316,  315,  313,  312,  310,  309,  307,  306,  304,  303,
  302,   300,   299,   297,   296,   295,   293,  292,  291,  289,  288,  287,  286,  284,  283,  282,  281,  280,
  278,   277,   276,   275,   274,   273,   271,  270,  269,  268,  267,  266,  265,  264,  263,  262,  261,  260,
  259,   258,   257,   256,   255,   254,   253,  252,  251,  250,  249,  248,  247,  246,  245,  244,  243,  242,
  241,   240,   240,   239,   238,   237,   236,  235,  234,  234,  233,  232,  231,  230,  229,  229,  228,  227,
  226,   225,   225,   224,   223,   222,   222,  221,  220,  219,  219,  218,  217,  217,  216,  215,  214,  214,
  213,   212,   212,   211,   210,   210,   209,  208,  208,  207,  206,  206,  205,  204,  204,  203,  202,  202,
  201,   201,   200,   199,   199,   198,   197,  197,  196,  196,  195,  195,  194,  193,  193,  192,  192,  191,
  191,   190,   189,   189,   188,   188,   187,  187,  186,  186,  185,  185,  184,  184,  183,  183,  182,  182,
  181,   181,   180,   180,   179,   179,   178,  178,  177,  177,  176,  176,  175,  175,  174,  174,  173,  173,
  172,   172,   172,   171,   171,   170,   170,  169,  169,  168,  168,  168,  167,  167,  166,  166,  165,  165,
  165,   164,   164,   163,   163,   163,   162,  162,  161,  161,  161,  160,  160,  159,  159,  159,  158,  158,
  157,   157,   157,   156,   156,   156,   155,  155,  154,  154,  154,  153,  153,  153,  152,  152,  152,  151,
  151,   151,   150,   150,   149,   149,   149,  148,  148,  148,  147,  147,  147,  146,  146,  146,  145,  145,
  145,   144,   144,   144,   144,   143,   143,  143,  142,  142,  142,  141,  141,  141,  140,  140,  140,  140,
  139,   139,   139,   138,   138,   138,   137,  137,  137,  137,  136,  136,  136,  135,  135,  135,  135,  134,
  134,   134,   134,   133,   133,   133,   132,  132,  132,  132,  131,  131,  131,  131,  130,  130,  130,  130,
  129,   129,   129,   129,   128,   128,   128,  128,
};
#endif
const int8_t g_GbiLog2WeightBase = 3;
const int8_t g_GbiWeightBase = (1 << g_GbiLog2WeightBase);
const int8_t g_GbiWeights[GBI_NUM] = { -2, 3, 4, 5, 10 };
const int8_t g_GbiSearchOrder[GBI_NUM] = { GBI_DEFAULT, GBI_DEFAULT - 2, GBI_DEFAULT + 2, GBI_DEFAULT - 1, GBI_DEFAULT + 1 };
int8_t g_GbiCodingOrder[GBI_NUM];
int8_t g_GbiParsingOrder[GBI_NUM];

int8_t getGbiWeight(uint8_t gbiIdx, uint8_t uhRefFrmList)
{
  // Weghts for the model: P0 + w * (P1 - P0) = (1-w) * P0 + w * P1
  // Retuning  1-w for P0 or w for P1
  return (uhRefFrmList == REF_PIC_LIST_0 ? g_GbiWeightBase - g_GbiWeights[gbiIdx] : g_GbiWeights[gbiIdx]);
}

void resetGbiCodingOrder(bool bRunDecoding, const CodingStructure &cs)
{
  // Form parsing order: { GBI_DEFAULT, GBI_DEFAULT+1, GBI_DEFAULT-1, GBI_DEFAULT+2, GBI_DEFAULT-2, ... }
  g_GbiParsingOrder[0] = GBI_DEFAULT;
  for (int i = 1; i <= (GBI_NUM >> 1); ++i)
  {
    g_GbiParsingOrder[2 * i - 1] = GBI_DEFAULT + (int8_t)i;
    g_GbiParsingOrder[2 * i] = GBI_DEFAULT - (int8_t)i;
  }

  // Form encoding order
  if (!bRunDecoding)
  {
    for (int i = 0; i < GBI_NUM; ++i)
    {
      g_GbiCodingOrder[(uint32_t)g_GbiParsingOrder[i]] = i;
    }
  }
}

uint32_t deriveWeightIdxBits(uint8_t gbiIdx) // Note: align this with TEncSbac::codeGbiIdx and TDecSbac::parseGbiIdx
{
  uint32_t numBits = 1;
  uint8_t  gbiCodingIdx = (uint8_t)g_GbiCodingOrder[gbiIdx];

  if (GBI_NUM > 2 && gbiCodingIdx != 0)
  {
    uint32_t prefixNumBits = GBI_NUM - 2;
    uint32_t step = 1;
    uint8_t  prefixSymbol = gbiCodingIdx;

    // Truncated unary code
    uint8_t idx = 1;
    for (int ui = 0; ui < prefixNumBits; ++ui)
    {
      if (prefixSymbol == idx)
      {
        ++numBits;
        break;
      }
      else
      {
        ++numBits;
        idx += step;
      }
    }
  }
  return numBits;
}

#if JVET_M0102_INTRA_SUBPARTITIONS // define the sbb sizes
uint32_t g_log2SbbSize[2][MAX_CU_DEPTH+1][MAX_CU_DEPTH+1][2] =
{
  //===== luma =====
  {
    { {0,0}, {0,1}, {0,2}, {0,3}, {0,4}, {0,4}, {0,4}, {0,4} },
    { {1,0}, {1,1}, {1,2}, {1,3}, {1,3}, {1,3}, {1,3}, {1,3} },
    { {2,0}, {2,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {3,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {4,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {4,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {4,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {4,0}, {3,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} }
  },
  //===== chroma =====
  {
    { {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0} },
    { {0,0}, {1,1}, {1,1}, {1,1}, {1,1}, {1,1}, {1,1}, {1,1} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} },
    { {0,0}, {1,1}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2}, {2,2} }
  },
};
#endif
// initialize ROM variables
void initROM()
{
  int c;

#if RExt__HIGH_BIT_DEPTH_SUPPORT
  {
    c = 64;
    const double s = sqrt((double)c) * (64 << COM16_C806_TRANS_PREC);


    for (int k = 0; k < c; k++)
    {
      for (int n = 0; n < c; n++)
      {
        double w0, v;
        const double PI = 3.14159265358979323846;

        // DCT-II
        w0 = k == 0 ? sqrt(0.5) : 1;
        v = cos(PI*(n + 0.5)*k / c) * w0 * sqrt(2.0 / c);
        short sv = (short)(s * v + (v > 0 ? 0.5 : -0.5));
        if (g_aiT64[0][0][c*c + k*c + n] != sv)
        {
          msg(VTMWARNING, "trap");
        }
      }
    }
  }
#endif

  // g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...
  // g_aucLog2[ x ]: log2(x), if x=1 -> 0, x=2 -> 1, x=4 -> 2, x=8 -> 3, x=16 -> 4, ...
  ::memset(g_aucLog2, 0, sizeof(g_aucLog2));
  c = 0;
  for( int i = 0, n = 0; i <= MAX_CU_SIZE; i++ )
  {
    g_aucNextLog2[i] = i <= 1 ? 0 : c + 1;

    if( i == ( 1 << n ) )
    {
      c = n;
      n++;
    }

    g_aucPrevLog2[i] = c;
    g_aucLog2    [i] = c;
  }


  gp_sizeIdxInfo = new SizeIndexInfoLog2();
  gp_sizeIdxInfo->init(MAX_CU_SIZE);


  generateTrafoBlockSizeScaling(*gp_sizeIdxInfo);

  SizeIndexInfoLog2 sizeInfo;
  sizeInfo.init(MAX_CU_SIZE);

#if JVET_M0102_INTRA_SUBPARTITIONS
  for( int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++ )
  {
#endif
  // initialize scan orders
  for (uint32_t blockHeightIdx = 0; blockHeightIdx < sizeInfo.numAllHeights(); blockHeightIdx++)
  {
    for (uint32_t blockWidthIdx = 0; blockWidthIdx < sizeInfo.numAllWidths(); blockWidthIdx++)
    {
      const uint32_t blockWidth  = sizeInfo.sizeFrom(blockWidthIdx);
      const uint32_t blockHeight = sizeInfo.sizeFrom(blockHeightIdx);
      const uint32_t totalValues = blockWidth * blockHeight;

      //--------------------------------------------------------------------------------------------------

      //non-grouped scan orders

      for (uint32_t scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);
        ScanElement *       scan     = nullptr;

        if (blockWidthIdx < sizeInfo.numWidths() && blockHeightIdx < sizeInfo.numHeights())
        {
          scan = new ScanElement[totalValues];
        }

#if JVET_M0102_INTRA_SUBPARTITIONS
        g_scanOrder[ch][SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx] = scan;
#else
        g_scanOrder[SCAN_UNGROUPED][scanType][blockWidthIdx][blockHeightIdx] = scan;
#endif

        if (scan == nullptr)
        {
          continue;
        }

        ScanGenerator fullBlockScan(blockWidth, blockHeight, blockWidth, scanType);

        for (uint32_t scanPosition = 0; scanPosition < totalValues; scanPosition++)
        {
          const int rasterPos = fullBlockScan.GetNextIndex( 0, 0 );
          const int posY      = rasterPos / blockWidth;
          const int posX      = rasterPos - ( posY * blockWidth );

          scan[scanPosition].idx = rasterPos;
          scan[scanPosition].x   = posX;
          scan[scanPosition].y   = posY;
        }
      }

      //--------------------------------------------------------------------------------------------------

      //grouped scan orders
#if JVET_M0102_INTRA_SUBPARTITIONS
      const uint32_t* log2Sbb        = g_log2SbbSize[ch][ g_aucLog2[blockWidth] ][ g_aucLog2[blockHeight] ];
      const uint32_t  log2CGWidth    = log2Sbb[0];
      const uint32_t  log2CGHeight   = log2Sbb[1];
#else
      const uint32_t  log2CGWidth    = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;
      const uint32_t  log2CGHeight   = (blockWidth & 3) + (blockHeight & 3) > 0 ? 1 : 2;
#endif

      const uint32_t  groupWidth     = 1 << log2CGWidth;
      const uint32_t  groupHeight    = 1 << log2CGHeight;
#if JVET_M0257
      const uint32_t  widthInGroups = std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, blockWidth) >> log2CGWidth;
      const uint32_t  heightInGroups = std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, blockHeight) >> log2CGHeight;
#else
      const uint32_t  widthInGroups  = blockWidth >> log2CGWidth;
      const uint32_t  heightInGroups = blockHeight >> log2CGHeight;
#endif

      const uint32_t  groupSize      = groupWidth    * groupHeight;
      const uint32_t  totalGroups    = widthInGroups * heightInGroups;

      for (uint32_t scanTypeIndex = 0; scanTypeIndex < SCAN_NUMBER_OF_TYPES; scanTypeIndex++)
      {
        const CoeffScanType scanType = CoeffScanType(scanTypeIndex);

        ScanElement *scan = new ScanElement[totalValues];

#if JVET_M0102_INTRA_SUBPARTITIONS
        g_scanOrder[ch][SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx] = scan;
#else
        g_scanOrder[SCAN_GROUPED_4x4][scanType][blockWidthIdx][blockHeightIdx] = scan;
#endif

#if JVET_M0257
        if ( blockWidth > JVET_C0024_ZERO_OUT_TH || blockHeight > JVET_C0024_ZERO_OUT_TH )
        {
          for (uint32_t i = 0; i < totalValues; i++)
          {
            scan[i].idx = totalValues - 1;
            scan[i].x   = blockWidth - 1;
            scan[i].y   = blockHeight - 1;
          }
        }
#endif

        ScanGenerator fullBlockScan(widthInGroups, heightInGroups, groupWidth, scanType);

        for (uint32_t groupIndex = 0; groupIndex < totalGroups; groupIndex++)
        {
          const uint32_t groupPositionY  = fullBlockScan.GetCurrentY();
          const uint32_t groupPositionX  = fullBlockScan.GetCurrentX();
          const uint32_t groupOffsetX    = groupPositionX * groupWidth;
          const uint32_t groupOffsetY    = groupPositionY * groupHeight;
          const uint32_t groupOffsetScan = groupIndex     * groupSize;

          ScanGenerator groupScan(groupWidth, groupHeight, blockWidth, scanType);

          for (uint32_t scanPosition = 0; scanPosition < groupSize; scanPosition++)
          {
            const int rasterPos = groupScan.GetNextIndex( groupOffsetX, groupOffsetY );
            const int posY      = rasterPos / blockWidth;
            const int posX      = rasterPos - ( posY * blockWidth );

            scan[groupOffsetScan + scanPosition].idx = rasterPos;
            scan[groupOffsetScan + scanPosition].x   = posX;
            scan[groupOffsetScan + scanPosition].y   = posY;
          }

          fullBlockScan.GetNextIndex(0, 0);
        }
      }

      //--------------------------------------------------------------------------------------------------
    }
  }
#if JVET_M0102_INTRA_SUBPARTITIONS
  }
#endif

  for( int idxH = MAX_CU_DEPTH - MIN_CU_LOG2; idxH >= 0; --idxH )
  {
    for( int idxW = MAX_CU_DEPTH - MIN_CU_LOG2; idxW >= 0; --idxW )
    {
      int numW   = 1 << idxW;
      int numH   = 1 << idxH;
      int ratioW = std::max( 0, idxW - idxH );
      int ratioH = std::max( 0, idxH - idxW );
      int sum    = std::max( (numW >> ratioW), (numH >> ratioH) ) - 1;
      for( int y = 0; y < numH; y++ )
      {
        int idxY = y >> ratioH;
        for( int x = 0; x < numW; x++ )
        {
          int idxX = x >> ratioW;
          g_triangleMvStorage[TRIANGLE_DIR_135][idxH][idxW][y][x] = (idxX == idxY) ? 2 : (idxX > idxY ? 0 : 1);
          g_triangleMvStorage[TRIANGLE_DIR_45][idxH][idxW][y][x] = (idxX + idxY == sum) ? 2 : (idxX + idxY > sum ? 1 : 0);
        }
      }
    }
  }
}

void destroyROM()
{
  unsigned numWidths = gp_sizeIdxInfo->numAllWidths();
  unsigned numHeights = gp_sizeIdxInfo->numAllHeights();

#if JVET_M0102_INTRA_SUBPARTITIONS
  for( uint32_t ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++ )
  {
    for( uint32_t groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++ )
    {
      for( uint32_t scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++ )
      {
        for( uint32_t blockWidthIdx = 0; blockWidthIdx <= numWidths; blockWidthIdx++ )
        {
          for( uint32_t blockHeightIdx = 0; blockHeightIdx <= numHeights; blockHeightIdx++ )
          {
            delete[] g_scanOrder[ch][groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx];
            g_scanOrder[ch][groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx] = nullptr;
          }
        }
      }
    }
  }
#else
  for (uint32_t groupTypeIndex = 0; groupTypeIndex < SCAN_NUMBER_OF_GROUP_TYPES; groupTypeIndex++)
  {
    for (uint32_t scanOrderIndex = 0; scanOrderIndex < SCAN_NUMBER_OF_TYPES; scanOrderIndex++)
    {
      for (uint32_t blockWidthIdx = 0; blockWidthIdx <= numWidths; blockWidthIdx++)
      {
        for (uint32_t blockHeightIdx = 0; blockHeightIdx <= numHeights; blockHeightIdx++)
        {
          delete[] g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx];
          g_scanOrder[groupTypeIndex][scanOrderIndex][blockWidthIdx][blockHeightIdx] = nullptr;
        }
      }
    }
  }
#endif

  delete gp_sizeIdxInfo;
  gp_sizeIdxInfo = nullptr;
}



void generateTrafoBlockSizeScaling(SizeIndexInfo& sizeIdxInfo)
{
  for (SizeType y = 0; y < sizeIdxInfo.numHeights(); y++)
  {
    for (SizeType x = 0; x < sizeIdxInfo.numWidths(); x++)
    {
      SizeType h = sizeIdxInfo.sizeFrom(y);
      SizeType w = sizeIdxInfo.sizeFrom(x);
      double factor = sqrt(h) * sqrt(w) / (double)(1 << ((g_aucLog2[h] + g_aucLog2[w]) / 2));

      g_BlockSizeTrafoScale[h][w][0] = ((int)(factor + 0.9) != 1) ? (int)(factor * (double)(1 << ADJ_QUANT_SHIFT)) : 1;
      g_BlockSizeTrafoScale[h][w][1] = ((int)(factor + 0.9) != 1) ? (int)((double)(1 << ADJ_DEQUANT_SHIFT) / factor + 0.5) : 1;
    }
  }
}

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================

const int g_quantScales[SCALING_LIST_REM_NUM] =
{
  26214,23302,20560,18396,16384,14564
};

const int g_invQuantScales[SCALING_LIST_REM_NUM] =
{
  40,45,51,57,64,72
};

//--------------------------------------------------------------------------------------------------
//structures
//--------------------------------------------------------------------------------------------------
//coefficients
//--------------------------------------------------------------------------------------------------

const uint8_t g_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize] =
{
  //0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,29,30,31,32,33,33,34,34,35,35,36,36,37,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,63,63,63,63,63,63 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,63,63,63,63,63,63 }
};

// ====================================================================================================================
// Intra prediction
// ====================================================================================================================

const uint8_t g_aucIntraModeNumFast_UseMPM_2D[7 - MIN_CU_LOG2 + 1][7 - MIN_CU_LOG2 + 1] =
{
  {3, 3, 3, 3, 2, 2},  //   4x4,   4x8,   4x16,   4x32,   4x64,   4x128,
  {3, 3, 3, 3, 3, 2},  //   8x4,   8x8,   8x16,   8x32,   8x64,   8x128,
  {3, 3, 3, 3, 3, 2},  //  16x4,  16x8,  16x16,  16x32,  16x64,  16x128,
  {3, 3, 3, 3, 3, 2},  //  32x4,  32x8,  32x16,  32x32,  32x64,  32x128,
  {2, 3, 3, 3, 3, 2},  //  64x4,  64x8,  64x16,  64x32,  64x64,  64x128,
  {2, 2, 2, 2, 2, 3},  // 128x4, 128x8, 128x16, 128x32, 128x64, 128x128,
};

const uint8_t g_aucIntraModeNumFast_UseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  8,  //   4x4
  8,  //   8x8
  3,  //  16x16
  3,  //  32x32
  3,  //  64x64
  3   // 128x128
};
const uint8_t g_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH] =
{
  3,  //   2x2
  9,  //   4x4
  9,  //   8x8
  4,  //  16x16   33
  4,  //  32x32   33
  5,  //  64x64   33
  5   // 128x128
};

const uint8_t g_chroma422IntraAngleMappingTable[NUM_INTRA_MODE] =
//                                                               H                                                               D                                                               V
//0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, DM
{ 0, 1, 2, 2, 2, 2, 2, 2, 2, 3,  4,  6,  8, 10, 12, 13, 14, 16, 18, 20, 22, 23, 24, 26, 28, 30, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 44, 44, 45, 46, 46, 46, 47, 48, 48, 48, 49, 50, 51, 52, 52, 52, 53, 54, 54, 54, 55, 56, 56, 56, 57, 58, 59, 60, DM_CHROMA_IDX };


#if !REMOVE_BIN_DECISION_TREE
// ====================================================================================================================
// Decision tree templates
// ====================================================================================================================

const DecisionTreeTemplate g_mtSplitDTT = compile(
  decision( DTT_SPLIT_DO_SPLIT_DECISION,
  /*0*/ DTT_SPLIT_NO_SPLIT,
  /*1*/ decision( DTT_SPLIT_HV_DECISION,
        /*0*/ decision( DTT_SPLIT_H_IS_BT_12_DECISION,
              /*0*/ DTT_SPLIT_TT_HORZ,
              /*1*/ DTT_SPLIT_BT_HORZ ),
        /*1*/ decision( DTT_SPLIT_V_IS_BT_12_DECISION,
              /*0*/ DTT_SPLIT_TT_VERT,
              /*1*/ DTT_SPLIT_BT_VERT ) ) ) );

#endif


// ====================================================================================================================
// Misc.
// ====================================================================================================================
SizeIndexInfo*           gp_sizeIdxInfo = NULL;
int                      g_BlockSizeTrafoScale[MAX_CU_SIZE + 1][MAX_CU_SIZE + 1][2];
int8_t                    g_aucLog2    [MAX_CU_SIZE + 1];
int8_t                    g_aucNextLog2[MAX_CU_SIZE + 1];
int8_t                    g_aucPrevLog2[MAX_CU_SIZE + 1];

UnitScale g_miScaling( MIN_CU_LOG2, MIN_CU_LOG2 );


// ====================================================================================================================
// Scanning order & context model mapping
// ====================================================================================================================

// scanning order table
#if JVET_M0102_INTRA_SUBPARTITIONS
ScanElement *g_scanOrder[2][SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
#else
ScanElement *g_scanOrder[SCAN_NUMBER_OF_GROUP_TYPES][SCAN_NUMBER_OF_TYPES][MAX_CU_SIZE / 2 + 1][MAX_CU_SIZE / 2 + 1];
#endif

const uint32_t g_uiMinInGroup[LAST_SIGNIFICANT_GROUPS] = { 0,1,2,3,4,6,8,12,16,24,32,48,64,96 };
const uint32_t g_uiGroupIdx[MAX_TU_SIZE] = { 0,1,2,3,4,4,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9, 10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11
,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12
,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13 };
const uint32_t g_auiGoRiceParsCoeff[32] =
{
  0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3
};
const uint32_t g_auiGoRicePosCoeff0[3][32] =
{
  {0, 0, 0, 0, 0, 1, 2,    2, 2, 2, 2, 2, 4, 4,    4, 4, 4, 4,  4,  4,  4,  4,  4,  8,  8,  8,  8,  8,     8,  8,  8,  8},
  {1, 1, 1, 1, 2, 3, 4,    4, 4, 6, 6, 6, 8, 8,    8, 8, 8, 8, 12, 12, 12, 12, 12, 12, 12, 12, 16, 16,    16, 16, 16, 16},
  {1, 1, 2, 2, 2, 3, 4,    4, 4, 6, 6, 6, 8, 8,    8, 8, 8, 8, 12, 12, 12, 12, 12, 12, 12, 16, 16, 16,    16, 16, 16, 16}
};
#if !JVET_M0470
const uint32_t g_auiGoRiceRange[MAX_GR_ORDER_RESIDUAL] =
{
  6, 5, 6, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION, COEF_REMAIN_BIN_REDUCTION
};
#endif

#if HEVC_USE_SCALING_LISTS
const char *MatrixType[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
    "INTRA2X2_LUMA",
    "INTRA2X2_CHROMAU",
    "INTRA2X2_CHROMAV",
    "INTER2X2_LUMA",
    "INTER2X2_CHROMAU",
    "INTER2X2_CHROMAV"
  },
  {
    "INTRA4X4_LUMA",
    "INTRA4X4_CHROMAU",
    "INTRA4X4_CHROMAV",
    "INTER4X4_LUMA",
    "INTER4X4_CHROMAU",
    "INTER4X4_CHROMAV"
  },
  {
    "INTRA8X8_LUMA",
    "INTRA8X8_CHROMAU",
    "INTRA8X8_CHROMAV",
    "INTER8X8_LUMA",
    "INTER8X8_CHROMAU",
    "INTER8X8_CHROMAV"
  },
  {
    "INTRA16X16_LUMA",
    "INTRA16X16_CHROMAU",
    "INTRA16X16_CHROMAV",
    "INTER16X16_LUMA",
    "INTER16X16_CHROMAU",
    "INTER16X16_CHROMAV"
  },
  {
   "INTRA32X32_LUMA",
   "INTRA32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTRA32X32_CHROMAV_FROM16x16_CHROMAV",
   "INTER32X32_LUMA",
   "INTER32X32_CHROMAU_FROM16x16_CHROMAU",
   "INTER32X32_CHROMAV_FROM16x16_CHROMAV"
  },
};

const char *MatrixType_DC[SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM] =
{
  {
  },
  {
  },
  {
  },
  {
    "INTRA16X16_LUMA_DC",
    "INTRA16X16_CHROMAU_DC",
    "INTRA16X16_CHROMAV_DC",
    "INTER16X16_LUMA_DC",
    "INTER16X16_CHROMAU_DC",
    "INTER16X16_CHROMAV_DC"
  },
  {
    "INTRA32X32_LUMA_DC",
    "INTRA32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTRA32X32_CHROMAV_DC_FROM16x16_CHROMAV",
    "INTER32X32_LUMA_DC",
    "INTER32X32_CHROMAU_DC_FROM16x16_CHROMAU",
    "INTER32X32_CHROMAV_DC_FROM16x16_CHROMAV"
  },
};

const int g_quantTSDefault4x4[4 * 4] =
{
  16,16,16,16,
  16,16,16,16,
  16,16,16,16,
  16,16,16,16
};

const int g_quantIntraDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,21,24,
  16,16,16,16,17,19,22,25,
  16,16,17,18,20,22,25,29,
  16,16,18,21,24,27,31,36,
  17,17,20,24,30,35,41,47,
  18,19,22,27,35,44,54,65,
  21,22,25,31,41,54,70,88,
  24,25,29,36,47,65,88,115
};

const int g_quantInterDefault8x8[8 * 8] =
{
  16,16,16,16,17,18,20,24,
  16,16,16,17,18,20,24,25,
  16,16,17,18,20,24,25,28,
  16,17,18,20,24,25,28,33,
  17,18,20,24,25,28,33,41,
  18,20,24,25,28,33,41,54,
  20,24,25,28,33,41,54,71,
  24,25,28,33,41,54,71,91
};

const uint32_t g_scalingListSize [SCALING_LIST_SIZE_NUM] = { 4, 16, 64, 256, 1024, 4096, 16384 };
const uint32_t g_scalingListSizeX[SCALING_LIST_SIZE_NUM] = { 2,  4,  8,  16,   32,   64,   128 };
#endif

#if !JVET_M0328_KEEP_ONE_WEIGHT_GROUP
const Pel g_trianglePelWeightedLuma[TRIANGLE_DIR_NUM][2][7] =
{
  { // TRIANGLE_DIR_135
    { 1, 2, 4, 6, 7, 0, 0 },
    { 1, 2, 3, 4, 5, 6, 7 }
  },
  { // TRIANGLE_DIR_45
    { 7, 6, 4, 2, 1, 0, 0 },
    { 7, 6, 5, 4, 3, 2, 1 }
  }
};
const Pel g_trianglePelWeightedChroma[2][TRIANGLE_DIR_NUM][2][7] =
{
  { // 444 format
    { // TRIANGLE_DIR_135
      { 1, 2, 4, 6, 7, 0, 0 },
      { 1, 2, 3, 4, 5, 6, 7 }
    },
    { // TRIANGLE_DIR_45
      { 7, 6, 4, 2, 1, 0, 0 },
      { 7, 6, 5, 4, 3, 2, 1 }
    }
  },
  { // 420 format
    { // TRIANGLE_DIR_135
      { 1, 4, 7, 0, 0, 0, 0 },
      { 2, 4, 6, 0, 0, 0, 0 }
    },
    { // TRIANGLE_DIR_45
      { 7, 4, 1, 0, 0, 0, 0 },
      { 6, 4, 2, 0, 0, 0, 0 }
    }
  }
};

const uint8_t g_triangleWeightLengthLuma[2] = { 5, 7 };
const uint8_t g_triangleWeightLengthChroma[2][2] = { { 5, 7 }, { 3, 3 } };
#endif

uint8_t g_triangleMvStorage[TRIANGLE_DIR_NUM][MAX_CU_DEPTH - MIN_CU_LOG2 + 1][MAX_CU_DEPTH - MIN_CU_LOG2 + 1][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];

#if !JVET_M0883_TRIANGLE_SIGNALING
const uint8_t g_triangleCombination[TRIANGLE_MAX_NUM_CANDS][3] =
{
  { 0, 1, 0 }, { 1, 0, 1 }, { 1, 0, 2 }, { 0, 0, 1 }, { 0, 2, 0 },
  { 1, 0, 3 }, { 1, 0, 4 }, { 1, 1, 0 }, { 0, 3, 0 }, { 0, 4, 0 },
  { 0, 0, 2 }, { 0, 1, 2 }, { 1, 1, 2 }, { 0, 0, 4 }, { 0, 0, 3 },
  { 0, 1, 3 }, { 0, 1, 4 }, { 1, 1, 4 }, { 1, 1, 3 }, { 1, 2, 1 },
  { 1, 2, 0 }, { 0, 2, 1 }, { 0, 4, 3 }, { 1, 3, 0 }, { 1, 3, 2 },
  { 1, 3, 4 }, { 1, 4, 0 }, { 1, 3, 1 }, { 1, 2, 3 }, { 1, 4, 1 },
  { 0, 4, 1 }, { 0, 2, 3 }, { 1, 4, 2 }, { 0, 3, 2 }, { 1, 4, 3 },
  { 0, 3, 1 }, { 0, 2, 4 }, { 1, 2, 4 }, { 0, 4, 2 }, { 0, 3, 4 },
};

const uint8_t g_triangleIdxBins[TRIANGLE_MAX_NUM_CANDS] =
{
   2,  2,  4,  4,  4,  4,  6,  6,  6,  6,
   6,  6,  6,  6,  8,  8,  8,  8,  8,  8,
   8,  8,  8,  8,  8,  8,  8,  8,  8,  8,
  10, 10, 10, 10, 10, 10, 10, 10, 10, 10
};
#endif
//! \}
