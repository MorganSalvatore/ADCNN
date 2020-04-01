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

/** \file     Contexts.cpp
 *  \brief    Classes providing probability descriptions and contexts (also contains context initialization values)
 */

#include "Contexts.h"

#include <algorithm>
#include <cstring>
#include <limits>

#if !JVET_M0453_CABAC_ENGINE
const uint8_t ProbModelTables::m_NextState[128][2] =
{
  {   2,  1 },{   0,  3 },{   4,  0 },{   1,  5 },{   6,  2 },{   3,  7 },{   8,  4 },{   5,  9 },
  {  10,  4 },{   5, 11 },{  12,  8 },{   9, 13 },{  14,  8 },{   9, 15 },{  16, 10 },{  11, 17 },
  {  18, 12 },{  13, 19 },{  20, 14 },{  15, 21 },{  22, 16 },{  17, 23 },{  24, 18 },{  19, 25 },
  {  26, 18 },{  19, 27 },{  28, 22 },{  23, 29 },{  30, 22 },{  23, 31 },{  32, 24 },{  25, 33 },
  {  34, 26 },{  27, 35 },{  36, 26 },{  27, 37 },{  38, 30 },{  31, 39 },{  40, 30 },{  31, 41 },
  {  42, 32 },{  33, 43 },{  44, 32 },{  33, 45 },{  46, 36 },{  37, 47 },{  48, 36 },{  37, 49 },
  {  50, 38 },{  39, 51 },{  52, 38 },{  39, 53 },{  54, 42 },{  43, 55 },{  56, 42 },{  43, 57 },
  {  58, 44 },{  45, 59 },{  60, 44 },{  45, 61 },{  62, 46 },{  47, 63 },{  64, 48 },{  49, 65 },
  {  66, 48 },{  49, 67 },{  68, 50 },{  51, 69 },{  70, 52 },{  53, 71 },{  72, 52 },{  53, 73 },
  {  74, 54 },{  55, 75 },{  76, 54 },{  55, 77 },{  78, 56 },{  57, 79 },{  80, 58 },{  59, 81 },
  {  82, 58 },{  59, 83 },{  84, 60 },{  61, 85 },{  86, 60 },{  61, 87 },{  88, 60 },{  61, 89 },
  {  90, 62 },{  63, 91 },{  92, 64 },{  65, 93 },{  94, 64 },{  65, 95 },{  96, 66 },{  67, 97 },
  {  98, 66 },{  67, 99 },{ 100, 66 },{  67,101 },{ 102, 68 },{  69,103 },{ 104, 68 },{  69,105 },
  { 106, 70 },{  71,107 },{ 108, 70 },{  71,109 },{ 110, 70 },{  71,111 },{ 112, 72 },{  73,113 },
  { 114, 72 },{  73,115 },{ 116, 72 },{  73,117 },{ 118, 74 },{  75,119 },{ 120, 74 },{  75,121 },
  { 122, 74 },{  75,123 },{ 124, 76 },{  77,125 },{ 124, 76 },{  77,125 },{ 126,126 },{ 127,127 }
};

const uint32_t ProbModelTables::m_EstFracBits[128] =
{
  0x07b23, 0x085f9, 0x074a0, 0x08cbc, 0x06ee4, 0x09354, 0x067f4, 0x09c1b, 0x060b0, 0x0a62a, 0x05a9c, 0x0af5b, 0x0548d, 0x0b955, 0x04f56, 0x0c2a9,
  0x04a87, 0x0cbf7, 0x045d6, 0x0d5c3, 0x04144, 0x0e01b, 0x03d88, 0x0e937, 0x039e0, 0x0f2cd, 0x03663, 0x0fc9e, 0x03347, 0x10600, 0x03050, 0x10f95,
  0x02d4d, 0x11a02, 0x02ad3, 0x12333, 0x0286e, 0x12cad, 0x02604, 0x136df, 0x02425, 0x13f48, 0x021f4, 0x149c4, 0x0203e, 0x1527b, 0x01e4d, 0x15d00,
  0x01c99, 0x166de, 0x01b18, 0x17017, 0x019a5, 0x17988, 0x01841, 0x18327, 0x016df, 0x18d50, 0x015d9, 0x19547, 0x0147c, 0x1a083, 0x0138e, 0x1a8a3,
  0x01251, 0x1b418, 0x01166, 0x1bd27, 0x01068, 0x1c77b, 0x00f7f, 0x1d18e, 0x00eda, 0x1d91a, 0x00e19, 0x1e254, 0x00d4f, 0x1ec9a, 0x00c90, 0x1f6e0,
  0x00c01, 0x1fef8, 0x00b5f, 0x208b1, 0x00ab6, 0x21362, 0x00a15, 0x21e46, 0x00988, 0x2285d, 0x00934, 0x22ea8, 0x008a8, 0x239b2, 0x0081d, 0x24577,
  0x007c9, 0x24ce6, 0x00763, 0x25663, 0x00710, 0x25e8f, 0x006a0, 0x26a26, 0x00672, 0x26f23, 0x005e8, 0x27ef8, 0x005ba, 0x284b5, 0x0055e, 0x29057,
  0x0050c, 0x29bab, 0x004c1, 0x2a674, 0x004a7, 0x2aa5e, 0x0046f, 0x2b32f, 0x0041f, 0x2c0ad, 0x003e7, 0x2ca8d, 0x003ba, 0x2d323, 0x0010c, 0x3bfbb
};

const BinFracBits ProbModelTables::m_BinFracBits_128[128] =
{
  {{0x07b23, 0x085f9}}, {{0x085f9, 0x07b23}},   {{0x074a0, 0x08cbc}}, {{0x08cbc, 0x074a0}},   {{0x06ee4, 0x09354}}, {{0x09354, 0x06ee4}},   {{0x067f4, 0x09c1b}}, {{0x09c1b, 0x067f4}},
  {{0x060b0, 0x0a62a}}, {{0x0a62a, 0x060b0}},   {{0x05a9c, 0x0af5b}}, {{0x0af5b, 0x05a9c}},   {{0x0548d, 0x0b955}}, {{0x0b955, 0x0548d}},   {{0x04f56, 0x0c2a9}}, {{0x0c2a9, 0x04f56}},
  {{0x04a87, 0x0cbf7}}, {{0x0cbf7, 0x04a87}},   {{0x045d6, 0x0d5c3}}, {{0x0d5c3, 0x045d6}},   {{0x04144, 0x0e01b}}, {{0x0e01b, 0x04144}},   {{0x03d88, 0x0e937}}, {{0x0e937, 0x03d88}},
  {{0x039e0, 0x0f2cd}}, {{0x0f2cd, 0x039e0}},   {{0x03663, 0x0fc9e}}, {{0x0fc9e, 0x03663}},   {{0x03347, 0x10600}}, {{0x10600, 0x03347}},   {{0x03050, 0x10f95}}, {{0x10f95, 0x03050}},
  {{0x02d4d, 0x11a02}}, {{0x11a02, 0x02d4d}},   {{0x02ad3, 0x12333}}, {{0x12333, 0x02ad3}},   {{0x0286e, 0x12cad}}, {{0x12cad, 0x0286e}},   {{0x02604, 0x136df}}, {{0x136df, 0x02604}},
  {{0x02425, 0x13f48}}, {{0x13f48, 0x02425}},   {{0x021f4, 0x149c4}}, {{0x149c4, 0x021f4}},   {{0x0203e, 0x1527b}}, {{0x1527b, 0x0203e}},   {{0x01e4d, 0x15d00}}, {{0x15d00, 0x01e4d}},
  {{0x01c99, 0x166de}}, {{0x166de, 0x01c99}},   {{0x01b18, 0x17017}}, {{0x17017, 0x01b18}},   {{0x019a5, 0x17988}}, {{0x17988, 0x019a5}},   {{0x01841, 0x18327}}, {{0x18327, 0x01841}},
  {{0x016df, 0x18d50}}, {{0x18d50, 0x016df}},   {{0x015d9, 0x19547}}, {{0x19547, 0x015d9}},   {{0x0147c, 0x1a083}}, {{0x1a083, 0x0147c}},   {{0x0138e, 0x1a8a3}}, {{0x1a8a3, 0x0138e}},
  {{0x01251, 0x1b418}}, {{0x1b418, 0x01251}},   {{0x01166, 0x1bd27}}, {{0x1bd27, 0x01166}},   {{0x01068, 0x1c77b}}, {{0x1c77b, 0x01068}},   {{0x00f7f, 0x1d18e}}, {{0x1d18e, 0x00f7f}},
  {{0x00eda, 0x1d91a}}, {{0x1d91a, 0x00eda}},   {{0x00e19, 0x1e254}}, {{0x1e254, 0x00e19}},   {{0x00d4f, 0x1ec9a}}, {{0x1ec9a, 0x00d4f}},   {{0x00c90, 0x1f6e0}}, {{0x1f6e0, 0x00c90}},
  {{0x00c01, 0x1fef8}}, {{0x1fef8, 0x00c01}},   {{0x00b5f, 0x208b1}}, {{0x208b1, 0x00b5f}},   {{0x00ab6, 0x21362}}, {{0x21362, 0x00ab6}},   {{0x00a15, 0x21e46}}, {{0x21e46, 0x00a15}},
  {{0x00988, 0x2285d}}, {{0x2285d, 0x00988}},   {{0x00934, 0x22ea8}}, {{0x22ea8, 0x00934}},   {{0x008a8, 0x239b2}}, {{0x239b2, 0x008a8}},   {{0x0081d, 0x24577}}, {{0x24577, 0x0081d}},
  {{0x007c9, 0x24ce6}}, {{0x24ce6, 0x007c9}},   {{0x00763, 0x25663}}, {{0x25663, 0x00763}},   {{0x00710, 0x25e8f}}, {{0x25e8f, 0x00710}},   {{0x006a0, 0x26a26}}, {{0x26a26, 0x006a0}},
  {{0x00672, 0x26f23}}, {{0x26f23, 0x00672}},   {{0x005e8, 0x27ef8}}, {{0x27ef8, 0x005e8}},   {{0x005ba, 0x284b5}}, {{0x284b5, 0x005ba}},   {{0x0055e, 0x29057}}, {{0x29057, 0x0055e}},
  {{0x0050c, 0x29bab}}, {{0x29bab, 0x0050c}},   {{0x004c1, 0x2a674}}, {{0x2a674, 0x004c1}},   {{0x004a7, 0x2aa5e}}, {{0x2aa5e, 0x004a7}},   {{0x0046f, 0x2b32f}}, {{0x2b32f, 0x0046f}},
  {{0x0041f, 0x2c0ad}}, {{0x2c0ad, 0x0041f}},   {{0x003e7, 0x2ca8d}}, {{0x2ca8d, 0x003e7}},   {{0x003ba, 0x2d323}}, {{0x2d323, 0x003ba}},   {{0x0010c, 0x3bfbb}}, {{0x3bfbb, 0x0010c}}
};

const uint32_t ProbModelTables::m_EstFracProb[128] =
{
  0x041b5, 0x03df6, 0x04410, 0x03bbc, 0x04636, 0x039a3, 0x048e6, 0x036f6, 0x04bd3, 0x0340c, 0x04e5d, 0x03185, 0x050fa, 0x02eeb, 0x0534b, 0x02c9b,
  0x0557e, 0x02a6a, 0x057b1, 0x02839, 0x059e4, 0x02608, 0x05bba, 0x02433, 0x05d8f, 0x0225e, 0x05f58, 0x02097, 0x060f7, 0x01efa, 0x06288, 0x01d69,
  0x06427, 0x01bcb, 0x06581, 0x01a72, 0x066d4, 0x0191f, 0x0682f, 0x017c6, 0x0693e, 0x016b7, 0x06a80, 0x01576, 0x06b7e, 0x01478, 0x06ca1, 0x01356,
  0x06da2, 0x01255, 0x06e88, 0x01170, 0x06f67, 0x01092, 0x0703e, 0x00fba, 0x07116, 0x00ee3, 0x071b7, 0x00e42, 0x0728f, 0x00d6a, 0x07323, 0x00cd6,
  0x073e9, 0x00c11, 0x0747d, 0x00b7d, 0x0751e, 0x00add, 0x075b2, 0x00a49, 0x0761b, 0x009e0, 0x07697, 0x00964, 0x07719, 0x008e2, 0x07794, 0x00867,
  0x077f1, 0x0080b, 0x0785b, 0x007a1, 0x078c9, 0x00733, 0x07932, 0x006ca, 0x0798f, 0x0066d, 0x079c6, 0x00636, 0x07a23, 0x005da, 0x07a7f, 0x0057d,
  0x07ab7, 0x00546, 0x07afb, 0x00502, 0x07b32, 0x004cb, 0x07b7d, 0x00480, 0x07b9c, 0x00461, 0x07bf8, 0x00405, 0x07c17, 0x003e6, 0x07c55, 0x003a9,
  0x07c8c, 0x00371, 0x07cbf, 0x0033f, 0x07cd0, 0x0032e, 0x07cf6, 0x00308, 0x07d2c, 0x002d1, 0x07d52, 0x002ab, 0x07d71, 0x0028c, 0x07f46, 0x000b5
};

const uint8_t ProbModelTables::m_LPSTable_64_4[64][4] =
{
  { 128, 176, 208, 240 },
  { 128, 167, 197, 227 },
  { 128, 158, 187, 216 },
  { 123, 150, 178, 205 },
  { 116, 142, 169, 195 },
  { 111, 135, 160, 185 },
  { 105, 128, 152, 175 },
  { 100, 122, 144, 166 },
  {  95, 116, 137, 158 },
  {  90, 110, 130, 150 },
  {  85, 104, 123, 142 },
  {  81,  99, 117, 135 },
  {  77,  94, 111, 128 },
  {  73,  89, 105, 122 },
  {  69,  85, 100, 116 },
  {  66,  80,  95, 110 },
  {  62,  76,  90, 104 },
  {  59,  72,  86,  99 },
  {  56,  69,  81,  94 },
  {  53,  65,  77,  89 },
  {  51,  62,  73,  85 },
  {  48,  59,  69,  80 },
  {  46,  56,  66,  76 },
  {  43,  53,  63,  72 },
  {  41,  50,  59,  69 },
  {  39,  48,  56,  65 },
  {  37,  45,  54,  62 },
  {  35,  43,  51,  59 },
  {  33,  41,  48,  56 },
  {  32,  39,  46,  53 },
  {  30,  37,  43,  50 },
  {  29,  35,  41,  48 },
  {  27,  33,  39,  45 },
  {  26,  31,  37,  43 },
  {  24,  30,  35,  41 },
  {  23,  28,  33,  39 },
  {  22,  27,  32,  37 },
  {  21,  26,  30,  35 },
  {  20,  24,  29,  33 },
  {  19,  23,  27,  31 },
  {  18,  22,  26,  30 },
  {  17,  21,  25,  28 },
  {  16,  20,  23,  27 },
  {  15,  19,  22,  25 },
  {  14,  18,  21,  24 },
  {  14,  17,  20,  23 },
  {  13,  16,  19,  22 },
  {  12,  15,  18,  21 },
  {  12,  14,  17,  20 },
  {  11,  14,  16,  19 },
  {  11,  13,  15,  18 },
  {  10,  12,  15,  17 },
  {  10,  12,  14,  16 },
  {   9,  11,  13,  15 },
  {   9,  11,  12,  14 },
  {   8,  10,  12,  14 },
  {   8,   9,  11,  13 },
  {   7,   9,  11,  12 },
  {   7,   9,  10,  12 },
  {   7,   8,  10,  11 },
  {   6,   8,   9,  11 },
  {   6,   7,   9,  10 },
  {   6,   7,   8,   9 },
  {   2,   2,   2,   2 }
};
#endif

const uint8_t ProbModelTables::m_RenormTable_32[32] =
{
  6,  5,  4,  4,
  3,  3,  3,  3,
  2,  2,  2,  2,
  2,  2,  2,  2,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1,
  1,  1,  1,  1
};

#if JVET_M0453_CABAC_ENGINE
const BinFracBits ProbModelTables::m_binFracBits[256] = {
  { { 0x0005c, 0x48000 } }, { { 0x00116, 0x3b520 } }, { { 0x001d0, 0x356cb } }, { { 0x0028b, 0x318a9 } },
  { { 0x00346, 0x2ea40 } }, { { 0x00403, 0x2c531 } }, { { 0x004c0, 0x2a658 } }, { { 0x0057e, 0x28beb } },
  { { 0x0063c, 0x274ce } }, { { 0x006fc, 0x26044 } }, { { 0x007bc, 0x24dc9 } }, { { 0x0087d, 0x23cfc } },
  { { 0x0093f, 0x22d96 } }, { { 0x00a01, 0x21f60 } }, { { 0x00ac4, 0x2122e } }, { { 0x00b89, 0x205dd } },
  { { 0x00c4e, 0x1fa51 } }, { { 0x00d13, 0x1ef74 } }, { { 0x00dda, 0x1e531 } }, { { 0x00ea2, 0x1db78 } },
  { { 0x00f6a, 0x1d23c } }, { { 0x01033, 0x1c970 } }, { { 0x010fd, 0x1c10b } }, { { 0x011c8, 0x1b903 } },
  { { 0x01294, 0x1b151 } }, { { 0x01360, 0x1a9ee } }, { { 0x0142e, 0x1a2d4 } }, { { 0x014fc, 0x19bfc } },
  { { 0x015cc, 0x19564 } }, { { 0x0169c, 0x18f06 } }, { { 0x0176d, 0x188de } }, { { 0x0183f, 0x182e8 } },
  { { 0x01912, 0x17d23 } }, { { 0x019e6, 0x1778a } }, { { 0x01abb, 0x1721c } }, { { 0x01b91, 0x16cd5 } },
  { { 0x01c68, 0x167b4 } }, { { 0x01d40, 0x162b6 } }, { { 0x01e19, 0x15dda } }, { { 0x01ef3, 0x1591e } },
  { { 0x01fcd, 0x15480 } }, { { 0x020a9, 0x14fff } }, { { 0x02186, 0x14b99 } }, { { 0x02264, 0x1474e } },
  { { 0x02343, 0x1431b } }, { { 0x02423, 0x13f01 } }, { { 0x02504, 0x13afd } }, { { 0x025e6, 0x1370f } },
  { { 0x026ca, 0x13336 } }, { { 0x027ae, 0x12f71 } }, { { 0x02894, 0x12bc0 } }, { { 0x0297a, 0x12821 } },
  { { 0x02a62, 0x12494 } }, { { 0x02b4b, 0x12118 } }, { { 0x02c35, 0x11dac } }, { { 0x02d20, 0x11a51 } },
  { { 0x02e0c, 0x11704 } }, { { 0x02efa, 0x113c7 } }, { { 0x02fe9, 0x11098 } }, { { 0x030d9, 0x10d77 } },
  { { 0x031ca, 0x10a63 } }, { { 0x032bc, 0x1075c } }, { { 0x033b0, 0x10461 } }, { { 0x034a5, 0x10173 } },
  { { 0x0359b, 0x0fe90 } }, { { 0x03693, 0x0fbb9 } }, { { 0x0378c, 0x0f8ed } }, { { 0x03886, 0x0f62b } },
  { { 0x03981, 0x0f374 } }, { { 0x03a7e, 0x0f0c7 } }, { { 0x03b7c, 0x0ee23 } }, { { 0x03c7c, 0x0eb89 } },
  { { 0x03d7d, 0x0e8f9 } }, { { 0x03e7f, 0x0e671 } }, { { 0x03f83, 0x0e3f2 } }, { { 0x04088, 0x0e17c } },
  { { 0x0418e, 0x0df0e } }, { { 0x04297, 0x0dca8 } }, { { 0x043a0, 0x0da4a } }, { { 0x044ab, 0x0d7f3 } },
  { { 0x045b8, 0x0d5a5 } }, { { 0x046c6, 0x0d35d } }, { { 0x047d6, 0x0d11c } }, { { 0x048e7, 0x0cee3 } },
  { { 0x049fa, 0x0ccb0 } }, { { 0x04b0e, 0x0ca84 } }, { { 0x04c24, 0x0c85e } }, { { 0x04d3c, 0x0c63f } },
  { { 0x04e55, 0x0c426 } }, { { 0x04f71, 0x0c212 } }, { { 0x0508d, 0x0c005 } }, { { 0x051ac, 0x0bdfe } },
  { { 0x052cc, 0x0bbfc } }, { { 0x053ee, 0x0b9ff } }, { { 0x05512, 0x0b808 } }, { { 0x05638, 0x0b617 } },
  { { 0x0575f, 0x0b42a } }, { { 0x05888, 0x0b243 } }, { { 0x059b4, 0x0b061 } }, { { 0x05ae1, 0x0ae83 } },
  { { 0x05c10, 0x0acaa } }, { { 0x05d41, 0x0aad6 } }, { { 0x05e74, 0x0a907 } }, { { 0x05fa9, 0x0a73c } },
  { { 0x060e0, 0x0a575 } }, { { 0x06219, 0x0a3b3 } }, { { 0x06354, 0x0a1f5 } }, { { 0x06491, 0x0a03b } },
  { { 0x065d1, 0x09e85 } }, { { 0x06712, 0x09cd4 } }, { { 0x06856, 0x09b26 } }, { { 0x0699c, 0x0997c } },
  { { 0x06ae4, 0x097d6 } }, { { 0x06c2f, 0x09634 } }, { { 0x06d7c, 0x09495 } }, { { 0x06ecb, 0x092fa } },
  { { 0x0701d, 0x09162 } }, { { 0x07171, 0x08fce } }, { { 0x072c7, 0x08e3e } }, { { 0x07421, 0x08cb0 } },
  { { 0x0757c, 0x08b26 } }, { { 0x076da, 0x089a0 } }, { { 0x0783b, 0x0881c } }, { { 0x0799f, 0x0869c } },
  { { 0x07b05, 0x0851f } }, { { 0x07c6e, 0x083a4 } }, { { 0x07dd9, 0x0822d } }, { { 0x07f48, 0x080b9 } },
  { { 0x080b9, 0x07f48 } }, { { 0x0822d, 0x07dd9 } }, { { 0x083a4, 0x07c6e } }, { { 0x0851f, 0x07b05 } },
  { { 0x0869c, 0x0799f } }, { { 0x0881c, 0x0783b } }, { { 0x089a0, 0x076da } }, { { 0x08b26, 0x0757c } },
  { { 0x08cb0, 0x07421 } }, { { 0x08e3e, 0x072c7 } }, { { 0x08fce, 0x07171 } }, { { 0x09162, 0x0701d } },
  { { 0x092fa, 0x06ecb } }, { { 0x09495, 0x06d7c } }, { { 0x09634, 0x06c2f } }, { { 0x097d6, 0x06ae4 } },
  { { 0x0997c, 0x0699c } }, { { 0x09b26, 0x06856 } }, { { 0x09cd4, 0x06712 } }, { { 0x09e85, 0x065d1 } },
  { { 0x0a03b, 0x06491 } }, { { 0x0a1f5, 0x06354 } }, { { 0x0a3b3, 0x06219 } }, { { 0x0a575, 0x060e0 } },
  { { 0x0a73c, 0x05fa9 } }, { { 0x0a907, 0x05e74 } }, { { 0x0aad6, 0x05d41 } }, { { 0x0acaa, 0x05c10 } },
  { { 0x0ae83, 0x05ae1 } }, { { 0x0b061, 0x059b4 } }, { { 0x0b243, 0x05888 } }, { { 0x0b42a, 0x0575f } },
  { { 0x0b617, 0x05638 } }, { { 0x0b808, 0x05512 } }, { { 0x0b9ff, 0x053ee } }, { { 0x0bbfc, 0x052cc } },
  { { 0x0bdfe, 0x051ac } }, { { 0x0c005, 0x0508d } }, { { 0x0c212, 0x04f71 } }, { { 0x0c426, 0x04e55 } },
  { { 0x0c63f, 0x04d3c } }, { { 0x0c85e, 0x04c24 } }, { { 0x0ca84, 0x04b0e } }, { { 0x0ccb0, 0x049fa } },
  { { 0x0cee3, 0x048e7 } }, { { 0x0d11c, 0x047d6 } }, { { 0x0d35d, 0x046c6 } }, { { 0x0d5a5, 0x045b8 } },
  { { 0x0d7f3, 0x044ab } }, { { 0x0da4a, 0x043a0 } }, { { 0x0dca8, 0x04297 } }, { { 0x0df0e, 0x0418e } },
  { { 0x0e17c, 0x04088 } }, { { 0x0e3f2, 0x03f83 } }, { { 0x0e671, 0x03e7f } }, { { 0x0e8f9, 0x03d7d } },
  { { 0x0eb89, 0x03c7c } }, { { 0x0ee23, 0x03b7c } }, { { 0x0f0c7, 0x03a7e } }, { { 0x0f374, 0x03981 } },
  { { 0x0f62b, 0x03886 } }, { { 0x0f8ed, 0x0378c } }, { { 0x0fbb9, 0x03693 } }, { { 0x0fe90, 0x0359b } },
  { { 0x10173, 0x034a5 } }, { { 0x10461, 0x033b0 } }, { { 0x1075c, 0x032bc } }, { { 0x10a63, 0x031ca } },
  { { 0x10d77, 0x030d9 } }, { { 0x11098, 0x02fe9 } }, { { 0x113c7, 0x02efa } }, { { 0x11704, 0x02e0c } },
  { { 0x11a51, 0x02d20 } }, { { 0x11dac, 0x02c35 } }, { { 0x12118, 0x02b4b } }, { { 0x12494, 0x02a62 } },
  { { 0x12821, 0x0297a } }, { { 0x12bc0, 0x02894 } }, { { 0x12f71, 0x027ae } }, { { 0x13336, 0x026ca } },
  { { 0x1370f, 0x025e6 } }, { { 0x13afd, 0x02504 } }, { { 0x13f01, 0x02423 } }, { { 0x1431b, 0x02343 } },
  { { 0x1474e, 0x02264 } }, { { 0x14b99, 0x02186 } }, { { 0x14fff, 0x020a9 } }, { { 0x15480, 0x01fcd } },
  { { 0x1591e, 0x01ef3 } }, { { 0x15dda, 0x01e19 } }, { { 0x162b6, 0x01d40 } }, { { 0x167b4, 0x01c68 } },
  { { 0x16cd5, 0x01b91 } }, { { 0x1721c, 0x01abb } }, { { 0x1778a, 0x019e6 } }, { { 0x17d23, 0x01912 } },
  { { 0x182e8, 0x0183f } }, { { 0x188de, 0x0176d } }, { { 0x18f06, 0x0169c } }, { { 0x19564, 0x015cc } },
  { { 0x19bfc, 0x014fc } }, { { 0x1a2d4, 0x0142e } }, { { 0x1a9ee, 0x01360 } }, { { 0x1b151, 0x01294 } },
  { { 0x1b903, 0x011c8 } }, { { 0x1c10b, 0x010fd } }, { { 0x1c970, 0x01033 } }, { { 0x1d23c, 0x00f6a } },
  { { 0x1db78, 0x00ea2 } }, { { 0x1e531, 0x00dda } }, { { 0x1ef74, 0x00d13 } }, { { 0x1fa51, 0x00c4e } },
  { { 0x205dd, 0x00b89 } }, { { 0x2122e, 0x00ac4 } }, { { 0x21f60, 0x00a01 } }, { { 0x22d96, 0x0093f } },
  { { 0x23cfc, 0x0087d } }, { { 0x24dc9, 0x007bc } }, { { 0x26044, 0x006fc } }, { { 0x274ce, 0x0063c } },
  { { 0x28beb, 0x0057e } }, { { 0x2a658, 0x004c0 } }, { { 0x2c531, 0x00403 } }, { { 0x2ea40, 0x00346 } },
  { { 0x318a9, 0x0028b } }, { { 0x356cb, 0x001d0 } }, { { 0x3b520, 0x00116 } }, { { 0x48000, 0x0005c } },
};

const uint16_t ProbModelTables::m_inistateToCount[128] = {
  614,   647,   681,   718,   756,   797,   839,   884,   932,   982,   1034,  1089,  1148,  1209,  1274,  1342,
  1414,  1490,  1569,  1653,  1742,  1835,  1933,  2037,  2146,  2261,  2382,  2509,  2643,  2785,  2934,  3091,
  3256,  3430,  3614,  3807,  4011,  4225,  4452,  4690,  4941,  5205,  5483,  5777,  6086,  6412,  6755,  7116,
  7497,  7898,  8320,  8766,  9235,  9729,  10249, 10798, 11375, 11984, 12625, 13300, 14012, 14762, 15551, 16384,
  16384, 17216, 18005, 18755, 19467, 20142, 20783, 21392, 21969, 22518, 23038, 23532, 24001, 24447, 24869, 25270,
  25651, 26012, 26355, 26681, 26990, 27284, 27562, 27826, 28077, 28315, 28542, 28756, 28960, 29153, 29337, 29511,
  29676, 29833, 29982, 30124, 30258, 30385, 30506, 30621, 30730, 30834, 30932, 31025, 31114, 31198, 31277, 31353,
  31425, 31493, 31558, 31619, 31678, 31733, 31785, 31835, 31883, 31928, 31970, 32011, 32049, 32086, 32120, 32153
};
#endif

void BinProbModel_Std::init( int qp, int initId )
{
  int slope     = ( ( initId >>  4 )  * 5 ) - 45;
  int offset    = ( ( initId  & 15 ) << 3 ) - 16;
  int inistate  = ( ( slope   * qp ) >> 4 ) + offset;
#if JVET_M0453_CABAC_ENGINE
  const int p1 = m_inistateToCount[inistate < 0 ? 0 : inistate > 127 ? 127 : inistate];
  m_state[0]   = p1 & MASK_0;
  m_state[1]   = p1 & MASK_1;
#else
  if( inistate >= 64 )
  {
    m_State     = ( std::min( 62, inistate - 64 ) << 1 ) + 1;
  }
  else
  {
    m_State     = ( std::min( 62, 63 - inistate ) << 1 );
  }
#endif
}




CtxSet::CtxSet( std::initializer_list<CtxSet> ctxSets )
{
  uint16_t  minOffset = std::numeric_limits<uint16_t>::max();
  uint16_t  maxOffset = 0;
  for( auto iter = ctxSets.begin(); iter != ctxSets.end(); iter++ )
  {
    minOffset = std::min<uint16_t>( minOffset, (*iter).Offset              );
    maxOffset = std::max<uint16_t>( maxOffset, (*iter).Offset+(*iter).Size );
  }
  Offset  = minOffset;
  Size    = maxOffset - minOffset;
}





const std::vector<uint8_t>& ContextSetCfg::getInitTable( unsigned initId )
{
  VTMCHECK( initId >= (unsigned)sm_InitTables.size(),
         "Invalid initId (" << initId << "), only " << sm_InitTables.size() << " tables defined." );
  return sm_InitTables[initId];
}


CtxSet ContextSetCfg::addCtxSet( std::initializer_list<std::initializer_list<uint8_t>> initSet2d )
{
  const std::size_t startIdx  = sm_InitTables[0].size();
  const std::size_t numValues = ( *initSet2d.begin() ).size();
        std::size_t setId     = 0;
  for( auto setIter = initSet2d.begin(); setIter != initSet2d.end() && setId < sm_InitTables.size(); setIter++, setId++ )
  {
    const std::initializer_list<uint8_t>& initSet   = *setIter;
    std::vector<uint8_t>&           initTable = sm_InitTables[setId];
    VTMCHECK( initSet.size() != numValues,
           "Number of init values do not match for all sets (" << initSet.size() << " != " << numValues << ")." );
    initTable.resize( startIdx + numValues );
    std::size_t elemId = startIdx;
    for( auto elemIter = ( *setIter ).begin(); elemIter != ( *setIter ).end(); elemIter++, elemId++ )
    {
      initTable[elemId] = *elemIter;
    }
  }
  return CtxSet( (uint16_t)startIdx, (uint16_t)numValues );
}



#define CNU 154 // dummy initialization value for unused context models 'Context model Not Used'
#if JVET_M0453_CABAC_ENGINE
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables(NUMBER_OF_SLICE_TYPES + 1);
#else
std::vector<std::vector<uint8_t>> ContextSetCfg::sm_InitTables( NUMBER_OF_SLICE_TYPES );
#endif

// clang-format off
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0421_SPLIT_SIG
  // |-------- do split ctx -------------------|
  { 122, 124, 141, 108, 125, 156, 138, 126, 143, },
  { 93, 139, 171, 124, 125, 141, 139, 141, 158, },
  { 138, 154, 172, 124, 140, 142, 154, 127, 175, },
#if JVET_M0453_CABAC_ENGINE
  { 9, 13, 8, 8, 13, 12, 5, 10, 12, },
#endif
#else
#if JVET_M0453_CABAC_ENGINE
  {  107, 110, 127, 106, 123, 140,},
  {  138, 140, 142, 106, 123, 125,},
  {  152, 126, 159, 150, 138, 126,},
  {    8,   8,  12,  12,  12,   9,},
#else
  { 137, 125, 127, 107, 138, 140, },
  { 138, 111, 143, 107, 138, 140, },
  { 138, 141, 158, 151, 124, 126, },
#endif
#endif
});

#if JVET_M0421_SPLIT_SIG
const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet
({
  { 138, 140, 142, 136, 138, 140, },
  { 139, 126, 142, 107, 138, 125, },
  { 139, 125, 127, 136, 153, 126, },
#if JVET_M0453_CABAC_ENGINE
  { 0, 8, 8, 12, 12, 8, },
#endif
});

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet
({
  { 154, 168, 155, 139, 155, },
  { 169, 168, 170, 153, 170, },
  { 154, 168, 140, 153, 169, },
#if JVET_M0453_CABAC_ENGINE
  { 10, 9, 9, 8, 8, },
#endif
});

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet
({
  { 154, 140, 154, 140, },
  { 169, 155, 154, 140, },
  { 154, 170, 154, 170, },
#if JVET_M0453_CABAC_ENGINE
  { 12, 12, 12, 12, },
#endif
});
#else
const CtxSet ContextSetCfg::BTSplitFlag = ContextSetCfg::addCtxSet
({
  // |-------- 1st bin, 9 ctx for luma + 3 ctx for chroma------| |--2nd bin--| |3rd bin|
#if JVET_M0453_CABAC_ENGINE
  {  137, 125, 141, 123, 125, 141,  78, 124, 140, CNU, CNU, CNU, 169, 155, 154, 140,},
  {  138, 140, 142, 138, 125, 141, 107, 124, 140, CNU, CNU, CNU, 169, 170, 139, 155,},
  {  139, 141, 157, 124, 111, 142, 138, 139, 141, 153, 140, 156, 154, 169, 139, 155,},
  {    4,   9,   9,   9,   9,   9,   8,  12,   8,   9,  13,  13,   5,   8,   8,  13,},
#else
  { 137, 125, 141, 123, 125, 141, 78, 124, 140, CNU, CNU, CNU, 169, 155, 154, 154, },
  { 123, 140, 156, 138, 125, 141, 122, 124, 140, CNU, CNU, CNU, 169, 155, 139, 169, },
  { 139, 141, 157, 139, 155, 142, 153, 125, 141, 154, 154, 154, 154, 154, 154, 140, },
#endif
});
#endif

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 197, 214, 216, },
  { 197, 198, 185, },
  { 40, 138, 154, },
  { 5, 8, 8, },
#else
  { 183, 185, 186, },
  { 168, 199, 200, },
  { CNU, CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 111, },
  { 111, },
  { 153, },
  { 5, },
#else
  { 125, },
  { 110, },
  { CNU, },
#endif
});

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 138, },
  { 154, },
  { 153, },
  { 8, },
#else
  { 167, },
  { 138, },
  { CNU, },
#endif
});

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 120, },
  { 122, },
  { CNU, },
  { 8, },
#else
  { 136, },
  { 167, },
  { CNU, },
#endif
});

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 154, },
  { 154, },
  { CNU, },
  { 10, },
#else
  { 154, },
  { 154, },
  { CNU, },
#endif
});

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 213, },
  { 244, },
  { CNU, },
  { 1, },
#else
  { 213, },
  { 169, },
  { CNU, },
#endif
});

const CtxSet ContextSetCfg::PartSize = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  {  CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU,},
  {  CNU, CNU, CNU, CNU,},
  { DWS, DWS, DWS, DWS, }
#else
  {  154, 139, 154, 154,},
  {  154, 139, 154, 154,},
  {  184, CNU, CNU, CNU,},
#endif
});

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
#if JVET_M0502_PRED_MODE_CTX
  { 192, 168, },
  { 165, 139, },
  { CNU, CNU, },
  { 5, 2, },
#else
  {  193,},
  {  151,},
  {  CNU,},
  {    1,},
#endif
#else
#if JVET_M0502_PRED_MODE_CTX
  { 178, 178, },
  { 194, 194, },
  { CNU, CNU, },
#else
  { 178, },
  { 194, },
  { CNU, },
#endif
#endif
});

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 90, 212, CNU, },
  { 118, 212, CNU, },
  { 119, 169, CNU, },
  { 8, 8, DWS, },
#else
  { 151, 183, CNU, },
  { 165, 183, CNU, },
  { 122, 184, CNU, },
#endif
});

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 154, },
  { 154, },
  { 170, },
  { 6, },
#else
    { 183, },
    { 154, },
    { 156, },
#endif
});

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  {  137, 139, 140,},
  {  138, 139, 169,},
  {  154, 139, 154,},
  {    5,   8,   9,},
#else
  { 152, 139, 154,},
  { 138, 139, 169,},
  { 109, 139, 154,},
#endif
});

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet
({
  {  154, 154, 154,},
  {  154, 154, 154,},
  {  154, 154, 154,},
#if JVET_M0453_CABAC_ENGINE
  { DWS, DWS, DWS, }
#endif
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 111, 125, 110, 94, 192, },
  { 126, 111, 110, 94, 208, },
  { CNU, CNU, CNU, CNU, CNU, },
  { 0, 0, 4, 5, 0, },
#else
  { 111, 110, 95, 78, 193, },
  { 126, 111, 95, 93, 194, },
  { CNU, CNU, CNU, CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 125, 139, },
  { 138, 168, },
  { CNU, CNU, },
  { 4, 5, },
#else
  { 139, 139, },
  { 138, 168, },
  { CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 179, 169, 171, },
  { 180, 168, 155, },
  { CNU, CNU, CNU, },
  { 8, 5, 4, },
#else
  { 196, 184, 171, },
  { 181, 169, 185, },
  { CNU, CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 138, },
  { 153, },
  { CNU, },
  { 4, },
#else
  { 123, },
  { 138, },
  { CNU, },
#endif
});

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet
( {
#if JVET_M0453_CABAC_ENGINE
#if JVET_M0381_ONE_CTX_FOR_SUBBLOCK_MRG_IDX
  { 109, },
  { 95, },
  { CNU, },
  { 0, },
#else
  {  109, 168, 168, 153, CNU,},
  {   95, 154, 139, 153, CNU,},
  {  CNU, CNU, CNU, CNU, CNU,},
  {    0,   5,   9,   8, DWS,},
#endif
#else
#if JVET_M0381_ONE_CTX_FOR_SUBBLOCK_MRG_IDX
  { 123, },
  { 109, },
  { CNU, },
#else
  { 123, 154, 154, 168, CNU, },
  { 109, 154, 139, 168, CNU, },
  { CNU, CNU, CNU, CNU, CNU, },
#endif
#endif
} );

const CtxSet ContextSetCfg::GBiIdx = ContextSetCfg::addCtxSet
({
  // 4 ctx for 1st bin; 1 ctx for each of rest bins
#if JVET_M0453_CABAC_ENGINE
  { 228, CNU, CNU, CNU, 125, 155, 175, },
  { 242, CNU, CNU, CNU, 154, 170, 237, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { 4, DWS, DWS, DWS, 4, 0, 0, },
#else
  { 199, CNU, CNU, CNU, 124, 169, 127, },
  { 154, CNU, CNU, CNU, 124, 185, 143, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 169, 183, },
  { 155, 154, },
  { 141, 156, },
  { 9, 5, },
#else
  { 169, 183, },
  { 155, 198, },
  { CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 109, },
  { 95, },
  { 110, },
  { 4, },
#else
  { 94, },
  { 95, },
  { CNU, },
#endif
});

const CtxSet ContextSetCfg::QtCbf[] =
{
#if JVET_M0102_INTRA_SUBPARTITIONS
#if JVET_M0453_CABAC_ENGINE
  ContextSetCfg::addCtxSet
  ({
    { 141, 127, 139, 140, },
    { 142, 127, 139, 140, },
    { CNU, 111, 124, 111, },
    { 1, 5, 9, 8, },
  }),
#else
  ContextSetCfg::addCtxSet
  ({
    { 140, 141, CNU, CNU},
    { 155, 127, CNU, CNU},
    { CNU, 126, CNU, CNU},
  }),
#endif
#else
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    {  155, 127,},
    {  141, 127,},
    {  CNU, 126,},
    {    4,   5,  },
#else
    { 140, 141, },
    { 155, 127, },
    { CNU, 126, },
#endif
  }),
#endif
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 163, 154, CNU, CNU, CNU, },
    { 164, 154, CNU, CNU, CNU, },
    { 109, CNU, CNU, CNU, CNU, },
    { 5, 8, DWS, DWS, DWS, },
#else
    { 149, 168, CNU, CNU, CNU, },
    { 164, 154, CNU, CNU, CNU, },
    { 109, CNU, CNU, CNU, CNU, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 161, 154, },
    { 192, 154, },
    { 151, 155, },
    { 5, 5, },
#else
    { 192, 153, },
    { 178, 139, },
    { 122, 140, },
#endif
  }),
};

const CtxSet ContextSetCfg::SigCoeffGroup[] =
{
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 105, 155, },
    { 106, 156, },
    { 107, 158, },
    { 8, 5, },
#else
    { 106, 170, },
    { 121, 141, },
    { 107, 158, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 91, 155, },
    { 90, 141, },
    { 76, 127, },
    { 5, 8, },
#else
    { 91, 140, },
    { 105, 155, },
    { 105, 126, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
    { CNU, CNU, },
    { CNU, CNU, },
    { CNU, CNU, },
#if JVET_M0453_CABAC_ENGINE
    { DWS, DWS, }
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
    { CNU, CNU, },
    { CNU, CNU, },
    { CNU, CNU, },
#if JVET_M0453_CABAC_ENGINE
    { DWS, DWS, }
#endif
  }),
};

const CtxSet ContextSetCfg::SigFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 88, 166, 152, 182, 168, 154, 0, 167, 182, 168, 183, 155, 193, 213, 183, 183, 169, 185, },
    { 132, 152, 167, 168, 183, 140, 177, 182, 168, 154, 169, 155, 180, 213, 183, 169, 184, 156, },
    { 89, 138, 153, 139, 154, 140, 134, 139, 139, 140, 140, 141, 137, 170, 169, 170, 141, 157, },
    { 12, 9, 9, 9, 9, 10, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 9, },
#else
    { 105, 152, 167, 153, 168, 169, 104, 167, 182, 183, 183, 170, 209, 213, 183, 183, 169, 185, },
    { 119, 152, 167, 168, 183, 140, 134, 182, 168, 183, 169, 185, 166, 228, 183, 198, 184, 156, },
    { 105, 138, 153, 154, 125, 111, 105, 139, 154, 155, 155, 127, 137, 185, 169, 185, 171, 159, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 72, 167, 153, 168, 154, 155, 180, 199, 183, 199, 199, 186, },
    { 133, 138, 153, 139, 154, 140, 181, 229, 169, 229, 170, 157, },
    { 43, 153, 168, 169, 154, 155, 152, 215, 155, 201, 171, 143, },
    { 9, 9, 12, 9, 13, 13, 5, 5, 8, 8, 8, 9, },
#else
    { 148, 167, 153, 168, 154, 140, 166, 199, 183, 199, 199, 172, },
    { 134, 168, 168, 169, 169, 170, 196, 244, 184, 244, 200, 172, },
    { 104, 168, 168, 169, 140, 141, 167, 215, 155, 172, 171, 158, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 152, 156, 201, 186, 186, 187, 182, 248, 188, 232, 188, 205, 182, 223, 223, 223, 223, 223, },
    { 123, 142, 157, 172, 172, 218, 138, 249, 248, 248, 219, 223, 139, 223, 223, 223, 223, 223, },
    { 93, 142, 157, 143, 188, 175, 138, 238, 205, 238, 253, 237, 139, 223, 223, 223, 223, 253, },
    { 9, 12, 8, 8, 8, 8, 8, 8, 8, 8, 8, 5, 8, 0, 0, 0, 0, 0, },
#else
    { 152, 127, 173, 201, 187, 173, 197, 203, 188, 217, 188, 189, 182, 223, 223, 223, 223, 223, },
    { 123, 142, 202, 172, 172, 203, 138, 188, 233, 203, 203, 191, 139, 223, 223, 223, 223, 223, },
    { 108, 157, 158, 158, 218, 189, 123, 191, 159, 190, 205, 236, 79, 223, 253, 223, 223, 253, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 182, 171, 143, 158, 172, 189, 183, 223, 223, 223, 223, 223, },
    { 168, 156, 173, 216, 172, 219, 169, 223, 223, 223, 223, 223, },
    { 152, 173, 157, 187, 204, 253, 170, 223, 223, 223, 223, 223, },
    { 8, 9, 12, 8, 8, 8, 4, 0, 2, 2, 2, 2, },
#else
    { 182, 171, 143, 158, 172, 202, 168, 223, 223, 223, 223, 223, },
    { 168, 156, 173, 201, 157, 203, 198, 223, 223, 223, 223, 223, },
    { 152, 173, 157, 187, 189, 251, 170, 223, 223, 253, 223, 223, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 123, 173, 223, 191, 232, 251, 212, 223, 223, 236, 206, 223, 192, 223, 223, 223, 223, 223, },
    { 123, 175, 223, 175, 218, 223, 138, 223, 223, 223, 222, 223, 196, 223, 223, 223, 223, 223, },
    { 107, 174, 223, 238, 251, 223, 63, 223, 223, 238, 223, 238, 12, 223, 223, 223, 223, 223, },
    { 8, 8, 4, 8, 8, 8, 8, 0, 0, 4, 8, 5, 4, 2, 2, 2, 2, 1, },
#else
    { 137, 142, 190, 188, 202, 189, 241, 191, 191, 189, 189, 190, 195, 223, 223, 223, 223, 223, },
    { 123, 187, 191, 173, 173, 248, 138, 191, 191, 191, 203, 191, 196, 223, 223, 223, 223, 223, },
    { 107, 143, 205, 188, 233, 205, 63, 251, 191, 253, 206, 252, 62, 223, 223, 223, 223, 223, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 167, 201, 223, 248, 219, 223, 181, 223, 223, 223, 223, 223, },
    { 167, 171, 223, 175, 248, 223, 152, 223, 223, 223, 223, 223, },
    { 166, 234, 223, 236, 248, 223, 108, 223, 223, 223, 223, 223, },
    { 8, 8, 5, 8, 8, 8, 5, 1, 2, 2, 2, 2, },
#else
    { 167, 200, 175, 188, 174, 175, 196, 223, 223, 223, 223, 223, },
    { 167, 156, 237, 158, 188, 205, 182, 223, 223, 223, 223, 223, },
    { 166, 174, 159, 247, 188, 189, 168, 223, 223, 223, 238, 223, },
#endif
  }),
};


const CtxSet ContextSetCfg::ParFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 121, 105, 136, 152, 138, 183, 90, 122, 167, 153, 168, 135, 152, 153, 168, 139, 151, 153, 139, 168, 154, },
    { 121, 119, 136, 137, 138, 153, 104, 122, 138, 153, 139, 106, 138, 153, 168, 139, 137, 153, 168, 139, 139, },
    { 121, 135, 137, 152, 138, 153, 91, 137, 138, 153, 139, 151, 138, 153, 139, 139, 138, 168, 139, 154, 139, },
    { 8, 9, 12, 13, 13, 13, 10, 13, 13, 13, 13, 13, 13, 13, 13, 13, 10, 13, 13, 13, 13, },
#else
    { 91, 104, 136, 152, 153, 153, 105, 137, 167, 153, 168, 121, 167, 153, 168, 139, 151, 153, 139, 168, 154, },
    { 106, 134, 151, 152, 138, 168, 120, 137, 138, 153, 139, 136, 138, 153, 168, 139, 137, 153, 168, 139, 139, },
    { 121, 135, 137, 138, 153, 153, 136, 123, 138, 153, 139, 152, 153, 153, 139, 139, 138, 168, 139, 154, 139, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 151, 120, 152, 138, 153, 153, 136, 168, 154, 168, 154, },
    { 135, 120, 137, 138, 138, 153, 136, 153, 168, 139, 154, },
    { 136, 135, 152, 153, 138, 153, 136, 168, 154, 139, 154, },
    { 8, 10, 12, 12, 13, 13, 10, 10, 13, 13, 13, },
#else
    { 135, 135, 152, 138, 153, 124, 151, 168, 169, 153, 139, },
    { 120, 150, 152, 153, 153, 153, 166, 168, 168, 139, 154, },
    { 136, 121, 167, 168, 138, 153, 137, 139, 154, 139, 154, },
#endif
  }),
};

const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 31, 73, 118, 75, 152, 109, 42, 44, 105, 107, 109, 0, 119, 136, 152, 124, 118, 136, 138, 153, 140, },
    { 14, 116, 86, 119, 106, 152, 0, 72, 120, 151, 138, 116, 90, 107, 152, 153, 104, 107, 123, 153, 154, },
    { 90, 72, 119, 135, 137, 138, 43, 60, 106, 137, 109, 58, 106, 108, 109, 124, 121, 138, 139, 154, 155, },
    { 4, 1, 8, 8, 4, 2, 5, 9, 9, 8, 9, 9, 9, 9, 8, 9, 9, 8, 9, 8, 8, },
#else
    { 30, 0, 102, 104, 106, 152, 57, 44, 120, 136, 123, 87, 134, 151, 152, 153, 89, 121, 152, 153, 125, },
    { 88, 0, 102, 149, 150, 152, 101, 103, 150, 151, 138, 102, 105, 122, 167, 153, 90, 107, 123, 153, 154, },
    { 90, 41, 149, 121, 122, 123, 58, 105, 92, 108, 109, 104, 92, 123, 109, 124, 151, 138, 139, 154, 140, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 119, 101, 134, 151, 107, 123, 118, 122, 124, 140, 155, },
    { 117, 0, 90, 106, 92, 93, 147, 136, 138, 154, 140, },
    { 194, 40, 120, 122, 122, 138, 103, 121, 153, 154, 155, },
    { 2, 5, 8, 8, 8, 6, 6, 8, 8, 8, 7, },
#else
    { 102, 101, 90, 107, 122, 93, 118, 121, 153, 125, 140, },
    { 0, 0, 105, 151, 107, 93, 103, 136, 138, 154, 125, },
    { 165, 11, 120, 122, 137, 138, 75, 106, 138, 154, 155, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 43, 177, 181, 168, 154, 170, 133, 167, 139, 154, 155, 164, 153, 154, 169, 155, 181, 183, 169, 185, 186, },
    { 101, 133, 137, 153, 139, 140, 134, 138, 139, 154, 155, 136, 153, 154, 140, 170, 138, 154, 155, 170, 186, },
    { 134, 120, 123, 153, 139, 140, 92, 124, 154, 125, 111, 138, 154, 140, 155, 141, 154, 140, 185, 171, 143, },
    { 8, 5, 9, 9, 12, 9, 9, 10, 13, 12, 10, 9, 10, 10, 10, 10, 8, 9, 8, 8, 10, },
#else
    { 89, 132, 151, 138, 124, 125, 119, 152, 153, 154, 140, 135, 153, 139, 169, 155, 151, 168, 169, 170, 171, },
    { 118, 101, 137, 138, 139, 140, 149, 138, 139, 154, 155, 136, 153, 154, 140, 170, 152, 139, 140, 155, 186, },
    { 135, 120, 108, 153, 139, 140, 151, 153, 139, 125, 140, 123, 154, 140, 155, 126, 139, 140, 170, 156, 142, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 0, 178, 153, 154, 140, 140, 196, 170, 186, 157, 188, },
    { 0, 135, 153, 139, 125, 140, 182, 155, 156, 142, 159, },
    { 163, 136, 153, 154, 125, 140, 183, 170, 201, 187, 174, },
    { 6, 9, 10, 12, 12, 10, 5, 9, 8, 8, 9, },
#else
    { 102, 164, 138, 139, 154, 140, 181, 155, 171, 157, 143, },
    { 132, 136, 153, 154, 140, 155, 167, 155, 156, 142, 173, },
    { 165, 151, 153, 154, 125, 126, 168, 155, 186, 172, 143, },
#endif
  }),
};

const CtxSet ContextSetCfg::LastX[] =
{
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 111, 111, 110, 111, 111, 139, 111, 126, 111, 139, 126, 126, 111, 111, 169, 154, 111, 110, 110, 139, CNU, CNU, CNU, CNU, CNU, },
    { 125, 110, 109, 125, 125, 123, 111, 111, 95, 123, 126, 111, 110, 95, 169, 154, 140, 139, 139, 138, CNU, CNU, CNU, CNU, CNU, },
    { 125, 140, 124, 111, 111, 109, 111, 126, 125, 123, 111, 141, 111, 125, 79, 155, 142, 170, 140, 183, CNU, CNU, CNU, CNU, CNU, },
    { 8, 5, 5, 5, 4, 4, 5, 4, 4, 0, 5, 1, 0, 0, 0, 1, 1, 0, 0, 0, DWS, DWS, DWS, DWS, DWS, },
#else
    { 111, 125, 124, 111, 111, 109, 111, 111, 125, 109, 140, 126, 111, 111, 139, 140, 111, 125, 95, 138, CNU, CNU, CNU, CNU, CNU, },
    { 125, 110, 109, 111, 125, 123, 111, 111, 95, 123, 140, 126, 125, 95, 169, 125, 140, 110, 124, 152, CNU, CNU, CNU, CNU, CNU, },
    { 140, 140, 124, 140, 126, 109, 140, 141, 125, 94, 111, 127, 111, 140, 93, 141, 186, 141, 125, 197, CNU, CNU, CNU, CNU, CNU, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 122, 124, 63, CNU, },
    { 138, 123, 92, CNU, },
    { 138, 108, 47, CNU, },
    { 2, 1, 1, DWS, },
#else
    { 123, 109, 63, CNU, },
    { 138, 123, 92, CNU, },
    { 123, 108, 62, CNU, },
#endif
  }),
};

const CtxSet ContextSetCfg::LastY[] =
{
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 125, 125, 139, 125, 111, 139, 111, 111, 110, 110, 140, 126, 125, 125, 140, 139, 111, 110, 124, 181, CNU, CNU, CNU, CNU, CNU, },
    { 95, 95, 109, 110, 110, 108, 125, 111, 124, 123, 140, 111, 110, 124, 139, 125, 126, 110, 124, 182, CNU, CNU, CNU, CNU, CNU, },
    { 110, 110, 109, 125, 111, 123, 111, 126, 95, 108, 111, 127, 111, 95, 78, 169, 157, 141, 125, 138, CNU, CNU, CNU, CNU, CNU, },
    { 8, 5, 8, 5, 5, 4, 5, 5, 4, 0, 5, 5, 1, 0, 0, 1, 4, 1, 0, 0, DWS, DWS, DWS, DWS, DWS, },
#else
    { 125, 110, 139, 125, 125, 109, 111, 111, 110, 109, 140, 126, 110, 110, 154, 140, 111, 125, 109, 181, CNU, CNU, CNU, CNU, CNU, },
    { 110, 95, 94, 125, 110, 123, 140, 111, 95, 123, 125, 111, 110, 95, 154, 125, 111, 95, 94, 137, CNU, CNU, CNU, CNU, CNU, },
    { 110, 110, 109, 125, 111, 123, 111, 141, 95, 108, 111, 142, 111, 95, 63, 140, 157, 141, 110, 152, CNU, CNU, CNU, CNU, CNU, },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
#if JVET_M0453_CABAC_ENGINE
    { 122, 124, 123, CNU, },
    { 108, 123, 121, CNU, },
    { 123, 123, 91, CNU, },
    { 2, 2, 2, DWS, },
#else
    { 108, 94, 122, CNU, },
    { 108, 93, 92, CNU, },
    { 108, 123, 77, CNU, },
#endif
  }),
};


const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 153, },
  { 168, },
  { 168, },
  { 10, },
#else
  { 168, },
  { 168, },
  { CNU, },
#endif
});

#if JVET_M0444_SMVD
const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet
( {
  { 154, },
  { 125, },
  { CNU, },
#if JVET_M0453_CABAC_ENGINE
  { 8, },
#endif
} );
#endif

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 47, },
  { 244, },
  { 199, },
  { 0, },
#else
  { 92, },
  { 214, },
  { 184, },
#endif
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 47, },
  { 95, },
  { 95, },
  { 0, },
#else
  { 77, },
  { 111, },
  { 110, },
#endif
});

#if !JVET_M0464_UNI_MTS
const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  {  154,  13,},
  {  152,  57,},
  {  139,   0,},
  {    4,   1,},
#else
  { 124, 61, },
  { 138, 46, },
  { 109, 42, },
#endif
});
#endif

const CtxSet ContextSetCfg::TransquantBypassFlag = ContextSetCfg::addCtxSet
({
  {  154,},
  {  154,},
  {  154,},
#if JVET_M0453_CABAC_ENGINE
  { DWS, }
#endif
});

const CtxSet ContextSetCfg::RdpcmFlag = ContextSetCfg::addCtxSet
({
  {  139, 139,},
  {  139, 139,},
  {  CNU, CNU,},
#if JVET_M0453_CABAC_ENGINE
  { DWS, DWS, }
#endif
});

const CtxSet ContextSetCfg::RdpcmDir = ContextSetCfg::addCtxSet
({
  {  139, 139,},
  {  139, 139,},
  {  CNU, CNU,},
#if JVET_M0453_CABAC_ENGINE
  { DWS, DWS, }
#endif
});

#if JVET_M0464_UNI_MTS
const CtxSet ContextSetCfg::MTSIndex = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { CNU, 155, 155, 140, 140, CNU, 216, 153, 153, 0, CNU, },
  { CNU, 155, 155, 140, 140, CNU, 233, 167, 153, 0, CNU, },
  { CNU, CNU, 140, 140, 140, CNU, 219, 138, 153, 0, CNU, },
  { DWS, 8, 8, 8, 8, DWS, 4, 8, 9, 3, DWS, },
#else
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
#endif
});
#else
const CtxSet ContextSetCfg::EMTTuIndex = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  {  122, 136, CNU, CNU,},
  {  151, 150, CNU, CNU,},
  {  121, 136, CNU, CNU,},
  {    9,   9, DWS, DWS,},
#else
  { 153, 138, CNU, CNU, },
  { 138, 167, CNU, CNU, },
  { 167, 123, CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::EMTCuFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  {  216, 158, 172, 201, 200, CNU,},
  {  202, 158, 158, 158, 187, CNU,},
  {  CNU, CNU, 141, 171, 171, CNU,},
  {    9,   8,   9,   8,   8, DWS,},
#else
  { 155, 141, 155, 155, 140, CNU, },
  { 141, 141, 141, 126, 155, CNU, },
  { CNU, CNU, 140, 155, 155, CNU, },
#endif
});
#endif

#if JVET_M0102_INTRA_SUBPARTITIONS
const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet
({
  { 152, 154, },
  { 166, 154, },
  { 152, 154, },
#if JVET_M0453_CABAC_ENGINE
  { 8, 5, },
#endif
});
#endif

#if JVET_M0140_SBT
const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet
( {
  { 168, 183, },
  { 197, 183, },
  { CNU, CNU, },
#if JVET_M0453_CABAC_ENGINE
  { 4, 8, },
#endif
} );

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet
( {
  { 168, },
  { 168, },
  { CNU, },
#if JVET_M0453_CABAC_ENGINE
  { 9, },
#endif
} );

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet
( {
  { 139, 154, 139, },
  { 139, 154, 139, },
  { CNU, CNU, CNU, },
#if JVET_M0453_CABAC_ENGINE
  { 8, 5, 4, },
#endif
} );

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet
( {
  { 154, },
  { 154, },
  { CNU, },
#if JVET_M0453_CABAC_ENGINE
  { 13, },
#endif
} );
#endif

const CtxSet ContextSetCfg::CrossCompPred = ContextSetCfg::addCtxSet
({
  {  154, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
  {  154, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
  {  154, 154, 154, 154, 154, 154, 154, 154, 154, 154,},
#if JVET_M0453_CABAC_ENGINE
  { DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, }
#endif
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet
({
  {  154,},
  {  154,},
  {  154,},
#if JVET_M0453_CABAC_ENGINE
  { DWS, }
#endif
});

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet
({
  {  154,},
  {  154,},
  {  154,},
#if JVET_M0453_CABAC_ENGINE
  { DWS, }
#endif
});

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
#if JVET_M0246_AFFINE_AMVR
  { 212, 199, 215, 180, 183, 242, },
  { 213, 229, 244, 166, 198, 244, },
  { CNU, CNU, CNU, 152, CNU, CNU, },
  { 1, 4, 4, 5, 1, 0, },
#else
  {  227, 214, 230, 195,},
  {  213, 229, 230, 166,},
  {  CNU, CNU, CNU, CNU,},
  {    1,   4,   4,   5,},
#endif
#else
#if JVET_M0246_AFFINE_AMVR
  { 212, 214, 230, 182, 212, 214 },
  { 212, 214, 230, 182, 212, 214 },
  { CNU, CNU, CNU, CNU, CNU, CNU },
#else
  { 212, 214, 230, 182, },
  { 212, 214, 230, 182, },
  { CNU, CNU, CNU, CNU, },
#endif
#endif
});

const CtxSet ContextSetCfg::ctbAlfFlag =
{
  ContextSetCfg::addCtxSet
  ( {
#if JVET_M0453_CABAC_ENGINE
    { 154, 186, 174, 183, 233, 250, 168, 248, 250, },
    { 139, 186, 203, 183, 247, 249, 183, 232, 249, },
    { 219, 236, 238, 232, 249, 235, 246, 234, 251, },
    { 0, 0, 4, 0, 0, 1, 0, 0, 1, },
#else
    { 138, 141, 173, 122, 170, 203, 151, 170, 203, },
    { 153, 156, 188, 137, 185, 218, 152, 185, 218, },
    { 155, 205, 253, 168, 187, 234, 168, 187, 220, },
#endif
  } )
};

#if ADCNN
const CtxSet ContextSetCfg::ctbCnnlfFlag =
{
	ContextSetCfg::addCtxSet
	({
		{ 100, 100, 100, 100, 100, 100, 100, 100, 100 },
		{ 153, 153, 153, 153, 153, 153, 153, 153, 153 },
		{ 200, 200, 200, 200, 200, 200, 200, 200, 200 },
})
};
#endif

const CtxSet ContextSetCfg::MHIntraFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 225, },
  { 197, },
  { CNU, },
  { 1, },
#else
  { 226, },
  { 227, },
  { CNU, },
#endif
});

const CtxSet ContextSetCfg::MHIntraPredMode = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 156, CNU, CNU, CNU, },
  { 156, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, },
  { 9, DWS, DWS, DWS, },
#else
  { 155, CNU, CNU, CNU, },
  { 141, CNU, CNU, CNU, },
  { CNU, CNU, CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::TriangleFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 149, 123, 123, },
  { 151, 152, 138, },
  { CNU, CNU, CNU, },
  { 8, 12, 9, },
#else
  { 165, 137, 153, },
  { 106, 122, 138, },
  { CNU, CNU, CNU, },
#endif
});

const CtxSet ContextSetCfg::TriangleIdx = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
#else
  { 155, },
  { 126, },
  { CNU, },
#endif
});
// clang-format on

#if JVET_M0483_IBC
const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet
({
#if JVET_M0453_CABAC_ENGINE
  { 0, 154, 141, },
  { 0, 153, 140, },
  { 132, 153, 125, },
  { 5, 5, 8, },
#else
  { 165, 137, 153, },
  { 106, 122, 138, },
  { CNU, CNU, CNU, },
#endif
});
#endif

const unsigned ContextSetCfg::NumberOfContexts = (unsigned)ContextSetCfg::sm_InitTables[0].size();


// combined sets
const CtxSet ContextSetCfg::Sao = { ContextSetCfg::SaoMergeFlag, ContextSetCfg::SaoTypeIdx };



template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore()
  : m_CtxBuffer ()
  , m_Ctx       ( nullptr )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( bool dummy )
  : m_CtxBuffer ( ContextSetCfg::NumberOfContexts )
  , m_Ctx       ( m_CtxBuffer.data() )
{}

template <class BinProbModel>
CtxStore<BinProbModel>::CtxStore( const CtxStore<BinProbModel>& ctxStore )
  : m_CtxBuffer ( ctxStore.m_CtxBuffer )
  , m_Ctx       ( m_CtxBuffer.data() )
{}

template <class BinProbModel>
void CtxStore<BinProbModel>::init( int qp, int initId )
{
  const std::vector<uint8_t>& initTable = ContextSetCfg::getInitTable( initId );
  VTMCHECK( m_CtxBuffer.size() != initTable.size(),
        "Size of init table (" << initTable.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
#if JVET_M0453_CABAC_ENGINE
  const std::vector<uint8_t> &rateInitTable = ContextSetCfg::getInitTable(NUMBER_OF_SLICE_TYPES);
  VTMCHECK(m_CtxBuffer.size() != rateInitTable.size(),
        "Size of rate init table (" << rateInitTable.size() << ") does not match size of context buffer ("
                                    << m_CtxBuffer.size() << ").");
#endif
  int clippedQP = Clip3( 0, MAX_QP, qp );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].init( clippedQP, initTable[k] );
#if JVET_M0453_CABAC_ENGINE
    m_CtxBuffer[k].setLog2WindowSize(rateInitTable[k]);
#endif
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::setWinSizes( const std::vector<uint8_t>& log2WindowSizes )
{
  VTMCHECK( m_CtxBuffer.size() != log2WindowSizes.size(),
        "Size of window size table (" << log2WindowSizes.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].setLog2WindowSize( log2WindowSizes[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::loadPStates( const std::vector<uint16_t>& probStates )
{
  VTMCHECK( m_CtxBuffer.size() != probStates.size(),
        "Size of prob states table (" << probStates.size() << ") does not match size of context buffer (" << m_CtxBuffer.size() << ")." );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    m_CtxBuffer[k].setState( probStates[k] );
  }
}

template <class BinProbModel>
void CtxStore<BinProbModel>::savePStates( std::vector<uint16_t>& probStates ) const
{
  probStates.resize( m_CtxBuffer.size(), uint16_t(0) );
  for( std::size_t k = 0; k < m_CtxBuffer.size(); k++ )
  {
    probStates[k] = m_CtxBuffer[k].getState();
  }
}





template class CtxStore<BinProbModel_Std>;





Ctx::Ctx()                                  : m_BPMType( BPM_Undefined )                        {}
Ctx::Ctx( const BinProbModel_Std*   dummy ) : m_BPMType( BPM_Std   ), m_CtxStore_Std  ( true )  {}

Ctx::Ctx( const Ctx& ctx )
  : m_BPMType         ( ctx.m_BPMType )
  , m_CtxStore_Std    ( ctx.m_CtxStore_Std    )
{
  ::memcpy( m_GRAdaptStats, ctx.m_GRAdaptStats, sizeof( unsigned ) * RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS );
}

