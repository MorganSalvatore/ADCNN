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

/** \file     TAppDecLib.cpp
    \brief    Decoder application class
*/

#include <list>
#include <vector>
#include <stdio.h>
#include <fcntl.h>

#include "ColourRemapping.h"
#include "DecoderLib/AnnexBread.h"
#include "DecoderLib/NALread.h"

//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 - create internal class
 - initialize internal class
 - until the end of the bitstream, call decoding function in DecLib class
 - delete allocated buffers
 - destroy internal class
 .
 */

ColourRemapping::ColourRemapping(): m_pcSeiColourRemappingInfoPrevious( NULL )
{

}


ColourRemapping::~ColourRemapping()
{
  reset();
}

void ColourRemapping::reset()
{
  delete m_pcSeiColourRemappingInfoPrevious;
  m_pcSeiColourRemappingInfoPrevious = NULL;
}


void applyColourRemapping(const PelUnitBuf& pic, SEIColourRemappingInfo& criSEI, const SPS &activeSPS, std::ofstream& outstream);

void ColourRemapping::outputColourRemapPic(Picture* pcPic, std::ofstream& outstream)
{
  const SPS &sps=*pcPic->cs->sps;
  SEIMessages colourRemappingInfo = getSeisByType(pcPic->SEIs, SEI::COLOUR_REMAPPING_INFO );
  SEIColourRemappingInfo *seiColourRemappingInfo = ( colourRemappingInfo.size() > 0 ) ? (SEIColourRemappingInfo*) *(colourRemappingInfo.begin()) : NULL;

  if (colourRemappingInfo.size() > 1)
  {
    msg( VTMWARNING, "Warning: Got multiple Colour Remapping Information SEI messages. Using first.");
  }
  if (seiColourRemappingInfo)
  {
    applyColourRemapping(pcPic->getRecoBuf(), *seiColourRemappingInfo, sps, outstream );

    // save the last CRI SEI received
    if (m_pcSeiColourRemappingInfoPrevious == NULL)
    {
      m_pcSeiColourRemappingInfoPrevious = new SEIColourRemappingInfo();
    }
    m_pcSeiColourRemappingInfoPrevious->copyFrom(*seiColourRemappingInfo);
  }
  else  // using the last CRI SEI received
  {
    // TODO: prevent persistence of CRI SEI across C(L)VS.
    if (m_pcSeiColourRemappingInfoPrevious != NULL)
    {
      if (m_pcSeiColourRemappingInfoPrevious->m_colourRemapPersistenceFlag == false)
      {
        msg( VTMWARNING, "Warning No SEI-CRI message is present for the current picture, persistence of the CRI is not managed\n");
      }
      applyColourRemapping(pcPic->getRecoBuf(), *m_pcSeiColourRemappingInfoPrevious, sps, outstream);
    }
  }
}

// compute lut from SEI
// use at lutPoints points aligned on a power of 2 value
// SEI Lut must be in ascending values of coded Values
static std::vector<int>
initColourRemappingInfoLut(const int                                          bitDepth_in,     // bit-depth of the input values of the LUT
                           const int                                          nbDecimalValues, // Position of the fixed point
                           const std::vector<SEIColourRemappingInfo::CRIlut> &lut,
                           const int                                          maxValue, // maximum output value
                           const int                                          lutOffset)
{
  const int lutPoints = (1 << bitDepth_in) + 1 ;
  std::vector<int> retLut(lutPoints);

  // missing values: need to define default values before first definition (check codedValue[0] == 0)
  int iTargetPrev = (lut.size() && lut[0].codedValue == 0) ? lut[0].targetValue: 0;
  int startPivot = (lut.size())? ((lut[0].codedValue == 0)? 1: 0): 1;
  int iCodedPrev  = 0;
  // set max value with the coded bit-depth
  // + ((1 << nbDecimalValues) - 1) is for the added bits
  const int maxValueFixedPoint = (maxValue << nbDecimalValues) + ((1 << nbDecimalValues) - 1);

  int iValue = 0;

  for ( int iPivot=startPivot ; iPivot < (int)lut.size(); iPivot++ )
  {
    int iCodedNext  = lut[iPivot].codedValue;
    int iTargetNext = lut[iPivot].targetValue;

    // ensure correct bit depth and avoid overflow in lut address
    int iCodedNext_bitDepth = std::min(iCodedNext, (1 << bitDepth_in));

    const int divValue =  (iCodedNext - iCodedPrev > 0)? (iCodedNext - iCodedPrev): 1;
    const int lutValInit = (lutOffset + iTargetPrev) << nbDecimalValues;
    const int roundValue = divValue / 2;
    for ( ; iValue<iCodedNext_bitDepth; iValue++ )
    {
      int value = iValue;
      int interpol = ((((value-iCodedPrev) * (iTargetNext - iTargetPrev)) << nbDecimalValues) + roundValue) / divValue;
      retLut[iValue]  = std::min(lutValInit + interpol , maxValueFixedPoint);
    }
    iCodedPrev  = iCodedNext;
    iTargetPrev = iTargetNext;
  }
  // fill missing values if necessary
  if(iCodedPrev < (1 << bitDepth_in)+1)
  {
    int iCodedNext  = (1 << bitDepth_in);
    int iTargetNext = (1 << bitDepth_in) - 1;

    const int divValue =  (iCodedNext - iCodedPrev > 0)? (iCodedNext - iCodedPrev): 1;
    const int lutValInit = (lutOffset + iTargetPrev) << nbDecimalValues;
    const int roundValue = divValue / 2;

    for ( ; iValue<=iCodedNext; iValue++ )
    {
      int value = iValue;
      int interpol = ((((value-iCodedPrev) * (iTargetNext - iTargetPrev)) << nbDecimalValues) + roundValue) / divValue;
      retLut[iValue]  = std::min(lutValInit + interpol , maxValueFixedPoint);
    }
  }
  return retLut;
}

static void
initColourRemappingInfoLuts(std::vector<int>      (&preLut)[3],
                            std::vector<int>      (&postLut)[3],
                            SEIColourRemappingInfo &pCriSEI,
                            const int               maxBitDepth)
{
  int internalBitDepth = pCriSEI.m_colourRemapBitDepth;
  for ( int c=0 ; c<3 ; c++ )
  {
    std::sort(pCriSEI.m_preLut[c].begin(), pCriSEI.m_preLut[c].end()); // ensure preLut is ordered in ascending values of codedValues
    preLut[c] = initColourRemappingInfoLut(pCriSEI.m_colourRemapInputBitDepth, maxBitDepth - pCriSEI.m_colourRemapInputBitDepth, pCriSEI.m_preLut[c], ((1 << internalBitDepth) - 1), 0); //Fill preLut

    std::sort(pCriSEI.m_postLut[c].begin(), pCriSEI.m_postLut[c].end()); // ensure postLut is ordered in ascending values of codedValues
    postLut[c] = initColourRemappingInfoLut(pCriSEI.m_colourRemapBitDepth, maxBitDepth - pCriSEI.m_colourRemapBitDepth, pCriSEI.m_postLut[c], (1 << internalBitDepth) - 1, 0); //Fill postLut
  }
}

// apply lut.
// Input lut values are aligned on power of 2 boundaries
static int
applyColourRemappingInfoLut1D(int inVal, const std::vector<int> &lut, const int inValPrecisionBits)
{
  const int roundValue = (inValPrecisionBits)? 1 << (inValPrecisionBits - 1): 0;
  inVal = std::min(std::max(0, inVal), (int)(((lut.size()-1) << inValPrecisionBits)));
  int index  = (int) std::min((inVal >> inValPrecisionBits), (int)(lut.size()-2));
  int outVal = (( inVal - (index<<inValPrecisionBits) ) * (lut[index+1] - lut[index]) + roundValue) >> inValPrecisionBits;
  outVal +=  lut[index] ;

  return outVal;
}

static int
applyColourRemappingInfoMatrix(const int (&colourRemapCoeffs)[3], const int postOffsetShift, const int p0, const int p1, const int p2, const int offset)
{
  int YUVMat = (colourRemapCoeffs[0]* p0 + colourRemapCoeffs[1]* p1 + colourRemapCoeffs[2]* p2  + offset) >> postOffsetShift;
  return YUVMat;
}

static void
setColourRemappingInfoMatrixOffset(int (&matrixOffset)[3], int offset0, int offset1, int offset2)
{
  matrixOffset[0] = offset0;
  matrixOffset[1] = offset1;
  matrixOffset[2] = offset2;
}

static void
setColourRemappingInfoMatrixOffsets(      int  (&matrixInputOffset)[3],
                                          int  (&matrixOutputOffset)[3],
                                    const int  bitDepth,
                                    const bool crInputFullRangeFlag,
                                    const int  crInputMatrixCoefficients,
                                    const bool crFullRangeFlag,
                                    const int  crMatrixCoefficients)
{
  // set static matrix offsets
  int crInputOffsetLuma = (crInputFullRangeFlag)? 0:-(16 << (bitDepth-8));
  int crOffsetLuma = (crFullRangeFlag)? 0:(16 << (bitDepth-8));
  int crInputOffsetChroma = 0;
  int crOffsetChroma = 0;

  switch(crInputMatrixCoefficients)
  {
    case MATRIX_COEFFICIENTS_RGB:
      crInputOffsetChroma = 0;
      if(!crInputFullRangeFlag)
      {
        msg( VTMERROR, "VTMWARNING: crInputMatrixCoefficients set to MATRIX_COEFFICIENTS_RGB and crInputFullRangeFlag not set\n");
        crInputOffsetLuma = 0;
      }
      break;
    case MATRIX_COEFFICIENTS_UNSPECIFIED:
    case MATRIX_COEFFICIENTS_BT709:
    case MATRIX_COEFFICIENTS_BT2020_NON_CONSTANT_LUMINANCE:
      crInputOffsetChroma = -(1 << (bitDepth-1));
      break;
    default:
      msg( VTMERROR, "VTMWARNING: crInputMatrixCoefficients set to undefined value: %d\n", crInputMatrixCoefficients);
  }

  switch(crMatrixCoefficients)
  {
    case MATRIX_COEFFICIENTS_RGB:
      crOffsetChroma = 0;
      if(!crFullRangeFlag)
      {
        msg( VTMERROR, "VTMWARNING: crMatrixCoefficients set to MATRIX_COEFFICIENTS_RGB and crInputFullRangeFlag not set\n");
        crOffsetLuma = 0;
      }
      break;
    case MATRIX_COEFFICIENTS_UNSPECIFIED:
    case MATRIX_COEFFICIENTS_BT709:
    case MATRIX_COEFFICIENTS_BT2020_NON_CONSTANT_LUMINANCE:
      crOffsetChroma = (1 << (bitDepth-1));
      break;
    default:
      msg( VTMERROR, "VTMWARNING: crMatrixCoefficients set to undefined value: %d\n", crMatrixCoefficients);
  }

  setColourRemappingInfoMatrixOffset(matrixInputOffset, crInputOffsetLuma, crInputOffsetChroma, crInputOffsetChroma);
  setColourRemappingInfoMatrixOffset(matrixOutputOffset, crOffsetLuma, crOffsetChroma, crOffsetChroma);
}

void applyColourRemapping(const PelUnitBuf& pic, SEIColourRemappingInfo& criSEI, const SPS &activeSPS, std::ofstream& outstream)
{
  const int maxBitDepth = 16;

  // create colour remapped picture
  if( !criSEI.m_colourRemapCancelFlag && pic.chromaFormat!=CHROMA_400) // 4:0:0 not supported.
  {
    PelBuf picY  = pic.Y();
    PelBuf picCb = pic.Cb();
    PelBuf picCr = pic.Cr();
    const int          iHeight         = picY.width;
    const int          iWidth          = picY.height;

    PelStorage cRemapped;
    cRemapped.create( pic.chromaFormat, Area( Position(), pic.Y()));

    const int  iStrideIn   = picY.stride;
    const int  iCStrideIn  = picCb.stride;

    PelBuf remapY  = cRemapped.Y();
    PelBuf remapCb = cRemapped.Cb();
    PelBuf remapCr = cRemapped.Cr();
    const int  iStrideOut  = remapY.stride;
    const int  iCStrideOut = remapCb.stride;
    const bool b444        = ( pic.chromaFormat == CHROMA_444 );
    const bool b422        = ( pic.chromaFormat == CHROMA_422 );
    const bool b420        = ( pic.chromaFormat == CHROMA_420 );

    std::vector<int> preLut[3];
    std::vector<int> postLut[3];
    int matrixInputOffset[3];
    int matrixOutputOffset[3];
    const Pel *YUVIn[MAX_NUM_COMPONENT];
    Pel *YUVOut[MAX_NUM_COMPONENT];
    YUVIn[COMPONENT_Y]  = picY.buf;
    YUVIn[COMPONENT_Cb] = picCb.buf;
    YUVIn[COMPONENT_Cr] = picCr.buf;
    YUVOut[COMPONENT_Y]  = remapY.buf;
    YUVOut[COMPONENT_Cb] = remapCb.buf;
    YUVOut[COMPONENT_Cr] = remapCr.buf;

    const int bitDepth = criSEI.m_colourRemapBitDepth;
//    BitDepths        bitDepthsCriFile;
//    bitDepthsCriFile.recon[CHANNEL_TYPE_LUMA]   = bitDepth;
//    bitDepthsCriFile.recon[CHANNEL_TYPE_CHROMA] = bitDepth; // Different bitdepth is not implemented

    const int postOffsetShift = criSEI.m_log2MatrixDenom;
    const int matrixRound = 1 << (postOffsetShift - 1);
    const int postLutInputPrecision = (maxBitDepth - criSEI.m_colourRemapBitDepth);

    if ( ! criSEI.m_colourRemapVideoSignalInfoPresentFlag ) // setting default
    {
      setColourRemappingInfoMatrixOffsets(matrixInputOffset, matrixOutputOffset, maxBitDepth,
          activeSPS.getVuiParameters()->getVideoFullRangeFlag(), activeSPS.getVuiParameters()->getMatrixCoefficients(),
          activeSPS.getVuiParameters()->getVideoFullRangeFlag(), activeSPS.getVuiParameters()->getMatrixCoefficients());
    }
    else
    {
      setColourRemappingInfoMatrixOffsets(matrixInputOffset, matrixOutputOffset, maxBitDepth,
          activeSPS.getVuiParameters()->getVideoFullRangeFlag(), activeSPS.getVuiParameters()->getMatrixCoefficients(),
          criSEI.m_colourRemapFullRangeFlag, criSEI.m_colourRemapMatrixCoefficients);
    }

    // add matrix rounding to output matrix offsets
    matrixOutputOffset[0] = (matrixOutputOffset[0] << postOffsetShift) + matrixRound;
    matrixOutputOffset[1] = (matrixOutputOffset[1] << postOffsetShift) + matrixRound;
    matrixOutputOffset[2] = (matrixOutputOffset[2] << postOffsetShift) + matrixRound;

    // Merge   matrixInputOffset and matrixOutputOffset to matrixOutputOffset
    matrixOutputOffset[0] += applyColourRemappingInfoMatrix(criSEI.m_colourRemapCoeffs[0], 0, matrixInputOffset[0], matrixInputOffset[1], matrixInputOffset[2], 0);
    matrixOutputOffset[1] += applyColourRemappingInfoMatrix(criSEI.m_colourRemapCoeffs[1], 0, matrixInputOffset[0], matrixInputOffset[1], matrixInputOffset[2], 0);
    matrixOutputOffset[2] += applyColourRemappingInfoMatrix(criSEI.m_colourRemapCoeffs[2], 0, matrixInputOffset[0], matrixInputOffset[1], matrixInputOffset[2], 0);

    // rescaling output: include CRI/output frame difference
    const int scaleShiftOut_neg = abs(bitDepth - maxBitDepth);
    const int scaleOut_round = 1 << (scaleShiftOut_neg-1);

    initColourRemappingInfoLuts(preLut, postLut, criSEI, maxBitDepth);

    VTMCHECK(pic.chromaFormat == CHROMA_400, "Chroma format (400) not supported");
    const int hs = ::getComponentScaleX(ComponentID(COMPONENT_Cb), pic.chromaFormat);
    const int maxOutputValue = (1 << bitDepth) - 1;

    for( int y = 0; y < iHeight; y++ )
    {
      for( int x = 0; x < iWidth; x++ )
      {
        const int xc = (x>>hs);
        bool computeChroma = b444 || ((b422 || !(y&1)) && !(x&1));

        int YUVPre_0 = applyColourRemappingInfoLut1D(YUVIn[COMPONENT_Y][x], preLut[0], 0);
        int YUVPre_1 = applyColourRemappingInfoLut1D(YUVIn[COMPONENT_Cb][xc], preLut[1], 0);
        int YUVPre_2 = applyColourRemappingInfoLut1D(YUVIn[COMPONENT_Cr][xc], preLut[2], 0);

        int YUVMat_0 = applyColourRemappingInfoMatrix(criSEI.m_colourRemapCoeffs[0], postOffsetShift, YUVPre_0, YUVPre_1, YUVPre_2, matrixOutputOffset[0]);
        int YUVLutB_0 = applyColourRemappingInfoLut1D(YUVMat_0, postLut[0], postLutInputPrecision);
        YUVOut[COMPONENT_Y][x] = std::min(maxOutputValue, (YUVLutB_0 + scaleOut_round) >> scaleShiftOut_neg);

        if( computeChroma )
        {
          int YUVMat_1 = applyColourRemappingInfoMatrix(criSEI.m_colourRemapCoeffs[1], postOffsetShift, YUVPre_0, YUVPre_1, YUVPre_2, matrixOutputOffset[1]);
          int YUVLutB_1 = applyColourRemappingInfoLut1D(YUVMat_1, postLut[1], postLutInputPrecision);
          YUVOut[COMPONENT_Cb][xc] = std::min(maxOutputValue, (YUVLutB_1 + scaleOut_round) >> scaleShiftOut_neg);

          int YUVMat_2 = applyColourRemappingInfoMatrix(criSEI.m_colourRemapCoeffs[2], postOffsetShift, YUVPre_0, YUVPre_1, YUVPre_2, matrixOutputOffset[2]);
          int YUVLutB_2 = applyColourRemappingInfoLut1D(YUVMat_2, postLut[2], postLutInputPrecision);
          YUVOut[COMPONENT_Cr][xc] = std::min(maxOutputValue, (YUVLutB_2 + scaleOut_round) >> scaleShiftOut_neg);
        }
      }

      YUVIn[COMPONENT_Y]  += iStrideIn;
      YUVOut[COMPONENT_Y] += iStrideOut;
      if( !(b420 && !(y&1)) )
      {
         YUVIn[COMPONENT_Cb]  += iCStrideIn;
         YUVIn[COMPONENT_Cr]  += iCStrideIn;
         YUVOut[COMPONENT_Cb] += iCStrideOut;
         YUVOut[COMPONENT_Cr] += iCStrideOut;
      }
    }
    //Write remapped picture in display order
    //find a write
//tbd     cRemapped.dump( outstream, bitDepthsCriFile, true );
    cRemapped.destroy();
  }
}

//! \}
