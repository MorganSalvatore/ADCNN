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

/** \file     Reshape.cpp
    \brief    common reshaper class
*/
#include "Reshape.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#if JVET_M0427_INLOOP_RESHAPER
 //! \ingroup CommonLib
 //! \{

 // ====================================================================================================================
 // Constructor / destructor / create / destroy
 // ====================================================================================================================

Reshape::Reshape()
{
  m_CTUFlag = false;
  m_recReshaped = false;
  m_reshape = true;
}

Reshape::~Reshape()
{
}

void  Reshape::createDec(int bitDepth)
{
  m_lumaBD = bitDepth;
  m_reshapeLUTSize = 1 << m_lumaBD;
  m_initCW = m_reshapeLUTSize / PIC_CODE_CW_BINS;
  if (m_fwdLUT.empty())
    m_fwdLUT.resize(m_reshapeLUTSize, 0);
  if (m_invLUT.empty())
    m_invLUT.resize(m_reshapeLUTSize, 0);
  if (m_binCW.empty())
    m_binCW.resize(PIC_CODE_CW_BINS, 0);
  if (m_reshapePivot.empty())
    m_reshapePivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (m_chromaAdjHelpLUT.empty())
    m_chromaAdjHelpLUT.resize(PIC_CODE_CW_BINS, 1<<CSCALE_FP_PREC);
}

void  Reshape::destroy()
{
}

/**
-Perform inverse of a one dimension LUT
\param   InputLUT  describing the input LUT
\retval  OutputLUT describing the inversed LUT of InputLUT
\param   lut_size  size of LUT in number of samples
*/
void Reshape::reverseLUT(std::vector<Pel>& inputLUT, std::vector<Pel>& outputLUT, uint16_t lutSize)
{
  int i, j;
  outputLUT[m_reshapePivot[m_sliceReshapeInfo.reshaperModelMinBinIdx]] = m_sliceReshapeInfo.reshaperModelMinBinIdx*m_initCW;
  for (i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
  {
    int16_t X1 = m_reshapePivot[i];
    int16_t X2 = m_reshapePivot[i + 1];
    outputLUT[X2] = (i + 1)*m_initCW;
    int16_t Y1 = outputLUT[X1];
    int16_t Y2 = outputLUT[X2];

    if (X2 !=X1)
    {
      int32_t scale = (int32_t)(Y2 - Y1) * (1 << FP_PREC) / (int32_t)(X2 - X1);
      for (j = X1 + 1; j < X2; j++)
      {
        outputLUT[j] = (Pel)((scale*(int32_t)(j - X1) + (1 << (FP_PREC - 1))) >> FP_PREC) + Y1;
      }
    }
  }

  for (i = 0; i < m_reshapePivot[m_sliceReshapeInfo.reshaperModelMinBinIdx]; i++)
    outputLUT[i] = outputLUT[m_reshapePivot[m_sliceReshapeInfo.reshaperModelMinBinIdx]];
  for (i = m_reshapePivot[m_sliceReshapeInfo.reshaperModelMaxBinIdx + 1]; i < m_reshapeLUTSize; i++)
    outputLUT[i] = outputLUT[m_reshapePivot[m_sliceReshapeInfo.reshaperModelMaxBinIdx + 1]];

  bool clipRange = ((m_sliceReshapeInfo.reshaperModelMinBinIdx > 0) && (m_sliceReshapeInfo.reshaperModelMaxBinIdx < (PIC_CODE_CW_BINS - 1)));
  for (i = 0; i < lutSize; i++)
  {
    if (clipRange) outputLUT[i] = Clip3((Pel)(16<<(m_lumaBD-8)), (Pel)(235<<(m_lumaBD-8)), outputLUT[i]);
    else           outputLUT[i] = Clip3((Pel)0, (Pel)((1<<m_lumaBD)-1), outputLUT[i]);
  }
}


/** compute chroma residuce scale for TU
* \param average luma pred of TU
* \return chroma residue scale
*/
int  Reshape::calculateChromaAdj(Pel avgLuma)
{
  int lumaIdx = Clip3<int>(0, (1<<m_lumaBD) - 1, avgLuma);
  int iAdj = m_chromaAdjHelpLUT[getPWLIdxInv(lumaIdx)];
  return(iAdj);
}


/** find inx of PWL for inverse mapping
* \param average luma pred of TU
* \return idx of PWL for inverse mapping
*/
int Reshape::getPWLIdxInv(int lumaVal)
{
  int idxS = 0;
  if (lumaVal < m_reshapePivot[m_sliceReshapeInfo.reshaperModelMinBinIdx + 1])
    return m_sliceReshapeInfo.reshaperModelMinBinIdx;
  else if (lumaVal >= m_reshapePivot[m_sliceReshapeInfo.reshaperModelMaxBinIdx])
    return m_sliceReshapeInfo.reshaperModelMaxBinIdx;
  else
  {
    for (idxS = m_sliceReshapeInfo.reshaperModelMinBinIdx; (idxS < m_sliceReshapeInfo.reshaperModelMaxBinIdx); idxS++)
    {
      if (lumaVal < m_reshapePivot[idxS + 1])     break;
    }
    return idxS;
  }
}

/**
-copy Slice reshaper info structure
\param   tInfo describing the target Slice reshaper info structure
\param   sInfo describing the source Slice reshaper info structure
*/
void Reshape::copySliceReshaperInfo(SliceReshapeInfo& tInfo, SliceReshapeInfo& sInfo)
{
  tInfo.sliceReshaperModelPresentFlag = sInfo.sliceReshaperModelPresentFlag;
  if (sInfo.sliceReshaperModelPresentFlag)
  {
    tInfo.reshaperModelMaxBinIdx = sInfo.reshaperModelMaxBinIdx;
    tInfo.reshaperModelMinBinIdx = sInfo.reshaperModelMinBinIdx;
    memcpy(tInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof(int)*(PIC_CODE_CW_BINS));
    tInfo.maxNbitsNeededDeltaCW = sInfo.maxNbitsNeededDeltaCW;
  }
  tInfo.sliceReshaperEnableFlag = sInfo.sliceReshaperEnableFlag;
  if (sInfo.sliceReshaperEnableFlag)
    tInfo.enableChromaAdj = sInfo.enableChromaAdj;
  else
    tInfo.enableChromaAdj = 0;
}

/** Construct reshaper from syntax
* \param void
* \return void
*/
void Reshape::constructReshaper()
{
  int pwlFwdLUTsize = PIC_CODE_CW_BINS;
  int pwlFwdBinLen = m_reshapeLUTSize / PIC_CODE_CW_BINS;

  for (int i = 0; i < m_sliceReshapeInfo.reshaperModelMinBinIdx; i++)
    m_binCW[i] = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMaxBinIdx + 1; i < PIC_CODE_CW_BINS; i++)
    m_binCW[i] = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
    m_binCW[i] = (uint16_t)(m_sliceReshapeInfo.reshaperModelBinCWDelta[i] + (int)m_initCW);

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_reshapePivot[i + 1] = m_reshapePivot[i] + m_binCW[i];
    int16_t Y1 = m_reshapePivot[i];
    int16_t Y2 = m_reshapePivot[i + 1];

    m_fwdLUT[i*pwlFwdBinLen] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)Y1);

    int log2PwlFwdBinLen = g_aucLog2[pwlFwdBinLen];

    int32_t scale = ((int32_t)(Y2 - Y1) * (1 << FP_PREC) + (1 << (log2PwlFwdBinLen - 1))) >> (log2PwlFwdBinLen);
    for (int j = 1; j < pwlFwdBinLen; j++)
    {
      int tempVal = Y1 + (((int32_t)scale * (int32_t)j + (1 << (FP_PREC - 1))) >> FP_PREC);
      m_fwdLUT[i*pwlFwdBinLen + j] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)tempVal);
    }
  }
  reverseLUT(m_fwdLUT, m_invLUT, m_reshapeLUTSize);
  updateChromaScaleLUT();
}

/** generate chroma residue scaling LUT
* \param void
* \return void
*/
void Reshape::updateChromaScaleLUT()
{
  const int16_t  CW_bin_SC_LUT[2 * PIC_ANALYZE_CW_BINS] = { 16384, 16384, 16384, 16384, 16384, 16384, 16384, 8192, 8192, 8192, 8192, 5461, 5461, 5461, 5461, 4096, 4096, 4096, 4096, 3277, 3277, 3277, 3277, 2731, 2731, 2731, 2731, 2341, 2341, 2341, 2048, 2048, 2048, 1820, 1820, 1820, 1638, 1638, 1638, 1638, 1489, 1489, 1489, 1489, 1365, 1365, 1365, 1365, 1260, 1260, 1260, 1260, 1170, 1170, 1170, 1170, 1092, 1092, 1092, 1092, 1024, 1024, 1024, 1024 }; //p=11
  for (int i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    uint16_t binCW = m_lumaBD > 10 ? (m_binCW[i] >> (m_lumaBD - 10)) : m_lumaBD < 10 ? (m_binCW[i] << (10 -m_lumaBD)): m_binCW[i];
    if ((i < m_sliceReshapeInfo.reshaperModelMinBinIdx) || (i > m_sliceReshapeInfo.reshaperModelMaxBinIdx))
      m_chromaAdjHelpLUT[i] = 1 << CSCALE_FP_PREC;
    else
      m_chromaAdjHelpLUT[i] = CW_bin_SC_LUT[Clip3((uint16_t)1, (uint16_t)64, (uint16_t)(binCW >> 1)) - 1];
  }
}
#endif


//
//! \}
