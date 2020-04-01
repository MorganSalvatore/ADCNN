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

/** \file     EncReshape.cpp
\brief    encoder reshaper class
*/
#include "EncReshape.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#if JVET_M0427_INLOOP_RESHAPER
//! \ingroup EncLib
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncReshape::EncReshape()
{
  m_CTUFlag      = false;
  m_srcReshaped  = false;
  m_recReshaped  = false;
  m_reshape      = true;
  m_exceedSTD    = false;
  m_tcase        = 0;
  m_rateAdpMode  = 0;
  m_chromaAdj    = 0;
}

EncReshape::~EncReshape()
{
}

void  EncReshape::createEnc(int picWidth, int picHeight, uint32_t maxCUWidth, uint32_t maxCUHeight, int bitDepth)
{
  m_lumaBD = bitDepth;
  m_reshapeLUTSize = 1 << m_lumaBD;
  m_initCWAnalyze = m_reshapeLUTSize / PIC_ANALYZE_CW_BINS;
  m_initCW = m_reshapeLUTSize / PIC_CODE_CW_BINS;

  if (m_fwdLUT.empty())
    m_fwdLUT.resize(m_reshapeLUTSize, 0);
  if (m_invLUT.empty())
    m_invLUT.resize(m_reshapeLUTSize,0);
  if (m_binCW.empty())
    m_binCW.resize(PIC_ANALYZE_CW_BINS);
  if (m_binImportance.empty())
    m_binImportance.resize(PIC_ANALYZE_CW_BINS);
  if (m_reshapePivot.empty())
    m_reshapePivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (m_chromaAdjHelpLUT.empty())
    m_chromaAdjHelpLUT.resize(PIC_CODE_CW_BINS, 1<<CSCALE_FP_PREC);

  m_sliceReshapeInfo.setUseSliceReshaper(true);
  m_sliceReshapeInfo.setSliceReshapeChromaAdj(true);
  m_sliceReshapeInfo.setSliceReshapeModelPresentFlag(true);
  m_sliceReshapeInfo.reshaperModelMinBinIdx = 0;
  m_sliceReshapeInfo.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1;
  memset(m_sliceReshapeInfo.reshaperModelBinCWDelta, 0, (PIC_CODE_CW_BINS) * sizeof(int));

  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_maxCUWidth = maxCUWidth;
  m_maxCUHeight = maxCUHeight;
  m_widthInCtus = (m_picWidth + m_maxCUWidth - 1) / m_maxCUWidth;
  m_heightInCtus = (m_picHeight + m_maxCUHeight - 1) / m_maxCUHeight;
  m_numCtuInFrame = m_widthInCtus * m_heightInCtus;
}

void  EncReshape::destroy()
{
}

/**
-Perform HDR set up
\param   pcPic describe pointer of current coding picture
\param   sliceType describe the slice type
*/
#if JVET_M0483_IBC
void EncReshape::preAnalyzerHDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT)
#else
void EncReshape::preAnalyzerHDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT, bool isIBC)
#endif
{
  if (m_lumaBD >= 10)
  {
    m_sliceReshapeInfo.sliceReshaperEnableFlag = true;
    if (reshapeCW.rspIntraPeriod == 1)
    {
      if (pcPic->getPOC() == 0)          { m_sliceReshapeInfo.sliceReshaperModelPresentFlag = true;  }
      else                               { m_sliceReshapeInfo.sliceReshaperModelPresentFlag = false; }
    }
    else
    {
#if JVET_M0483_IBC
      if (sliceType == I_SLICE )                                              { m_sliceReshapeInfo.sliceReshaperModelPresentFlag = true;  }
#else
      if (sliceType == I_SLICE || (sliceType == P_SLICE && isIBC))            { m_sliceReshapeInfo.sliceReshaperModelPresentFlag = true;  }
#endif
      else                                                                    { m_sliceReshapeInfo.sliceReshaperModelPresentFlag = false; }
    }
#if JVET_M0483_IBC
    if (sliceType == I_SLICE  && isDualT)                                     { m_sliceReshapeInfo.enableChromaAdj = 0;                   }
#else
    if ((sliceType == I_SLICE || (sliceType == P_SLICE && isIBC)) && isDualT) { m_sliceReshapeInfo.enableChromaAdj = 0;                   }
#endif
    else                                                                      { m_sliceReshapeInfo.enableChromaAdj = 1;                   }
  }
  else
  {
    m_sliceReshapeInfo.sliceReshaperEnableFlag = false;
    m_sliceReshapeInfo.sliceReshaperModelPresentFlag = false;
  }
}

/**
-Perform picture analysis for SDR
\param   pcPic describe pointer of current coding picture
\param   sliceType describe the slice type
\param   reshapeCW describe some input info
*/
#if JVET_M0483_IBC
void EncReshape::preAnalyzerSDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT)
#else
void EncReshape::preAnalyzerSDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT, bool isIBC)
#endif
{
  m_sliceReshapeInfo.sliceReshaperModelPresentFlag = true;
  m_sliceReshapeInfo.sliceReshaperEnableFlag = true;

  int modIP = pcPic->getPOC() - pcPic->getPOC() / reshapeCW.rspFpsToIp * reshapeCW.rspFpsToIp;
#if JVET_M0483_IBC
  if (sliceType == I_SLICE || (reshapeCW.rspIntraPeriod == -1 && modIP == 0))
#else
  if (sliceType == I_SLICE || (reshapeCW.rspIntraPeriod == -1 && modIP == 0) || (sliceType == P_SLICE && isIBC))
#endif
  {
    if (m_sliceReshapeInfo.sliceReshaperModelPresentFlag == true)
    {
      int stdMin = 16 <<(m_lumaBD-8);
      int stdMax = 235 << (m_lumaBD - 8);
      int  binLen = m_reshapeLUTSize / PIC_ANALYZE_CW_BINS;

      m_reshapeCW = reshapeCW;
      m_initCWAnalyze = binLen;

      for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
      {
        m_binImportance[b] = 0;
        m_binCW[b] = binLen;
      }

      int startBinIdx = stdMin / binLen;
      int endBinIdx = stdMax / binLen;
      m_sliceReshapeInfo.reshaperModelMinBinIdx = startBinIdx;
      m_sliceReshapeInfo.reshaperModelMaxBinIdx = endBinIdx;

      PelBuf picY = pcPic->getOrigBuf(COMPONENT_Y);
      const int width = picY.width;
      const int height = picY.height;
      const int stride = picY.stride;

      double blockBinVarSum[PIC_ANALYZE_CW_BINS] = { 0.0 };
      uint32_t   bockBinCnt[PIC_ANALYZE_CW_BINS] = { 0 };

      const int PIC_ANALYZE_WIN_SIZE = 5;
      const uint32_t winSize = PIC_ANALYZE_WIN_SIZE;
      const uint32_t winLens = (winSize - 1) >> 1;

      int64_t tempSq = 0;
      int64_t leftSum = 0, leftSumSq = 0;
      int64_t *leftColSum = new int64_t[width];
      int64_t *leftColSumSq = new int64_t[width];
      memset(leftColSum, 0, width * sizeof(int64_t));
      memset(leftColSumSq, 0, width * sizeof(int64_t));
      int64_t topSum = 0, topSumSq = 0;
      int64_t *topRowSum = new int64_t[height];
      int64_t *topRowSumSq = new int64_t[height];
      memset(topRowSum, 0, height * sizeof(int64_t));
      memset(topRowSumSq, 0, height * sizeof(int64_t));
      int64_t *topColSum = new int64_t[width];
      int64_t *topColSumSq = new int64_t[width];
      memset(topColSum, 0, width * sizeof(int64_t));
      memset(topColSumSq, 0, width * sizeof(int64_t));

      for (uint32_t y = 0; y < height; y++)
      {
        for (uint32_t x = 0; x < width; x++)
        {
          const Pel pxlY = picY.buf[x];
          int64_t sum = 0;
          int64_t sumSq = 0;
          uint32_t numPixInPart = 0;

          uint32_t y1 = std::max((int)(y - winLens), 0);
          uint32_t y2 = std::min((int)(y + winLens), (height - 1));
          uint32_t x1 = std::max((int)(x - winLens), 0);
          uint32_t x2 = std::min((int)(x + winLens), (width - 1));


          uint32_t bx = 0, by = 0;
          const Pel *pWinY = &picY.buf[0];
          numPixInPart = (x2 - x1 + 1) * (y2 - y1 + 1);

          if (x == 0 && y == 0)           // for the 1st Pixel, calc all points
          {
            for (by = y1; by <= y2; by++)
            {
              for (bx = x1; bx <= x2; bx++)
              {
                tempSq = pWinY[bx] * pWinY[bx];
                leftSum += pWinY[bx];
                leftSumSq += tempSq;
                leftColSum[bx] += pWinY[bx];
                leftColSumSq[bx] += tempSq;
                topColSum[bx] += pWinY[bx];
                topColSumSq[bx] += tempSq;
                topRowSum[by] += pWinY[bx];
                topRowSumSq[by] += tempSq;
              }
              pWinY += stride;
            }
            topSum = leftSum;
            topSumSq = leftSumSq;
            sum = leftSum;
            sumSq = leftSumSq;
          }
          else if (x == 0 && y > 0)       // for the 1st column, calc the bottom stripe
          {
            if (y < height - winLens)
            {
              pWinY += winLens*stride;
              topRowSum[y + winLens] = 0;
              topRowSumSq[y + winLens] = 0;
              for (bx = x1; bx <= x2; bx++)
              {
                topRowSum[y + winLens] += pWinY[bx];
                topRowSumSq[y + winLens] += pWinY[bx] * pWinY[bx];
              }
              topSum += topRowSum[y + winLens];
              topSumSq += topRowSumSq[y + winLens];
            }
            if (y > winLens)
            {
              topSum -= topRowSum[y - 1 - winLens];
              topSumSq -= topRowSumSq[y - 1 - winLens];
            }

            memset(leftColSum, 0, width * sizeof(int64_t));
            memset(leftColSumSq, 0, width * sizeof(int64_t));
            pWinY = &picY.buf[0];
            pWinY -= (y <= winLens ? y : winLens)*stride;
            for (by = y1; by <= y2; by++)
            {
              for (bx = x1; bx <= x2; bx++)
              {
                leftColSum[bx] += pWinY[bx];
                leftColSumSq[bx] += pWinY[bx] * pWinY[bx];
              }
              pWinY += stride;
            }

            leftSum = topSum;
            leftSumSq = topSumSq;
            sum = topSum;
            sumSq = topSumSq;
          }

          else if (x > 0)
          {
            if (x < width - winLens)
            {
              pWinY -= (y <= winLens ? y : winLens)*stride;
              if (y == 0)                 // for the 1st row, calc the right stripe
              {
                leftColSum[x + winLens] = 0;
                leftColSumSq[x + winLens] = 0;
                for (by = y1; by <= y2; by++)
                {
                  leftColSum[x + winLens] += pWinY[x + winLens];
                  leftColSumSq[x + winLens] += pWinY[x + winLens] * pWinY[x + winLens];
                  pWinY += stride;
                }
              }
              else                        // for the main area, calc the B-R point
              {
                leftColSum[x + winLens] = topColSum[x + winLens];
                leftColSumSq[x + winLens] = topColSumSq[x + winLens];
                if (y < height - winLens)
                {
                  pWinY = &picY.buf[0];
                  pWinY += winLens * stride;
                  leftColSum[x + winLens] += pWinY[x + winLens];
                  leftColSumSq[x + winLens] += pWinY[x + winLens] * pWinY[x + winLens];
                }
                if (y > winLens)
                {
                  pWinY = &picY.buf[0];
                  pWinY -= (winLens + 1) * stride;
                  leftColSum[x + winLens] -= pWinY[x + winLens];
                  leftColSumSq[x + winLens] -= pWinY[x + winLens] * pWinY[x + winLens];
                }
              }
              topColSum[x + winLens] = leftColSum[x + winLens];
              topColSumSq[x + winLens] = leftColSumSq[x + winLens];
              leftSum += leftColSum[x + winLens];
              leftSumSq += leftColSumSq[x + winLens];
            }
            if (x > winLens)
            {
              leftSum -= leftColSum[x - 1 - winLens];
              leftSumSq -= leftColSumSq[x - 1 - winLens];
            }
            sum = leftSum;
            sumSq = leftSumSq;
          }

          double average = double(sum) / numPixInPart;
          double variance = double(sumSq) / numPixInPart - average * average;
          uint32_t binNum = (uint32_t)(pxlY/PIC_ANALYZE_CW_BINS);

          if (m_lumaBD > 10)
          {
            average = average / (double)(1<<(m_lumaBD - 10));
            variance = variance / (double)(1 << (2*m_lumaBD - 20));
            binNum = (uint32_t)((pxlY>>(m_lumaBD - 10)) / PIC_ANALYZE_CW_BINS);
          }
          else if (m_lumaBD < 10)
          {
            average = average * (double)(1 << (10 - m_lumaBD));
            variance = variance * (double)(1 << (20-2*m_lumaBD));
            binNum = (uint32_t)((pxlY << (10 - m_lumaBD)) / PIC_ANALYZE_CW_BINS);
          }
          double varLog10 = log10(variance + 1.0);
          blockBinVarSum[binNum] += varLog10;
          bockBinCnt[binNum]++;
        }
        picY.buf += stride;
      }

      delete[] topColSum;
      delete[] topColSumSq;
      delete[] topRowSum;
      delete[] topRowSumSq;
      delete[] leftColSum;
      delete[] leftColSumSq;

      for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
      {
        if (bockBinCnt[b] > 0)
          blockBinVarSum[b] = blockBinVarSum[b] / bockBinCnt[b];
      }

      m_reshape = true;
      m_exceedSTD = false;
      m_useAdpCW = false;
      m_chromaWeight = 1.0;
      m_sliceReshapeInfo.enableChromaAdj = 1;
      bool   intraAdp = false;
      bool   interAdp = true;
      double reshapeTH1 = 0.0;
      double reshapeTH2 = 5.0;
      deriveReshapeParametersSDRfromStats(bockBinCnt, blockBinVarSum, &reshapeTH1, &reshapeTH2, &intraAdp, &interAdp);

      if (m_rateAdpMode == 2 && reshapeCW.rspBaseQP <= 22)
      {
        intraAdp = false;
        interAdp = false;
      }

      m_sliceReshapeInfo.sliceReshaperEnableFlag = intraAdp;

      if (!intraAdp && !interAdp)
      {
        m_sliceReshapeInfo.sliceReshaperModelPresentFlag = false;
        m_reshape = false;
        return;
      }

      if (m_exceedSTD)
      {
        startBinIdx = 2;
        endBinIdx = 29;
        for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
        {
          if (bockBinCnt[b] > 0 && b < startBinIdx)
            startBinIdx = b;
          if (bockBinCnt[b] > 0 && b > endBinIdx)
            endBinIdx = b;
        }
        m_sliceReshapeInfo.reshaperModelMinBinIdx = startBinIdx;
        m_sliceReshapeInfo.reshaperModelMaxBinIdx = endBinIdx;
      }

      m_initCWAnalyze = m_lumaBD > 10 ? (m_initCWAnalyze >> (m_lumaBD - 10)) : m_lumaBD < 10 ? (m_initCWAnalyze << (10 - m_lumaBD)) : m_initCWAnalyze;
      if (reshapeCW.rspBaseQP <= 22 && m_rateAdpMode == 1)
      {
        for (int i = 0; i < PIC_ANALYZE_CW_BINS; i++)
        {
          if (i >= startBinIdx && i <= endBinIdx)
            m_binCW[i] = m_initCWAnalyze + 1;
          else
            m_binCW[i] = 0;
        }
      }
      else if (m_useAdpCW)
      {
        double Alpha = 1.0, Beta = 0.0;
        deriveReshapeParameters(blockBinVarSum, startBinIdx, endBinIdx, m_reshapeCW, Alpha, Beta);
        for (int i = 0; i < PIC_ANALYZE_CW_BINS; i++)
        {
          if (i >= startBinIdx && i <= endBinIdx)
            m_binCW[i] = (uint32_t)round(Alpha*blockBinVarSum[i] + Beta);
          else
            m_binCW[i] = 0;
        }
      }
      else
      {
        for (int b = startBinIdx; b <= endBinIdx; b++)
        {
          if (blockBinVarSum[b] < reshapeTH1)
            m_binImportance[b] = 2;
          else if (blockBinVarSum[b] > reshapeTH2)
            m_binImportance[b] = 3;
          else
            m_binImportance[b] = 1;
        }

        for (int i = 0; i < PIC_ANALYZE_CW_BINS; i++)
        {
          if (m_binImportance[i] == 0)
            m_binCW[i] = 0;
          else if (m_binImportance[i] == 1)
            m_binCW[i] = m_initCWAnalyze + 1;
          else if (m_binImportance[i] == 2)
            m_binCW[i] = m_reshapeCW.binCW[0];
          else if (m_binImportance[i] == 3)
            m_binCW[i] = m_reshapeCW.binCW[1];
          else
            THROW("SDR Reshape Bin Importance not supported");
        }
      }
      if (m_reshapeCW.rspPicSize <= 1497600 && reshapeCW.rspIntraPeriod == -1 && modIP == 0 && sliceType != I_SLICE)
      {
        m_sliceReshapeInfo.sliceReshaperEnableFlag = false;
      }

    }
    m_chromaAdj = m_sliceReshapeInfo.enableChromaAdj;
#if JVET_M0483_IBC
    if (sliceType == I_SLICE && isDualT)
#else
    if ((sliceType == I_SLICE || (sliceType == P_SLICE && isIBC)) && isDualT)
#endif
    {
        m_sliceReshapeInfo.enableChromaAdj = 0;
    }
  }
  else // Inter slices
  {
    m_sliceReshapeInfo.sliceReshaperModelPresentFlag = false;
    m_sliceReshapeInfo.enableChromaAdj = m_chromaAdj;

    if (!m_reshape)
    {
      m_sliceReshapeInfo.sliceReshaperEnableFlag = false;
    }
    else
    {
      const int cTid = m_reshapeCW.rspTid;
      bool enableRsp = m_tcase == 5 ? false : (m_tcase < 5 ? (cTid < m_tcase + 1 ? false : true) : (cTid <= 10 - m_tcase ? true : false));
      m_sliceReshapeInfo.sliceReshaperEnableFlag = enableRsp;
    }
  }
}

// Bubble Sort to  descending order with index
void EncReshape::bubbleSortDsd(double* array, int * idx, int n)
{
  int i, j;
  bool swapped;
  for (i = 0; i < n - 1; i++)
  {
    swapped = false;
    for (j = 0; j < n - i - 1; j++)
    {
      if (array[j] < array[j + 1])
      {
        swap(&array[j], &array[j + 1]);
        swap(&idx[j], &idx[j + 1]);
        swapped = true;
      }
    }
    if (swapped == false)
      break;
  }
}

void EncReshape::deriveReshapeParametersSDRfromStats(uint32_t * blockBinCnt, double *blockBinVarSum, double* reshapeTH1, double* reshapeTH2, bool *intraAdp, bool *interAdp)
{
  int    binIdxSortDsd[PIC_ANALYZE_CW_BINS]    = { 0 };
  double binVarSortDsd[PIC_ANALYZE_CW_BINS]    = { 0.0 };
  double binHist[PIC_ANALYZE_CW_BINS]          = { 0.0 };
  double binVarSortDsdCDF[PIC_ANALYZE_CW_BINS] = { 0.0 };
  double maxBinVar = 0.0, meanBinVar = 0.0, minBinVar = 5.0;
  int    nonZeroBinCt = 0;
  int    firstBinVarLessThanVal1 = 0;
  int    firstBinVarLessThanVal2 = 0;
  int    firstBinVarLessThanVal3 = 0;
  int    firstBinVarLessThanVal4 = 0;

  for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
  {
    binHist[b] = (double) blockBinCnt[b] / (double)(m_reshapeCW.rspPicSize);
    if (binHist[b] > 0.001)
    {
      nonZeroBinCt++;
      meanBinVar += blockBinVarSum[b];
      if (blockBinVarSum[b] > maxBinVar)        {        maxBinVar = blockBinVarSum[b];      }
      if (blockBinVarSum[b] < minBinVar)        {        minBinVar = blockBinVarSum[b];      }
    }
    binVarSortDsd[b] = blockBinVarSum[b];
    binIdxSortDsd[b] = b;
  }
  if ((binHist[0] + binHist[1] + binHist[PIC_ANALYZE_CW_BINS - 2] + binHist[PIC_ANALYZE_CW_BINS - 1]) > 0.01)   {    m_exceedSTD = true;  }
  if ((binHist[PIC_ANALYZE_CW_BINS - 2] + binHist[PIC_ANALYZE_CW_BINS - 1]) > 0.01)   {    *interAdp = false;    return;   }
  else                                                                                {    *interAdp = true;               }

  meanBinVar = meanBinVar / (double)nonZeroBinCt;
  bubbleSortDsd(binVarSortDsd, binIdxSortDsd, PIC_ANALYZE_CW_BINS);
  binVarSortDsdCDF[0] = binHist[binIdxSortDsd[0]];

  for (int b = 1; b < PIC_ANALYZE_CW_BINS; b++)
  {
    binVarSortDsdCDF[b] = binVarSortDsdCDF[b - 1] + binHist[binIdxSortDsd[b]];
  }

  for (int b = 0; b < PIC_ANALYZE_CW_BINS - 1; b++)
  {
    if (binVarSortDsd[b] > 3.5)     {      firstBinVarLessThanVal1 = b + 1;    }
    if (binVarSortDsd[b] > 3.0)     {      firstBinVarLessThanVal2 = b + 1;    }
    if (binVarSortDsd[b] > 2.5)     {      firstBinVarLessThanVal3 = b + 1;    }
    if (binVarSortDsd[b] > 2.0)     {      firstBinVarLessThanVal4 = b + 1;    }
  }

  m_reshapeCW.binCW[0] = 38;
  m_reshapeCW.binCW[1] = 28;

  if (m_reshapeCW.rspIntraPeriod == -1)
  {
    *intraAdp = true;
    if (m_reshapeCW.rspPicSize > 1497600)
    {
      m_reshapeCW.binCW[0] = 36;
      *reshapeTH1 = 2.4;
      *reshapeTH2 = 4.5;
      m_rateAdpMode = 2;

      if (meanBinVar >= 2.52)
      {
        if (binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5)
        {
          *reshapeTH1 = 2.5;
          *reshapeTH2 = 3.0;
        }
        else if (binVarSortDsdCDF[firstBinVarLessThanVal2] < 0.1 && binVarSortDsdCDF[firstBinVarLessThanVal1] > 0.02)
        {
          *reshapeTH1 = 2.2;
        }
        else if (binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.25)
        {
          m_reshapeCW.binCW[1] = 30;
          *reshapeTH1 = 2.0;
          m_rateAdpMode = 0;
        }
        else
        {
          m_reshapeCW.binCW[1] = 30;
          m_rateAdpMode = 1;
        }
      }
    }
    else if (m_reshapeCW.rspPicSize > 660480)
    {
      m_reshapeCW.binCW[0] = 34;
      *reshapeTH1 = 3.4;
      *reshapeTH2 = 4.0;
      m_rateAdpMode = 2;

      if (binVarSortDsdCDF[firstBinVarLessThanVal4] > 0.6)
      {
        if (maxBinVar < 3.5)
        {
          m_useAdpCW = true;
          m_reshapeCW.binCW[0] = 38;
        }
        else
        {
          m_reshapeCW.binCW[0] = 40;
          *reshapeTH1 = 2.2;
          *reshapeTH2 = 4.5;
          m_rateAdpMode = 0;
        }
      }
      else
      {
        if (maxBinVar > 3.3)
        {
          m_reshapeCW.binCW[1] = 30;
        }
        else
        {
          m_reshapeCW.binCW[1] = 28;
        }
      }
    }
    else if (m_reshapeCW.rspPicSize > 249600)
    {
      m_reshapeCW.binCW[0] = 36;
      *reshapeTH1 = 2.5;
      *reshapeTH2 = 4.5;

      if (m_exceedSTD)
      {
        m_reshapeCW.binCW[0] = 36;
        m_reshapeCW.binCW[1] = 30;
      }
      if (minBinVar > 2.6)
      {
        *reshapeTH1 = 3.0;
      }
      else {
        double diff1 = binVarSortDsdCDF[firstBinVarLessThanVal4] - binVarSortDsdCDF[firstBinVarLessThanVal3];
        double diff2 = binVarSortDsdCDF[firstBinVarLessThanVal2] - binVarSortDsdCDF[firstBinVarLessThanVal1];
        if (diff1 > 0.4 || binVarSortDsdCDF[firstBinVarLessThanVal1] > 0.1)
        {
          m_useAdpCW = true;
          m_rateAdpMode = 1;
        }
        else if (diff2 <= 0.1 && binVarSortDsdCDF[firstBinVarLessThanVal4] > 0.99 && binVarSortDsdCDF[firstBinVarLessThanVal3] > 0.642 && binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.03)
        {
          m_useAdpCW = true;
          m_rateAdpMode = 1;
        }
        else
        {
          m_rateAdpMode = 2;
        }
      }
    }
    else
    {
      m_reshapeCW.binCW[0] = 36;
      *reshapeTH1 = 2.6;
      *reshapeTH2 = 4.5;

      if (binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5 && maxBinVar < 4.7)
      {
        *reshapeTH1 = 3.2;
        m_rateAdpMode = 1;
      }
    }
  }
  else if (m_reshapeCW.rspIntraPeriod == 1)
  {
    *intraAdp = true;
    if (m_reshapeCW.rspPicSize > 5184000)
    {
      *reshapeTH1 = 2.0;
      *reshapeTH2 = 3.0;
      m_rateAdpMode = 2;

      if (maxBinVar > 2.4)
      {
        if (binVarSortDsdCDF[firstBinVarLessThanVal4] > 0.88)
        {
          if (maxBinVar < 2.695)
          {
            *reshapeTH2 = 2.2;
          }
          else
          {
            if (binVarSortDsdCDF[firstBinVarLessThanVal3] < 0.45)
            {
              *reshapeTH1 = 2.5;
              *reshapeTH2 = 4.0;
              m_reshapeCW.binCW[0] = 36;
              m_sliceReshapeInfo.enableChromaAdj = 0;
              m_rateAdpMode = 0;
            }
            else
            {
              m_useAdpCW = true;
              m_reshapeCW.binCW[0] = 36;
              m_reshapeCW.binCW[1] = 30;
            }
          }
        }
        else
        {
          if (maxBinVar > 2.8)
          {
            *reshapeTH1 = 2.2;
            *reshapeTH2 = 4.0;
            m_reshapeCW.binCW[0] = 36;
            m_sliceReshapeInfo.enableChromaAdj = 0;
          }
          else
          {
            m_useAdpCW = true;
            m_reshapeCW.binCW[0] = 38;
            m_reshapeCW.binCW[1] = 28;
          }
        }
      }
      else
      {
        if (maxBinVar > 2.24)
        {
          m_useAdpCW = true;
          m_reshapeCW.binCW[0] = 34;
          m_reshapeCW.binCW[1] = 30;
        }
      }
    }
    else if (m_reshapeCW.rspPicSize > 1497600)
    {
      *reshapeTH1 = 2.0;
      *reshapeTH2 = 4.5;
      m_rateAdpMode = 2;

      if (binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.25)
      {
        int firstVarCDFLargerThanVal = 1;
        for (int b = 0; b < PIC_ANALYZE_CW_BINS; b++)
        {
          if (binVarSortDsdCDF[b] > 0.7)
          {
            firstVarCDFLargerThanVal = b;
            break;
          }
        }
        if (meanBinVar < 2.52 || binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5)
        {
          *reshapeTH1 = 2.2;
          *reshapeTH2 = (binVarSortDsd[firstVarCDFLargerThanVal] + binVarSortDsd[firstVarCDFLargerThanVal - 1]) / 2.0;
        }
        else
        {
          m_reshapeCW.binCW[1] = 30;
          *reshapeTH2 = 2.8;
        }
      }
      else if (binVarSortDsdCDF[firstBinVarLessThanVal2] < 0.1 && binVarSortDsdCDF[firstBinVarLessThanVal1] > 0.02)
      {
        m_reshapeCW.binCW[0] = 36;
        *reshapeTH1 = 3.5;
        m_rateAdpMode = 1;
      }
    }
    else if (m_reshapeCW.rspPicSize > 660480)
    {
      *reshapeTH1 = 2.5;
      *reshapeTH2 = 4.5;
      m_rateAdpMode = 1;

      if (binVarSortDsdCDF[firstBinVarLessThanVal4] > 0.6)
      {
        if (maxBinVar < 3.5)
        {
          *reshapeTH1 = 2.0;
        }
      }
      else
      {
        if (maxBinVar > 3.3)
        {
          m_reshapeCW.binCW[0] = 35;
        }
        else
        {
          *reshapeTH1 = 2.8;
          m_reshapeCW.binCW[0] = 35;
        }
      }
    }
    else if (m_reshapeCW.rspPicSize > 249600)
    {
      m_rateAdpMode = 1;
      m_reshapeCW.binCW[0] = 36;
      *reshapeTH1 = 2.5;
      *reshapeTH2 = 4.5;
    }
    else
    {
      if (binVarSortDsdCDF[firstBinVarLessThanVal2] < 0.33 && m_reshapeCW.rspFps>40)
      {
        *intraAdp = false;
        *interAdp = false;
      }
      else
      {
        m_rateAdpMode = 1;
        m_reshapeCW.binCW[0] = 36;
        *reshapeTH1 = 3.0;
        *reshapeTH2 = 4.0;
      }
    }
  }
  else
  {
    if (m_reshapeCW.rspPicSize > 5184000)
    {
      m_reshapeCW.binCW[0] = 40;
      *reshapeTH2 = 4.0;
      m_rateAdpMode = 2;

      if (maxBinVar < 2.4)
      {
        *reshapeTH1 = 3.0;
        if (m_reshapeCW.rspBaseQP <= 22)
          m_tcase = 3;
      }
      else if (maxBinVar > 3.0)
      {
        if (minBinVar > 1)
        {
          m_reshapeCW.binCW[0] = 36;
          *reshapeTH1 = 2.8;
          *reshapeTH2 = 3.5;
          m_sliceReshapeInfo.enableChromaAdj = 0;
          m_chromaWeight = 1.05;
          m_rateAdpMode = 0;
        }
        else
        {
          m_reshapeCW.binCW[0] = 36;
          *reshapeTH1 = 2.2;
          *reshapeTH2 = 3.5;
          m_sliceReshapeInfo.enableChromaAdj = 0;
          m_chromaWeight = 0.95;
        }
      }
      else
      {
        *reshapeTH1 = 1.5;
      }
    }
    else if (m_reshapeCW.rspPicSize > 1497600)
    {
      *reshapeTH1 = 2.5;
      *reshapeTH2 = 4.5;
      m_rateAdpMode = 1;

      if (meanBinVar < 2.52)
      {
        *intraAdp = true;
        m_rateAdpMode = 0;
        m_tcase = 9;
      }
      else
      {
        if (binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5)
        {
          *reshapeTH2 = 3.0;
          *intraAdp = true;
        }
        else if (binVarSortDsdCDF[firstBinVarLessThanVal2] < 0.1 && binVarSortDsdCDF[firstBinVarLessThanVal1] > 0.02)
        {
          *reshapeTH1 = 3.0;
          *intraAdp = true;
          m_rateAdpMode = 0;
          m_tcase = 9;
        }
        else if (binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.25)
        {
          *reshapeTH1 = 2.4;
          m_reshapeCW.binCW[0] = 36;
        }
        else
        {
          *reshapeTH1 = 2.4;
          m_reshapeCW.binCW[0] = 36;
        }
      }
    }
    else if (m_reshapeCW.rspPicSize > 660480)
    {
      *intraAdp = true;
      m_rateAdpMode = 1;

      if (binVarSortDsdCDF[firstBinVarLessThanVal4] > 0.6)
      {
        if (maxBinVar < 3.5)
        {
          *reshapeTH1 = 2.1;
          *reshapeTH2 = 3.5;
        }
        else
        {
          *reshapeTH1 = 2.4;
          *reshapeTH2 = 4.5;
          m_reshapeCW.binCW[0] = 40;
          m_rateAdpMode = 0;
        }
      }
      else
      {
        if (maxBinVar > 3.3)
        {
          *reshapeTH1 = 3.5;
          *reshapeTH2 = 3.8;
        }
        else
        {
          *reshapeTH1 = 3.0;
          *reshapeTH2 = 4.0;
          m_reshapeCW.binCW[1] = 30;
        }
      }
    }
    else if (m_reshapeCW.rspPicSize > 249600)
    {
      m_reshapeCW.binCW[1] = 30;
      *reshapeTH1 = 2.5;
      *reshapeTH2 = 4.5;
      *intraAdp = true;
      m_rateAdpMode = 1;

      if (minBinVar > 2.6)
      {
        *reshapeTH1 = 3.2;
        m_rateAdpMode = 0;
        m_tcase = 9;
      }
      else {
        double diff1 = binVarSortDsdCDF[firstBinVarLessThanVal4] - binVarSortDsdCDF[firstBinVarLessThanVal3];
        double diff2 = binVarSortDsdCDF[firstBinVarLessThanVal2] - binVarSortDsdCDF[firstBinVarLessThanVal1];
        if (diff1 > 0.4 || binVarSortDsdCDF[firstBinVarLessThanVal1] > 0.1)
        {
          *reshapeTH1 = 2.9;
          *intraAdp = false;
        }
        else
        {
          if (diff2 > 0.1)
          {
            *reshapeTH1 = 2.5;
          }
          else
          {
            *reshapeTH1 = 2.9;
            if (binVarSortDsdCDF[firstBinVarLessThanVal4] > 0.99 && binVarSortDsdCDF[firstBinVarLessThanVal3] > 0.642 && binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.03)
            {
              m_rateAdpMode = 0;
              m_tcase = 9;
            }
          }
        }
      }
    }
    else
    {
      m_reshapeCW.binCW[0] = 36;
      m_reshapeCW.binCW[1] = 30;
      *reshapeTH1 = 2.6;
      *reshapeTH2 = 4.5;
      *intraAdp = true;
      m_rateAdpMode = 1;
      if (binVarSortDsdCDF[firstBinVarLessThanVal2] > 0.5 && maxBinVar < 4.7)
      {
        *reshapeTH1 = 3.4;
      }
    }
  }
}

void EncReshape::deriveReshapeParameters(double *array, int start, int end, ReshapeCW respCW, double &alpha, double &beta)
{
  double minVar = 10.0, maxVar = 0.0;
  for (int b = start; b <= end; b++)
  {
    if (array[b] < minVar)       minVar = array[b];
    if (array[b] > maxVar)       maxVar = array[b];
  }
  double maxCW = (double)respCW.binCW[0];
  double minCW = (double)respCW.binCW[1];
  alpha = (minCW - maxCW) / (maxVar - minVar);
  beta = (maxCW*maxVar - minCW*minVar) / (maxVar - minVar);
}

/**
-Init reshaping LUT  from dQP model
*/
void EncReshape::initLUTfromdQPModel()
{
  initModelParam();
  int pwlFwdLUTsize = PIC_CODE_CW_BINS;
  int pwlFwdBinLen = m_reshapeLUTSize / PIC_CODE_CW_BINS;
  int p1 = m_dQPModel.scaleFracPrec;
  int p2 = m_dQPModel.offsetFracPrec;
  int totalShift = p1 + p2;
  int scaleFP = (1 - 2 * m_dQPModel.scaleSign)  * m_dQPModel.scaleAbs;
  int offsetFP = (1 - 2 * m_dQPModel.offsetSign) * m_dQPModel.offsetAbs;
  int maxQP = (1 - 2 * m_dQPModel.maxQPSign)  * m_dQPModel.maxQPAbs;
  int minQP = (1 - 2 * m_dQPModel.minQPSign)  * m_dQPModel.minQPAbs;
  int maxFP = maxQP * (1 << totalShift);
  int minFP = minQP * (1 << totalShift);
  int temp, signval, absval;
  int dQPDiv6FP;
  int32_t * slopeLUT = new int32_t[m_reshapeLUTSize]();
  int32_t * fwdLUTHighPrec = new int32_t[m_reshapeLUTSize]();

  for (int i = 0; i < m_reshapeLUTSize; i++)
  {
    int inputY = m_lumaBD < 10 ? i << (10 - m_lumaBD) : m_lumaBD > 10 ? i >> (m_lumaBD - 10) : i;
    temp = int64_t((scaleFP*inputY) * (1 << p2)) + int64_t(offsetFP * (1 << p1));
    temp = temp > maxFP ? maxFP : temp < minFP ? minFP : temp;
    signval = temp >= 0 ? 1 : -1;
    absval = signval * temp;
    dQPDiv6FP = signval * (((absval + 3) / 6 + (1 << (totalShift - 17))) >> (totalShift - 16));
    slopeLUT[i] = calcEXP2(dQPDiv6FP);
  }

  if (m_dQPModel.fullRangeInputFlag == 0)
  {
    for (int i = 0; i < (16 << (m_lumaBD - 8)); i++)                    {      slopeLUT[i] = 0;    }
    for (int i = (235 << (m_lumaBD - 8)); i < m_reshapeLUTSize; i++)    {      slopeLUT[i] = 0;    }
  }

  for (int i = 0; i < m_reshapeLUTSize - 1; i++)
    fwdLUTHighPrec[i + 1] = fwdLUTHighPrec[i] + slopeLUT[i];
  if (slopeLUT != nullptr)   {    delete[] slopeLUT;    slopeLUT = nullptr;  }

  int max_Y = (fwdLUTHighPrec[m_reshapeLUTSize - 1] + (1 << 7)) >> 8;
  int Roffset = max_Y >> 1;
  for (int i = 0; i < m_reshapeLUTSize; i++)
  {
    m_fwdLUT[i] = (short)(((fwdLUTHighPrec[i] >> 8) * (m_reshapeLUTSize - 1) + Roffset) / max_Y);
  }

  if (fwdLUTHighPrec != nullptr)   {    delete[] fwdLUTHighPrec;    fwdLUTHighPrec = nullptr;  }
  m_sliceReshapeInfo.reshaperModelMinBinIdx = 1;
  m_sliceReshapeInfo.reshaperModelMaxBinIdx = 14;

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    int16_t X1 = i * pwlFwdBinLen;
    m_reshapePivot[i] = m_fwdLUT[X1];
  }
  m_reshapePivot[pwlFwdLUTsize] = ((1 << m_lumaBD) - 1);

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_binCW[i] = m_reshapePivot[i + 1] - m_reshapePivot[i];
  }

  int maxAbsDeltaCW = 0, absDeltaCW = 0, deltaCW = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
  {
    deltaCW = (int)m_binCW[i] - (int)m_initCW;
    m_sliceReshapeInfo.reshaperModelBinCWDelta[i] = deltaCW;
    absDeltaCW = (deltaCW < 0) ? (-deltaCW) : deltaCW;
    if (absDeltaCW > maxAbsDeltaCW)     {      maxAbsDeltaCW = absDeltaCW;    }
  }
  m_sliceReshapeInfo.maxNbitsNeededDeltaCW = g_aucLog2[maxAbsDeltaCW << 1];

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    int16_t Y1 = m_reshapePivot[i];
    int16_t Y2 = m_reshapePivot[i + 1];
    m_fwdLUT[i*pwlFwdBinLen] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)Y1);
    int log2PwlFwdBinLen = g_aucLog2[pwlFwdBinLen];
    int32_t scale = ((int32_t)(Y2 - Y1) * (1 << FP_PREC) + (1 << (log2PwlFwdBinLen - 1))) >> (log2PwlFwdBinLen);
    for (int j = 1; j < pwlFwdBinLen; j++)
    {
      int tempVal = Y1 + (((int32_t)scale * (int32_t)j + (1 << (FP_PREC - 1))) >> FP_PREC);
      m_fwdLUT[i*pwlFwdBinLen + j] = Clip3((Pel)0, (Pel)((1<<m_lumaBD) -1), (Pel)tempVal);
    }
  }
  reverseLUT(m_fwdLUT, m_invLUT, m_reshapeLUTSize);
  updateChromaScaleLUT();
}


/**
-Perform fixe point exp2 calculation
\param   val  input value
\retval  output value = exp2(val)
*/
int EncReshape::calcEXP2(int val)
{
  int32_t i, f, r, s;
  r = 0x00000e20;

  i = ((int32_t)(val)+0x8000) & ~0xffff;
  f = (int32_t)(val)-i;
  s = ((15 << 16) - i) >> 16;

  r = (r * f + 0x3e1cc333) >> 17;
  r = (r * f + 0x58bd46a6) >> 16;
  r = r * f + 0x7ffde4a3;
  return (uint32_t)r >> s;
}

void EncReshape::constructReshaperSDR()
{
  int bdShift = m_lumaBD - 10;
  int usedCW = 0;
  int totCW = bdShift != 0 ? (bdShift > 0 ? m_reshapeLUTSize / (1<<bdShift) : m_reshapeLUTSize * (1 << (-bdShift))) : m_reshapeLUTSize;
  int histBins = PIC_ANALYZE_CW_BINS;
  int histLenth = totCW/histBins;
  int log2HistLenth = g_aucLog2[histLenth];
  int16_t *tempFwdLUT = new int16_t[m_reshapeLUTSize + 1]();
  int i, j;
  int cwScaleBins1, cwScaleBins2;
  int maxAllowedCW = totCW-1;

  cwScaleBins1 = m_reshapeCW.binCW[0];
  cwScaleBins2 = m_reshapeCW.binCW[1];

  for (i = 0; i < histBins; i++)
    usedCW += m_binCW[i];

  if (usedCW > maxAllowedCW)
  {
    int cnt0 = 0, cnt1 = 0, cnt2 = 0;
    for (i = 0; i < histBins; i++)
    {
      if (m_binCW[i] == histLenth + 1)             cnt0++;
      else if (m_binCW[i] == cwScaleBins1)         cnt1++;
      else if (m_binCW[i] == cwScaleBins2)         cnt2++;
    }

    int resCW = usedCW - maxAllowedCW;
    int cwReduce1 = (cwScaleBins1 - histLenth - 1) * cnt1;
    int cwReduce2 = (histLenth + 1 - cwScaleBins2) * cnt0;

    if (resCW <= cwReduce1)
    {
      int idx = 0;
      while (resCW > 0)
      {
        if (m_binCW[idx] > (histLenth + 1))
        {
          m_binCW[idx]--;
          resCW--;
        }
        idx++;
        if (idx == histBins)
          idx = 0;
      }
    }
    else if (resCW > cwReduce1 && resCW <= (cwReduce1 + cwReduce2))
    {
      resCW -= cwReduce1;
      int idx = 0;
      while (resCW > 0)
      {
        if (m_binCW[idx] > cwScaleBins2 && m_binCW[idx] < cwScaleBins1)
        {
          m_binCW[idx]--;
          resCW--;
        }
        idx++;
        if (idx == histBins)
          idx = 0;
      }
      for (i = 0; i < histBins; i++)
      {
        if (m_binCW[i] == cwScaleBins1)
          m_binCW[i] = histLenth + 1;
      }
    }
    else if (resCW > (cwReduce1 + cwReduce2))
    {
      resCW -= (cwReduce1 + cwReduce2);
      int idx = 0;
      while (resCW > 0)
      {
        if (m_binCW[idx] > 0 && m_binCW[idx] < (histLenth + 1))
        {
          m_binCW[idx]--;
          resCW--;
        }
        idx++;
        if (idx == histBins)
          idx = 0;
      }
      for (i = 0; i < histBins; i++)
      {
        if (m_binCW[i] == histLenth + 1)
          m_binCW[i] = cwScaleBins2;
        if (m_binCW[i] == cwScaleBins1)
          m_binCW[i] = histLenth + 1;
      }
    }
  }

  if (bdShift != 0)
  {
    for (int i = 0; i < PIC_ANALYZE_CW_BINS; i++)
    {
      m_binCW[i] = bdShift > 0 ? m_binCW[i] * (1 << bdShift) : m_binCW[i] / (1 << (-bdShift));
    }
  }

  for (int i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    m_binCW[i] = m_binCW[2 * i] + m_binCW[2 * i + 1];
  }
  m_sliceReshapeInfo.reshaperModelMinBinIdx = 0;
  m_sliceReshapeInfo.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1;
  for (int i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    if (m_binCW[i] > 0)
    {
      m_sliceReshapeInfo.reshaperModelMinBinIdx = i;
      break;
    }
  }
  for (int i = PIC_CODE_CW_BINS - 1; i >= 0; i--)
  {
    if (m_binCW[i] > 0)
    {
      m_sliceReshapeInfo.reshaperModelMaxBinIdx = i;
      break;
    }
  }

  int maxAbsDeltaCW = 0, absDeltaCW = 0, deltaCW = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
  {
    deltaCW = (int)m_binCW[i] - (int)m_initCW;
    m_sliceReshapeInfo.reshaperModelBinCWDelta[i] = deltaCW;
    absDeltaCW = (deltaCW < 0) ? (-deltaCW) : deltaCW;
    if (absDeltaCW > maxAbsDeltaCW)      {      maxAbsDeltaCW = absDeltaCW;    }
  }
  m_sliceReshapeInfo.maxNbitsNeededDeltaCW = std::max(1, (int)g_aucLog2[maxAbsDeltaCW << 1]);

  histLenth = m_initCW;
  log2HistLenth = g_aucLog2[histLenth];

  int sumBins = 0;
  for (i = 0; i < PIC_CODE_CW_BINS; i++)   { sumBins += m_binCW[i];  }
  VTMCHECK(sumBins >= m_reshapeLUTSize, "SDR CW assignment is wrong!!");
  memset(tempFwdLUT, 0, (m_reshapeLUTSize + 1) * sizeof(int16_t));
  tempFwdLUT[0] = 0;

  for (i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    tempFwdLUT[(i + 1)*histLenth] = tempFwdLUT[i*histLenth] + m_binCW[i];
    int16_t Y1 = tempFwdLUT[i*histLenth];
    int16_t Y2 = tempFwdLUT[(i + 1)*histLenth];
    m_reshapePivot[i + 1] = Y2;
    int32_t scale = ((int32_t)(Y2 - Y1) * (1 << FP_PREC) + (1 << (log2HistLenth - 1))) >> (log2HistLenth);
    m_fwdLUT[i*histLenth] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)Y1);
    for (j = 1; j < histLenth; j++)
    {
      tempFwdLUT[i*histLenth + j] = Y1 + (((int32_t)scale * (int32_t)j + (1 << (FP_PREC - 1))) >> FP_PREC);
      m_fwdLUT[i*histLenth + j] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)tempFwdLUT[i*histLenth + j]);
    }
  }

  for (i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    int start = i*histLenth;
    int end = (i + 1)*histLenth - 1;
    m_cwLumaWeight[i] = m_fwdLUT[end] - m_fwdLUT[start];
  }

  if (tempFwdLUT != nullptr)   {     delete[] tempFwdLUT;    tempFwdLUT = nullptr;  }

  reverseLUT(m_fwdLUT, m_invLUT, m_reshapeLUTSize);
  updateChromaScaleLUT();
}

#endif
//
//! \}
