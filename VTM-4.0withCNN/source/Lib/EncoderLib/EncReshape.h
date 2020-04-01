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

 /** \file     EncReshape.h
     \brief    encoder reshaping header and class (header)
 */

#ifndef __ENCRESHAPE__
#define __ENCRESHAPE__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#include "CommonLib/Reshape.h"
#if JVET_M0427_INLOOP_RESHAPER

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct ModelInfo
{
  unsigned fullRangeInputFlag;
  unsigned scaleIntPrec;
  unsigned scaleInt;
  unsigned scaleFracPrec;
  unsigned scaleFrac;
  unsigned scaleSign;
  unsigned offsetIntPrec;
  unsigned offsetInt;
  unsigned offsetFracPrec;
  unsigned offsetFrac;
  unsigned offsetSign;
  unsigned minMaxQPAbsPrec;
  unsigned maxQPAbs;
  unsigned maxQPSign;
  unsigned minQPAbs;
  unsigned minQPSign;
  unsigned scaleAbs;
  unsigned offsetAbs;
};

class EncReshape : public Reshape
{
private:
  bool                    m_srcReshaped;
  int                     m_picWidth;
  int                     m_picHeight;
  uint32_t                m_maxCUWidth;
  uint32_t                m_maxCUHeight;
  uint32_t                m_widthInCtus;
  uint32_t                m_heightInCtus;
  uint32_t                m_numCtuInFrame;
  bool                    m_exceedSTD;
  std::vector<uint32_t>   m_binImportance;
  int                     m_tcase;
  int                     m_rateAdpMode;
  bool                    m_useAdpCW;
  uint16_t                m_initCWAnalyze;
  ModelInfo               m_dQPModel;
  ReshapeCW               m_reshapeCW;
  Pel                     m_cwLumaWeight[PIC_CODE_CW_BINS];
  double                  m_chromaWeight;
  int                     m_chromaAdj;
public:

  EncReshape();
  ~EncReshape();

  void createEnc( int picWidth, int picHeight, uint32_t maxCUWidth, uint32_t maxCUHeight, int bitDepth);
  void destroy();

  bool getSrcReshaped() { return m_srcReshaped; }
  void setSrcReshaped(bool b) { m_srcReshaped = b; }
#if JVET_M0483_IBC
  void preAnalyzerSDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT);
  void preAnalyzerHDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT);
#else
  void preAnalyzerSDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT, bool isIBC);
  void preAnalyzerHDR(Picture *pcPic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT, bool isIBC);
#endif
  void bubbleSortDsd(double *array, int * idx, int n);
  void swap(int *xp, int *yp) { int temp = *xp;  *xp = *yp;  *yp = temp; }
  void swap(double *xp, double *yp) { double temp = *xp;  *xp = *yp;  *yp = temp; }
  void deriveReshapeParametersSDRfromStats(uint32_t *, double*, double* reshapeTH1, double* reshapeTH2, bool *intraAdp, bool *interAdp);
  void deriveReshapeParameters(double *array, int start, int end, ReshapeCW respCW, double &alpha, double &beta);
  void initLUTfromdQPModel();
  int  calcEXP2(int val);
  void constructReshaperSDR();
  ReshapeCW * getReshapeCW() { return &m_reshapeCW; }
  Pel * getWeightTable() { return m_cwLumaWeight; }
  double getCWeight() { return m_chromaWeight; }

  void initModelParam(double scale = 0.015, double offset = -7.5, int QPMax = 6, int QPMin = -3)
  {
    /// dQP model:  dQP = clip3(QPMin, QPMax, dScale*Y+dOffset);
    m_dQPModel.fullRangeInputFlag = 0;
    m_dQPModel.scaleIntPrec = 0;
    m_dQPModel.scaleFracPrec = 16;
    m_dQPModel.offsetIntPrec = 3;
    m_dQPModel.offsetFracPrec = 1;
    m_dQPModel.minMaxQPAbsPrec = 3;
    m_dQPModel.scaleSign = scale < 0 ? 1 : 0;
    m_dQPModel.scaleAbs = unsigned((scale < 0 ? -scale : scale) * (1 << m_dQPModel.scaleFracPrec));
    m_dQPModel.scaleInt = m_dQPModel.scaleAbs >> m_dQPModel.scaleFracPrec;
    m_dQPModel.scaleFrac = m_dQPModel.scaleAbs - (m_dQPModel.scaleInt << m_dQPModel.scaleFracPrec);
    m_dQPModel.offsetSign = offset < 0 ? 1 : 0;
    m_dQPModel.offsetAbs = unsigned((offset < 0 ? -offset : offset) * (1 << m_dQPModel.offsetFracPrec));
    m_dQPModel.offsetInt = m_dQPModel.offsetAbs >> m_dQPModel.offsetFracPrec;
    m_dQPModel.offsetFrac = m_dQPModel.offsetAbs - (m_dQPModel.offsetInt << m_dQPModel.offsetFracPrec);
    m_dQPModel.maxQPSign = QPMax < 0 ? 1 : 0;
    m_dQPModel.maxQPAbs = m_dQPModel.maxQPSign ? -QPMax : QPMax;
    m_dQPModel.minQPSign = QPMin < 0 ? 1 : 0;
    m_dQPModel.minQPAbs = m_dQPModel.minQPSign ? -QPMin : QPMin;
  }
};// END CLASS DEFINITION EncReshape

//! \}
#endif
#endif
