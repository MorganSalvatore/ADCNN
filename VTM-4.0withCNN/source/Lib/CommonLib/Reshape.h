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

 /** \file     Reshape.h
     \brief    reshaping header and class (header)
 */

#ifndef __RESHAPE__
#define __RESHAPE__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "CommonDef.h"
#include "Rom.h"
#include "CommonLib/Picture.h"
#if JVET_M0427_INLOOP_RESHAPER
//! \ingroup CommonLib
//! \{
// ====================================================================================================================
// Class definition
// ====================================================================================================================

class Reshape
{
protected:
  SliceReshapeInfo        m_sliceReshapeInfo;
  bool                    m_CTUFlag;
  bool                    m_recReshaped;
  std::vector<Pel>        m_invLUT;
  std::vector<Pel>        m_fwdLUT;
  std::vector<int>        m_chromaAdjHelpLUT;
  std::vector<uint16_t>   m_binCW;
  uint16_t                m_initCW;
  bool                    m_reshape;
  std::vector<Pel>        m_reshapePivot;
  int                     m_lumaBD;
  int                     m_reshapeLUTSize;
public:
  Reshape();
  ~Reshape();

  void createDec(int bitDepth);
  void destroy();

  void reverseLUT(std::vector<Pel>& inputLUT, std::vector<Pel>& outputLUT, uint16_t lutSize);
  std::vector<Pel>&  getFwdLUT() { return m_fwdLUT; }
  std::vector<Pel>&  getInvLUT() { return m_invLUT; }
  std::vector<int>&  getChromaAdjHelpLUT() { return m_chromaAdjHelpLUT; }

  bool getCTUFlag()              { return m_CTUFlag; }
  void setCTUFlag(bool b       ) { m_CTUFlag = b; }

  bool getRecReshaped()          { return m_recReshaped; }
  void setRecReshaped(bool b)    { m_recReshaped = b; }
  int  calculateChromaAdj(Pel avgLuma);
  int  getPWLIdxInv(int lumaVal);
  SliceReshapeInfo& getSliceReshaperInfo() { return m_sliceReshapeInfo; }
  void copySliceReshaperInfo(SliceReshapeInfo& tInfo, SliceReshapeInfo& sInfo);

  void constructReshaper();
  void updateChromaScaleLUT();
  bool getReshapeFlag() { return m_reshape; }
  void setReshapeFlag(bool b) { m_reshape = b; }
};// END CLASS DEFINITION Reshape

//! \}
#endif //
#endif // __RESHAPE__


