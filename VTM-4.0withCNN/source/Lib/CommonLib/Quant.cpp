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

/** \file     Quant.cpp
    \brief    transform and quantization class
*/

#include "Quant.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "CrossCompPrediction.h"

#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>



//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================


// ====================================================================================================================
// QpParam constructor
// ====================================================================================================================

QpParam::QpParam(const int           qpy,
                 const ChannelType   chType,
                 const int           qpBdOffset,
                 const int           chromaQPOffset,
                 const ChromaFormat  chFmt,
                 const int           dqp )
{
  int baseQp;

  if(isLuma(chType))
  {
    baseQp = qpy + qpBdOffset;
  }
  else
  {
    baseQp = Clip3( -qpBdOffset, (chromaQPMappingTableSize - 1), qpy + chromaQPOffset );

    if(baseQp < 0)
    {
      baseQp = baseQp + qpBdOffset;
    }
    else
    {
      baseQp = getScaledChromaQP(baseQp, chFmt) + qpBdOffset;
    }
  }

  baseQp = Clip3( 0, MAX_QP+qpBdOffset, baseQp + dqp );

  Qp =baseQp;
  per=baseQp/6;
  rem=baseQp%6;
}

QpParam::QpParam(const TransformUnit& tu, const ComponentID &compIDX, const int QP /*= -MAX_INT*/)
{
  int chromaQpOffset = 0;
  ComponentID compID = MAP_CHROMA(compIDX);

  if (isChroma(compID))
  {
    chromaQpOffset += tu.cs->pps->getQpOffset( compID );
    chromaQpOffset += tu.cs->slice->getSliceChromaQpDelta( compID );
    chromaQpOffset += tu.cs->pps->getPpsRangeExtension().getChromaQpOffsetListEntry( tu.cu->chromaQpAdj ).u.offset[int( compID ) - 1];
  }

#if HM_QTBT_AS_IN_JEM_QUANT
  int dqp = 0;
#else
  int dqp = ( TU::needsQP3Offset(tu, compID) ? -3 : 0 );
#endif

  *this = QpParam(QP <= -MAX_INT ? tu.cu->qp : QP, toChannelType(compID), tu.cs->sps->getQpBDOffset(toChannelType(compID)), chromaQpOffset, tu.chromaFormat, dqp);
}


// ====================================================================================================================
// Quant class member functions
// ====================================================================================================================

Quant::Quant( const Quant* other )
{
#if HEVC_USE_SCALING_LISTS
  xInitScalingList( other );
#endif
}

Quant::~Quant()
{
#if HEVC_USE_SCALING_LISTS
  xDestroyScalingList();
#endif
}


#if HEVC_USE_SIGN_HIDING
// To minimize the distortion only. No rate is considered.
void Quant::xSignBitHidingHDQ( TCoeff* pQCoef, const TCoeff* pCoef, TCoeff* deltaU, const CoeffCodingContext& cctx, const int maxLog2TrDynamicRange )
{
  const uint32_t width     = cctx.width();
  const uint32_t height    = cctx.height();
  const uint32_t groupSize = 1 << cctx.log2CGSize();

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  int lastCG = -1;
  int absSum = 0 ;
  int n ;

  for( int subSet = (width*height-1) >> cctx.log2CGSize(); subSet >= 0; subSet-- )
  {
    int  subPos = subSet << cctx.log2CGSize();
    int  firstNZPosInCG=groupSize , lastNZPosInCG=-1 ;
    absSum = 0 ;

    for(n = groupSize-1; n >= 0; --n )
    {
      if( pQCoef[ cctx.blockPos( n + subPos ) ] )
      {
        lastNZPosInCG = n;
        break;
      }
    }

    for(n = 0; n <groupSize; n++ )
    {
      if( pQCoef[ cctx.blockPos( n + subPos ) ] )
      {
        firstNZPosInCG = n;
        break;
      }
    }

    for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )
    {
      absSum += int(pQCoef[ cctx.blockPos( n + subPos ) ]);
    }

    if(lastNZPosInCG>=0 && lastCG==-1)
    {
      lastCG = 1 ;
    }

    if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
    {
      uint32_t signbit = (pQCoef[cctx.blockPos(subPos+firstNZPosInCG)]>0?0:1) ;
      if( signbit!=(absSum&0x1) )  //compare signbit with sum_parity
      {
        TCoeff curCost    = std::numeric_limits<TCoeff>::max();
        TCoeff minCostInc = std::numeric_limits<TCoeff>::max();
        int minPos =-1, finalChange=0, curChange=0;

        for( n = (lastCG==1?lastNZPosInCG:groupSize-1) ; n >= 0; --n )
        {
          uint32_t blkPos   = cctx.blockPos( n+subPos );
          if(pQCoef[ blkPos ] != 0 )
          {
            if(deltaU[blkPos]>0)
            {
              curCost = - deltaU[blkPos];
              curChange=1 ;
            }
            else
            {
              //curChange =-1;
              if(n==firstNZPosInCG && abs(pQCoef[blkPos])==1)
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = deltaU[blkPos];
                curChange =-1;
              }
            }
          }
          else
          {
            if(n<firstNZPosInCG)
            {
              uint32_t thisSignBit = (pCoef[blkPos]>=0?0:1);
              if(thisSignBit != signbit )
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = - (deltaU[blkPos])  ;
                curChange = 1 ;
              }
            }
            else
            {
              curCost = - (deltaU[blkPos])  ;
              curChange = 1 ;
            }
          }

          if( curCost<minCostInc)
          {
            minCostInc = curCost ;
            finalChange = curChange ;
            minPos = blkPos ;
          }
        } //CG loop

        if(pQCoef[minPos] == entropyCodingMaximum || pQCoef[minPos] == entropyCodingMinimum)
        {
          finalChange = -1;
        }

        if(pCoef[minPos]>=0)
        {
          pQCoef[minPos] += finalChange ;
        }
        else
        {
          pQCoef[minPos] -= finalChange ;
        }
      } // Hide
    }
    if(lastCG==1)
    {
      lastCG=0 ;
    }
  } // TU loop

  return;
}
#endif

void Quant::dequant(const TransformUnit &tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  const SPS            *sps                = tu.cs->sps;
  const CompArea       &area               = tu.blocks[compID];
  const uint32_t            uiWidth            = area.width;
  const uint32_t            uiHeight           = area.height;
  const TCoeff   *const piQCoef            = tu.getCoeffs(compID).buf;
        TCoeff   *const piCoef             = dstCoeff.buf;
  const uint32_t            numSamplesInBlock  = uiWidth * uiHeight;
  const int             maxLog2TrDynamicRange = sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const TCoeff          transformMinimum   = -(1 << maxLog2TrDynamicRange);
  const TCoeff          transformMaximum   =  (1 << maxLog2TrDynamicRange) - 1;
#if HEVC_USE_SCALING_LISTS
  const bool            enableScalingLists = getUseScalingList(uiWidth, uiHeight, (tu.transformSkip[compID] != 0));
  const int             scalingListType    = getScalingListType(tu.cu->predMode, compID);
#endif
  const int             channelBitDepth    = sps->getBitDepth(toChannelType(compID));

#if HEVC_USE_SCALING_LISTS
  VTMCHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
#endif
  VTMCHECK(uiWidth > m_uiMaxTrSize, "Unsupported transformation size");

  // Represents scaling through forward transform
#if JVET_M0464_UNI_MTS
  const bool bClipTransformShiftTo0 = tu.mtsIdx!=1 && sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
#else
  const bool bClipTransformShiftTo0 = (tu.transformSkip[compID] != 0) && sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
#endif
  const int  originalTransformShift = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);
  const int  iTransformShift        = bClipTransformShiftTo0 ? std::max<int>(0, originalTransformShift) : originalTransformShift;

  const int QP_per = cQP.per;
  const int QP_rem = cQP.rem;

#if HM_QTBT_AS_IN_JEM_QUANT
#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
  const bool needsScalingCorrection = TU::needsBlockSizeTrafoScale( tu, compID );
  const int  NEScale    = TU::needsSqrt2Scale( tu, compID ) ? 181 : 1;
#else
  const bool needsScalingCorrection = TU::needsBlockSizeTrafoScale( tu.block( compID ) );
  const int  NEScale    = TU::needsSqrt2Scale( tu.blocks[compID] ) ? 181 : 1;
#endif
#if HEVC_USE_SCALING_LISTS
  const int  rightShift = (needsScalingCorrection ?   8 : 0 ) + (IQUANT_SHIFT - (iTransformShift + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);
#else
  const int  rightShift = (needsScalingCorrection ?   8 : 0 ) + (IQUANT_SHIFT - (iTransformShift + QP_per));
#endif
#else
#if HEVC_USE_SCALING_LISTS
  const int  rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);
#else
  const int  rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per));
#endif
#endif

#if HEVC_USE_SCALING_LISTS
  if(enableScalingLists)
  {
    //from the dequantization equation:
    //iCoeffQ                         = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) + iAdd ) >> rightShift
    //(sizeof(Intermediate_Int) * 8)  =              inputBitDepth    +    dequantCoefBits                   - rightShift
    const uint32_t             dequantCoefBits     = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const uint32_t             targetInputBitDepth = std::min<uint32_t>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    const uint32_t uiLog2TrWidth  = g_aucLog2[uiWidth];
    const uint32_t uiLog2TrHeight = g_aucLog2[uiHeight];
    int *piDequantCoef        = getDequantCoeff(scalingListType, QP_rem, uiLog2TrWidth - 1, uiLog2TrHeight - 1);

    if(rightShift > 0)
    {
      const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);

      for( int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
#if HM_QTBT_AS_IN_JEM_QUANT
        const Intermediate_Int iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[n] * NEScale) + iAdd ) >> rightShift;
#else
        const Intermediate_Int iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[n]) + iAdd ) >> rightShift;
#endif

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const int leftShift = -rightShift;

      for( int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
#if HM_QTBT_AS_IN_JEM_QUANT
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[n] * NEScale) << leftShift;
#else
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[n]) << leftShift;
#endif

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
  else
  {
#endif
#if HM_QTBT_AS_IN_JEM_QUANT
    const int scale     = g_invQuantScales[QP_rem] * NEScale;
#else
    const int scale     = g_invQuantScales[QP_rem];
#endif
    const int scaleBits = ( IQUANT_SHIFT + 1 );

    //from the dequantisation equation:
    //iCoeffQ                         = Intermediate_Int((int64_t(clipQCoef) * scale + iAdd) >> rightShift);
    //(sizeof(Intermediate_Int) * 8)  =                    inputBitDepth   + scaleBits      - rightShift
    const uint32_t             targetInputBitDepth = std::min<uint32_t>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);

      for( int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const int leftShift = -rightShift;

      for( int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
#if HEVC_USE_SCALING_LISTS
  }
#endif
}

void Quant::init( uint32_t uiMaxTrSize,
                  bool bUseRDOQ,
                  bool bUseRDOQTS,
#if T0196_SELECTIVE_RDOQ
                  bool useSelectiveRDOQ
#endif
                  )
{

  // TODO: pass to init() a single variable containing (quantization) flags,
  //       instead of variables that don't have to do with this class

  m_uiMaxTrSize  = uiMaxTrSize;
  m_useRDOQ      = bUseRDOQ;
  m_useRDOQTS    = bUseRDOQTS;
#if T0196_SELECTIVE_RDOQ
  m_useSelectiveRDOQ     = useSelectiveRDOQ;
#endif
}

#if ENABLE_SPLIT_PARALLELISM
void Quant::copyState( const Quant& other )
{
  m_dLambda = other.m_dLambda;
  memcpy( m_lambdas, other.m_lambdas, sizeof( m_lambdas ) );
}
#endif

#if HEVC_USE_SCALING_LISTS
/** set quantized matrix coefficient for encode
 * \param scalingList            quantized matrix address
 * \param format                 chroma format
 * \param maxLog2TrDynamicRange
 * \param bitDepths              reference to bit depth array for all channels
 */
void Quant::setScalingList(ScalingList *scalingList, const int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths)
{
  const int minimumQp = 0;
  const int maximumQp = SCALING_LIST_REM_NUM;

  for(uint32_t size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(uint32_t list = 0; list < SCALING_LIST_NUM; list++)
    {
      for(int qp = minimumQp; qp < maximumQp; qp++)
      {
        xSetScalingListEnc(scalingList,list,size,qp);
        xSetScalingListDec(*scalingList,list,size,qp);
      }
    }
  }
}
/** set quantized matrix coefficient for decode
 * \param scalingList quantized matrix address
 * \param format      chroma format
 */
void Quant::setScalingListDec(const ScalingList &scalingList)
{
  const int minimumQp = 0;
  const int maximumQp = SCALING_LIST_REM_NUM;

  for(uint32_t size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(uint32_t list = 0; list < SCALING_LIST_NUM; list++)
    {
      for(int qp = minimumQp; qp < maximumQp; qp++)
      {
        xSetScalingListDec(scalingList,list,size,qp);
      }
    }
  }
}


/** set quantized matrix coefficient for encode
 * \param scalingList quantized matrix address
 * \param listId List index
 * \param sizeId size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
void Quant::xSetScalingListEnc(ScalingList *scalingList, uint32_t listId, uint32_t sizeId, int qp)
{
  uint32_t width  = g_scalingListSizeX[sizeId];
  uint32_t height = g_scalingListSizeX[sizeId];
  uint32_t ratio  = g_scalingListSizeX[sizeId]/std::min(MAX_MATRIX_SIZE_NUM,(int)g_scalingListSizeX[sizeId]);
  int *quantcoeff;
  int *coeff  = scalingList->getScalingListAddress(sizeId,listId);
  quantcoeff  = getQuantCoeff(listId, qp, sizeId, sizeId);

  int quantScales = g_quantScales[qp];

  processScalingListEnc(coeff,
                        quantcoeff,
                        (quantScales << LOG2_SCALING_LIST_NEUTRAL_VALUE),
                        height, width, ratio,
                        std::min(MAX_MATRIX_SIZE_NUM, (int)g_scalingListSizeX[sizeId]),
                        scalingList->getScalingListDC(sizeId,listId));
}

/** set quantized matrix coefficient for decode
 * \param scalingList quantaized matrix address
 * \param listId List index
 * \param sizeId size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
void Quant::xSetScalingListDec(const ScalingList &scalingList, uint32_t listId, uint32_t sizeId, int qp)
{
  uint32_t width  = g_scalingListSizeX[sizeId];
  uint32_t height = g_scalingListSizeX[sizeId];
  uint32_t ratio  = g_scalingListSizeX[sizeId]/std::min(MAX_MATRIX_SIZE_NUM,(int)g_scalingListSizeX[sizeId]);
  int *dequantcoeff;
  const int *coeff  = scalingList.getScalingListAddress(sizeId,listId);

  dequantcoeff = getDequantCoeff(listId, qp, sizeId, sizeId);

  int invQuantScale = g_invQuantScales[qp];

  processScalingListDec(coeff,
                        dequantcoeff,
                        invQuantScale,
                        height, width, ratio,
                        std::min(MAX_MATRIX_SIZE_NUM, (int)g_scalingListSizeX[sizeId]),
                        scalingList.getScalingListDC(sizeId,listId));
}

/** set flat matrix value to quantized coefficient
 */
void Quant::setFlatScalingList(const int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths)
{
  const int minimumQp = 0;
  const int maximumQp = SCALING_LIST_REM_NUM;

  for(uint32_t sizeX = 0; sizeX < SCALING_LIST_SIZE_NUM; sizeX++)
  {
    for(uint32_t sizeY = 0; sizeY < SCALING_LIST_SIZE_NUM; sizeY++)
    {
      for(uint32_t list = 0; list < SCALING_LIST_NUM; list++)
      {
        for(int qp = minimumQp; qp < maximumQp; qp++)
        {
          xSetFlatScalingList( list, sizeX, sizeY, qp );
        }
      }
    }
  }
}

/** set flat matrix value to quantized coefficient
 * \param list List ID
 * \param size size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
void Quant::xSetFlatScalingList(uint32_t list, uint32_t sizeX, uint32_t sizeY, int qp)
{
  uint32_t i,num = g_scalingListSizeX[sizeX]*g_scalingListSizeX[sizeY];
  int *quantcoeff;
  int *dequantcoeff;

  int quantScales    = g_quantScales   [qp];
  int invQuantScales = g_invQuantScales[qp] << 4;

  quantcoeff   = getQuantCoeff(list, qp, sizeX, sizeY);
  dequantcoeff = getDequantCoeff(list, qp, sizeX, sizeY);

  for(i=0;i<num;i++)
  {
    *quantcoeff++ = quantScales;
    *dequantcoeff++ = invQuantScales;
  }
}

/** set quantized matrix coefficient for encode
 * \param coeff quantaized matrix address
 * \param quantcoeff quantaized matrix address
 * \param quantScales Q(QP%6)
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
void Quant::processScalingListEnc( int *coeff, int *quantcoeff, int quantScales, uint32_t height, uint32_t width, uint32_t ratio, int sizuNum, uint32_t dc)
{
  for(uint32_t j=0;j<height;j++)
  {
    for(uint32_t i=0;i<width;i++)
    {
      quantcoeff[j*width + i] = quantScales / coeff[sizuNum * (j / ratio) + i / ratio];
    }
  }

  if(ratio > 1)
  {
    quantcoeff[0] = quantScales / dc;
  }
}

/** set quantized matrix coefficient for decode
 * \param coeff quantaized matrix address
 * \param dequantcoeff quantaized matrix address
 * \param invQuantScales IQ(QP%6))
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
void Quant::processScalingListDec( const int *coeff, int *dequantcoeff, int invQuantScales, uint32_t height, uint32_t width, uint32_t ratio, int sizuNum, uint32_t dc)
{
  for(uint32_t j=0;j<height;j++)
  {
    for(uint32_t i=0;i<width;i++)
    {
      dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * (j / ratio) + i / ratio];
    }
  }

  if(ratio > 1)
  {
    dequantcoeff[0] = invQuantScales * dc;
  }
}

/** initialization process of scaling list array
 */
void Quant::xInitScalingList( const Quant* other )
{
  m_isScalingListOwner = other == nullptr;

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
        {
          if( m_isScalingListOwner )
          {
            m_quantCoef   [sizeIdX][sizeIdY][listId][qp] = new int    [g_scalingListSizeX[sizeIdX]*g_scalingListSizeX[sizeIdY]];
            m_dequantCoef [sizeIdX][sizeIdY][listId][qp] = new int    [g_scalingListSizeX[sizeIdX]*g_scalingListSizeX[sizeIdY]];
          }
          else
          {
            m_quantCoef   [sizeIdX][sizeIdY][listId][qp] = other->m_quantCoef   [sizeIdX][sizeIdY][listId][qp];
            m_dequantCoef [sizeIdX][sizeIdY][listId][qp] = other->m_dequantCoef [sizeIdX][sizeIdY][listId][qp];
          }
        } // listID loop
      }
    }
  }
}

/** destroy quantization matrix array
 */
void Quant::xDestroyScalingList()
{
  if( !m_isScalingListOwner ) return;

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
      {
        for(uint32_t qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
        {
          if(m_quantCoef[sizeIdX][sizeIdY][listId][qp])
          {
            delete [] m_quantCoef[sizeIdX][sizeIdY][listId][qp];
          }
          if(m_dequantCoef[sizeIdX][sizeIdY][listId][qp])
          {
            delete [] m_dequantCoef[sizeIdX][sizeIdY][listId][qp];
          }
        }
      }
    }
  }
}
#endif

void Quant::quant(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx)
{
  const SPS &sps            = *tu.cs->sps;
  const CompArea &rect      = tu.blocks[compID];
#if HEVC_USE_SCALING_LISTS || HEVC_USE_SIGN_HIDING
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;
#endif
  const int channelBitDepth = sps.getBitDepth(toChannelType(compID));

  const CCoeffBuf &piCoef   = pSrc;
        CoeffBuf   piQCoef  = tu.getCoeffs(compID);

#if JVET_M0464_UNI_MTS
  const bool useTransformSkip      = tu.mtsIdx==1;
#else
  const bool useTransformSkip      = tu.transformSkip[compID];
#endif
  const int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(toChannelType(compID));

  {
#if HEVC_USE_SIGN_HIDING
    CoeffCodingContext cctx(tu, compID, tu.cs->slice->getSignDataHidingEnabledFlag());
#else
    CoeffCodingContext cctx(tu, compID);
#endif

    const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
    const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

#if HEVC_USE_SIGN_HIDING
    TCoeff deltaU[MAX_TU_SIZE * MAX_TU_SIZE];
#endif
#if HEVC_USE_SCALING_LISTS
    int scalingListType = getScalingListType(tu.cu->predMode, compID);
    VTMCHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
    const uint32_t uiLog2TrWidth = g_aucLog2[uiWidth];
    const uint32_t uiLog2TrHeight = g_aucLog2[uiHeight];
    int *piQuantCoeff = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrWidth-1, uiLog2TrHeight-1);

    const bool enableScalingLists             = getUseScalingList(uiWidth, uiHeight, useTransformSkip);
#endif
    const int  defaultQuantisationCoefficient = g_quantScales[cQP.rem];

    /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
     * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
     * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
     * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
     */
    // Represents scaling through forward transform
    int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

    if (useTransformSkip && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
    {
      iTransformShift = std::max<int>(0, iTransformShift);
    }

    int iWHScale = 1;
#if HM_QTBT_AS_IN_JEM_QUANT
#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
    if( TU::needsBlockSizeTrafoScale( tu, compID ) )
#else
    if( TU::needsBlockSizeTrafoScale( rect ) )
#endif
    {
      iTransformShift += ADJ_QUANT_SHIFT;
      iWHScale = 181;
    }
#endif

    const int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;
    // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

    const int64_t iAdd = int64_t(tu.cs->slice->isIRAP() ? 171 : 85) << int64_t(iQBits - 9);
#if HEVC_USE_SIGN_HIDING
    const int qBits8 = iQBits - 8;
#endif

    for (int uiBlockPos = 0; uiBlockPos < piQCoef.area(); uiBlockPos++)
    {
      const TCoeff iLevel   = piCoef.buf[uiBlockPos];
      const TCoeff iSign    = (iLevel < 0 ? -1: 1);

#if HEVC_USE_SCALING_LISTS
      const int64_t  tmpLevel = (int64_t)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);
#else
      const int64_t  tmpLevel = (int64_t)abs(iLevel) * defaultQuantisationCoefficient;
#endif

      const TCoeff quantisedMagnitude = TCoeff((tmpLevel * iWHScale + iAdd ) >> iQBits);
#if HEVC_USE_SIGN_HIDING
      deltaU[uiBlockPos] = (TCoeff)((tmpLevel * iWHScale - ((int64_t)quantisedMagnitude<<iQBits) )>> qBits8);
#endif

      uiAbsSum += quantisedMagnitude;
      const TCoeff quantisedCoefficient = quantisedMagnitude * iSign;

      piQCoef.buf[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
    } // for n
#if HEVC_USE_SIGN_HIDING
    if( cctx.signHiding() && uiWidth>=4 && uiHeight>=4 )
    {
      if(uiAbsSum >= 2) //this prevents TUs with only one coefficient of value 1 from being tested
      {
        xSignBitHidingHDQ(piQCoef.buf, piCoef.buf, deltaU, cctx, maxLog2TrDynamicRange);
      }
    }
#endif
  } //if RDOQ
  //return;
}

bool Quant::xNeedRDOQ(TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, const QpParam &cQP)
{
  const SPS &sps            = *tu.cs->sps;
  const CompArea &rect      = tu.blocks[compID];
#if HEVC_USE_SCALING_LISTS
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;
#endif
  const int channelBitDepth = sps.getBitDepth(toChannelType(compID));

  const CCoeffBuf piCoef    = pSrc;

#if JVET_M0464_UNI_MTS
  const bool useTransformSkip      = tu.mtsIdx==1;
#else
  const bool useTransformSkip      = tu.transformSkip[compID];
#endif
  const int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(toChannelType(compID));

#if HEVC_USE_SCALING_LISTS
  int scalingListType = getScalingListType(tu.cu->predMode, compID);
  VTMCHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  const uint32_t uiLog2TrWidth  = g_aucLog2[uiWidth];
  const uint32_t uiLog2TrHeight = g_aucLog2[uiHeight];
  int *piQuantCoeff         = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrWidth-1, uiLog2TrHeight-1);

  const bool enableScalingLists             = getUseScalingList(uiWidth, uiHeight, (useTransformSkip != 0));
#endif
  const int  defaultQuantisationCoefficient = g_quantScales[cQP.rem];

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
    * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
    * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
    * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
    */

  // Represents scaling through forward transform
  int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  if (useTransformSkip && sps.getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  {
    iTransformShift = std::max<int>(0, iTransformShift);
  }

  int iWHScale = 1;
#if HM_QTBT_AS_IN_JEM_QUANT
#if JVET_M0119_NO_TRANSFORM_SKIP_QUANTISATION_ADJUSTMENT
  if( TU::needsBlockSizeTrafoScale( tu, compID ) )
#else
  if( TU::needsBlockSizeTrafoScale( rect ) )
#endif
  {
    iTransformShift += ADJ_QUANT_SHIFT;
    iWHScale = 181;
  }
#endif

  const int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;
  // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

  // iAdd is different from the iAdd used in normal quantization
  const int64_t iAdd = int64_t(compID == COMPONENT_Y ? 171 : 256) << (iQBits - 9);

  for (int uiBlockPos = 0; uiBlockPos < rect.area(); uiBlockPos++)
  {
    const TCoeff iLevel   = piCoef.buf[uiBlockPos];
#if HEVC_USE_SCALING_LISTS
    const int64_t  tmpLevel = (int64_t)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);
#else
    const int64_t  tmpLevel = (int64_t)abs(iLevel) * defaultQuantisationCoefficient;
#endif
    const TCoeff quantisedMagnitude = TCoeff((tmpLevel * iWHScale + iAdd ) >> iQBits);

    if (quantisedMagnitude != 0)
    {
      return true;
    }
  } // for n
  return false;
}


void Quant::transformSkipQuantOneSample(TransformUnit &tu, const ComponentID &compID, const TCoeff &resiDiff, TCoeff &coeff, const uint32_t &uiPos, const QpParam &cQP, const bool bUseHalfRoundingPoint)
{
  const SPS           &sps = *tu.cs->sps;
  const CompArea      &rect                           = tu.blocks[compID];
#if HEVC_USE_SCALING_LISTS
  const uint32_t           uiWidth                        = rect.width;
  const uint32_t           uiHeight                       = rect.height;
#endif
  const int            maxLog2TrDynamicRange          = sps.getMaxLog2TrDynamicRange(toChannelType(compID));
  const int            channelBitDepth                = sps.getBitDepth(toChannelType(compID));
  const int            iTransformShift                = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);
#if HEVC_USE_SCALING_LISTS
  const int            scalingListType                = getScalingListType(tu.cu->predMode, compID);
  const bool           enableScalingLists             = getUseScalingList(uiWidth, uiHeight, true);
#endif
  const int            defaultQuantisationCoefficient = g_quantScales[cQP.rem];

#if HEVC_USE_SCALING_LISTS
  VTMCHECK( scalingListType >= SCALING_LIST_NUM, "Invalid scaling list" );

  const uint32_t uiLog2TrWidth      = g_aucLog2[uiWidth];
  const uint32_t uiLog2TrHeight     = g_aucLog2[uiHeight];
  const int *const piQuantCoeff = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrWidth-1, uiLog2TrHeight-1);
#endif

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
  * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
  * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
  * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
  */

  const int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;
  // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset
  const int iAdd = int64_t(bUseHalfRoundingPoint ? 256 : (tu.cs->slice->isIRAP() ? 171 : 85)) << int64_t(iQBits - 9);
  TCoeff transformedCoefficient;

  // transform-skip
  if (iTransformShift >= 0)
  {
    transformedCoefficient = resiDiff << iTransformShift;
  }
  else // for very high bit depths
  {
    const int iTrShiftNeg  = -iTransformShift;
    const int offset       = 1 << (iTrShiftNeg - 1);
    transformedCoefficient = ( resiDiff + offset ) >> iTrShiftNeg;
  }

  // quantization
  const TCoeff iSign = (transformedCoefficient < 0 ? -1: 1);

#if HEVC_USE_SCALING_LISTS
  const int quantisationCoefficient = enableScalingLists ? piQuantCoeff[uiPos] : defaultQuantisationCoefficient;

  const int64_t tmpLevel = (int64_t)abs(transformedCoefficient) * quantisationCoefficient;
#else
  const int64_t tmpLevel = (int64_t)abs(transformedCoefficient) * defaultQuantisationCoefficient;
#endif

  const TCoeff quantisedCoefficient = (TCoeff((tmpLevel + iAdd ) >> iQBits)) * iSign;

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;
  coeff = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
}

void Quant::invTrSkipDeQuantOneSample(TransformUnit &tu, const ComponentID &compID, const TCoeff &inSample, Pel &reconSample, const uint32_t &uiPos, const QpParam &cQP)
{
  const SPS           &sps                    = *tu.cs->sps;
  const CompArea      &rect                   = tu.blocks[compID];
#if HEVC_USE_SCALING_LISTS
  const uint32_t           uiWidth                = rect.width;
  const uint32_t           uiHeight               = rect.height;
#endif
  const int            QP_per                 = cQP.per;
  const int            QP_rem                 = cQP.rem;
  const int            maxLog2TrDynamicRange  = sps.getMaxLog2TrDynamicRange(toChannelType(compID));
  const int            channelBitDepth        = sps.getBitDepth(toChannelType(compID));
  const int            iTransformShift        = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);
#if HEVC_USE_SCALING_LISTS
  const int            scalingListType        = getScalingListType(tu.cu->predMode, compID);
  const bool           enableScalingLists     = getUseScalingList(uiWidth, uiHeight, true);

  VTMCHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  const int rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);
#else
  const int rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per));
#endif

  const TCoeff transformMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff transformMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  // De-quantisation

  TCoeff dequantisedSample;

#if HEVC_USE_SCALING_LISTS
  if (enableScalingLists)
  {
    const uint32_t             dequantCoefBits = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const uint32_t             targetInputBitDepth = std::min<uint32_t>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum =  (1 << (targetInputBitDepth - 1)) - 1;

    const uint32_t uiLog2TrWidth  = g_aucLog2[uiWidth];
    const uint32_t uiLog2TrHeight = g_aucLog2[uiHeight];
    int *piDequantCoef        = getDequantCoeff(scalingListType,QP_rem,uiLog2TrWidth-1, uiLog2TrHeight-1);

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ = ((Intermediate_Int(clipQCoef) * piDequantCoef[uiPos]) + iAdd) >> rightShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum, transformMaximum, iCoeffQ));
    }
    else
    {
      const int              leftShift = -rightShift;
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ = (Intermediate_Int(clipQCoef) * piDequantCoef[uiPos]) << leftShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum, transformMaximum, iCoeffQ));
    }
  }
  else
  {
#endif
    const int scale = g_invQuantScales[QP_rem];
    const int scaleBits = (IQUANT_SHIFT + 1);

    const uint32_t             targetInputBitDepth = std::min<uint32_t>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum = (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum, transformMaximum, iCoeffQ));
    }
    else
    {
      const int              leftShift = -rightShift;
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ = (Intermediate_Int(clipQCoef) * scale) << leftShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum, transformMaximum, iCoeffQ));
    }
#if HEVC_USE_SCALING_LISTS
  }
#endif

  // Inverse transform-skip

  if (iTransformShift >= 0)
  {
    const TCoeff offset = iTransformShift == 0 ? 0 : (1 << (iTransformShift - 1));
    reconSample = Pel((dequantisedSample + offset) >> iTransformShift);
  }
  else //for very high bit depths
  {
    const int iTrShiftNeg = -iTransformShift;
    reconSample = Pel(dequantisedSample << iTrShiftNeg);
  }
}


//! \}
