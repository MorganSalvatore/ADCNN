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

/** \file     CABACWriter.h
 *  \brief    Writer for low level syntax
 */

#ifndef __CABACWRITER__
#define __CABACWRITER__

#include "CommonLib/BitStream.h"
#include "CommonLib/ContextModelling.h"
#include "BinEncoder.h"


//! \ingroup EncoderLib
//! \{


class EncCu;
class CABACWriter
{
public:
  CABACWriter(BinEncIf& binEncoder)   : m_BinEncoder(binEncoder), m_Bitstream(0) { m_TestCtx = m_BinEncoder.getCtx(); m_EncCu = NULL; }
  virtual ~CABACWriter() {}

public:
  void        initCtxModels             ( const Slice&                  slice );
  void        setEncCu(EncCu* pcEncCu) { m_EncCu = pcEncCu; }
  SliceType   getCtxInitId              ( const Slice&                  slice );
  void        initBitstream             ( OutputBitstream*              bitstream )           { m_Bitstream = bitstream; m_BinEncoder.init( m_Bitstream ); }

  const Ctx&  getCtx                    ()                                            const   { return m_BinEncoder.getCtx();  }
  Ctx&        getCtx                    ()                                                    { return m_BinEncoder.getCtx();  }

  void        start                     ()                                                    { m_BinEncoder.start(); }
  void        resetBits                 ()                                                    { m_BinEncoder.resetBits(); }
  uint64_t    getEstFracBits            ()                                            const   { return m_BinEncoder.getEstFracBits(); }
  uint32_t    getNumBins                ()                                                    { return m_BinEncoder.getNumBins(); }
  bool        isEncoding                ()                                                    { return m_BinEncoder.isEncoding(); }

public:
  // slice segment data (clause 7.3.8.1)
  void        end_of_slice              ();

  // coding tree unit (clause 7.3.8.2)
  void        coding_tree_unit          (       CodingStructure&        cs,       const UnitArea&   area,       int (&qps)[2],  unsigned ctuRsAddr,  bool skipSao = false );

  // sao (clause 7.3.8.3)
  void        sao                       ( const Slice&                  slice,    unsigned          ctuRsAddr );
  void        sao_block_pars            ( const SAOBlkParam&            saoPars,  const BitDepths&  bitDepths,  bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo );
  void        sao_offset_pars           ( const SAOOffset&              ctbPars,  ComponentID       compID,     bool sliceEnabled,  int bitDepth );
  // coding (quad)tree (clause 7.3.8.4)
  void        coding_tree               ( const CodingStructure&        cs,       Partitioner&      pm,         CUCtx& cuCtx, Partitioner* pPartitionerChroma = nullptr, CUCtx* pCuCtxChroma = nullptr);
#if JVET_M0421_SPLIT_SIG
  void        split_cu_mode             ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm );
#else
  void        split_cu_flag             ( bool                          split,    const CodingStructure& cs,    Partitioner& pm );
  void        split_cu_mode_mt          ( const PartSplit               split,    const CodingStructure& cs,    Partitioner& pm );
#endif

  // coding unit (clause 7.3.8.5)
  void        coding_unit               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        cu_transquant_bypass_flag ( const CodingUnit&             cu );
  void        cu_skip_flag              ( const CodingUnit&             cu );
  void        pred_mode                 ( const CodingUnit&             cu );
  void        pcm_data                  ( const CodingUnit&             cu,       Partitioner&      pm );
  void        pcm_flag                  ( const CodingUnit&             cu,       Partitioner&      pm );
  void        cu_pred_data              ( const CodingUnit&             cu );
  void        cu_gbi_flag               ( const CodingUnit&             cu );
  void        extend_ref_line           (const PredictionUnit&          pu );
  void        extend_ref_line           (const CodingUnit&              cu );
  void        intra_luma_pred_modes     ( const CodingUnit&             cu );
  void        intra_luma_pred_mode      ( const PredictionUnit&         pu );
  void        intra_chroma_pred_modes   ( const CodingUnit&             cu );
  void        intra_chroma_lmc_mode     ( const PredictionUnit&         pu );
  void        intra_chroma_pred_mode    ( const PredictionUnit&         pu );
  void        cu_residual               ( const CodingUnit&             cu,       Partitioner&      pm,         CUCtx& cuCtx );
  void        rqt_root_cbf              ( const CodingUnit&             cu );
#if JVET_M0140_SBT
  void        sbt_mode                  ( const CodingUnit&             cu );
#endif
  void        end_of_ctu                ( const CodingUnit&             cu,       CUCtx&            cuCtx );

  // prediction unit (clause 7.3.8.6)
  void        prediction_unit           ( const PredictionUnit&         pu );
  void        merge_flag                ( const PredictionUnit&         pu );
  void        affine_flag               ( const CodingUnit&             cu );
  void        subblock_merge_flag       ( const CodingUnit&             cu );
  void        merge_idx                 ( const PredictionUnit&         pu );
  void        mmvd_merge_idx(const PredictionUnit&         pu);
  void        imv_mode                  ( const CodingUnit&             cu );
#if JVET_M0246_AFFINE_AMVR
  void        affine_amvr_mode          ( const CodingUnit&             cu );
#endif
  void        inter_pred_idc            ( const PredictionUnit&         pu );
  void        ref_idx                   ( const PredictionUnit&         pu,       RefPicList        eRefList );
  void        mvp_flag                  ( const PredictionUnit&         pu,       RefPicList        eRefList );

  void        MHIntra_flag              ( const PredictionUnit&         pu );
  void        MHIntra_luma_pred_modes   ( const CodingUnit&             cu );
  void        triangle_mode             ( const CodingUnit&             cu );
#if JVET_M0444_SMVD
  void        smvd_mode              ( const PredictionUnit&         pu );
#endif

  // pcm samples (clause 7.3.8.7)
  void        pcm_samples               ( const TransformUnit&          tu );

  // transform tree (clause 7.3.8.8)
#if JVET_M0102_INTRA_SUBPARTITIONS
  void        transform_tree            ( const CodingStructure&        cs,       Partitioner&      pm,     CUCtx& cuCtx, ChromaCbfs& chromaCbfs, const PartSplit ispType = TU_NO_ISP, const int subTuIdx = -1 );
  void        cbf_comp                  ( const CodingStructure&        cs,       bool              cbf,    const CompArea& area, unsigned depth, const bool prevCbCbf = false, const bool useISP = false );
#else
  void        transform_tree            ( const CodingStructure&        cs,       Partitioner&      pm,     CUCtx& cuCtx,   ChromaCbfs& chromaCbfs );
  void        cbf_comp                  ( const CodingStructure&        cs,       bool              cbf,    const CompArea& area, unsigned depth, const bool prevCbCbf = false );
#endif

  // mvd coding (clause 7.3.8.9)
#if JVET_M0246_AFFINE_AMVR
  void        mvd_coding                ( const Mv &rMvd, int8_t imv );
#else
  void        mvd_coding                ( const Mv &rMvd, uint8_t imv );
#endif
  // transform unit (clause 7.3.8.10)
  void        transform_unit            ( const TransformUnit&          tu,       CUCtx&            cuCtx,  ChromaCbfs& chromaCbfs );
  void        cu_qp_delta               ( const CodingUnit&             cu,       int               predQP, const int8_t qp );
  void        cu_chroma_qp_offset       ( const CodingUnit&             cu );

  // residual coding (clause 7.3.8.11)
  void        residual_coding           ( const TransformUnit&          tu,       ComponentID       compID );
#if JVET_M0464_UNI_MTS
  void        mts_coding                ( const TransformUnit&          tu,       ComponentID       compID );
#else
  void        transform_skip_flag       ( const TransformUnit&          tu,       ComponentID       compID );
  void        emt_tu_index              ( const TransformUnit&          tu );
  void        emt_cu_flag               ( const CodingUnit&             cu );
#endif
#if JVET_M0102_INTRA_SUBPARTITIONS
  void        isp_mode                  ( const CodingUnit&             cu );
#endif
  void        explicit_rdpcm_mode       ( const TransformUnit&          tu,       ComponentID       compID );
#if JVET_M0297_32PT_MTS_ZERO_OUT
  void        last_sig_coeff            ( CoeffCodingContext&           cctx,     const TransformUnit& tu, ComponentID       compID );
#else
  void        last_sig_coeff            ( CoeffCodingContext&           cctx );
#endif
  void        residual_coding_subblock  ( CoeffCodingContext&           cctx,     const TCoeff*     coeff, const int stateTransTable, int& state );

  // cross component prediction (clause 7.3.8.12)
  void        cross_comp_pred           ( const TransformUnit&          tu,       ComponentID       compID );

  void        codeAlfCtuEnableFlags     ( CodingStructure& cs, ChannelType channel, AlfSliceParam* alfParam);
  void        codeAlfCtuEnableFlags     ( CodingStructure& cs, ComponentID compID, AlfSliceParam* alfParam);
  void        codeAlfCtuEnableFlag      ( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, AlfSliceParam* alfParam = NULL );

#if WMZ_CNNLF
  void        codeCnnlfCtuEnableFlags(CodingStructure& cs, ChannelType channel, CnnlfSliceParam* cnnlfParam);
  void        codeCnnlfCtuEnableFlags(CodingStructure& cs, ComponentID compID, CnnlfSliceParam* cnnlfParam);
  void        codeCnnlfCtuEnableFlag(CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, CnnlfSliceParam* cnnlfParam = NULL);
#endif

private:
  void        unary_max_symbol          ( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol );
  void        unary_max_eqprob          ( unsigned symbol,                                   unsigned maxSymbol );
  void        exp_golomb_eqprob         ( unsigned symbol, unsigned count );
#if !REMOVE_BIN_DECISION_TREE
  void        encode_sparse_dt          ( DecisionTree& dt, unsigned toCodeId );
#endif

  // statistic
  unsigned    get_num_written_bits()    { return m_BinEncoder.getNumWrittenBits(); }

  void  xWriteTruncBinCode(uint32_t uiSymbol, uint32_t uiMaxSymbol);

private:
  BinEncIf&         m_BinEncoder;
  OutputBitstream*  m_Bitstream;
  Ctx               m_TestCtx;
  EncCu*            m_EncCu;
};



class CABACEncoder
{
public:
  CABACEncoder()
    : m_CABACWriterStd      ( m_BinEncoderStd )
    , m_CABACEstimatorStd   ( m_BitEstimatorStd )
    , m_CABACWriter         { &m_CABACWriterStd,   }
    , m_CABACEstimator      { &m_CABACEstimatorStd }
  {}

  CABACWriter*                getCABACWriter          ( const SPS*   sps   )        { return m_CABACWriter   [0]; }
  CABACWriter*                getCABACEstimator       ( const SPS*   sps   )        { return m_CABACEstimator[0]; }
private:
  BinEncoder_Std      m_BinEncoderStd;
  BitEstimator_Std    m_BitEstimatorStd;
  CABACWriter         m_CABACWriterStd;
  CABACWriter         m_CABACEstimatorStd;
  CABACWriter*        m_CABACWriter   [BPM_NUM-1];
  CABACWriter*        m_CABACEstimator[BPM_NUM-1];
};

//! \}

#endif //__CABACWRITER__
