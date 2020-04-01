#ifndef __ENCCNNLOOPFILTER__
#define __ENCCNNLOOPFILTER__

#include "CommonLib/CNNLoopFilter.h"
#include "CABACWriter.h"
#if ADCNN
class EncCNNLoopFilter : public CNNLoopFilter
{

private:
	uint8_t*               m_ctuEnableFlagTmp[MAX_NUM_COMPONENT];
	CnnlfSliceParam        m_cnnlfSliceParamTemp;
	CABACWriter*           m_CABACEstimator;
	CtxCache*              m_CtxCache;
	double                 m_lambda[MAX_NUM_COMPONENT];
	const double           FracBitsScale = 1.0 / double(1 << SCALE_BITS);

public:
	EncCNNLoopFilter();
	virtual ~EncCNNLoopFilter() {}
	void CNNLFProcess(CodingStructure& cs, const double *lambdas, CnnlfSliceParam& cnnlfSliceParam, int QP);
	void initCABACEstimator(CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice);
	void create(const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE]);
	void destroy();

private:
	void   cnnlfEncoder(CodingStructure& cs, CnnlfSliceParam& cnnlfSliceParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& cnnUnitBuf, PelUnitBuf& recUnitBuf, const ChannelType channel);
	void   copyCnnlfSliceParam(CnnlfSliceParam& cnnlfSliceParamDst, CnnlfSliceParam& cnnlfSliceParamSrc, ChannelType channel);
	double deriveCtbCnnlfEnableFlags(CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& cnnUnitBuf, const PelUnitBuf& recUnitBuf, ChannelType channel);
	double getOrgCost(const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& recUnitBuf, ChannelType channel);
	double getCnnCost(const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& recUnitBuf, ChannelType channel);
	void setEnableFlag(CnnlfSliceParam& cnnlfSlicePara, ChannelType channel, bool val);
	void setEnableFlag(CnnlfSliceParam& cnnlfSlicePara, ChannelType channel, uint8_t** ctuFlags);
	void setCtuEnableFlag(uint8_t** ctuFlags, ChannelType channel, uint8_t val);
	void copyCtuEnableFlag(uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel);
	uint64_t xCalcSSD(const CPelBuf& refBuf, const CPelBuf& cmpBuf, ChannelType ch);
	int lengthTruncatedUnary(int symbol, int maxSymbol);

};
#endif

#endif
