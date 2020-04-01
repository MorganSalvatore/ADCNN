#include "EncCNNLoopFilter.h"
#include "CommonLib/Picture.h"
#include "CommonLib/CodingStructure.h"

#if ADCNN
#define CnnlfCtx(c) SubCtx( Ctx::ctbCnnlfFlag, c )

EncCNNLoopFilter::EncCNNLoopFilter()
	: m_CABACEstimator(nullptr)
{
}

void EncCNNLoopFilter::create(const int picWidth, const int picHeight, const ChromaFormat chromaFormatIDC, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE])
{
	CNNLoopFilter::create(picWidth, picHeight, chromaFormatIDC, maxCUWidth, maxCUHeight, maxCUDepth, inputBitDepth);
	for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
	{
		m_ctuEnableFlagTmp[compIdx] = new uint8_t[m_numCTUsInPic];
	}
}

void EncCNNLoopFilter::destroy()
{
	for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
	{
		if (m_ctuEnableFlagTmp[compIdx])
		{
			delete[] m_ctuEnableFlagTmp[compIdx];
			m_ctuEnableFlagTmp[compIdx] = nullptr;
		}
	}
	CNNLoopFilter::destroy();
}

void EncCNNLoopFilter::initCABACEstimator(CABACEncoder* cabacEncoder, CtxCache* ctxCache, Slice* pcSlice)
{
	m_CABACEstimator = cabacEncoder->getCABACEstimator(pcSlice->getSPS());
	m_CtxCache = ctxCache;
	m_CABACEstimator->initCtxModels(*pcSlice);
	m_CABACEstimator->resetBits();
}

void EncCNNLoopFilter::CNNLFProcess(CodingStructure& cs, const double *lambdas, CnnlfSliceParam& cnnlfSliceParam, int QP)
{
	// ctu enable flag
	for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
	{
		m_ctuEnableFlag[compIdx] = cs.picture->getCnnlfCtuEnableFlag(compIdx);
	}
	cnnlfSliceParam.reset();
	int shiftLuma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_LUMA]);
	int shiftChroma = 2 * DISTORTION_PRECISION_ADJUSTMENT(m_inputBitDepth[CHANNEL_TYPE_CHROMA]);
	m_lambda[COMPONENT_Y] = lambdas[COMPONENT_Y] * double(1 << shiftLuma);
	m_lambda[COMPONENT_Cb] = lambdas[COMPONENT_Cb] * double(1 << shiftChroma);
	m_lambda[COMPONENT_Cr] = lambdas[COMPONENT_Cr] * double(1 << shiftChroma);

	PelUnitBuf orgUnitBuf = cs.getOrgBuf();
	PelUnitBuf recUnitBuf = cs.getRecoBuf();
	m_tempCnnBuf.copyFrom(recUnitBuf);
	PelUnitBuf cnnUnitBuf = m_tempCnnBuf.getBuf(cs.area);

	VTMCHECK(orgUnitBuf.bufs.size() != cnnUnitBuf.bufs.size(), "Error buf size.");

	// run CNNLF
	runCNNLF(cs, recUnitBuf, cnnUnitBuf, QP);

	// on/off control
	cnnlfEncoder(cs, cnnlfSliceParam, orgUnitBuf, cnnUnitBuf, recUnitBuf, CHANNEL_TYPE_LUMA);
	if (cnnlfSliceParam.enabledFlag[COMPONENT_Y])
	{
		cnnlfEncoder(cs, cnnlfSliceParam, orgUnitBuf, cnnUnitBuf, recUnitBuf, CHANNEL_TYPE_CHROMA);
	}
}

double EncCNNLoopFilter::deriveCtbCnnlfEnableFlags(CodingStructure& cs, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& cnnUnitBuf, const PelUnitBuf& recUnitBuf, ChannelType channel)
{
	TempCtx        ctxTempStart(m_CtxCache);
	TempCtx        ctxTempBest(m_CtxCache);
	const ComponentID compIDFirst = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cb;
	const ComponentID compIDLast = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cr;
	int ctuRsAddr = 0;
	double cost = 0;
	for (int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
	{
		for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth)
		{
			const int width = (xPos + m_maxCUWidth > m_picWidth) ? (m_picWidth - xPos) : m_maxCUWidth;
			const int height = (yPos + m_maxCUHeight > m_picHeight) ? (m_picHeight - yPos) : m_maxCUHeight;
			const UnitArea area(m_chromaFormat, Area(xPos, yPos, width, height));

			for (int compID = compIDFirst; compID <= compIDLast; compID++)
			{
				const ComponentID comp = ComponentID(compID);
				const CompArea& compArea = area.block(comp);
				const ChannelType ch = toChannelType(comp);

				CPelBuf orgBufCUs = orgUnitBuf.get(comp).subBuf(compArea.pos(), compArea.size());
				CPelBuf recBufCUs = recUnitBuf.get(comp).subBuf(compArea.pos(), compArea.size());
				CPelBuf cnnBufCUs = cnnUnitBuf.get(comp).subBuf(compArea.pos(), compArea.size());

				ctxTempStart = CnnlfCtx(m_CABACEstimator->getCtx());   // store the previous ctx
				m_CABACEstimator->resetBits();
				m_ctuEnableFlag[compID][ctuRsAddr] = 1;
				m_CABACEstimator->codeCnnlfCtuEnableFlag(cs, ctuRsAddr, compID, &m_cnnlfSliceParamTemp);
				double costOn = (double)xCalcSSD(orgBufCUs, cnnBufCUs, ch);
				costOn += m_lambda[compID] * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();
				ctxTempBest = CnnlfCtx(m_CABACEstimator->getCtx());   //  store the cnn ctx

				m_CABACEstimator->getCtx() = CnnlfCtx(ctxTempStart);  // use the previou ctx for comparision
				m_CABACEstimator->resetBits();
				m_ctuEnableFlag[compID][ctuRsAddr] = 0;
				m_CABACEstimator->codeCnnlfCtuEnableFlag(cs, ctuRsAddr, compID, &m_cnnlfSliceParamTemp);

				double costOff = (double)xCalcSSD(orgBufCUs, recBufCUs, ch);
				costOff += m_lambda[compID] * FracBitsScale*(double)m_CABACEstimator->getEstFracBits();

				if (costOn < costOff)
				{
					cost += costOn;
					m_CABACEstimator->getCtx() = CnnlfCtx(ctxTempBest);  // change the ctx
					m_ctuEnableFlag[compID][ctuRsAddr] = 1;
				}
				else
				{
					cost += costOff;
					m_ctuEnableFlag[compID][ctuRsAddr] = 0;
				}
			}
			ctuRsAddr++;
		}
	}
	if (isChroma(channel))
	{
		setEnableFlag(m_cnnlfSliceParamTemp, channel, m_ctuEnableFlag);
		const int cnnlfChromaIdc = m_cnnlfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_cnnlfSliceParamTemp.enabledFlag[COMPONENT_Cr];
		cost += lengthTruncatedUnary(cnnlfChromaIdc, 3) * m_lambda[channel];
	}
	return cost;
}

void EncCNNLoopFilter::cnnlfEncoder(CodingStructure& cs, CnnlfSliceParam& cnnlfSliceParam, const PelUnitBuf& orgUnitBuf, const PelUnitBuf& cnnUnitBuf, PelUnitBuf& recUnitBuf, const ChannelType channel)
{
	const TempCtx  ctxStart(m_CtxCache, CnnlfCtx(m_CABACEstimator->getCtx()));
	TempCtx        ctxBest(m_CtxCache);

	double costMin = MAX_DOUBLE;
	m_cnnlfSliceParamTemp = cnnlfSliceParam;

	//1. get unfiltered distortion
	double cost = getOrgCost(orgUnitBuf, recUnitBuf, channel);
	cost /= 1.001; // slight preference for unfiltered choice
	if (cost < costMin)
	{
		costMin = cost;
		setEnableFlag(cnnlfSliceParam, channel, false);
		ctxBest = CnnlfCtx(ctxStart);
		setCtuEnableFlag(m_ctuEnableFlagTmp, channel, 0);
	}

	//2. all CTUs are on
	setEnableFlag(m_cnnlfSliceParamTemp, channel, true);
	m_CABACEstimator->getCtx() = CnnlfCtx(ctxStart);
	cost = getCnnCost(orgUnitBuf, cnnUnitBuf, channel);
	if (cost < costMin)
	{
		costMin = cost;
		copyCnnlfSliceParam(cnnlfSliceParam, m_cnnlfSliceParamTemp, channel);
		ctxBest = CnnlfCtx(m_CABACEstimator->getCtx());
		setCtuEnableFlag(m_ctuEnableFlagTmp, channel, 1);
	}

	//3. CTU decision
	setEnableFlag(m_cnnlfSliceParamTemp, channel, true);
	m_CABACEstimator->getCtx() = CnnlfCtx(ctxStart);
	cost = deriveCtbCnnlfEnableFlags(cs, orgUnitBuf, cnnUnitBuf, recUnitBuf, channel);
	if (cost < costMin)
	{
		costMin = cost;
		ctxBest = CnnlfCtx(m_CABACEstimator->getCtx());
		copyCnnlfSliceParam(cnnlfSliceParam, m_cnnlfSliceParamTemp, channel);
		copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
	}
	m_CABACEstimator->getCtx() = CnnlfCtx(ctxBest);
	copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, channel);

	//4. filtering
	const ComponentID compIDFirst = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cb;
	const ComponentID compIDLast = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cr;

	for (int compIdx = compIDFirst; compIdx <= compIDLast; compIdx++)
	{
		ComponentID compID = (ComponentID)compIdx;
		if (cnnlfSliceParam.enabledFlag[compID])
		{
			const PreCalcValues& pcv = *cs.pcv;
			int ctuIdx = 0;
			const int chromaScaleX = getComponentScaleX(compID, recUnitBuf.chromaFormat);
			const int chromaScaleY = getComponentScaleY(compID, recUnitBuf.chromaFormat);

			for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
			{
				for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
				{
					const int width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
					const int height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;
					Area blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);

					if (m_ctuEnableFlag[compID][ctuIdx])
					{
						filterBlk(recUnitBuf, cnnUnitBuf, blk, compID);
					}
					ctuIdx++;
				}
			}
		}
	}
}

void EncCNNLoopFilter::copyCnnlfSliceParam(CnnlfSliceParam& cnnlfSliceParamDst, CnnlfSliceParam& cnnlfSliceParamSrc, ChannelType channel)
{
	if (isLuma(channel))
	{
		cnnlfSliceParamDst.enabledFlag[COMPONENT_Y] = cnnlfSliceParamSrc.enabledFlag[COMPONENT_Y];
	}
	else
	{
		cnnlfSliceParamDst.enabledFlag[COMPONENT_Cb] = cnnlfSliceParamSrc.enabledFlag[COMPONENT_Cb];
		cnnlfSliceParamDst.enabledFlag[COMPONENT_Cr] = cnnlfSliceParamSrc.enabledFlag[COMPONENT_Cr];
	}
}
double EncCNNLoopFilter::getCnnCost(const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& cnnUnitBuf, ChannelType channel)
{
	double cost = 0;
	if (isLuma(channel))
	{
		cost += xCalcSSD(orgUnitBuf.get(COMPONENT_Y), cnnUnitBuf.get(COMPONENT_Y), toChannelType(COMPONENT_Y));
	}
	else
	{
		cost += xCalcSSD(orgUnitBuf.get(COMPONENT_Cb), cnnUnitBuf.get(COMPONENT_Cb), toChannelType(COMPONENT_Cb));
		cost += xCalcSSD(orgUnitBuf.get(COMPONENT_Cr), cnnUnitBuf.get(COMPONENT_Cr), toChannelType(COMPONENT_Cr));
		const int cnnlfChromaIdc = m_cnnlfSliceParamTemp.enabledFlag[COMPONENT_Cb] * 2 + m_cnnlfSliceParamTemp.enabledFlag[COMPONENT_Cr];
		cost += lengthTruncatedUnary(cnnlfChromaIdc, 3) * m_lambda[channel];
	}
	return cost;
}

double EncCNNLoopFilter::getOrgCost(const CPelUnitBuf& orgUnitBuf, const CPelUnitBuf& recUnitBuf, ChannelType channel)
{
	double dist = 0;
	if (isLuma(channel))
	{
		dist = (double)xCalcSSD(orgUnitBuf.get(COMPONENT_Y), recUnitBuf.get(COMPONENT_Y), toChannelType(COMPONENT_Y));
	}
	else
	{
		dist += (double)xCalcSSD(orgUnitBuf.get(COMPONENT_Cb), recUnitBuf.get(COMPONENT_Cb), toChannelType(COMPONENT_Cb));
		dist += (double)xCalcSSD(orgUnitBuf.get(COMPONENT_Cr), recUnitBuf.get(COMPONENT_Cr), toChannelType(COMPONENT_Cr));
		dist += (double)lengthTruncatedUnary(0, 3) * m_lambda[COMPONENT_Cb];
	}
	return dist;
}

uint64_t EncCNNLoopFilter::xCalcSSD(const CPelBuf& refBuf, const CPelBuf& cmpBuf, ChannelType ch)
{
	int iWidth = refBuf.width;
	int iHeight = refBuf.height;
	int orgStride = refBuf.stride;
	int cmpStride = cmpBuf.stride;
	const Pel* pOrg = refBuf.buf;
	const Pel* pCmp = cmpBuf.buf;
	uint64_t uiSSD = 0;
	int x, y;
	int iTemp;

	for (y = 0; y < iHeight; y++)
	{
		for (x = 0; x < iWidth; x++)
		{
			iTemp = pOrg[x] - pCmp[x];
			uiSSD += (iTemp * iTemp);
		}
		pOrg += orgStride;
		pCmp += cmpStride;
	}
	return uiSSD;
}

int EncCNNLoopFilter::lengthTruncatedUnary(int symbol, int maxSymbol)
{
	if (maxSymbol == 0)
	{
		return 0;
	}

	bool codeLast = (maxSymbol > symbol);
	int bins = 0;
	int numBins = 0;
	while (symbol--)
	{
		bins <<= 1;
		bins++;
		numBins++;
	}
	if (codeLast)
	{
		bins <<= 1;
		numBins++;
	}

	return numBins;
}

void EncCNNLoopFilter::setEnableFlag(CnnlfSliceParam& cnnlfSlicePara, ChannelType channel, bool val)
{
	if (channel == CHANNEL_TYPE_LUMA)
	{
		cnnlfSlicePara.enabledFlag[COMPONENT_Y] = val;
	}
	else
	{
		cnnlfSlicePara.enabledFlag[COMPONENT_Cb] = cnnlfSlicePara.enabledFlag[COMPONENT_Cr] = val;
	}
}

void EncCNNLoopFilter::setEnableFlag(CnnlfSliceParam& cnnlfSlicePara, ChannelType channel, uint8_t** ctuFlags)
{
	const ComponentID compIDFirst = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cb;
	const ComponentID compIDLast = isLuma(channel) ? COMPONENT_Y : COMPONENT_Cr;
	for (int compId = compIDFirst; compId <= compIDLast; compId++)
	{
		cnnlfSlicePara.enabledFlag[compId] = false;
		for (int i = 0; i < m_numCTUsInPic; i++)
		{
			if (ctuFlags[compId][i])
			{
				cnnlfSlicePara.enabledFlag[compId] = true;
				break;
			}
		}
	}
}

void EncCNNLoopFilter::copyCtuEnableFlag(uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel)
{
	if (isLuma(channel))
	{
		memcpy(ctuFlagsDst[COMPONENT_Y], ctuFlagsSrc[COMPONENT_Y], sizeof(uint8_t) * m_numCTUsInPic);
	}
	else
	{
		memcpy(ctuFlagsDst[COMPONENT_Cb], ctuFlagsSrc[COMPONENT_Cb], sizeof(uint8_t) * m_numCTUsInPic);
		memcpy(ctuFlagsDst[COMPONENT_Cr], ctuFlagsSrc[COMPONENT_Cr], sizeof(uint8_t) * m_numCTUsInPic);
	}
}

void EncCNNLoopFilter::setCtuEnableFlag(uint8_t** ctuFlags, ChannelType channel, uint8_t val)
{
	if (isLuma(channel))
	{
		memset(ctuFlags[COMPONENT_Y], val, sizeof(uint8_t) * m_numCTUsInPic);
	}
	else
	{
		memset(ctuFlags[COMPONENT_Cb], val, sizeof(uint8_t) * m_numCTUsInPic);
		memset(ctuFlags[COMPONENT_Cr], val, sizeof(uint8_t) * m_numCTUsInPic);
	}
}

#endif