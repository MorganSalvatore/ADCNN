#ifndef __CNNLOOPFILTER__
#define __CNNLOOPFILTER__

#define NOMINMAX
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/cc/ops/math_ops.h"

#include "CommonDef.h"
#include "Unit.h"
#include "CodingStructure.h"


using namespace tensorflow;


class CNNLoopFilter
{
public:
	CNNLoopFilter();
	virtual ~CNNLoopFilter();
	void initTF(std::string pbpath, std::string workingmode, std::string gpuid);
	void create(const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE]);
	void destroy();
	void CNNLFProcess(CodingStructure& cs, CnnlfSliceParam& cnnlfSliceParam, int QP);
	Session *session;

protected:
	PelStorage                   m_tempCnnBuf;
	uint8_t*                     m_ctuEnableFlag[MAX_NUM_COMPONENT];
	int                          m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];
	int                          m_picWidth;
	int                          m_picHeight;
	int                          m_maxCUWidth;
	int                          m_maxCUHeight;
	int                          m_maxCUDepth;
	int                          m_numCTUsInWidth;
	int                          m_numCTUsInHeight;
	int                          m_numCTUsInPic;
	ChromaFormat                 m_chromaFormat;

	void runCNNLF(CodingStructure& cs, const PelUnitBuf& recUnitBuf, PelUnitBuf& cnnUnitBuf, int QP);
	void filterBlk(PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compID);
};

#endif