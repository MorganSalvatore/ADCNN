#include "CNNLoopFilter.h"

#if ADCNN
//#include "CodingStructure.h"
#include "Picture.h"
#include "CommonLib/UnitTools.h"



CNNLoopFilter::CNNLoopFilter()
{
	for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
	{
		m_ctuEnableFlag[compIdx] = nullptr;
	}
}

void CNNLoopFilter::initTF(std::string pbpath, std::string workingmode, std::string gpuid)
{
	if (workingmode == "CPU")
	{
		_putenv_s("CUDA_VISIBLE_DEVICES", "-1");
	}
	else if (workingmode == "GPU")
	{
		if (gpuid != "")
		{
			_putenv_s("CUDA_VISIBLE_DEVICES", gpuid.c_str());
		}
	}

	std::cout << "start initalize session" << std::endl;
	Status status = NewSession(SessionOptions(), &session);
	if (!status.ok()) {
		std::cout << status.ToString() << std::endl;
	}

	//加载tensorflow模型pb文件
	GraphDef graph_def;
	status = ReadBinaryProto(Env::Default(), pbpath, &graph_def);
	if (!status.ok()) {
		std::cout << status.ToString() << std::endl;
	}
	status = session->Create(graph_def);
	if (!status.ok()) {
		std::cout << status.ToString() << std::endl;
	}
	std::cout << "pb加载成功" << std::endl;
}

void CNNLoopFilter::create(const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE])
{
	std::memcpy(m_inputBitDepth, inputBitDepth, sizeof(m_inputBitDepth));
	m_picWidth = picWidth;
	m_picHeight = picHeight;
	m_maxCUWidth = maxCUWidth;
	m_maxCUHeight = maxCUHeight;
	m_maxCUDepth = maxCUDepth;
	m_numCTUsInWidth = (m_picWidth / m_maxCUWidth) + ((m_picWidth % m_maxCUWidth) ? 1 : 0);
	m_numCTUsInHeight = (m_picHeight / m_maxCUHeight) + ((m_picHeight % m_maxCUHeight) ? 1 : 0);
	m_numCTUsInPic = m_numCTUsInHeight * m_numCTUsInWidth;
	m_chromaFormat = format;

	//temporary picture buffer
	UnitArea picArea(format, Area(0, 0, picWidth, picHeight));

	m_tempCnnBuf.destroy();
	m_tempCnnBuf.create(picArea);
}

void CNNLoopFilter::destroy()
{
	m_tempCnnBuf.destroy();
	session->Close();
}

CNNLoopFilter::~CNNLoopFilter()
{
	destroy();
}

void CNNLoopFilter::CNNLFProcess(CodingStructure& cs, CnnlfSliceParam& cnnlfSliceParam, int QP)
{
	// set CTU enable flags
	for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
	{
		m_ctuEnableFlag[compIdx] = cs.picture->getCnnlfCtuEnableFlag(compIdx);
	}
	const PreCalcValues& pcv = *cs.pcv;

	for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
	{
		std::cout << cnnlfSliceParam.enabledFlag[compIdx] << std::endl;
		int ctuIdx = 0;
		for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
		{
			for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
			{
				std::cout << int(m_ctuEnableFlag[compIdx][ctuIdx]);
				std::cout << " ";
				ctuIdx++;
			}
			std::cout << std::endl;
		}
	}

	if (!cnnlfSliceParam.enabledFlag[COMPONENT_Y] && !cnnlfSliceParam.enabledFlag[COMPONENT_Cb] && !cnnlfSliceParam.enabledFlag[COMPONENT_Cr])
	{
		return;
	}



	PelUnitBuf recYuv = cs.getRecoBuf();
	m_tempCnnBuf.copyFrom(recYuv);
	PelUnitBuf cnnYuv = m_tempCnnBuf.getBuf(cs.area);

	// run CNNLF
	runCNNLF(cs, recYuv, cnnYuv, QP);

	
	int ctuIdx = 0;
	for (int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight)
	{
		for (int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth)
		{
			const int width = (xPos + pcv.maxCUWidth > pcv.lumaWidth) ? (pcv.lumaWidth - xPos) : pcv.maxCUWidth;
			const int height = (yPos + pcv.maxCUHeight > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUHeight;

			// LUMA
			if (m_ctuEnableFlag[COMPONENT_Y][ctuIdx])
			{
				Area blk(xPos, yPos, width, height);
				filterBlk(recYuv, cnnYuv, blk, COMPONENT_Y);
			}

			// CHROMA
			for (int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
			{
				ComponentID compID = ComponentID(compIdx);
				const int chromaScaleX = getComponentScaleX(compID, recYuv.chromaFormat);
				const int chromaScaleY = getComponentScaleY(compID, recYuv.chromaFormat);

				if (m_ctuEnableFlag[compIdx][ctuIdx])
				{
					Area blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
					filterBlk(recYuv, cnnYuv, blk, compID);
				}
			}
			ctuIdx++;
		}
	}
}

void writeComp2Tensor(PelUnitBuf& rec, Tensor& x, ComponentID comp, int hei, int wid, int bitdepth)
{
	Pel *p = rec.get(comp).bufAt(0, 0);
	int stride = rec.get(comp).stride;

	auto tensormap = x.tensor<float, 4>();

	for (int i = 0; i < hei; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			tensormap(0, i, j, 0) = p[j] / float((1 << bitdepth) - 1);
		}
		p += stride;
	}
}

void writeTensor2Comp(PelUnitBuf& rec, Tensor& in, Tensor& res, ComponentID comp, int hei, int wid, int bitdepth)
{
	Pel *p = rec.get(comp).bufAt(0, 0);
	int stride = rec.get(comp).stride;

	auto in_map = in.tensor<float, 4>();
	auto res_map = res.tensor<float, 4>();

	int max_range = (1 << bitdepth) - 1;

	for (int i = 0; i < hei; i++)
	{
		for (int j = 0; j < wid; j++)
		{
			p[j] = Clip3(0, max_range, int((in_map(0, i, j, 0) + res_map(0, i, j, 0)) * max_range + 0.5));
		}
		p += stride;
	}
}

void CNNLoopFilter::runCNNLF(CodingStructure& cs, const PelUnitBuf& recUnitBuf, PelUnitBuf& cnnUnitBuf, int QP)
{
	//将Y写入tensor
	Tensor y(DT_FLOAT, TensorShape({ 1, m_picHeight, m_picWidth, 1 }));
	writeComp2Tensor(cnnUnitBuf, y, COMPONENT_Y, m_picHeight, m_picWidth, m_inputBitDepth[CHANNEL_TYPE_LUMA]);

	//将U写入tensor
	Tensor u(DT_FLOAT, TensorShape({ 1, m_picHeight / 2, m_picWidth / 2, 1 }));
	writeComp2Tensor(cnnUnitBuf, u, COMPONENT_Cb, m_picHeight / 2, m_picWidth / 2, m_inputBitDepth[CHANNEL_TYPE_CHROMA]);

	//将V写入tensor
	Tensor v(DT_FLOAT, TensorShape({ 1, m_picHeight / 2, m_picWidth / 2, 1 }));
	writeComp2Tensor(cnnUnitBuf, v, COMPONENT_Cr, m_picHeight / 2, m_picWidth / 2, m_inputBitDepth[CHANNEL_TYPE_CHROMA]);

	//将QP写入tensor
	//Tensor QPmap(DT_FLOAT, TensorShape({ 1, m_picHeight , m_picWidth , 1 }));
	//auto tensormap_qp = QPmap.tensor<float, 4>();
	//for (int i = 0; i < m_picHeight ; i++)
	//{
	//	for (int j = 0; j < m_picWidth ; j++)
	//	{
	//		tensormap_qp(0, i, j, 0) = QP / float(MAX_QP);
	//	}
	//}
	Tensor QPmap(DT_FLOAT, TensorShape({ 1 }));
	auto tensormap_qp = QPmap.tensor<float, 1>();
	tensormap_qp(0) = QP / float(MAX_QP);


	//将CU写入tensor
	Tensor cumap_y(DT_FLOAT, TensorShape({ 1, m_picHeight, m_picWidth, 1 }));
	auto tensormap_cuy = cumap_y.tensor<float, 4>();

	Tensor cumap_uv(DT_FLOAT, TensorShape({ 1, m_picHeight / 2, m_picWidth / 2, 1 }));
	auto tensormap_cuuv = cumap_uv.tensor<float, 4>();

	const PreCalcValues& pcv = *cs.pcv;
	//遍历所有CTU
	for (int y = 0; y < pcv.heightInCtus; y++)
	{
		for (int x = 0; x < pcv.widthInCtus; x++)
		{
			const UnitArea ctuArea(pcv.chrFormat, Area(x << pcv.maxCUWidthLog2, y << pcv.maxCUHeightLog2, pcv.maxCUWidth, pcv.maxCUWidth));

			//遍历luma的所有CU
			for (auto &currCU : cs.traverseCUs(CS::getArea(cs, ctuArea, CH_L), CH_L))
			{
				CodingUnit& cu = currCU;
				const Area area = cu.Y();
				int cu_x = area.x;
				int cu_y = area.y;
				int cu_wid = area.width;
				int cu_hei = area.height;
				double pelsum = 0;
				for (int j = cu_y; j < cu_y + cu_hei; j++)
				{
					for (int k = cu_x; k < cu_x + cu_wid; k++)
					{
						if (j == cu_y || j == cu_y + cu_hei - 1 || k == cu_x || k == cu_x + cu_wid - 1)
							tensormap_cuy(0, j, k, 0) = 1.0;
						else
							tensormap_cuy(0, j, k, 0) = 0.5;
					}
				}

			}

			//遍历chroma的所有CU
			for (auto &currCU : cs.traverseCUs(CS::getArea(cs, ctuArea, CH_C), CH_C))
			{
				CodingUnit& cu = currCU;
				const Area area = cu.Cb();
				int cu_x = area.x;
				int cu_y = area.y;
				int cu_wid = area.width;
				int cu_hei = area.height;
				double pelsum_u = 0;
				double pelsum_v = 0;
				for (int j = cu_y; j < cu_y + cu_hei; j++)
				{
					for (int k = cu_x; k < cu_x + cu_wid; k++)
					{
						if (j == cu_y || j == cu_y + cu_hei - 1 || k == cu_x || k == cu_x + cu_wid - 1)
							tensormap_cuuv(0, j, k, 0) = 1.0;
						else
							tensormap_cuuv(0, j, k, 0) = 0.5;
					}
				}
			}
		}
	}


	//构造输入输出的vector<Tensor>
	//std::vector<std::pair<string, Tensor>> inputs = { { "input_Y", y },{ "input_U", u },{ "input_V", v },{ "CU_Y", cumap_y },{ "CU_UV",cumap_uv },{ "QP", QPmap } };
	std::vector<std::pair<string, Tensor>> inputs = { { "Sinput_Y", y },{ "Sinput_U", u },{ "Sinput_V", v },{ "SCU_Y", cumap_y },{ "SCU_UV",cumap_uv },{ "SQP", QPmap } };
	std::vector<Tensor> outputs;
	//status = session->Run(inputs, { "stage2_Y/res_Y/BiasAdd","stage2_U/res_U/BiasAdd","stage2_V/res_V/BiasAdd" }, {}, &outputs);
	session->Run(inputs, { "student/stage2_Y/res_Y/BiasAdd","student/stage2_U/res_U/BiasAdd","student/stage2_V/res_V/BiasAdd" }, {}, &outputs);

	Tensor res_y = outputs[0];
	writeTensor2Comp(cnnUnitBuf, y, res_y, COMPONENT_Y, m_picHeight, m_picWidth, m_inputBitDepth[CHANNEL_TYPE_LUMA]);

	Tensor res_u = outputs[1];
	writeTensor2Comp(cnnUnitBuf, u, res_u, COMPONENT_Cb, m_picHeight / 2, m_picWidth / 2, m_inputBitDepth[CHANNEL_TYPE_CHROMA]);

	Tensor res_v = outputs[2];
	writeTensor2Comp(cnnUnitBuf, v, res_v, COMPONENT_Cr, m_picHeight / 2, m_picWidth / 2, m_inputBitDepth[CHANNEL_TYPE_CHROMA]);

	

}

void CNNLoopFilter::filterBlk(PelUnitBuf &recUnitBuf, const CPelUnitBuf& cnnUnitBuf, const Area& blk, const ComponentID compId)
{
	const CPelBuf cnnBlk = cnnUnitBuf.get(compId).subBuf(blk.pos(), blk.size());
	PelBuf recBlk = recUnitBuf.get(compId).subBuf(blk.pos(), blk.size());
	recBlk.copyFrom(cnnBlk);
}
#endif