# ADCNN: Attention-based Dual-scale CNN In-Loop Filter for Versatile Video Coding

Repository of the paper "Attention-based Dual-scale CNN In-Loop Filter for Versatile Video Coding"

link: [IEEE Xplore](https://ieeexplore.ieee.org/document/8852743)



## Prerequisites

* windows 7 or 10

* visual studio 2017

* cmake

* Python 3.6 (Anaconda3-5.2.0 suggested)

* Tensorflow 1.10

* CUDA 9.2 & cudnn 7.2 (if using GPU to do inference)

  

## Inference

* Download this repo, and build the software using `cmake`

* Download pre-trained models [here]

* Download the one you need in the repo ([prebuilt tensorflow](https://github.com/fo40225/tensorflow-windows-wheel/tree/master/1.10.0/cpp)). For example, if your CPU support AVX2 and you want to do inference with GPU, download `libtensorflow-gpu-windows-x86_64-1.10.0-avx2cuda92cudnn72.7z`. If your CPU support SSE2 and you want to do inference with only CPU, download `libtensorflow-cpu-windows-x86_64-1.10.0-sse2.7z`.

* Add `include` & `lib` of `libtensorflow` into the generated solution file in Visual Studio

* Compile the VS project to get `EncoderApp.exe` and `DecoderApp.exe`

* Run VTM using command line

  ```win cmd
  EncoderApp -c PATH/TO/CFG  -i PATH/TO/YUV  -wdt WIDTH_OF_INPUT  -hgt HEIGHT_OF_INPUT  -fr FRAMERATE_OF_INPUT  -f FRAMES_TO_BE_ENCODED  -q QP  -tf_pb_path PATH/TO/PB  --WorkingMode=MODE  --LoopFilterDisable=1  --SAO=0  --ALF=0  -b PATH/TO/BITSTREAM/FILE  -o PATH/TO/RECONSTRUCTED/YUV > PATH/TO/ENCODER/LOG.txt
  ```

  ```
  DecoderApp -b PATH/TO/BITSTREAM/FILE  -tf_pb_path PATH/TO/PB  --WorkingMode=MODE  > PATH/TO/DECODER/LOG.txt
  ```

  

# TODO

data generation & training 