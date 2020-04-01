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

* Clone this repo
```
  git clone https://github.com/MorganSalvatore/ADCNN.git
```

* Build the software using `cmake`

* Download one `prebuilt tensorflow` you need in [this repo](https://github.com/fo40225/tensorflow-windows-wheel/tree/master/1.10.0/cpp). For example, if your CPU support AVX2 and you want to do inference with GPU, download `libtensorflow-gpu-windows-x86_64-1.10.0-avx2cuda92cudnn72.7z`. If your CPU support SSE2 and you want to do inference with only CPU, download `libtensorflow-cpu-windows-x86_64-1.10.0-sse2.7z`.

* Open the generated solution file in Visual Studio. For the following projects, add `include` of `libtensorflow` and disable `Treat Warnings as Errors` settings :

  ​	`CommonLib` `EncoderLib` `DecoderLib` `EncoderApp` `DecoderApp`

* Add `lib/tensorflow.lib` of `libtensorflow` into `CommonLib` project.

* Compile the VS project to get `EncoderApp.exe` and `DecoderApp.exe`. Copy `bin/tensorflow.dll` of `libtensorflow` into the folder of exe files.

* Run VTM using command line

  ```win cmd
  EncoderApp -c PATH/TO/CFG  -i PATH/TO/YUV  -wdt WIDTH_OF_INPUT  -hgt HEIGHT_OF_INPUT  -fr FRAMERATE_OF_INPUT  -f FRAMES_TO_BE_ENCODED  -q QP  -tf_pb_path ./ckpt/XXX.pb  --WorkingMode=MODE  --LoopFilterDisable=1  --SAO=0  --ALF=0  -b PATH/TO/BITSTREAM/FILE  -o PATH/TO/RECONSTRUCTED/YUV > PATH/TO/ENCODER/LOG.txt
  ```

  ```
  DecoderApp -b PATH/TO/BITSTREAM/FILE  -tf_pb_path ./ckpt/XXX.pb  --WorkingMode=MODE  > PATH/TO/DECODER/LOG.txt
  ```

  

## TODO

data generation & training 