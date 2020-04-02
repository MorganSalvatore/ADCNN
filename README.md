# ADCNN: Attention-based Dual-scale CNN In-Loop Filter for Versatile Video Coding

Repository of the paper "Attention-based Dual-scale CNN In-Loop Filter for Versatile Video Coding"

link: [IEEE Xplore](https://ieeexplore.ieee.org/document/8852743)



## Prerequisites

* windows (7 or 10)

* visual studio (2015 update3 or 2017)

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

  ​	`CommonLib` `CommonAnalyserLib` `EncoderLib` `DecoderLib` `EncoderApp` `DecoderApp`

* Add `lib/tensorflow.lib` of `libtensorflow` into `CommonLib` project.

* Compile the VS project to get `EncoderApp.exe` and `DecoderApp.exe`. Copy `bin/tensorflow.dll` of `libtensorflow` into the folder of exe files.

* Run VTM using command line

  ```win cmd
  EncoderApp -c PATH/TO/CFG  -i PATH/TO/YUV  -wdt WIDTH_OF_INPUT  -hgt HEIGHT_OF_INPUT  -fr FRAMERATE_OF_INPUT  -f FRAMES_TO_BE_ENCODED  -q QP  --tf_pb_path=./ckpt/XXX.pb  --WorkingMode=MODE  --LoopFilterDisable=1  --SAO=0  --ALF=0  -b PATH/TO/BITSTREAM/FILE  -o PATH/TO/RECONSTRUCTED/YUV > PATH/TO/ENCODER/LOG.txt
  ```

  ```cmd
  DecoderApp -b PATH/TO/BITSTREAM/FILE  --tf_pb_path=./ckpt/XXX.pb  --WorkingMode=MODE  > PATH/TO/DECODER/LOG.txt
  ```


* `WorkingMode` could be `CPU` or `GPU` , you can specify GPU ID by `--GPUid=0` if you have multiple GPUs.

* An example is as follows:

  ```cmd
  EncoderApp -c ./VTM-4.0withCNN/cfg/encoder_randomaccess_vtm.cfg  -i D:/BlowingBubbles_416x240_50.yuv -wdt 416 -hgt 240 -fr 50 -f 10 -q 37 --tf_pb_path=./ckpt/RA_graph_ep150_val_+0.3398_+0.2563_2.50e-04.pb --WorkingMode=GPU --GPUid=1 --LoopFilterDisable=1  --SAO=0 --ALF=0 -b str.bin -o rec.yuv > encoder_log.txt
  ```

  

  

## TODO

data generation & training 