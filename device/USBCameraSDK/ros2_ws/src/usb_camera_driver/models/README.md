# CREStereo pre-trained ONNX models

The primary mapping profile uses the four-input
`crestereo_combined_iter2_360x640.onnx` cascade. It first predicts at 320x180
and refines at 640x360, matching the two-stage inference structure of the
upstream CREStereo example. A separate
`crestereo_init_iter2_180x320_fp16.onnx` reverse pass supplies the inexpensive
left-right confidence check used to reject reflections and occlusions. No
project-specific training or fine-tuning is used.

Source archive:
<https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/284_CREStereo/tensorrt/resources_iter2.tar.gz>

Upstream model and license: <https://github.com/megvii-research/CREStereo>

The two-input `crestereo_init_iter2_360x640.onnx` remains an offline/debug
baseline; it does not include the full-resolution refinement stage.

The deployment uses ONNX Runtime CUDA EP. TensorRT 10.3 FP16 engine building
was tested on the target NX, but did not finish within ten minutes and reached
about 2.6 GiB resident memory, so it is intentionally not the default.

SHA-256 checksums of the deployed files:

- `crestereo_init_iter2_180x320.onnx`:
  `2a8a1a096f0e2fe9538f5850bf82cfb8083945b888d1246420df9b68179a93ac`
- `crestereo_init_iter2_360x640.onnx`:
  `03f9016057d6905168321c4370be75b55d67e25646c4f1d4fabc7030f9cc085b`
- `crestereo_init_iter2_180x320_fp16.onnx`:
  `e3dd0d266a6caf02a70388c1a52f0b29970b57029c29d8d7d42d1a36f391628d`
- `crestereo_combined_iter2_360x640.onnx`:
  `d5ca84f85cde56d8a731788ef57097b0942808380e4cd2994bcf520772f6fb93`
