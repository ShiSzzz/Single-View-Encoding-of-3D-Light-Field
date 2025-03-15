# Single View Encoding of 3D Light Field
# 📖 Overview

This repository implements the method proposed in the paper "Single-View Encoding of 3D Light Field Based on Editable Field of View Gaussian Splatting", which enables real-time generation of high-resolution 8K light field images (96 viewpoints) from a single input image. By combining 3D Gaussian Splatting (3DGS) with light field encoding, the method achieves a 30x speedup over traditional approaches while maintaining optimal rendering quality. Key innovations include:

Adaptive Tile Rendering:  A pixel-by-pixel GS-based light field image rendering method to improve rendering speed

Viewpoint-Constrained 3D Reconstruction: Focuses on frontal light field synthesis for higher fidelity.

Memory Optimization: Reduces VRAM usage by 50% via lifecycle management and CUDA-aware strategies.


# 🚀 Features
🖼️ Single-View to 8K Light Field: Generate 7680×4320 light field images from a single RGB input.

⚡ Real-Time Performance: ~0.04s per 96-view rendering (tested on RTX 3090).

🎯 High-Quality Rendering: Achieves PSNR >29 dB and SSIM >0.97.

🧠 Generalizable: Trained on diverse scenes for robust performance.

# DATA
## ShapeNet cars
We use the ShapeNet-SRN class(cars) to generate our image dataset. The dataset we used can be found [here](https://pan.baidu.com/s/1tVTYlZURE3wZ7Y9bpmKv6w?pwd=78s3) . You can also directly download other types of models from ShapeNet to generate your own image dataset. We provide a Blender script for generating the image dataset.

# Code
We are currently organizing the work, and the code will be released soon.
