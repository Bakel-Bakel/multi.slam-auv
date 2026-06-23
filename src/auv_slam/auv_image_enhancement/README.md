# auv_image_enhancement

Underwater image restoration (spec §10.2, §15) — mandatory preprocessing for visual SLAM.

- `enhance.py` — pure OpenCV/numpy: `gray_world_white_balance`, `clahe_lab`,
  `udcp_dehaze`, dispatched by `enhance(img, method)`. Unit-tested in `test/`.
- `image_enhancer` node — `/cam/{left,right}/image_raw` -> `/cam/{left,right}/image_enhanced`.
  Param `method`: `none|wb|clahe|clahe_wb|full`. A learned model (FUnIE-GAN / Sea-thru)
  can be slotted behind the same parameter later.

Visual SLAM consumes the **enhanced** stream, never the raw one.
