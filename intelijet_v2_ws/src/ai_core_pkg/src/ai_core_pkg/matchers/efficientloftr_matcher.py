import copy
import os
import time

import torch
import kornia as K

from ai_core_pkg.matchers.base import BaseMatcher
from ai_core_pkg.matchers.utils import image_cv_to_tensor
from ai_core_pkg.matchers.efficientloftr import (
    LoFTR,
    reparameter,
    full_default_cfg,
    opt_default_cfg,
)

# No auto-download (unlike kornia's KF.LoFTR(pretrained=...), which pulls
# from torch.hub): EfficientLoFTR's checkpoints are only distributed via
# the authors' Google Drive, not a stable URL pip/torch.hub can fetch.
# Download eloftr_outdoor.ckpt manually from
# https://github.com/zju3dv/EfficientLoFTR and place it here (this path
# is under the docker-compose data/.cache mount, so it survives the
# desktop icon's down+up-every-click container restart the same way the
# kornia LoFTR download cache does).
DEFAULT_CHECKPOINT_PATH = os.path.expanduser(
    "~/.cache/efficientloftr/eloftr_outdoor.ckpt"
)


class EfficientLoFTRMatcher(BaseMatcher):
    """https://github.com/zju3dv/EfficientLoFTR - vendored model code in
    ai_core_pkg/matchers/efficientloftr/. Alternative to LoFTRMatcher
    (kornia's KF.LoFTR) - not a drop-in performance win by default,
    benchmark both on real scan images before switching MatcherFactory
    over to this one.
    """

    def __init__(self, device="cpu", variant="opt", checkpoint_path=None, verbose=False):
        if variant not in ("opt", "full"):
            raise ValueError(f"variant must be 'opt' or 'full', got {variant!r}")

        checkpoint_path = checkpoint_path or DEFAULT_CHECKPOINT_PATH
        if not os.path.isfile(checkpoint_path):
            raise FileNotFoundError(
                f"EfficientLoFTR checkpoint not found at {checkpoint_path}. "
                "Download eloftr_outdoor.ckpt from "
                "https://github.com/zju3dv/EfficientLoFTR (see its README's "
                "download link) and place it there - it isn't auto-downloaded."
            )

        self.device = device
        self.variant = variant
        self.verbose = verbose

        config = copy.deepcopy(full_default_cfg if variant == "full" else opt_default_cfg)
        if device == "cpu":
            # torch's flash/mem-efficient scaled_dot_product_attention
            # backends are CUDA-only - linear_attention.py's Attention
            # forces enable_flash=True/enable_math=False when
            # `flash and not fp32`, which raises on CPU tensors unless
            # no_flash disables that path entirely (falls back to the
            # plain einsum softmax attention, mathematically equivalent,
            # just not fused).
            config["coarse"]["no_flash"] = True

        self.model = LoFTR(config=config)
        state_dict = torch.load(checkpoint_path, map_location=device)["state_dict"]
        self.model.load_state_dict(state_dict)
        # "Essential for good performance" per the upstream README - fuses
        # the RepVGG backbone's multi-branch train-time convs into single
        # inference-time convs.
        self.model = reparameter(self.model)
        self.model = self.model.eval().to(device)

    def match(self, img0, img1):
        t0 = time.perf_counter()

        img0_tensor = K.color.rgb_to_grayscale(image_cv_to_tensor(img0, device=self.device))
        img1_tensor = K.color.rgb_to_grayscale(image_cv_to_tensor(img1, device=self.device))

        # Model requires both dims divisible by 32 (see upstream README) -
        # round down rather than up so this never samples padding pixels.
        img0_tensor = _round_to_multiple_of_32(img0_tensor)
        img1_tensor = _round_to_multiple_of_32(img1_tensor)

        batch = {"image0": img0_tensor, "image1": img1_tensor}

        with torch.inference_mode():
            self.model(batch)

        t1 = time.perf_counter()
        if self.verbose:
            print(f"EfficientLoFTR ({self.variant}) inference time: {(t1 - t0) * 1000:.2f} ms")

        mconf = batch["mconf"]
        if self.variant == "opt":
            # Upstream README: 'opt' model confidence isn't a 0-1
            # probability like 'full' - rescale for downstream code
            # (match_ratio thresholds etc.) that assumes a 0-1 range.
            lo = torch.minimum(mconf.min(), torch.tensor(20.0, device=mconf.device))
            hi = torch.maximum(mconf.max(), torch.tensor(30.0, device=mconf.device))
            mconf = (mconf - lo) / (hi - lo)

        return {
            "keypoints0": batch["mkpts0_f"],
            "keypoints1": batch["mkpts1_f"],
            "confidence": mconf,
        }


def _round_to_multiple_of_32(img_tensor):
    h, w = img_tensor.shape[-2:]
    new_h, new_w = (h // 32) * 32, (w // 32) * 32
    if (new_h, new_w) == (h, w):
        return img_tensor
    return torch.nn.functional.interpolate(
        img_tensor, size=(new_h, new_w), mode="bilinear", align_corners=False
    )
