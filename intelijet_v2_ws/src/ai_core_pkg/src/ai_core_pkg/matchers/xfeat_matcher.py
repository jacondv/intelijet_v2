import time

import torch
import kornia.feature as KF

from ai_core_pkg.matchers.base import BaseMatcher
from ai_core_pkg.matchers.utils import image_cv_to_tensor


class XFeatMatcher(BaseMatcher):
    """https://github.com/verlab/accelerated_features - built into kornia
    as kornia.feature.XFeat, no vendoring needed. Sparse matcher, much
    lighter/faster than LoFTR/EfficientLoFTR; use mode="sparse" for the
    standard top-k keypoint matching or mode="semi-dense" for XFeat*'s
    denser (but slower) matching.
    """

    def __init__(self, device="cpu", mode="sparse", top_k=4096, verbose=False):
        if mode not in ("sparse", "semi-dense"):
            raise ValueError(f"mode must be 'sparse' or 'semi-dense', got {mode!r}")

        self.device = device
        self.mode = mode
        self.top_k = top_k
        self.verbose = verbose
        self.model = KF.XFeat(top_k=top_k).to(device).eval()

    def match(self, img0, img1):
        t0 = time.perf_counter()

        img0_tensor = image_cv_to_tensor(img0, device=self.device)
        img1_tensor = image_cv_to_tensor(img1, device=self.device)

        with torch.inference_mode():
            if self.mode == "sparse":
                mkpts0, mkpts1 = self.model.match_xfeat(img0_tensor, img1_tensor)
            else:
                mkpts0, mkpts1 = self.model.match_xfeat_star(img0_tensor, img1_tensor)

        t1 = time.perf_counter()
        if self.verbose:
            print(f"XFeat ({self.mode}) inference time: {(t1 - t0) * 1000:.2f} ms")

        if not torch.is_tensor(mkpts0):
            mkpts0 = torch.from_numpy(mkpts0)
            mkpts1 = torch.from_numpy(mkpts1)

        # XFeat doesn't produce a per-match confidence score like
        # LoFTR/EfficientLoFTR's softmax-derived mconf - report None so
        # callers relying on optional confidence degrade gracefully.
        return {
            "keypoints0": mkpts0,
            "keypoints1": mkpts1,
            "confidence": None,
        }
