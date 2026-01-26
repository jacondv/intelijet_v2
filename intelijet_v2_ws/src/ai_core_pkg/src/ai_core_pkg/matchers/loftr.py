import torch
import kornia as K
import kornia.feature as KF
from ai_core_pkg.matchers.base import BaseMatcher
from ai_core_pkg.matchers.utils import image_cv_to_tensor

import time


class LoFTRMatcher(BaseMatcher):
    def __init__(self, device="cpu", pretrained="outdoor",verbose=False):
        self.device = device
        self.model = KF.LoFTR(pretrained=pretrained).to(device).eval()
        self.verbose = verbose

    def match(self, img0, img1):
        t0 = time.perf_counter()

        img0_tensor = image_cv_to_tensor(img0)
        img1_tensor = image_cv_to_tensor(img1)

        input_dict = {
            "image0": K.color.rgb_to_grayscale(img0_tensor),
            "image1": K.color.rgb_to_grayscale(img1_tensor),
        }

        with torch.inference_mode():
            corr = self.model(input_dict)

        t1 = time.perf_counter()
        if self.verbose:
            print(f"Matcher inference time: {(t1 - t0) * 1000:.2f} ms")

        return {
            "keypoints0": corr["keypoints0"],
            "keypoints1": corr["keypoints1"],
            "confidence": corr.get("confidence", None),
        }
