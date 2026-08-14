import time

import torch
import kornia.feature as KF

from ai_core_pkg.matchers.base import BaseMatcher
from ai_core_pkg.matchers.utils import image_cv_to_tensor


class DiskLightGlueMatcher(BaseMatcher):
    """DISK (https://arxiv.org/abs/2006.13566) sparse keypoints + LightGlue
    (https://github.com/cvg/LightGlue) learned matcher, both built into
    kornia - no vendoring needed. Faster than LoFTR and tends to beat it
    on outdoor/textured scenes; LoFTR (dense) still wins on low-texture
    indoor scenes.
    """

    def __init__(self, device="cpu", top_k=2048, verbose=False):
        self.device = device
        self.top_k = top_k
        self.verbose = verbose
        self.disk = KF.DISK.from_pretrained("depth").to(device).eval()
        self.lg_matcher = KF.LightGlueMatcher("disk").to(device).eval()

    def match(self, img0, img1):
        t0 = time.perf_counter()

        img0_tensor = image_cv_to_tensor(img0, device=self.device)
        img1_tensor = image_cv_to_tensor(img1, device=self.device)

        with torch.inference_mode():
            # DISK requires both dims divisible by 16.
            feats0, feats1 = self.disk(
                torch.cat([img0_tensor, img1_tensor], dim=0),
                n=self.top_k,
                pad_if_not_divisible=True,
            )

            kps0, descs0 = feats0.keypoints, feats0.descriptors
            kps1, descs1 = feats1.keypoints, feats1.descriptors

            lafs0 = KF.laf_from_center_scale_ori(
                kps0[None], torch.ones(1, len(kps0), 1, 1, device=self.device)
            )
            lafs1 = KF.laf_from_center_scale_ori(
                kps1[None], torch.ones(1, len(kps1), 1, 1, device=self.device)
            )

            dists, idxs = self.lg_matcher(
                descs0, descs1, lafs0, lafs1,
                hw1=img0_tensor.shape[-2:], hw2=img1_tensor.shape[-2:],
            )

        t1 = time.perf_counter()
        if self.verbose:
            print(f"DISK+LightGlue inference time: {(t1 - t0) * 1000:.2f} ms")

        mkpts0 = kps0[idxs[:, 0]]
        mkpts1 = kps1[idxs[:, 1]]
        # LightGlueMatcher returns per-match confidence in [0, 1]
        # (higher = better), unlike classic descriptor matchers whose
        # `dists` is an L2 distance (lower = better).
        confidence = dists.squeeze(-1) if dists.ndim > 1 else dists

        return {
            "keypoints0": mkpts0,
            "keypoints1": mkpts1,
            "confidence": confidence,
        }
