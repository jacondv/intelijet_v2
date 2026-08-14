from ai_core_pkg.matchers.loftr import LoFTRMatcher


class MatcherFactory:
    @staticmethod
    def create(name: str = 'LoFTR', device="cpu", **kwargs):
        name = name.lower()

        if name == "loftr":
            return LoFTRMatcher(device)
        elif name == "efficientloftr":
            # kwargs: variant="opt"|"full" (default "opt"),
            # checkpoint_path=... - see EfficientLoFTRMatcher docstring.
            from ai_core_pkg.matchers.efficientloftr_matcher import EfficientLoFTRMatcher
            return EfficientLoFTRMatcher(device, **kwargs)
        elif name == "xfeat":
            # kwargs: mode="sparse"|"semi-dense" (default "sparse"),
            # top_k=4096 - see XFeatMatcher docstring.
            from ai_core_pkg.matchers.xfeat_matcher import XFeatMatcher
            return XFeatMatcher(device, **kwargs)
        elif name == "disk_lightglue":
            # kwargs: top_k=2048 - see DiskLightGlueMatcher docstring.
            from ai_core_pkg.matchers.disk_lightglue_matcher import DiskLightGlueMatcher
            return DiskLightGlueMatcher(device, **kwargs)
        elif name == "disk":
            return None
        elif name == "superpoint_lg":
            return None
        else:
            raise ValueError(f"Unknown matcher: {name}")
