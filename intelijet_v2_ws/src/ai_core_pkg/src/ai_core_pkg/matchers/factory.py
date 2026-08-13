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
        elif name == "disk":
            return None
        elif name == "superpoint_lg":
            return None
        else:
            raise ValueError(f"Unknown matcher: {name}")
