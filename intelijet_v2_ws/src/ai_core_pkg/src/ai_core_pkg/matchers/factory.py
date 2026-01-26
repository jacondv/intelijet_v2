from ai_core_pkg.matchers.loftr import LoFTRMatcher


class MatcherFactory:
    @staticmethod
    def create(name:str='LoFTR', device="cpu"):
        name = name.lower()

        if name.lower() == "loftr":
            return LoFTRMatcher(device)
        elif name == "disk":
            return None
        elif name == "superpoint_lg":
            return None
        else:
            raise ValueError(f"Unknown matcher: {name}")
