from abc import ABC, abstractmethod

class BaseMatcher(ABC):
    @abstractmethod
    def match(self, img0, img1):
        """
        img0, img1: torch.Tensor [1, 3, H, W]
        return {
            "keypoints0": corr["keypoints0"],
            "keypoints1": corr["keypoints1"],
            "confidence": corr.get("confidence", None),
        }
        keypoints0: torch.Tensor [N, 2]
        keypoints1: torch.Tensor [N, 2]
        confidence: torch.Tensor [N]
        """
        pass
