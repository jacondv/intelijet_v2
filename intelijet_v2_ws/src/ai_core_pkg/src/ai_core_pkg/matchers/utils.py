
import cv2
import torch


def image_cv_to_tensor(img, device="cpu"):
    # OpenCV load (BGR, uint8, HxWx3)
    if img is None:
        raise FileNotFoundError(f"Cannot read image")

    # BGR -> RGB
    # img = cv2.resize(img, (480, 480))  # (width, height)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # to tensor, float32, normalize
    img = torch.from_numpy(img).to(device)
    img = img.permute(2, 0, 1).float() / 255.0  # [3, H, W]

    # add batch dim
    return img.unsqueeze(0)  # [1, 3, H, W]