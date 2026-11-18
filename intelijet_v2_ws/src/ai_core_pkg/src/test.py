import io
import time
# import requests


# def download_image(url: str, filename: str = "") -> str:
#     filename = url.split("/")[-1] if len(filename) == 0 else filename
#     # Download
#     bytesio = io.BytesIO(requests.get(url).content)
#     # Save file
#     with open(filename, "wb") as outfile:
#         outfile.write(bytesio.getbuffer())

#     return filename

# url_a = "https://github.com/kornia/data/raw/main/matching/kn_church-2.jpg"
# url_b = "https://github.com/kornia/data/raw/main/matching/kn_church-8.jpg"

# download_image(url_a)
# download_image(url_b)


import cv2
import kornia as K
import kornia.feature as KF
import matplotlib.pyplot as plt
import numpy as np
import torch
from kornia_moons.viz import draw_LAF_matches
import time

def load_image_cv_to_tensor(fname, device="cpu"):
    # OpenCV load (BGR, uint8, HxWx3)
    img = cv2.imread(fname, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {fname}")

    # BGR -> RGB
    # img = cv2.resize(img, (480, 480))  # (width, height)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # to tensor, float32, normalize
    img = torch.from_numpy(img).to(device)
    img = img.permute(2, 0, 1).float() / 255.0  # [3, H, W]

    # add batch dim
    return img.unsqueeze(0)  # [1, 3, H, W]

fname1 = "/mnt/c/WORK/projects/intelijet_v2/intelijet_v2_ws/src/ai_core_pkg/src/matches_1769067378842_pos.png"
fname2 = "/mnt/c/WORK/projects/intelijet_v2/intelijet_v2_ws/src/ai_core_pkg/src/matches_1769070894491_pre.png"

# img1 = K.io.load_image(fname1, K.io.ImageLoadType.RGB32)[None, ...]
# img2 = K.io.load_image(fname2, K.io.ImageLoadType.RGB32, device="cpu")[None, ...]
# img1 = K.geometry.resize(img1, (600, 375), antialias=True)
# img2 = K.geometry.resize(img2, (600, 375), antialias=True)

img1 = load_image_cv_to_tensor(fname1)
img2 = load_image_cv_to_tensor(fname2)

matcher = KF.LoFTR(pretrained="outdoor")

input_dict = {
    "image0": K.color.rgb_to_grayscale(img1),  # LofTR works on grayscale images only
    "image1": K.color.rgb_to_grayscale(img2),
}

with torch.inference_mode():
    t0 = time.perf_counter()
    correspondences = matcher(input_dict)
    t1 = time.perf_counter()
    print(f"Inference time: {(t1 - t0)*1000:.2f} ms")



for k, v in correspondences.items():
    print(k, v)



mkpts0 = correspondences["keypoints0"].cpu().numpy()
mkpts1 = correspondences["keypoints1"].cpu().numpy()

Fm, inliers = cv2.findFundamentalMat(mkpts0, mkpts1, cv2.USAC_MAGSAC, 0.5, 0.999, 100000)
inliers = inliers > 0
inlier_idx = np.where(inliers.ravel() == 1)[0]
selected_idx = np.random.choice(inlier_idx, size=20, replace=False)
new_inliers = np.zeros_like(inliers)
new_inliers[selected_idx] = 1
inliers = new_inliers


import matplotlib.pyplot as plt
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)


# print(inliers)
draw_LAF_matches(
    KF.laf_from_center_scale_ori(
        torch.from_numpy(mkpts0).view(1, -1, 2),
        torch.ones(mkpts0.shape[0]).view(1, -1, 1, 1),
        torch.ones(mkpts0.shape[0]).view(1, -1, 1),
    ),
    KF.laf_from_center_scale_ori(
        torch.from_numpy(mkpts1).view(1, -1, 2),
        torch.ones(mkpts1.shape[0]).view(1, -1, 1, 1),
        torch.ones(mkpts1.shape[0]).view(1, -1, 1),
    ),
    torch.arange(mkpts0.shape[0]).view(-1, 1).repeat(1, 2),
    K.tensor_to_image(img1),
    K.tensor_to_image(img2),
    inliers,
    draw_dict={
        "inlier_color": (0.2, 1, 0.2),
        "tentative_color": None,
        "feature_color": (0.2, 0.5, 1),
        "vertical": False,
    },
)

plt.savefig('/mnt/c/WORK/projects/intelijet_v2/intelijet_v2_ws/src/ai_core_pkg/src/result.png')
