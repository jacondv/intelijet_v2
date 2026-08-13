# Vendored/trimmed from https://github.com/zju3dv/EfficientLoFTR
# src/utils/misc.py (Apache-2.0). Only detect_NaN is kept - the rest of
# the original file pulls in pytorch_lightning and joblib, which are
# training-only dependencies not needed for inference here.
import torch
from loguru import logger


def detect_NaN(feat_0, feat_1):
    logger.info(f'NaN detected in feature')
    logger.info(f"#NaN in feat_0: {torch.isnan(feat_0).int().sum()}, #NaN in feat_1: {torch.isnan(feat_1).int().sum()}")
    feat_0[torch.isnan(feat_0)] = 0
    feat_1[torch.isnan(feat_1)] = 0
