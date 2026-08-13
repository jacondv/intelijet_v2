#!/usr/bin/env python3
"""One-off conversion: strip the EfficientLoFTR checkpoint down to a
plain {"state_dict": ...} dict of tensors, so it can be loaded with
torch.load() without pytorch_lightning installed.

Run this ONCE, with pytorch_lightning installed (temporarily - it's
only needed to unpickle the original .ckpt, which was saved by the
authors' pytorch_lightning training pipeline):

    pip3 install pytorch_lightning
    python3 scripts/strip_efficientloftr_checkpoint.py \
        data/.cache/efficientloftr/eloftr_outdoor.ckpt \
        data/.cache/efficientloftr/eloftr_outdoor.stripped.ckpt

Then point EfficientLoFTRMatcher at the .stripped.ckpt file (see
ai_core_pkg/matchers/efficientloftr_matcher.py's DEFAULT_CHECKPOINT_PATH)
and pytorch_lightning is no longer needed at all - can be uninstalled /
never added to the Dockerfile.
"""
import sys

import torch


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input.ckpt> <output.ckpt>")
        sys.exit(1)

    src, dst = sys.argv[1], sys.argv[2]
    state_dict = torch.load(src, map_location="cpu")["state_dict"]
    torch.save({"state_dict": state_dict}, dst)
    print(f"Wrote {dst} ({sum(t.numel() for t in state_dict.values() if torch.is_tensor(t))} params)")


if __name__ == "__main__":
    main()
