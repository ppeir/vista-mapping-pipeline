#!/usr/bin/env python3
"""
Convert SuperPoint state dict (MagicLeap .pth / old pickle format)
to TorchScript module (.pt zip format) expected by RTAB-Map's torch::load().

Usage:
  python3 convert_superpoint.py --input superpoint_v1.pth --output superpoint_v1.pt
"""

import argparse
import torch
import torch.nn as nn
import torch.nn.functional as F


class SuperPoint(nn.Module):
    """SuperPoint network – matches the architecture expected by RTAB-Map 0.21.4 SPDetector."""

    def __init__(self):
        super().__init__()
        c1, c2, c3, c4, c5 = 64, 64, 128, 128, 256

        # Shared encoder
        self.conv1a = nn.Conv2d(1, c1, kernel_size=3, stride=1, padding=1)
        self.conv1b = nn.Conv2d(c1, c1, kernel_size=3, stride=1, padding=1)
        self.conv2a = nn.Conv2d(c1, c2, kernel_size=3, stride=1, padding=1)
        self.conv2b = nn.Conv2d(c2, c2, kernel_size=3, stride=1, padding=1)
        self.conv3a = nn.Conv2d(c2, c3, kernel_size=3, stride=1, padding=1)
        self.conv3b = nn.Conv2d(c3, c3, kernel_size=3, stride=1, padding=1)
        self.conv4a = nn.Conv2d(c3, c4, kernel_size=3, stride=1, padding=1)
        self.conv4b = nn.Conv2d(c4, c4, kernel_size=3, stride=1, padding=1)

        # Detector head
        self.convPa = nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convPb = nn.Conv2d(c5, 65, kernel_size=1, stride=1, padding=0)

        # Descriptor head
        self.convDa = nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convDb = nn.Conv2d(c5, 256, kernel_size=1, stride=1, padding=0)

    def forward(self, x: torch.Tensor):
        # Shared encoder
        x = F.relu(self.conv1a(x))
        x = F.relu(self.conv1b(x))
        x = F.max_pool2d(x, kernel_size=2, stride=2)
        x = F.relu(self.conv2a(x))
        x = F.relu(self.conv2b(x))
        x = F.max_pool2d(x, kernel_size=2, stride=2)
        x = F.relu(self.conv3a(x))
        x = F.relu(self.conv3b(x))
        x = F.max_pool2d(x, kernel_size=2, stride=2)
        x = F.relu(self.conv4a(x))
        x = F.relu(self.conv4b(x))

        # Detector head
        cPa = F.relu(self.convPa(x))
        semi = self.convPb(cPa)

        # Descriptor head
        cDa = F.relu(self.convDa(x))
        desc = self.convDb(cDa)
        dn = torch.norm(desc, p=2, dim=1, keepdim=True)
        desc = desc / dn

        return semi, desc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input",  required=True, help="Path to state dict (.pth or old .pt)")
    parser.add_argument("--output", required=True, help="Path for TorchScript output (.pt)")
    args = parser.parse_args()

    print(f"Loading state dict from {args.input} ...")
    state_dict = torch.load(args.input, map_location="cpu")

    # Handle wrapped dicts (e.g. {'model': state_dict})
    if isinstance(state_dict, dict) and "model" in state_dict:
        state_dict = state_dict["model"]
    if isinstance(state_dict, dict) and "state_dict" in state_dict:
        state_dict = state_dict["state_dict"]

    model = SuperPoint()
    model.load_state_dict(state_dict)
    model.eval()
    print("State dict loaded OK.")

    # Trace with a dummy input (RTAB-Map uses grayscale float32, shape [1,1,H,W])
    dummy = torch.zeros(1, 1, 240, 320)
    with torch.no_grad():
        traced = torch.jit.trace(model, dummy)

    traced.save(args.output)
    print(f"TorchScript model saved to {args.output}")

    # Quick sanity check
    reloaded = torch.jit.load(args.output)
    semi, desc = reloaded(dummy)
    print(f"Sanity check OK — semi: {list(semi.shape)}, desc: {list(desc.shape)}")


if __name__ == "__main__":
    main()
