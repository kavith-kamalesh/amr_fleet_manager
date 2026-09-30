"""UNTRAINED placeholder: demonstrates the PyTorch -> ONNX export step only.

The weights are random, nothing here is trained, and no node in this repo
loads the exported file. Collision avoidance in this project is
reservation-based (see amr_fleet_manager/spatial_mutex.py), not learned.
The parameter count is whatever the layers contain, printed at run time.
"""
import os

import torch
import torch.nn as nn


class PlaceholderMLP(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc1 = nn.Linear(8, 64)
        self.fc2 = nn.Linear(64, 90)
        self.fc3 = nn.Linear(90, 1)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)


if __name__ == "__main__":
    model = PlaceholderMLP()
    total = sum(p.numel() for p in model.parameters())
    print(f"Untrained placeholder MLP: {total} parameters")
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "placeholder_policy.onnx")
    torch.onnx.export(model, torch.randn(1, 8), path,
                      input_names=["sensor_array"], output_names=["steering_cmd"])
    print(f"Exported to {path}")
