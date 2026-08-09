# pps/cloud_utils/coloring.py
"""Distance -> RGB color mapping used to visualize compare results.

Split out of the old pps/helper.py god-module (see docs/plan/phase_06_pps_cleanup.md).
"""
import numpy as np


def map_distances_to_colors(
    distances, 
    clip_max=150,
    highlight_range=(20, 40),
    out_of_range_color=(0.678, 0.847, 0.902) #Light blue
):
    """
    Map distances to RGB colors with smooth transitions:
      - dist < highlight_range[0] → red → green gradient
      - dist in highlight_range → pure green
      - dist > highlight_range[1] → green → blue gradient
      - dist > clip_max → out_of_range_color
    """
    distances = np.abs(distances)
    colors = np.zeros((len(distances), 3))

    low, high = highlight_range

    for i, d in enumerate(distances):
        if d > clip_max:
            colors[i] = out_of_range_color

        elif d < low:
            # Gradient red (1,0,0) → green (0,1,0)
            if d < 20:
                colors[i] = (0.5, 0, 0)  # Đỏ hoàn toàn cho khoảng cách rất nhỏ
            else:
                t = d / low if low > 0 else 0
                colors[i] = (1 - t, t, 0)
            # colors[i] = (0.5, 0, 0)

        elif d <= high:
            # Pure green
            colors[i] = (0, 1, 0)

        else:
            # Gradient green (0,1,0) → blue (0,0,1)
            colors[i] = (0, 0, 1)

    return colors


def assign_colors(tcloud, clip_max=150, highlight_range=(20, 40)):
    """
    Map the 'distances' field of a tensor PointCloud to 'colors'.
    
    Args:
        tcloud: o3d.t.geometry.PointCloud, must have 'distances' field
        clip_max: maximum distance to clip (values above get out_of_range_color)
        highlight_range: (low, high) range for pure green
    
    Returns:
        tcloud with updated 'colors' field (in-place)
    """
    import open3d as o3d
    print("Assigning colors based on distances...")

    if 'distances' not in tcloud.point:
        raise ValueError("PointCloud must have 'distances' field")
    
    distances = tcloud.point['distances'].cpu().numpy()  # CPU numpy array
    colors = map_distances_to_colors(distances, clip_max=clip_max, highlight_range=highlight_range)

    # Update tensor cloud colors
    tcloud.point['colors'] = o3d.core.Tensor(colors.astype(np.float32))
    return tcloud
