# ui/services/cloud_pipeline.py
"""Point-cloud conversion/coloring/save, extracted out of App.on_cloud_received
so it contains no widget access and can later be moved onto a worker thread
(see Phase 5) without touching Qt objects.
"""
import rospy

from pps.data_converter import CloudConverter
from pps.helper import assign_colors


class CloudPipelineService:
    def __init__(self):
        # Reused across calls instead of constructing a new CloudConverter
        # every time (previously done at every call site).
        self._converter = CloudConverter()

    def pointcloud2_to_o3d(self, msg):
        """Convert an incoming sensor_msgs/PointCloud2 to an Open3D tensor cloud."""
        return self._converter.pointcloud2_to_o3d_tensor(msg)

    def assign_colors_for_highlight(self, o3d_cloud, highlight_range):
        """Color an already-converted Open3D cloud by `highlight_range` (min, max)."""
        return assign_colors(o3d_cloud, highlight_range=highlight_range)

    def process_incoming(self, msg, highlight_range=None):
        """Convert `msg` to an Open3D cloud, optionally color it by
        `highlight_range` (min, max). Pass highlight_range=None to skip
        coloring (used for topics that don't need it)."""
        o3d_cloud = self.pointcloud2_to_o3d(msg)
        if highlight_range is not None:
            o3d_cloud = self.assign_colors_for_highlight(o3d_cloud, highlight_range)
        return o3d_cloud

    def to_vtk(self, o3d_cloud):
        return self._converter.o3d_to_vtk_polydata(o3d_cloud)

    def save_ply(self, o3d_cloud, filepath):
        """Save `o3d_cloud` to `filepath`. Returns filepath on success, None
        on failure (matches the previous App.save_job behavior - failures
        are logged, not raised, since the caller only used truthiness)."""
        try:
            self._converter.o3d_to_ply(o3d_cloud, filepath)
            rospy.loginfo(f"[CloudPipelineService] Saved cloud to {filepath}")
            return filepath
        except Exception as e:
            rospy.logerr(f"[CloudPipelineService] Failed to save cloud to {filepath}: {e}")
            return None
