# tunnel_ui/logic/cloud_manager.py
import threading
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from pps.helper import compute_heatmap_to_plane, load_ply,smooth_cloud,assign_colors

from pps.data_converter import cloudconverter
from pps.tunnel_processing import TunnelProcessing

from shared.config_loader import CONFIG as cfg

THICKNESS_TARGET = cfg.thickness.target  # Target thickness in meter -> convert mm to m
THICKNESS_TOLERANCE = cfg.thickness.tolerance  # Allowable tolerance in meter of thickness

class CloudManager(QObject):
    _instance = None  # Singleton instance
    compare_done = pyqtSignal(object,str)   # object = result is poitcloud before upsample 
    compare_done2 = pyqtSignal(object,str) # Object is pointcloud after upsample, it is prepare for export report
    def __new__(cls, ui=None):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance.lock = threading.Lock()
            cls._instance.ui = ui
            cls._instance.pre_cloud = None
            cls._instance.post_cloud = None
            cls._instance.thread = None
            cls._instance.is_running = False
            cls._instance.pre_filename = "None"
            cls._instance.post_filename = "None"
            cls.target_thickness = THICKNESS_TARGET
            cls.tolerance = THICKNESS_TOLERANCE
        return cls._instance


    # ---------------------------
    # NORMAL METHODS (not @classmethod)
    # ---------------------------
    def set_prescan(self, file):
 
        with self.lock:
            if isinstance(file, str):
                self.pre_cloud = load_ply(file)
                self.pre_filename = file
                import os
                from ui.models.job_info import JobInfo
                job_folder = os.path.dirname(file)
                job_info = JobInfo.load(job_folder)
                if job_info:
                    self.target_thickness = job_info.parameters.get("target_thickness",THICKNESS_TARGET)/1000
                    self.tolerance = job_info.parameters.get("tolerance",THICKNESS_TOLERANCE)/1000
            else:
                self.pre_filename = "None"
                self.pre_cloud = file


    def set_postscan(self, file):
        with self.lock:
            if isinstance(file, str):
                self.post_filename = file
                self.post_cloud = load_ply(file)
            else:
                self.post_filename="None"
                self.post_cloud = file

    def auto_crop(self):
        # Crop, remove ground side and back side wall
        def __auto_crop(cloud_o3d):
            #Downsample
            cloud_o3d = cloud_o3d.vocel_down_sample(voxel_size=0.015)
            cloud_o3d = cloudconverter.voxel_down_sample_spatial(voxel_size=0.015)
            # Auto crop boundary
            tunnel = TunnelProcessing(cloud_o3d)
            result = tunnel.run_processing_pipeline()
            return result

        with self.lock:
            if self.pre_cloud is not None:
                self.pre_cloud = __auto_crop(self.pre_cloud)
            if self.post_cloud is not None:
                self.post_cloud = __auto_crop(self.post_cloud)


    def align(self):
        from pps.cloud_processing.align_manager import PointCloudAlignerManager
        from pps.cloud_processing.icp_aligner import ICPConfig
        from pps.data_converter import cloudconverter

        ICP_THRESHOLDS = [0.5, 0.3, 0.02]      # coarse → fine
        ICP_MAX_ITERS = [20, 20, 30]           # coarse → fine
        ICP_ALIGN_AREA = None                  # hoặc [[xmin, xmax], [ymin, ymax], [zmin, zmax]]
        ICP_VOXEL_RADII = [0.25, 0.15, 0.01]  # coarse → fine
        aligner = PointCloudAlignerManager(
                    strategy="icp",
                    config=ICPConfig(
                        threshold=ICP_THRESHOLDS,
                        max_iters=ICP_MAX_ITERS,
                        align_area=ICP_ALIGN_AREA,
                        voxel_radii=ICP_VOXEL_RADII
                    )
                )

        pre = cloudconverter.tensor_to_o3d_legacy(self.pre_cloud)
        post = cloudconverter.tensor_to_o3d_legacy(self.post_cloud)

        aligner.align(post, pre)
        T = aligner.get_transformation_matrix()
        post.transform(T)
        self.pre_cloud =  pre
        self.post_cloud = post

    def compare(self):

        """Chạy compare trong thread riêng, gọi callback(result) khi xong."""
        if self.is_running:
            print("[CloudManager] Compare is already running.")
            return

        def task():
            self.is_running = True
            with self.lock:
                try:
                    if self.pre_cloud is None or self.post_cloud is None:
                        raise ValueError("Pre or Post cloud not loaded")
                    
                    tunnel = TunnelProcessing(self.pre_cloud)
                    pre_cloud = tunnel.run_processing_pipeline()
                    
                    tunnel = TunnelProcessing(self.post_cloud)
                    post_cloud = tunnel.run_processing_pipeline()

                    print(f"[CloudManager] Comparing clouds {self.post_filename} vs {self.pre_filename}")
                    cloud_compared, distance = compute_heatmap_to_plane(
                        source=post_cloud, 
                        target=pre_cloud, 
                        target_thickness=self.target_thickness, 
                        tolerance_thickness=self.tolerance, 
                        k=6
                    )
                    # cloud_compare is in tensor format

                    # tunnel = TunnelProcessing()
                    # cloud_compared = tunnel.run_upsample(cloud_compared, axis='x', min_gap=0.02,max_gap=0.5)
                    # cloud_compared = smooth_cloud(cloud_compared, k=8, m=2, threshold=20.0)
                    # cloud_compared = assign_colors(cloud_compared, highlight_range=[20,40])
                    cloud_compared_upsample = tunnel.run_upsample(cloud_compared)
                    self.compare_done.emit(cloud_compared, self.post_filename)
                    self.compare_done2.emit(cloud_compared_upsample, self.post_filename)

                    # from ui.tunnel_report.report_data_model import ReportHeader
                    # header = ReportHeader(
                    #     site_name = "Jacon Equipment",
                    #     job_name= "Jacon PPS test",
                    #     applied_thickness = 30,
                    #     tolerance = 10,
                    #     operator = "Danh Vo"
                    # )
                    # filepath = "/mnt/c/work/projects/intelijet_v2/data/Jobnumber1/cloud_compared_new2.ply"
                    # header.save(path=filepath.replace(".ply", ".json"))

                except Exception as e:
                    print(f"[CloudManager] Error: {e}")
                finally:
                    self.is_running = False
                    

        self.thread = threading.Thread(target=task, daemon=True)
        self.thread.start()

cloud_compare = CloudManager()

