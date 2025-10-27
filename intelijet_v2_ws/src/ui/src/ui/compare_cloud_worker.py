# tunnel_ui/logic/cloud_manager.py
import threading
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
from pps.helper import compute_heatmap_to_plane, load_ply,cloud_downsample

from pps.data_converter import cloudconverter
from pps.tunnel_processing import TunnelProcessing
class CloudManager(QObject):
    _instance = None  # Singleton instance
    compare_done = pyqtSignal(object,str)   # object = kết quả point cloud hoặc polydata

    def __new__(cls, ui=None):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance.lock = threading.Lock()
            cls._instance.ui = ui
            cls._instance.pre_cloud = None
            cls._instance.post_cloud = None
            cls._instance.thread = None
            cls._instance.is_running = False
            cls._instance.filename = "None"
        return cls._instance


    # ---------------------------
    # NORMAL METHODS (not @classmethod)
    # ---------------------------
    def set_prescan(self, file):
        with self.lock:
            if isinstance(file, str):
                self.pre_cloud = load_ply(file)
            else:
                self.pre_cloud = file


    def set_postscan(self, file):
        with self.lock:
            if isinstance(file, str):
                self.filename = file
                self.post_cloud = load_ply(file)
            else:
                self.post_cloud = file


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


                    print(f"[CloudManager] Comparing clouds {self.filename}")
                    cloud_compared, distance = compute_heatmap_to_plane(
                        source=post_cloud, target=pre_cloud
                    )

                    # tunnel = TunnelProcessing()
                    # cloud_compared = tunnel.run_upsample(cloud_compared, axis='x', min_gap=0.02,max_gap=0.5)

                    self.compare_done.emit(cloud_compared, self.filename)

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


