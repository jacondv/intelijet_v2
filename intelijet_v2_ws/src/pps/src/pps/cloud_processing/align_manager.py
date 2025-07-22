from pps.cloud_processing.icp_aligner import ICPAligner, ICPConfig
from pps.cloud_processing.base_aligner import AlignMethodConfig
import numpy as np
# Bạn có thể import thêm RANSACAligner, NDTAligner nếu có

class PointCloudAlignerManager:
    def __init__(self, strategy: str, config: AlignMethodConfig):
        self.config = config
        self.__strategy = strategy.lower()
        self.__aligner = self._create_aligner()
        self.__aligned_cloud = None
        self.__transform = np.eye(4, dtype=np.float64)
        

    def _create_aligner(self):
        if self.__strategy == "icp":
            if not isinstance(self.config, ICPConfig):
                raise TypeError("ICP strategy requires ICPConfig.")
            
            return ICPAligner(config=self.config)
        else:
            raise ValueError(f"Unknown strategy: {self.__strategy}")
    
    def get_aligned_cloud(self):
        return self.__aligned_cloud       

    def get_transformation_matrix(self):
        """
            return transform matrix affer aligned, default is eye matrix (4,4)
        """
        return self.__transform

    def align(self, source, target, init_transform=np.eye(4,dtype=np.float64)):
        self.__aligned_cloud = self.__aligner.align(source, target, init_transform)
        self.__transform = self.__aligner.get_transformation_matrix()
        return self.__aligned_cloud
    

    

