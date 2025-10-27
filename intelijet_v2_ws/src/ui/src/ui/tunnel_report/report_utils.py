
import numpy as np

class PLYProcessor:
    def __init__(self):
        self.distances = None
        self.pcd = None
        self.target_thickness = 0
        self.tolerance = 0

    def load(self, ply_path):
        import open3d as o3d
        if ply_path is None:
            return None
        try:
            # Kiểm tra nếu là TensorPointCloud
            if isinstance(ply_path, o3d.t.geometry.PointCloud):
                pcd = ply_path
            elif isinstance(ply_path, str):
                pcd = o3d.t.io.read_point_cloud(ply_path)
            else:
                raise TypeError("The cloud not type of TensorPointCloud which have on open3d >= 0.17")

            if "distances" not in pcd.point:
                raise ValueError("PLY file doesn't contain the [distances] field.")
            self.distances = np.abs(pcd.point["distances"].numpy())
        except Exception as e:
            print(f"Input cloud is not in Open3D Tensor format. {e}")
            return None

        return pcd
    
    def set_parameters(self, target_thickness, tolerance):
        self.target_thickness = target_thickness
        self.tolerance = tolerance

    def get_header(self):
        header = {
            
        }
        return 

    def export_distribution_chart(self,bins, save_path=None):
        # self.pcd = self.load_ply(self.ply_path)
        img,_,_ = self.__plot_distance_distribution(self.distances, bins, save_path=None)
        return img #image is base64 format for report teamplate html
    
    def avg_thickness(self):
        if self.distances is None or len(self.distances) == 0:
            return 0
        return np.average(self.distances)
    
    def shotcrete_volume(self):
        if self.distances is None or len(self.distances) == 0:
            return 0
        
        min_val = self.target_thickness - self.tolerance
        mask = self.distances[np.abs(self.distances) >= min_val]
        vol = np.sum(mask)/1000 * (0.02*0.02) # volumn in m3
        return vol
    
    

    def __plot_distance_distribution(self,distances, bins, save_path=None):
        """
        Plot distance distribution with arbitrary bin edges.
        distances_m: array of distances in meters
        bins_m: list of bin edges in meters (e.g., [0.02, 0.04, 0.06])
        save_path: nếu khác None, lưu hình ảnh ra file (png/jpg/pdf)
        Distances converted to millimeters.
        """
        import numpy as np
        import matplotlib.pyplot as plt
        import io
        import base64

        # Convert to millimeters
        if self.distances is None or len(self.distances) == 0:
            from PIL import Image
            from io import BytesIO

            img = Image.new("RGB", (100, 100), (255, 255, 255))  # RGB trắng hoàn toàn
            buffered = BytesIO()
            img.save(buffered, format="PNG")
            img_base64 = base64.b64encode(buffered.getvalue()).decode("utf-8")

            return f"data:image/png;base64,{img_base64}", 0, 0 
        total = len(distances)

        counts = []
        labels = []

        # < first bin
        counts.append(np.sum(distances < bins[0]))
        labels.append(f"< {bins[0]:.0f} mm")

        # middle bins
        for i in range(len(bins) - 1):
            low, high = bins[i], bins[i + 1]
            counts.append(np.sum((distances >= low) & (distances < high)))
            labels.append(f"[{low:.0f}, {high:.0f}) mm")

        # >= last bin
        counts.append(np.sum(distances >= bins[-1]))
        labels.append(f">= {bins[-1]:.0f} mm")

        # Percentages
        percents = [c / total * 100 for c in counts]

        # 🎨 Màu riêng cho từng cột
        colors = ["#CA150F", "#39EB16", "#09BCF3", "#1120F0", "#C110E0", "#BB00D4"]
        colors = (colors * ((len(labels) // len(colors)) + 1))[:len(labels)]

        # Plot bar chart
        fig, ax = plt.subplots()
        bars = ax.bar(labels, counts, color=colors, edgecolor='none', width=0.2)
        plt.rcParams.update({'font.size': 14})

        # Annotate bars
        for bar, count, pct in zip(bars, counts, percents):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2, height + total * 0.01,
                    f"({pct:.1f}%)",
                    ha='center', va='bottom', fontsize=14)

        ax.set_ylim(0, max(counts) * 1.2)
        ax.set_xlabel("Distance Range (mm)", fontsize=14)
        ax.set_ylabel("Number of Points", fontsize=14)
        ax.set_title("Distance Distribution of Point Cloud", fontsize=14)
        plt.tight_layout()

        # Chuyển figure thành PIL Image
        buf = io.BytesIO()
        fig.savefig(buf, format='png', dpi=150)
        buf.seek(0)
        img_base64 = base64.b64encode(buf.read()).decode('utf-8')
        plt.close(fig)

        # Lưu hình ảnh nếu save_path khác None
        if save_path:
            fig.savefig(save_path, dpi=300)
            plt.close(fig)  # đóng figure để không hiển thị
        # else:
        #     plt.show()  # nếu không có path thì show bình thường

        return f"data:image/png;base64,{img_base64}", counts, percents 
