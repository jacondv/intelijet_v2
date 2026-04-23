
import os
import numpy as np
from pps.helper import surface_area, filter_pcd_by_distance


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
            
            self.pcd = pcd

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
    

    def export_tunnel_view_image(self, out_path=None):
        
        img_base64 = self.__render_pointcloud_to_image(self.pcd, out_path=out_path)
        return img_base64
    
        
    def avg_thickness(self): # return in mm
        if self.distances is None or self.distances.size == 0:
            return None  # hoặc 0.0 nếu pipeline bắt buộc number

        # minimum valid thickness (mm)
        min_thickness_mm = max(self.target_thickness - 1 * self.tolerance,20)

        # mask valid distances
        abs_dist = np.abs(self.distances)
        mask_valid = (abs_dist >= min_thickness_mm)
        valid_ratio = np.mean(mask_valid) * 100

        MIN_VALID_RATIO = 1.0  # % – chỉnh theo yêu cầu kỹ thuật
        if valid_ratio < MIN_VALID_RATIO:
            return 0  # không đủ dữ liệu để ước tính


        mean_thickness_mm = np.mean(self.distances[mask_valid]) #mm

        return mean_thickness_mm

    def area(self):
        area = surface_area(self.pcd, radii=(0.1, 0.15))
        return area


    def volume(self):
        """
        Estimate sprayed volume (m³) from distance map.
        Distances are in mm.
        """

        import open3d as o3d

    def compute_thickness_metrics(self):
        if self.distances is None or self.distances.size == 0:
            return {
                "avg_thickness_mm": None,
                "total_area_m2": None,
                "reached_area_m2": None,
                "volume_m3": None
            }
        distances = self.distances
        mask = (distances > -25) & (distances < 25)
        distances[mask] = np.abs(distances[mask])
        distances = np.where(distances < -25, np.abs(distances), distances)

        min_thickness_mm = max(self.target_thickness - 1 * self.tolerance, 0)
        filtered_pcd = filter_pcd_by_distance(self.pcd, d_min=min_thickness_mm, d_max=1000)
        valid_area = surface_area(filtered_pcd, radii=(0.1, 0.15))  # m²
        total_area = surface_area(self.pcd, radii=(0.1, 0.15))  # m²

        mean_thickness_mm = distances.mean()  # mm

        volume_m3 = valid_area * mean_thickness_mm/1000

        
        return {
                "avg_thickness_mm": mean_thickness_mm,
                "total_area_m2": total_area,
                "reached_area_m2": valid_area,
                "volume_m3": volume_m3
            }
    
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


    def __render_pointcloud_to_image(self, pcd, out_path=None,
                                    width=800, height=600,
                                    fov_deg=60.0,
                                    point_size=2.0,
                                    background=(1.0, 1.0, 1.0, 1.0)):
        """
        Render an Open3D pointcloud to an image (base64 + optional file).
        Supports both legacy and tensor pointclouds.
        """
        import io, base64
        import numpy as np
        import open3d as o3d
        from PIL import Image

        # --- Convert to tensor pointcloud ---
        if isinstance(pcd, o3d.geometry.PointCloud):
            tpc = o3d.t.geometry.PointCloud.from_legacy(pcd)
        elif isinstance(pcd, o3d.t.geometry.PointCloud):
            tpc = pcd
        else:
            raise TypeError("pcd must be open3d.geometry.PointCloud or open3d.t.geometry.PointCloud")

        if not tpc.point["positions"].shape[0]:
            raise ValueError("Empty pointcloud")

        # --- Ensure colors ---
        if "colors" not in tpc.point:
            if isinstance(pcd, o3d.geometry.PointCloud) and pcd.has_colors():
                colors = np.asarray(pcd.colors, dtype=np.float32)
                if colors.max() > 1.0:
                    colors /= 255.0
                tpc.point["colors"] = o3d.core.Tensor(colors, dtype=o3d.core.Dtype.Float32)
            else:
                colors = np.ones((tpc.point["positions"].shape[0], 3), dtype=np.float32)
                tpc.point["colors"] = o3d.core.Tensor(colors)

        # --- Material ---
        try:
            mat = o3d.visualization.rendering.MaterialRecord()
            mat.shader = "defaultUnlit"
            mat.point_size = float(point_size)

            # --- Renderer ---
            renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)
            renderer.scene.set_background(np.array(background, dtype=np.float32))

            legacy_pc = tpc.to_legacy()
            renderer.scene.add_geometry("pc", legacy_pc, mat)

            # --- Auto-fit camera ---
            bounds = legacy_pc.get_axis_aligned_bounding_box()
            center = bounds.get_center()
            extent = bounds.get_extent()
            radius = np.linalg.norm(extent) * 0.5
            # eye = center + np.array([0, 0, radius * 3.0])
            eye = np.array([-4.0, 0.0, 0.0])
            up = np.array([0, 0, 1])

            cam = renderer.scene.camera
            cam.set_projection(fov_deg, width / height, 0.1, 1000.0,
                            o3d.visualization.rendering.Camera.FovType.Vertical)
            cam.look_at(center, eye, up)

            # --- Render ---
            img_o3d = renderer.render_to_image()

            # --- Convert to Base64 ---
            img_np = np.asarray(img_o3d)
            img_pil = Image.fromarray(img_np)
            buf = io.BytesIO()
            img_pil.save(buf, format="PNG")
            img_base64 = base64.b64encode(buf.getvalue()).decode("utf-8")
            img_base64_str = f"data:image/png;base64,{img_base64}"

            # --- Save file if requested ---
            if out_path:
                o3d.io.write_image(out_path, img_o3d)

            del renderer
            return img_base64_str
        
        except Exception as e:
            print(f"[PLYProcessor] Failed to render image from cloud: {e}")
            
            # Tạo ảnh trắng
            from PIL import Image
            import io, base64
            width, height = 800, 600  # kích thước mặc định
            img = Image.new("RGB", (width, height), (255, 255, 255))  # ảnh trắng
            buf = io.BytesIO()
            img.save(buf, format="PNG")
            img_base64 = base64.b64encode(buf.getvalue()).decode("utf-8")
            return f"data:image/png;base64,{img_base64}"

    

def delete_old_final_report(file_path: str):
    folder = os.path.dirname(file_path)

    if not os.path.exists(folder):
        print("Folder does not exist")
        return

    for filename in os.listdir(folder):
        # check file pdf + chứa "final"
        if filename.lower().endswith(".pdf") and "final" in filename.lower():
            full_path = os.path.join(folder, filename)

            try:
                os.remove(full_path)
                print(f"Deleted: {full_path}")
            except Exception as e:
                print(f"Failed to delete {full_path}: {e}")