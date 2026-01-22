
import open3d as o3d
import numpy as np
import copy
import cv2
import os

def cloud_to_image(pcd,
                          filename="output.png",
                          width=800,
                          height=800,
                          rot_x=0,
                          rot_y=0,
                          rot_z=0):
    """
    Render point cloud to an image and save to file (workaround Open3D 0.19 PNG issue)
    Rotation values are in degrees.
    Returns True nếu lưu thành công, False nếu có lỗi.
    """
    import open3d as o3d
    import numpy as np
    import copy
    import cv2
    import os

    try:
        # ---- copy cloud và gán màu ----
        pcd = copy.deepcopy(pcd)
        pcd.paint_uniform_color([0.7, 0.7, 0.7])

        # ---- rotation ----
        Rx = pcd.get_rotation_matrix_from_xyz(np.radians([rot_x, 0, 0]))
        Ry = pcd.get_rotation_matrix_from_xyz(np.radians([0, rot_y, 0]))
        Rz = pcd.get_rotation_matrix_from_xyz(np.radians([0, 0, rot_z]))
        pcd.rotate(Rz @ Ry @ Rx, center=pcd.get_center())

        # ---- Visualizer headless ----
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False, width=width, height=height)

        if not vis.create_window(visible=False):
            raise RuntimeError("OpenGL/EGL context not available")
        
        vis.add_geometry(pcd)

        opt = vis.get_render_option()
        opt.background_color = np.array([0, 0, 0])
        opt.point_size = 2.0
        opt.light_on = True

        # ----- Camera param ---------
        ctr = vis.get_view_control()
        param = ctr.convert_to_pinhole_camera_parameters()
        intrinsic = param.intrinsic
        extrinsic = param.extrinsic
        print(intrinsic, extrinsic)

        vis.poll_events()
        vis.update_renderer()

        # ---- capture float buffer và convert sang uint8 ----
        img = np.asarray(vis.capture_screen_float_buffer(do_render=True))
        img = (img * 255).astype(np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # ---- tạo thư mục nếu chưa tồn tại ----
        # os.makedirs(os.path.dirname(filename), exist_ok=True)

        # ---- lưu ảnh ----
        success = cv2.imwrite(filename, img)
        vis.destroy_window()

        if success:
            print(f"Saved image to {filename}")
        else:
            print(f"[ERROR] Failed to save image to {filename}")
        return success

    except Exception as e:
        print(f"[EXCEPTION] Failed to render or save image: {e}")
        return False

if __name__ == "__main__":
    import open3d as o3d
    import numpy as np
    import cv2


    pcd_file = "/mnt/c/work/projects/intelijet_v2/data/Projects/TEST4/ARM0Deg/ARM0Deg#20251120_143449#pre_scan_cloud_01.ply"
    pcd = o3d.io.read_point_cloud(pcd_file)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    pcd.normalize_normals()

    cloud_to_image(pcd, rot_x=-90, rot_y=90, rot_z=0)
