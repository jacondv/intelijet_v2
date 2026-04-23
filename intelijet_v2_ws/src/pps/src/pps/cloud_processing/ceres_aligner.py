import numpy as np
import pyceres
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
import numpy as np
import pyceres

class PointToPlaneCost(pyceres.CostFunction):
    def __init__(self, p, q, n):
        super().__init__()

        self.set_num_residuals(1)
        self.set_parameter_block_sizes([6])  # se3: (rx, ry, rz, tx, ty, tz)

        self.p = np.asarray(p, dtype=np.float64)
        self.q = np.asarray(q, dtype=np.float64)
        self.n = np.asarray(n, dtype=np.float64)

    def Evaluate(self, parameters, residuals, jacobians):
        x = parameters[0]

        omega = x[:3]   # rotation vector
        t = x[3:]       # translation

        theta = np.linalg.norm(omega)
        if theta < 1e-12:
            R = np.eye(3)
        else:
            k = omega / theta
            K = np.array([
                [0, -k[2], k[1]],
                [k[2], 0, -k[0]],
                [-k[1], k[0], 0]
            ])
            R = np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K @ K)

        p_trans = R @ self.p + t
        residuals[0] = self.n.dot(p_trans - self.q)

        # Jacobian (optional: để None vẫn chạy, nhưng chậm hơn)
        if jacobians is not None:
            J = np.zeros((1, 6))

            # dR/dw approx
            J_rot = -self.n @ (R @ skew(self.p))
            J_trans = self.n

            J[0, :3] = J_rot
            J[0, 3:] = J_trans

            jacobians[0][:] = J

        return True


def skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def rotation_matrix(omega):
    theta = np.linalg.norm(omega)
    if theta < 1e-12:
        return np.eye(3)

    k = omega / theta
    K = skew(k)

    return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K @ K)

from scipy.spatial import cKDTree

def ceres_refine_icp(src_cloud,
                     tgt_cloud,
                     init_T=np.eye(4),
                     max_iter=20,
                     distance_threshold=0.01,
                     loss_sigma=0.05):

    import open3d as o3d
    # ---------------------------
    # 1. Data
    # ---------------------------
    src_points = np.asarray(src_cloud.points, dtype=np.float64)
    tgt_points = np.asarray(tgt_cloud.points, dtype=np.float64)

    if not tgt_cloud.has_normals():
        tgt_cloud.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )

    tgt_cloud.orient_normals_consistent_tangent_plane(50)
    tgt_normals = np.asarray(tgt_cloud.normals, dtype=np.float64)

    tree = cKDTree(tgt_points)

    # ---------------------------
    # 2. Init pose
    # ---------------------------
    x = np.zeros(6, dtype=np.float64)

    R0 = init_T[:3, :3]
    t0 = init_T[:3, 3]

    angle = np.arccos(np.clip((np.trace(R0) - 1) / 2, -1, 1))

    if angle < 1e-12:
        omega = np.zeros(3)
    else:
        wx = (R0[2,1] - R0[1,2]) / (2*np.sin(angle))
        wy = (R0[0,2] - R0[2,0]) / (2*np.sin(angle))
        wz = (R0[1,0] - R0[0,1]) / (2*np.sin(angle))
        omega = angle * np.array([wx, wy, wz])

    x[:3] = omega
    x[3:] = t0

    # ---------------------------
    # 3. Build correspondences (1 lần)
    # ---------------------------
    R = rotation_matrix(x[:3])
    t = x[3:]

    src_trans = (R @ src_points.T).T + t

    dist, idx = tree.query(src_trans)
    mask = dist < distance_threshold

    src_sel = src_points[mask]
    tgt_sel = tgt_points[idx[mask]]
    n_sel = tgt_normals[idx[mask]]

    # ---------------------------
    # 4. Build problem
    # ---------------------------
    problem = pyceres.Problem()

    for p, q, n in zip(src_sel, tgt_sel, n_sel):
        cost = PointToPlaneCost(p, q, n)
        loss = pyceres.HuberLoss(loss_sigma)

        problem.add_residual_block(cost, loss, [x])

    # ---------------------------
    # 5. Solve (1 shot)
    # ---------------------------
    options = pyceres.SolverOptions()
    options.max_num_iterations = max_iter
    options.minimizer_progress_to_stdout = True

    summary = pyceres.SolverSummary()
    pyceres.solve(options, problem, summary)

    # ---------------------------
    # 6. Output
    # ---------------------------
    T = np.eye(4)
    T[:3, :3] = rotation_matrix(x[:3])
    T[:3, 3] = x[3:]

    return T, summary.final_cost