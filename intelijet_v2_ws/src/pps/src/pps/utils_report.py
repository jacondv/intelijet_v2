import os
import csv
from datetime import datetime
import numpy as np


def ensure_folder(path: str):
    """Tạo thư mục nếu chưa tồn tại."""
    os.makedirs(path, exist_ok=True)


def save_array_to_csv(data, output_dir="./report", filename=None, header=None, fmt="%.6f"):
    """
    Lưu mảng (list hoặc numpy array) ra file CSV.

    Args:
        data: list hoặc numpy array (1D hoặc 2D)
        output_dir: thư mục lưu file
        filename: tên file (mặc định: data_YYYYMMDD_HHMMSS.csv)
        header: danh sách tên cột (tuỳ chọn)
        fmt: định dạng số (áp dụng nếu dùng numpy.savetxt)
    """
    ensure_folder(output_dir)

    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"data_{timestamp}.csv"

    filepath = os.path.join(output_dir, filename)

    # Convert sang numpy array
    data = np.array(data)

    # Nếu 1D thì reshape thành 2D để lưu
    if data.ndim == 1:
        data = data.reshape(-1, 1)

    # Ghi file CSV
    with open(filepath, mode="w", newline="") as f:
        writer = csv.writer(f)
        if header:
            writer.writerow(header)
        writer.writerows(data)

    print(f"[INFO] CSV saved: {filepath}")
    return filepath


def distance_to_csv(distances, output_dir="./report", filename=None):
    """
    Lưu mảng distance ra file CSV.

    Args:
        distances: list hoặc numpy array
        output_dir: thư mục lưu file
        filename: tên file (mặc định: distance_YYYYMMDD_HHMMSS.csv)
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"distance_{timestamp}.csv"

    return save_array_to_csv(distances, output_dir, filename, header=["distances"])
