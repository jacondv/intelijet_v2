import os
import io
import base64
from PIL import Image
import numpy as np

from dataclasses import dataclass, asdict, field
from typing import Optional
import json
from datetime import datetime

@dataclass
class ReportHeader:
    site_name: str
    job_name: str
    applied_thickness: float
    tolerance: float
    date: str = field(default_factory=lambda: datetime.now().strftime("%Y-%m-%d"))  # default ngày hôm nay
    time: str = field(default_factory=lambda: datetime.now().strftime("%H:%M"))        # default giờ hiện tại
    operator: str = ""
    notes: Optional[str] = ""  # ghi chú

    def to_json(self, indent: int = 2) -> str:
        return json.dumps(asdict(self), indent=indent)

    @classmethod
    def from_json(cls, json_str: str):
        data = json.loads(json_str)
        return cls(**data)

    def save(self, path: str):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            f.write(self.to_json())
        print(f"ReportHeader saved to {path}")

class ReportData:
    def __init__(self):
        self.site_name = None
        self.job_name = None
        self.applied_thickness = None
        self.tolerance = None
        self.avg_thickness = None
        self.shotcrete_volume = None
        self.date = None
        self.time = None
        self.operator = None
        self.logo = None
        self.tunnel_view = None
        self.thickness_chart = None

    @classmethod
    def from_inputs(cls, site_name, job_name, applied_thickness, tolerance, avg_thickness, shotcrete_volume, logo,tunnel_view,thickness_chart, **kwargs):
        obj = cls()
        obj.site_name = site_name
        obj.job_name = job_name
        obj.applied_thickness = applied_thickness
        obj.tolerance = tolerance
        obj.avg_thickness = avg_thickness
        obj.shotcrete_volume = shotcrete_volume

        # Optional
        obj.date = kwargs.get("date")
        obj.time = kwargs.get("time")
        obj.operator = kwargs.get("operator")
        # --- Xử lý ảnh ---
        obj.logo = cls._process_image(logo)
        obj.tunnel_view = cls._process_image(tunnel_view)
        obj.thickness_chart = cls._process_image(thickness_chart)
    
        return obj

    @staticmethod
    def _process_image(img):
        """
        Nhận file path, PIL.Image hoặc numpy array,
        trả về (path nếu có, base64 URI)
        """
        if img is None:
            return None

        # Nếu là chuỗi base64 (data URI)
        if isinstance(img, str) and img.startswith("data:image"):
            return img  # Giữ nguyên vì đã là base64 hợp lệ
        # Nếu là path
        if isinstance(img, str):
            if not os.path.exists(img):
                raise FileNotFoundError(f"Image file not found: {img}")
            with open(img, "rb") as f:
                img_uri = "data:image/png;base64," + base64.b64encode(f.read()).decode()
            return img_uri

        # Nếu là PIL.Image
        if isinstance(img, Image.Image):
            buffer = io.BytesIO()
            img.save(buffer, format="PNG")
            img_uri = "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode()
            return img_uri

        # Nếu là numpy array
        if isinstance(img, np.ndarray):
            pil_img = Image.fromarray(img)
            buffer = io.BytesIO()
            pil_img.save(buffer, format="PNG")
            img_uri = "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode()
            return img_uri

        raise TypeError("Image must be path, PIL.Image.Image, or numpy.ndarray")

    def to_json(self):
        # this json should map which report teamplate html
        data = {
            "logo": self.logo,
            "site_name": self.site_name,
            "applied_thickness": self.applied_thickness,
            "tolerance": self.tolerance,
            "job_name": self.job_name,
            "avg_thickness": self.avg_thickness,
            "date": self.date,
            "time": self.time,
            "shotcrete_volume": self.shotcrete_volume,
            "tunnel_view_uri": self.tunnel_view,
            "thickness_chart_uri": self.thickness_chart,
            "create_date": datetime.now().strftime("%d/%m/%Y %H:%M")
        }
        return data

