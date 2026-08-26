import os
from datetime import datetime
import json
    
class JobInfo:

    PENDING = "pending"
    ACTIVE = "scheduled"
    FINISHED = "finished"
    INFO_FILE = "job_info.json"

    VALID_STATUSES = {PENDING, ACTIVE, FINISHED}

    def __init__(self, name: str, created: str = None,  status:str = None, description: str = "", parameters: dict = None):
        self.name = name
        self.created = created or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.status = status      # pending / scheduled / finished
        self.description = description
        self.parameters = parameters or {}

        if status not in self.VALID_STATUSES:
            raise ValueError(f"Invalid status '{status}'. Must be one of {self.VALID_STATUSES}")
        self.status = status
   
    @classmethod
    def load(cls, folder_path: str):
        import os
        """Load INFO_FILE từ folder job"""
        path = os.path.join(folder_path, cls.INFO_FILE)
        if not os.path.exists(path):
            print(f"[Warning] {cls.INFO_FILE} not found in {folder_path}")
            return None
        try:
            with open(path, "r") as f:
                data = json.load(f)
            return cls.from_dict(data)
        except (OSError, IOError, json.JSONDecodeError, ValueError) as e:
            print(f"[Error] Failed to load JobInfo: {e}")
            return None


    @classmethod
    def from_dict(cls, data: dict):
        status = data.get("status", cls.PENDING)
        if status == "active":  # legacy value, renamed to "scheduled"
            status = cls.ACTIVE
        return cls(
            name=data.get("name", ""),
            created=data.get("created"),
            status=status,
            description=data.get("description", ""),
            parameters=data.get("parameters", {})
        )


    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "created": self.created,
            "status": self.status,
            "description": self.description,
            "parameters": self.parameters
        }
    

    def save(self, path: str) -> bool:
        """Ghi model ra file JSON tại path, trả về True nếu thành công, False nếu lỗi."""
        try:
            full_name = os.path.join(path, self.INFO_FILE)
            with open(full_name, "w") as f:
                json.dump(self.to_dict(), f, indent=4)
            return True
        except (IOError, OSError, json.JSONDecodeError) as e:
            print(f"[Error] Lưu JobInfo thất bại: {e}")
            return False
        
