import os
import re
import time

FILENAME_TEMPLATE = "{job}#{timestamp_str}#{type}_{index}#SCAN{scan_id}.{ext}"
# example: job1#20240601_153000#prescan#01#SCAN01.ply
def generate_filename(folder: str, job: str, scan_type: str, ext="ply"):
    """
    scan_type: 'pre_scan' | 'post_scan' | 'compared'
    """

    os.makedirs(folder, exist_ok=True)

    scan_type = re.sub(r'[^a-zA-Z0-9_-]', '', scan_type)

    files = [f for f in os.listdir(folder) if f.endswith(f".{ext}")]

    # --- 1. Tìm scan_id hiện tại ---
    scan_ids = []

    for f in files:
        match = re.search(r"#SCAN(\d+)", f)
        if match:
            scan_ids.append(int(match.group(1))) 

    scan_ids = sorted(scan_ids)
    current_scan_id = scan_ids[-1] if scan_ids else 0

    # --- 2. Nếu prescan → tạo scan_id mới ---
    if scan_type == "pre_scan":
        new_scan_id = current_scan_id+1
    else:
        new_scan_id = current_scan_id

    # --- 3. Đếm index postscan ---
    pattern = re.compile(
        rf"^{job}#.*#post_scan_cloud_(\d{{2}})#SCAN(\d{{3}})\.{ext}$"
    )

    indices = []
    for f in files:
        m = pattern.match(f)
        if m:
            indices.append(int(m.group(1)))

    next_index = max(indices) + 1 if indices else 1
    index = f"{next_index:02d}"

    # --- 4. Timestamp riêng từng file ---
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    
    # --- 5. Format filename ---
    filename = FILENAME_TEMPLATE.format(
        job=job,
        timestamp_str=timestamp_str,
        type=scan_type,
        index=index,
        scan_id=f"{int(new_scan_id):03d}",
        ext=ext
    )

    return os.path.join(folder, filename)


def parse_filename(filename: str):
    import os
    import re

    name = os.path.basename(filename)

    pattern = r"""
        ^(?P<job>[^#]+)
        \#
        (?P<timestamp>[^#]+)
        \#
        (?P<type>.+?)_(?P<index>\d+)
        \#
        SCAN(?P<scan_id>\d+)
        \.
        (?P<ext>\w+)$
    """

    match = re.match(pattern, name, re.VERBOSE)
    if not match:
        return None

    return {
        "job": match.group("job"),
        "timestamp": match.group("timestamp"),
        "type": match.group("type"),
        "index": int(match.group("index")),
        "scan_id": int(match.group("scan_id")),
        "ext": match.group("ext"),
    }



