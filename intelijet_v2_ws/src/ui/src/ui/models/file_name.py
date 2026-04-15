import os
import re
import time

FILENAME_TEMPLATE = "{job}#{timestamp_str}#{type}#{index}#SCAN{scan_id}.{ext}"
# example: job1#20240601_153000#prescan#01#SCAN01.ply
def generate_filename(folder: str, job: str, scan_type: str, ext="ply"):
    """
    scan_type: 'pre_scan' | 'post_scan' | 'compared'
    """

    os.makedirs(folder, exist_ok=True)

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
    if scan_type == "pre_scan" or current_scan_id is None:
        scan_id = time.strftime("%Y%m%d%H%M%S")
        index = "00"   # prescan không cần index
    else:
        scan_id = current_scan_id
        # --- 3. Đếm index postscan ---
        pattern = re.compile(
            rf"^{job}#.*#postscan#(\d{{2}})#SCAN(\d{{3}})\.{ext}$"
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
        scan_id=f"{int(scan_id):03d}",
        ext=ext
    )

    return os.path.join(folder, filename)


def parse_filename(filename: str):
    name = os.path.basename(filename)
    parts = name.split("#")

    if len(parts) < 4:
        return None  # không đúng format

    job = parts[0]
    timestamp = parts[1]
    scan_type = parts[2]
    index = int(parts[3])
    if len(parts) == 5:
        last_part = parts[4]
        scan_part, ext = last_part.split(".")
    else:
        scan_part = 'SCAN000'  # default nếu thiếu

    scan_id = int(scan_part.replace("SCAN", ""))

    return {
        "job": job,
        "timestamp": timestamp,
        "type": scan_type,
        "index": index,
        "scan_id": scan_id,
        "ext": ext
    }

# # ===== TEST =====
# import tempfile

# def test_filename_generation():
#     with tempfile.TemporaryDirectory() as folder:
#         job = "J001"

#         print("=== TEST START ===")

#         # 1. prescan
#         f1 = generate_filename(folder, job, "prescan")
#         print("Prescan:", f1)

#         # 2. postscan 1
#         f2 = generate_filename(folder, job, "postscan")
#         print("Postscan01:", f2)

#         # 3. postscan 2
#         f3 = generate_filename(folder, job, "postscan")
#         print("Postscan02:", f3)

#         # 4. prescan mới
#         f4 = generate_filename(folder, job, "prescan")
#         print("Prescan:", f4)

#         # ===== CHECK REGEX =====
#         pattern = re.compile(
#             r"^J001#\d{8}_\d{6}#(prescan#|postscan#\d{2}#)SCAN\d{3}\.ply$"
#         )

#         for f in [f1, f2, f3, f4]:
#             print(f"Checking: {f}")
#             # assert pattern.match(f), f"❌ Sai format: {f}"

#         print("✅ All filename format correct!")

# def test_parse_filename_simple():
#     test_cases = [
#         # --- prescan ---
#         (
#             "J001#20260414_101530#prescan#SCAN001.ply",
#             {
#                 "job": "J001",
#                 "timestamp": "20260414_101530",
#                 "type": "prescan",
#                 "index": None,
#                 "scan_id": 1,
#                 "ext": "ply"
#             }
#         ),

#         # --- postscan ---
#         (
#             "J001#20260414_101531#postscan#01#SCAN001.ply",
#             {
#                 "job": "J001",
#                 "timestamp": "20260414_101531",
#                 "type": "postscan",
#                 "index": 1,
#                 "scan_id": 1,
#                 "ext": "ply"
#             }
#         ),

#         # --- postscan nhiều index ---
#         (
#             "J001#20260414_101532#postscan#12#SCAN002.ply",
#             {
#                 "job": "J001",
#                 "timestamp": "20260414_101532",
#                 "type": "postscan",
#                 "index": 12,
#                 "scan_id": 2,
#                 "ext": "ply"
#             }
#         ),

#         # --- sai format ---
#         (
#             "invalid_filename.ply",
#             None
#         ),
#     ]

#     print("=== TEST parse_filename_simple ===")

#     for filename, expected in test_cases:
#         result = parse_filename(filename)
#         print(f"result: {result}")

#         if result != expected:
#             print(f"❌ FAIL: {filename}")
#             print("Expected:", expected)
#             print("Got     :", result)
#         else:
#             print(f"✅ PASS: {filename}")

#     print("=== DONE ===")


# if __name__ == "__main__":
    # test_parse_filename_simple()
    # test_filename_generation()




