from pathlib import Path
from jinja2 import Template
from weasyprint import HTML

from report_controler import ReportGenerator

report = ReportGenerator()
pcd = "/mnt/c/work/projects/intelijet_v2/data/Jobnumber1/cloud_compared_new2.ply"
report.export(pcd)

# def uri_for(path):
#     p = Path(path)
#     if not p.exists():
#         raise FileNotFoundError(f"Missing image: {p}")
#     return p.resolve().as_uri()  # đúng cho Win / Linux / WSL

# # load template
# with open("assets/templates/tunnel_report.html", "r", encoding="utf-8") as f:
#     tmpl = Template(f.read())

# tunnel_uri = uri_for("assets/images/tunnel.png")
# chart_uri  = uri_for("assets/images/chart.png")
# logo_uri  = uri_for("assets/images/logo.png")

# html_filled = tmpl.render(
#     logo_path=logo_uri,
#     site_name="Site-001",
#     applied_thickness=30,
#     tolerance=10,
#     job_name="Tunnel Zone A",
#     avg_thickness="28.4",
#     date="09/09/2025",
#     time="12:34",
#     shotcrete_volume="3.25",
#     tunnel_view_path=tunnel_uri,
#     thickness_chart_path=chart_uri
# )

# # Option A: viết file HTML ra để mở kiểm tra trên trình duyệt
# with open("debug_out.html", "w", encoding="utf-8") as f:
#     f.write(html_filled)

# # Xuất PDF (base_url không cần khi dùng file:// URIs, nhưng harmless)
# HTML(string=html_filled, base_url=str(Path.cwd())).write_pdf("thickness_report.pdf")
# print("wrote thickness_report.pdf")
