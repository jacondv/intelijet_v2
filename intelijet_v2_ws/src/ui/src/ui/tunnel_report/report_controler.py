from weasyprint import HTML
from ui.tunnel_report.template_manager import render_template
from ui.tunnel_report.report_data_model import ReportData
from ui.tunnel_report.report_utils import PLYProcessor

class ReportGenerator:
    def __init__(self):
        pass        

    def create_pdf(self, report_data, output_path, debug_html=True):
        html_content = render_template('tunnel_report.html', data=report_data)
        HTML(string=html_content, base_url='.').write_pdf(output_path)

        if debug_html:
            debug_path = output_path.replace(".pdf", ".html")
            with open(debug_path, "w", encoding="utf-8") as f:
                f.write(html_content)
            print(f"[DEBUG] Saved intermediate HTML to: {debug_path}")

    def export(self,pcd,output_path=None):

        processor = PLYProcessor()
        processor.load(pcd)

        #initial data to test
        site_name="VMS"
        job_name = "Testjob"
        tolerance = 10
        applied_thickness = 30
        bins = [applied_thickness-tolerance, applied_thickness+tolerance]

        #---------------------
        thickness_chart_img = processor.export_distribution_chart(bins=bins, save_path=None)
        tunnel_view_img = "/mnt/c/work/projects/intelijet_v2/data/Jobnumber1/tunnel.png"
        shotcrete_volume = processor.shotcrete_volume()
        avg_thickness = processor.avg_thickness()

        data = ReportData.from_inputs(
            site_name=site_name,
            job_name=job_name,
            applied_thickness=applied_thickness,
            tolerance=10,
            avg_thickness=avg_thickness,
            shotcrete_volume=shotcrete_volume,
            logo="/mnt/c/work/projects/intelijet_v2/intelijet_v2_ws/src/ui/src/ui/tunnel_report/assets/images/logo.png",
            tunnel_view=tunnel_view_img,
            thickness_chart=thickness_chart_img
        )
       
        self.create_pdf(report_data=data.to_json(), output_path=output_path, debug_html=False)

