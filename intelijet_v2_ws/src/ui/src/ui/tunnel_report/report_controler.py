import os
from weasyprint import HTML
from ui.tunnel_report.template_manager import render_template
from ui.tunnel_report.report_data_model import ReportData
from ui.tunnel_report.report_utils import PLYProcessor
from shared.config_loader import CONFIG as cfg
from datetime import datetime

BASE_DIR = cfg.BASE_DIR

class ReportGenerator:
    def __init__(self):
        self.site_name=None  
        self.job_name=None      
        self.applied_thickness=30
        self.tolerance=10
        self.date = datetime.now().strftime("%d/%m/%Y")
        self.time = datetime.now().strftime("%H:%M:%S")


    def set_info(self, site_name="Unknown", job_name="Unknown",date=None, time=None,applied_thickness=30,tolerance=10):
        self.site_name=site_name
        self.job_name=job_name
        self.applied_thickness=applied_thickness
        self.tolerance=tolerance,
        self.date = date or self.date
        self.time = time or self.time


    def get_info(self) -> dict:
        return {
            "site_name": self.site_name,
            "job_name": self.job_name,
            "applied_thickness": self.applied_thickness,
            "tolerance": self.tolerance,
            "date": self.date,
            "time": self.time
        }


    def create_pdf(self, report_data, output_path, debug_html=True):

        try:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)

            html_content = render_template('tunnel_report.html', data=report_data)
            HTML(string=html_content, base_url='.').write_pdf(output_path)

            if debug_html:
                debug_path = output_path.replace(".pdf", ".html")
                with open(debug_path, "w", encoding="utf-8") as f:
                    f.write(html_content)
                print(f"[ReportGenerator] Saved intermediate HTML to: {debug_path}")

        except Exception as e:
            print(f"[ReportGenerator] Failed to create PDF at {output_path}: {e}")


    def export(self,pcd,output_path=None):

        processor = PLYProcessor()
        processor.load(pcd)

        #initial data to test
        site_name= self.site_name
        job_name = self.job_name
        tolerance = self.tolerance
        applied_thickness = self.applied_thickness
        date =  self.date
        time = self.time
        bins = [applied_thickness-tolerance, applied_thickness+tolerance]

        #---------------------
        thickness_chart_img = processor.export_distribution_chart(bins=bins, save_path=None)
        tunnel_view_img = f"{BASE_DIR}/intelijet_v2_ws/src/ui/src/ui/tunnel_report/assets/images/tunnel.png"
        shotcrete_volume = round(processor.shotcrete_volume(),2)
        avg_thickness = round(processor.avg_thickness(),2)
        

        data = ReportData.from_inputs(
            site_name=site_name,
            job_name=job_name,
            applied_thickness=applied_thickness,
            tolerance=tolerance,
            avg_thickness=avg_thickness,
            shotcrete_volume=shotcrete_volume,
            logo=f"{BASE_DIR}/intelijet_v2_ws/src/ui/src/ui/tunnel_report/assets/images/logo.png",
            tunnel_view=tunnel_view_img,
            thickness_chart=thickness_chart_img,
            date=date,
            time=time
        )
       
        self.create_pdf(report_data=data.to_json(), output_path=output_path, debug_html=False)

