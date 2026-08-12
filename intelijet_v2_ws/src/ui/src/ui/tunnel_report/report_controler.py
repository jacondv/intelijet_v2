import os
import traceback
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
        self.date = datetime.now().strftime("%d-%b-%Y")
        self.time = datetime.now().strftime("%H:%M:%S")
        self.create_date = datetime.now().strftime("%d-%b-%Y")


    def set_info(self, site_name="Unknown", job_name="Unknown",operator="Unknown",date=None, time=None,applied_thickness=30,tolerance=10):
        self.site_name=site_name
        self.job_name=job_name
        self.applied_thickness=applied_thickness
        self.tolerance=tolerance
        self.date = date or self.date
        self.time = time or self.time
        self.create_date = datetime.now().strftime("%d-%b-%Y")
        self.operator=operator

    def get_info(self) -> dict:
        return {
            "site_name": self.site_name,
            "job_name": self.job_name,
            "applied_thickness": self.applied_thickness,
            "tolerance": self.tolerance,
            "date": self.date,
            "time": self.time,
            "create_date": self.create_date,
            self.operator:"Unknown"
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
            # Re-raise instead of swallowing: export() below only returns
            # True unconditionally after calling this - if the real
            # write_pdf() failure stopped here, export() would report
            # success (and log "Report exported successfully") for a PDF
            # that was never actually written, and the caller
            # (ReportService.export -> shutil.copy) would then fail with a
            # confusing FileNotFoundError on the "successfully exported"
            # file instead of the real cause.
            # Full traceback, not just str(e) - "__init__() takes 1
            # positional argument but 3 were given" alone doesn't say
            # WHICH __init__ (write_pdf()'s internals? something in
            # render_template()?), and guessing from the message alone
            # already turned out wrong once (checked OffscreenRenderer's
            # actual open3d==0.19.0 signature directly - it takes
            # width/height fine, so the real culprit is elsewhere).
            print(f"[ReportGenerator] Failed to create PDF at {output_path}: {e}\n{traceback.format_exc()}")
            raise


    def export(self,pcd,output_path=None):

        #initial data to test
        site_name= self.site_name
        job_name = self.job_name
        tolerance = self.tolerance
        applied_thickness = self.applied_thickness
        date =  self.date
        time = self.time
        bins = [applied_thickness-tolerance, applied_thickness+tolerance]

        processor = PLYProcessor()
        processor.load(pcd)
        processor.set_parameters(
            target_thickness=applied_thickness,
            tolerance=tolerance
        )
        thickness_metrics = processor.compute_thickness_metrics()

        #---------------------
        thickness_chart_img = processor.export_distribution_chart(bins=bins)
        tunnel_view_img = processor.export_tunnel_view_image(out_path=None)
        
        shotcrete_volume = round(thickness_metrics["volume_m3"],1)
        avg_thickness = round(thickness_metrics["avg_thickness_mm"],0)
        reached_area = round(thickness_metrics["reached_area_m2"],1)
        total_area = round(thickness_metrics["total_area_m2"],1)    
        

        data = ReportData.from_inputs(
            site_name=site_name,
            job_name=job_name,
            applied_thickness=applied_thickness,
            tolerance=tolerance,
            avg_thickness=avg_thickness,
            shotcrete_volume=shotcrete_volume,
            total_area_m2=total_area,
            reached_area_m2=reached_area,
            logo=f"{BASE_DIR}/intelijet_v2_ws/src/ui/src/ui/tunnel_report/assets/images/logo.png",
            tunnel_view=tunnel_view_img,
            thickness_chart=thickness_chart_img,
            date=date,
            time=time
        )
        try:
            self.create_pdf(report_data=data.to_json(), output_path=output_path, debug_html=False)
            return True
        except Exception as e:
            print(f"[ReportGenerator] Error exporting report: {e}")
    

