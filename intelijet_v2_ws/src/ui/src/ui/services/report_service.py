# ui/services/report_service.py
"""PDF report export, extracted out of App.export_report. No widget access -
raises on failure instead of printing and swallowing, so the caller (App)
can decide how to surface the error to the operator (e.g. NotificationCenter).
"""
import os
import shutil
from datetime import datetime

import rospy

from ui.tunnel_report.report_controler import ReportGenerator
from ui.tunnel_report.report_utils import delete_old_final_report
from ui.models.job_info import JobInfo


class ReportService:
    def export(self, o3d_cloud, filename):
        """Render and write the PDF report for `o3d_cloud` to `filename`
        (a .ply path is accepted and turned into .pdf). Returns the final
        report path (the "finalreport" copy if one was made, else the
        plain export path) on success. Raises RuntimeError/other exceptions
        with a clear message on failure - never swallows errors silently."""
        rospy.loginfo(f"[ReportService] Start releasing the report: {filename}")

        job_folder = os.path.dirname(filename)
        project_name = os.path.basename(os.path.dirname(job_folder))

        basename = os.path.basename(filename)
        basename_parts = basename.split("#")
        job_name = basename_parts[0] if len(basename_parts) > 0 else "Unknown"

        try:
            dt = datetime.strptime(basename_parts[1], "%Y%m%d_%H%M%S").date()
        except Exception:
            dt = None
        try:
            tt = datetime.strptime(basename_parts[1], "%Y%m%d_%H%M%S").time()
        except Exception:
            tt = None

        report = ReportGenerator()
        job_info = JobInfo.load(job_folder)
        if job_info is not None:
            report.set_info(
                site_name=project_name,
                job_name=job_name,
                applied_thickness=job_info.parameters.get("target_thickness", 10),
                tolerance=job_info.parameters.get("tolerance", 10),
                operator="Unknown",
                date=dt.strftime("%d-%b-%Y") if dt else None,
                time=tt.strftime("%H:%M:%S") if tt else None,
            )
        else:
            report.set_info(
                site_name="Unknown",
                job_name=job_name,
                applied_thickness=40,
                tolerance=10,
                operator="Unknown",
                date=dt.strftime("%d-%b-%Y") if dt else None,
                time=tt.strftime("%H:%M:%S") if tt else None,
            )

        if filename.lower().endswith(".ply"):
            filename = filename.replace(".ply", ".pdf")

        if not report.export(pcd=o3d_cloud, output_path=filename):
            raise RuntimeError(f"ReportGenerator.export() failed for {filename}")

        rospy.loginfo(f"[ReportService] Report exported successfully: {filename}")

        final_report_name = filename
        parts = filename.split("#")
        if len(parts) >= 4:
            parts[2] = "finalreport"
            final_report_name = "#".join(parts)
            rospy.loginfo(f"[ReportService] final_report_name: {final_report_name}")
            # Only deletes old final reports with the same segments, so it
            # never wipes out final reports belonging to other jobs.
            delete_old_final_report(final_report_name)
            shutil.copy(filename, final_report_name)

        return final_report_name
