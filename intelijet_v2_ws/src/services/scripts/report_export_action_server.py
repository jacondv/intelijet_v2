#!/usr/bin/env python3
"""Renders the tunnel-compare PDF report in its own process, off the UI.

See action/ExportReport.action's docstring for why: report generation
(weasyprint, matplotlib, Open3D offscreen rendering) is mostly-Python and
CPU-heavy enough to hold the GIL for seconds at a time - running it as a
QThread inside the UI process still let it fully freeze the UI, since every
thread in one Python process shares one GIL. A separate ROS node has its
own interpreter/GIL and its own GPU/EGL context, so it can't block the UI
this way.
"""
import os
import shutil
import traceback

import rospy
import actionlib

from services.msg import ExportReportAction, ExportReportResult, ExportReportFeedback
from services.report.report_controler import ReportGenerator
from services.report.report_utils import delete_old_final_report


class ReportExportServer:

    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            "/export_report",
            ExportReportAction,
            execute_cb=self.execute,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("Report export server started")

    def fb(self, stage, progress):
        f = ExportReportFeedback()
        f.stage = stage
        f.progress = progress
        self.server.publish_feedback(f)

    def execute(self, goal):
        try:
            self.fb("load", 0.05)

            report = ReportGenerator()
            report.set_info(
                site_name=goal.site_name,
                job_name=goal.job_name,
                applied_thickness=goal.applied_thickness,
                tolerance=goal.tolerance,
                date=goal.date,
                time=goal.time,
            )

            # ReportGenerator.export()/PLYProcessor.load() already accept a
            # plain .ply path directly (o3d.t.io.read_point_cloud) - no need
            # to load the cloud here first.
            if not report.export(pcd=goal.compared_ply_path, output_path=goal.output_pdf_path):
                raise RuntimeError(f"ReportGenerator.export() failed for {goal.output_pdf_path}")

            self.fb("finalize", 0.95)

            # Mirrors the old ui.services.report_service.ReportService.export()
            # tail: copy the export to a "finalreport" name (job's canonical
            # latest report), pruning any previous finalreport for this same
            # job/scan first so old ones don't pile up.
            final_path = goal.output_pdf_path
            parts = goal.output_pdf_path.split("#")
            if len(parts) >= 4:
                parts[2] = "finalreport"
                final_path = "#".join(parts)
                delete_old_final_report(final_path)
                shutil.copy(goal.output_pdf_path, final_path)

            self.fb("done", 1.0)

            res = ExportReportResult(success=True, report_path=final_path)
            self.server.set_succeeded(res)
            rospy.loginfo("[ReportExportServer] Report exported: %s", final_path)

        except Exception as e:
            rospy.logerr(
                "[ReportExportServer] export failed for %s: %s\n%s",
                goal.output_pdf_path, e, traceback.format_exc()
            )
            self.server.set_aborted(ExportReportResult(success=False, report_path=""))


if __name__ == "__main__":
    rospy.init_node("report_export_action_server")
    ReportExportServer()
    rospy.spin()
