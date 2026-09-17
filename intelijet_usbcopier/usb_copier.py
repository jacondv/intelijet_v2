#!/usr/bin/env python3
"""
USB Data Copier
- Runs as a background service, shows GUI when USB is plugged in
- Browse into subfolders and select items to copy
- Pure tkinter, no extra dependencies
"""

import os
import sys
import shutil
import threading
import subprocess
import time
import json
import queue
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

# ============================================================
# CONFIG
# ============================================================
SOURCE_DATA_PATH = "/home/nuc/intelijet_v2/data/Projects/"   # <-- Change to your data path
SCAN_INTERVAL_MS = 2000                # USB scan every 2 seconds
# ============================================================

C = {
    "bg":      "#f0f2f5",
    "panel":   "#ffffff",
    "border":  "#d0d5dd",
    "accent":  "#0071e3",
    "accent2": "#005bb5",
    "danger":  "#d93025",
    "warn":    "#e67e00",
    "text":    "#1a1a2e",
    "muted":   "#6e7787",
    "success": "#1a8a3c",
    "hover":   "#e8f0fe",
    "item":    "#ffffff",
    "check":   "#0071e3",
}
SCALE = 2.0


# ══════════════════════════════════════════════════════════════
#  USB DETECTION  (shared, thread-safe)
# ══════════════════════════════════════════════════════════════
def detect_usb_drives():
    drives = []
    try:
        result = subprocess.run(
            ["lsblk", "-J", "-o", "NAME,MOUNTPOINT,LABEL,SIZE,TRAN,RM"],
            capture_output=True, text=True, timeout=5)
        data = json.loads(result.stdout)

        def walk(devices):
            for dev in devices:
                tran = dev.get("tran") or ""
                rm   = str(dev.get("rm") or "0")
                mp   = dev.get("mountpoint") or ""
                name = dev.get("name", "")
                if mp and (tran == "usb" or rm == "1"):
                    label = dev.get("label") or name
                    try:
                        st = shutil.disk_usage(mp)
                        free = fmt_size(st.free)
                        total = fmt_size(st.total)
                    except:
                        free = total = "?"
                    drives.append({"label": label, "mount": mp,
                                   "free": free, "total": total, "name": name})
                if dev.get("children"):
                    walk(dev["children"])
        walk(data.get("blockdevices", []))
    except Exception:
        # Fallback: scan /media /mnt /run/media
        for base in ["/media", "/mnt", "/run/media"]:
            base_p = Path(base)
            if not base_p.exists():
                continue
            for sub in base_p.iterdir():
                candidates = list(sub.iterdir()) if sub.is_dir() else [sub]
                for mp in candidates:
                    try:
                        if mp.is_dir() and mp.stat().st_dev != base_p.stat().st_dev:
                            st = shutil.disk_usage(str(mp))
                            drives.append({
                                "label": mp.name, "mount": str(mp),
                                "free": fmt_size(st.free), "total": fmt_size(st.total),
                                "name": mp.name,
                            })
                    except:
                        pass
    return drives


def fmt_size(n):
    for unit in ["B", "KB", "MB", "GB", "TB"]:
        if n < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} PB"

def calc_size(path):
    if path.is_file():
        return path.stat().st_size
    total = 0
    try:
        for f in path.rglob("*"):
            if f.is_file():
                total += f.stat().st_size
    except:
        pass
    return total


# ══════════════════════════════════════════════════════════════
#  MAIN GUI WINDOW
# ══════════════════════════════════════════════════════════════
class USBCopierWindow:
    def __init__(self, root, usb_drives):
        self.root = root
        self.root.title("USB Data Copier")
        self.root.geometry("1180x820")
        self.root.minsize(1000, 800)
        self.root.configure(bg=C["bg"])
        # Open maximized by default (touchscreen kiosk - no reason to make
        # the user resize it every time it pops up on a USB insert).
        # "-zoomed" is the X11/Linux equivalent of Windows' state("zoomed").
        try:
            self.root.attributes("-zoomed", True)
        except tk.TclError:
            try:
                self.root.state("zoomed")
            except tk.TclError:
                pass
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.source_root = Path(SOURCE_DATA_PATH)
        self.current_path = self.source_root   # current browsed path
        self.usb_drives = usb_drives
        self.selected_usb_idx = 0
        self.folder_vars = {}       # path_str -> BooleanVar
        self.folder_widgets = {}    # path_str -> row_frame
        self.is_copying = False
        self.cancel_flag = threading.Event()
        self.total_bytes = 0
        self.copied_bytes = 0

        self._apply_styles()
        self._build_ui()
        self._populate_usb_combo()
        self._load_dir(self.source_root)
        self._log("Service detected USB — showing window", "accent")
        for d in self.usb_drives:
            self._log(f"  {d['label']}  [{d['mount']}]  {d['free']} free / {d['total']}", "success")

    # ── STYLES ───────────────────────────────────────────────
    def _apply_styles(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except:
            pass
        style.configure("TScrollbar",
                        background=C["panel"], troughcolor=C["bg"],
                        arrowcolor=C["muted"],arrowsize=44, borderwidth=0, width=44)
        style.configure("Prog.Horizontal.TProgressbar",
                        troughcolor=C["border"], background=C["accent"],
                        borderwidth=0, thickness=18)
        style.configure("TCombobox",
                        padding=14, arrowsize=36, font=("Courier New", 18))

    # ── UI BUILD ─────────────────────────────────────────────
    def _build_ui(self):
        # Header
        hdr = tk.Frame(self.root, bg=C["panel"], height=72)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        tk.Label(hdr, text="⟢  USB DATA COPIER",
                 bg=C["panel"], fg=C["accent"],
                 font=("Courier New", 18, "bold")).pack(side="left", padx=24)
        self.lbl_usb_dot = tk.Label(hdr, text="● 0 USB connected",
                                     bg=C["panel"], fg=C["danger"],
                                     font=("Courier New", 13))
        self.lbl_usb_dot.pack(side="right", padx=24)
        tk.Frame(self.root, bg=C["border"], height=1).pack(fill="x")

        # Body: left (file browser) 70% | right (activity log) 30%.
        # `place` with relwidth enforces that split as an actual hard
        # percentage of the parent's width, unlike `grid`'s column
        # `weight` - weight only distributes space LEFT OVER once each
        # column's own natural content size is satisfied, so with the log
        # panel's own widgets (scrollbar, text box) demanding real width,
        # a plain 2:1 weight split still rendered the log far wider than
        # intended. `place` also keeps re-applying the percentage on every
        # resize on its own, no manual <Configure> recompute needed.
        body = tk.Frame(self.root, bg=C["bg"])
        body.pack(fill="both", expand=True)

        # x/width/height alongside relx/relwidth/relheight are additive
        # pixel offsets in Tk's place geometry manager - used here purely
        # to reproduce the outer/inter-panel margins the old pack layout
        # had (16px outer, 8px gap between panels), on top of the 70/30
        # split itself.
        left = tk.Frame(body, bg=C["bg"])
        left.place(relx=0.0, x=16, y=16, relwidth=0.70, width=-24, relheight=1.0, height=-32)

        right = tk.Frame(body, bg=C["bg"])
        right.place(relx=0.70, x=0, y=16, relwidth=0.30, width=-16, relheight=1.0, height=-32)

        self._build_left(left)
        self._build_right(right)

        # Bottom bar
        tk.Frame(self.root, bg=C["border"], height=1).pack(fill="x")
        self._build_bottom()

    def _build_left(self, parent):
        # ── USB selector
        usb_box = self._panel(parent)
        usb_box.pack(fill="x", pady=(0, 10))
        tk.Label(usb_box, text="USB DRIVE", bg=C["panel"], fg=C["muted"],
                 font=("Courier New", 11, "bold")).pack(anchor="w", padx=14, pady=(12, 4))
        usb_row = tk.Frame(usb_box, bg=C["panel"])
        usb_row.pack(fill="x", padx=14, pady=(0, 14))
        self.combo_usb = ttk.Combobox(usb_row, state="readonly", font=("Courier New", 18), style="TCombobox")
        self.combo_usb.pack(side="left", fill="x", expand=True, ipady=8)
        self.combo_usb.bind("<<ComboboxSelected>>", self._on_usb_select)
        self._btn(usb_row, "↻ Refresh", self._refresh_usb, ghost=True).pack(side="right", padx=(10, 0))

        # ── Breadcrumb path bar
        nav_box = self._panel(parent)
        nav_box.pack(fill="x", pady=(0, 14))
        nav_inner = tk.Frame(nav_box, bg=C["panel"])
        nav_inner.pack(fill="x", padx=14, pady=12)
        self._btn(nav_inner, "⬆ Up", self._go_up, ghost=True).pack(side="left")
        self.lbl_path = tk.Label(nav_inner, text="", bg=C["panel"], fg=C["accent2"],
                                  font=("Courier New", 14), anchor="w")
        self.lbl_path.pack(side="left", padx=(14, 0), fill="x", expand=True)

        # ── Folder / file list header
        hdr_row = tk.Frame(parent, bg=C["bg"])
        hdr_row.pack(fill="x", pady=(0, 8))
        tk.Label(hdr_row, text="SELECT ITEMS TO COPY",
                 bg=C["bg"], fg=C["muted"],
                 font=("Courier New", 16, "bold")).pack(side="left")
        self._btn(hdr_row, "None", self._deselect_all, ghost=True).pack(side="right")
        self._btn(hdr_row, "All",  self._select_all,   ghost=True).pack(side="right", padx=(0, 8))

        # ── Scrollable list
        list_outer = self._panel(parent)
        list_outer.pack(fill="both", expand=True)

        canvas = tk.Canvas(list_outer, bg=C["panel"], highlightthickness=0, bd=0)
        sb = ttk.Scrollbar(list_outer, orient="vertical", command=canvas.yview,  style="Vertical.TScrollbar")
        self.item_frame = tk.Frame(canvas, bg=C["panel"])
        self.item_frame.bind("<Configure>",
                             lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.item_frame, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            canvas.bind(seq, lambda e, c=canvas: c.yview_scroll(
                -1 if e.num != 5 else 1, "units"))
        self._list_canvas = canvas

    def _build_right(self, parent):
        tk.Label(parent, text="ACTIVITY LOG", bg=C["bg"], fg=C["muted"],
                 font=("Courier New", 8, "bold")).pack(anchor="w", pady=(0, 5))
        log_box = self._panel(parent)
        log_box.pack(fill="both", expand=True)
        self.log_txt = tk.Text(log_box, bg=C["panel"], fg=C["text"],
                                font=("Courier New", 11), relief="flat", bd=0,
                                state="disabled", wrap="word",
                                selectbackground=C["border"], padx=10, pady=10)
        log_sb = ttk.Scrollbar(log_box, command=self.log_txt.yview)
        self.log_txt.configure(yscrollcommand=log_sb.set)
        log_sb.pack(side="right", fill="y")
        self.log_txt.pack(fill="both", expand=True)
        for tag, col in [("accent", C["accent"]), ("success", C["success"]),
                         ("warn", C["warn"]), ("err", C["danger"]),
                         ("muted", C["muted"]), ("info", C["text"])]:
            self.log_txt.tag_config(tag, foreground=col)

    def _build_bottom(self):
        bar = tk.Frame(self.root, bg=C["panel"], height=230)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)

        prog_row = tk.Frame(bar, bg=C["panel"])
        prog_row.pack(fill="x", padx=24, pady=(14, 4))
        self.lbl_progress = tk.Label(prog_row, text="Ready", bg=C["panel"],
                                      fg=C["muted"], font=("Courier New", 12))
        self.lbl_progress.pack(side="left")
        self.lbl_pct = tk.Label(prog_row, text="", bg=C["panel"],
                                 fg=C["accent"], font=("Courier New", 12, "bold"))
        self.lbl_pct.pack(side="right")

        self.progress = ttk.Progressbar(bar, style="Prog.Horizontal.TProgressbar",
                                         mode="determinate")
        self.progress.pack(fill="x", padx=24, pady=(0, 10), ipady=10)

        btn_row = tk.Frame(bar, bg=C["panel"])
        btn_row.pack(fill="x", padx=24, pady=(0, 14))
        self.lbl_size = tk.Label(btn_row, text="", bg=C["panel"],
                                  fg=C["muted"], font=("Courier New", 12))
        self.lbl_size.pack(side="left")

        # "Remove USB" sits apart on the left of the action buttons (own
        # padding gap) - it's not part of the copy flow, so it shouldn't
        # look like just another step next to Cancel/Start Copy, but it
        # still needs to be reachable without hunting for it.
        self.btn_remove_usb = self._btn(btn_row, "⏏  Remove USB", self._remove_usb, warn=True)
        self.btn_remove_usb.pack(side="left", padx=(24, 0))

        self.btn_cancel = self._btn(btn_row, "✕  Cancel", self._cancel_copy, danger=True)
        self.btn_cancel.pack(side="right", padx=(10, 0))
        self.btn_cancel.configure(state="disabled")
        self.btn_copy = self._btn(btn_row, "⟶  Start Copy", self._start_copy, accent=True)
        self.btn_copy.pack(side="right")

    # ── WIDGET HELPERS ───────────────────────────────────────
    def _make_checkbox(self, parent, var):
        """A big tappable ☐/☑ label bound to `var`, sized to match the
        icon/name text in the same row (see the note at its call site)."""
        lbl = tk.Label(parent, text="☑" if var.get() else "☐",
                        bg=C["item"], fg=C["accent"],
                        font=("Courier New", 22), cursor="hand2")
        lbl.bind("<Button-1>", lambda e: var.set(not var.get()))
        var.trace_add("write", lambda *_: lbl.configure(
            text="☑" if var.get() else "☐"))
        return lbl

    def _panel(self, parent):
        f = tk.Frame(parent, bg=C["panel"],
                     highlightbackground=C["border"], highlightthickness=1)
        return f

    def _btn(self, parent, text, cmd, accent=False, danger=False, warn=False, ghost=False, **kw):
        if accent:
            bg, fg, abg = C["accent"], "#0f1117", "#00b894"
        elif danger:
            bg, fg, abg = C["danger"], "white", "#cc3355"
        elif warn:
            bg, fg, abg = C["warn"], "white", "#c96a00"
        else:
            bg, fg, abg = C["border"], C["text"], C["hover"]
        b = tk.Button(parent, text=text, command=cmd,
                      bg=bg, fg=fg, activebackground=abg, activeforeground=fg,
                      relief="flat", cursor="hand2",
                      font=("Courier New", 18, "bold" if (accent or danger or warn) else "normal"),
                      padx=18, pady=16, bd=0, **kw)
        return b

    # ── DIRECTORY BROWSER ────────────────────────────────────
    def _load_dir(self, path: Path):
        self.current_path = path
        # Update path label (relative to source root)
        try:
            rel = path.relative_to(self.source_root)
            display = f"/{rel}" if str(rel) != "." else "/"
        except ValueError:
            display = str(path)
        self.lbl_path.configure(text=display)

        # Clear list
        for w in self.item_frame.winfo_children():
            w.destroy()
        self.folder_vars.clear()
        self.folder_widgets.clear()

        if not path.exists():
            tk.Label(self.item_frame,
                     text=f"⚠  Path not found:\n{path}",
                     bg=C["panel"], fg=C["danger"],
                     font=("Courier New", 10), padx=16, pady=16,
                     justify="left").pack(anchor="w")
            return

        dirs  = sorted([p for p in path.iterdir() if p.is_dir()])
        files = sorted([p for p in path.iterdir() if p.is_file()])
        items = dirs + files

        if not items:
            tk.Label(self.item_frame, text="(empty folder)",
                     bg=C["panel"], fg=C["muted"],
                     font=("Courier New", 10), padx=16, pady=16).pack(anchor="w")
            return

        for idx, item in enumerate(items):
            is_dir = item.is_dir()
            var = tk.BooleanVar(value=False)
            self.folder_vars[str(item)] = var

            row = tk.Frame(self.item_frame, bg=C["item"], cursor="hand2")
            row.pack(fill="x",pady=14)
            self.folder_widgets[str(item)] = row

            # Hover
            row.bind("<Enter>", lambda e, r=row: r.configure(bg=C["hover"]))
            row.bind("<Leave>", lambda e, r=row: r.configure(bg=C["item"]))

            # Checkbox - a plain Label drawing ☐/☑ instead of tk.Checkbutton,
            # since the native indicator can't be sized up to match the
            # icon/name text next to it (no "-indicatordiameter" option on
            # stock Tk; this keeps it visually consistent with the row).
            cb = self._make_checkbox(row, var)
            cb.pack(side="left", padx=(12, 0), pady=12)

            # Icon
            icon_lbl = tk.Label(row, text="📁" if is_dir else "📄",
                                  bg=C["item"], font=("Courier New", 16))
            icon_lbl.pack(side="left", padx=(8, 0))

            # Name
            name_lbl = tk.Label(row, text=item.name,
                                  bg=C["item"], fg=C["text"],
                                  font=("Courier New", 14), anchor="w")
            name_lbl.pack(side="left", padx=(10, 0), fill="x", expand=True)

            # Size (files only)
            if not is_dir:
                sz = tk.Label(row, text=fmt_size(item.stat().st_size),
                               bg=C["item"], fg=C["muted"],
                               font=("Courier New", 12))
                sz.pack(side="right", padx=16)
            else:
                # "Open" button for dirs
                open_btn = tk.Button(row, text="Open →",
                                      bg=C["border"], fg=C["accent2"],
                                      activebackground=C["hover"],
                                      activeforeground=C["accent"],
                                      font=("Courier New", 16), relief="flat",
                                      padx=16, pady=10, cursor="hand2",
                                      command=lambda p=item: self._load_dir(p))
                open_btn.pack(side="right", padx=12)

            # Click row = toggle checkbox
            for w in (row, icon_lbl, name_lbl):
                w.bind("<Button-1>", lambda e, v=var: v.set(not v.get()))

            # Separator
            if idx < len(items) - 1:
                tk.Frame(self.item_frame, bg=C["border"], height=1).pack(fill="x")

        self._list_canvas.yview_moveto(0)

    def _go_up(self):
        if self.current_path != self.source_root:
            self._load_dir(self.current_path.parent)

    def _select_all(self):
        for v in self.folder_vars.values():
            v.set(True)

    def _deselect_all(self):
        for v in self.folder_vars.values():
            v.set(False)

    # ── USB COMBO ────────────────────────────────────────────
    def _populate_usb_combo(self):
        if self.usb_drives:
            vals = [f"{d['label']}  [{d['mount']}]  {d['free']} free"
                    for d in self.usb_drives]
            self.combo_usb["values"] = vals
            self.combo_usb.current(0)
            self.lbl_usb_dot.configure(
                text=f"● {len(self.usb_drives)} USB connected", fg=C["success"])
        else:
            self.combo_usb["values"] = []
            self.combo_usb.set("")
            self.lbl_usb_dot.configure(text="● No USB found", fg=C["danger"])

    def _refresh_usb(self):
        self.usb_drives = detect_usb_drives()
        self._populate_usb_combo()
        self._log(f"Refreshed: {len(self.usb_drives)} USB drive(s) found", "accent")

    def _on_usb_select(self, _=None):
        idx = self.combo_usb.current()
        if 0 <= idx < len(self.usb_drives):
            self.selected_usb_idx = idx
            d = self.usb_drives[idx]
            self._log(f"Selected USB: {d['mount']}", "accent")

    def _get_usb_mount(self):
        idx = self.combo_usb.current()
        if 0 <= idx < len(self.usb_drives):
            return self.usb_drives[idx]["mount"]
        return None

    # ── COPY ─────────────────────────────────────────────────
    def _get_selected(self):
        return [Path(p) for p, v in self.folder_vars.items() if v.get()]

    def _start_copy(self):
        if self.is_copying:
            return
        mount = self._get_usb_mount()
        if not mount:
            messagebox.showwarning("No USB", "Please connect a USB drive and select it.")
            return
        items = self._get_selected()
        if not items:
            messagebox.showwarning("Nothing selected", "Please select at least one item to copy.")
            return

        names = "\n".join(f"  • {i.name}" for i in items)
        if not messagebox.askyesno("Confirm Copy",
                                    f"Copy {len(items)} item(s) to:\n{mount}\n\n{names}\n\nProceed?"):
            return

        self.is_copying = True
        self.cancel_flag.clear()
        self.btn_copy.configure(state="disabled")
        self.btn_cancel.configure(state="normal")
        self.progress.configure(value=0)
        self._log("─" * 30, "muted")
        self._log(f"Starting copy → {mount}", "accent")

        threading.Thread(target=self._copy_worker,
                         args=(items, mount), daemon=True).start()

    def _copy_worker(self, items, dest_root):
        try:
            total = sum(calc_size(i) for i in items)
            self.total_bytes = total
            self.copied_bytes = 0
            self.root.after(0, lambda: self.lbl_size.configure(
                text=f"Total: {fmt_size(total)}"))

            errors = 0
            for item in items:
                if self.cancel_flag.is_set():
                    self.root.after(0, lambda: self._log("⚠ Cancelled by user", "warn"))
                    break
                dest = Path(dest_root) / item.name
                self.root.after(0, lambda n=item.name: self._log(f"Copying: {n}", "info"))
                try:
                    self._copy_item(item, dest)
                    self.root.after(0, lambda n=item.name: self._log(f"✓ {n}", "success"))
                except Exception as ex:
                    errors += 1
                    self.root.after(0, lambda n=item.name, e=str(ex):
                                    self._log(f"✗ {n}: {e}", "err"))

            if not self.cancel_flag.is_set():
                ok = len(items) - errors
                tag = "success" if errors == 0 else "warn"
                self.root.after(0, lambda: self._log(
                    f"Done! {ok}/{len(items)} item(s) copied successfully", tag))
                self.root.after(0, lambda: self.lbl_progress.configure(text="Done ✓"))
                self.root.after(0, lambda: self.progress.configure(value=100))
                if errors == 0:
                    self.root.after(0, lambda: messagebox.showinfo(
                        "Done", f"Successfully copied {ok} item(s) to USB!"))
                else:
                    self.root.after(0, lambda: messagebox.showwarning(
                        "Done with errors", f"Finished but {errors} error(s) occurred.\nCheck the log."))
        finally:
            self.is_copying = False
            self.root.after(0, lambda: self.btn_copy.configure(state="normal"))
            self.root.after(0, lambda: self.btn_cancel.configure(state="disabled"))

    def _copy_item(self, src, dest):
        if self.cancel_flag.is_set():
            return
        if src.is_file():
            shutil.copy2(str(src), str(dest))
            self.copied_bytes += src.stat().st_size
            self._update_progress()
        elif src.is_dir():
            dest.mkdir(parents=True, exist_ok=True)
            for child in src.iterdir():
                if self.cancel_flag.is_set():
                    return
                self._copy_item(child, dest / child.name)

    def _update_progress(self):
        if self.total_bytes > 0:
            pct = min(100, int(self.copied_bytes * 100 / self.total_bytes))
            cb, tb = self.copied_bytes, self.total_bytes
            self.root.after(0, lambda p=pct, c=cb, t=tb: (
                self.progress.configure(value=p),
                self.lbl_pct.configure(text=f"{p}%"),
                self.lbl_progress.configure(
                    text=f"{fmt_size(c)} / {fmt_size(t)}")
            ))

    def _cancel_copy(self):
        if self.is_copying:
            self.cancel_flag.set()
            self._log("Cancelling...", "warn")

    # ── SAFE REMOVE ──────────────────────────────────────────
    def _remove_usb(self):
        """Unmount the selected USB drive and close the window - lets the
        user pull the drive right after, instead of yanking it out while
        it's still mounted (the usual cause of corrupted USB filesystems)."""
        if self.is_copying:
            messagebox.showwarning(
                "Copy in progress",
                "A copy is still running. Cancel or wait for it to finish before removing the USB drive.")
            return

        idx = self.combo_usb.current()
        if not (0 <= idx < len(self.usb_drives)):
            messagebox.showwarning("No USB", "No USB drive selected.")
            return

        drive = self.usb_drives[idx]
        if not messagebox.askyesno(
                "Remove USB",
                f"Safely unmount and close this window for:\n\n{drive['label']}  [{drive['mount']}]\n\nProceed?"):
            return

        self.btn_remove_usb.configure(state="disabled")
        self._log(f"Unmounting {drive['mount']} ...", "warn")
        threading.Thread(target=self._unmount_worker, args=(drive,), daemon=True).start()

    def _unmount_worker(self, drive):
        device = f"/dev/{drive['name']}" if drive.get("name") else None
        error = None
        try:
            unmounted = False
            if device:
                result = subprocess.run(
                    ["udisksctl", "unmount", "-b", device],
                    capture_output=True, text=True, timeout=15)
                unmounted = result.returncode == 0
                if not unmounted:
                    error = result.stderr.strip() or result.stdout.strip()
            if not unmounted:
                # Fallback for systems without udisks/polkit set up for
                # unprivileged unmount - requires the service to already
                # have permission (e.g. running as the drive's owner/root).
                result = subprocess.run(
                    ["umount", drive["mount"]],
                    capture_output=True, text=True, timeout=15)
                unmounted = result.returncode == 0
                if not unmounted:
                    error = result.stderr.strip() or error or "unmount failed"

            if unmounted:
                self.root.after(0, lambda: self._on_unmount_done(drive, None))
            else:
                self.root.after(0, lambda: self._on_unmount_done(drive, error))
        except Exception as ex:
            self.root.after(0, lambda: self._on_unmount_done(drive, str(ex)))

    def _on_unmount_done(self, drive, error):
        self.btn_remove_usb.configure(state="normal")
        if error:
            self._log(f"✗ Unmount failed: {error}", "err")
            messagebox.showerror(
                "Unmount Failed",
                f"Could not safely unmount {drive['label']}:\n{error}\n\n"
                "Do not remove the USB drive - try again or unmount it manually.")
            return

        self._log(f"✓ {drive['label']} unmounted - safe to remove now", "success")
        messagebox.showinfo(
            "Safe to Remove",
            f"{drive['label']} has been safely unmounted.\nYou can now remove the USB drive.")
        self.root.withdraw()

    # ── LOG ──────────────────────────────────────────────────
    def _log(self, msg, tag="info"):
        def _do():
            self.log_txt.configure(state="normal")
            ts = time.strftime("%H:%M:%S")
            self.log_txt.insert("end", f"[{ts}] {msg}\n", tag)
            self.log_txt.see("end")
            self.log_txt.configure(state="disabled")
        self.root.after(0, _do)

    def _on_close(self):
        """Hide window instead of destroying, so service keeps running."""
        self.root.withdraw()


# ══════════════════════════════════════════════════════════════
#  BACKGROUND SERVICE  (runs in main thread via Tk mainloop)
# ══════════════════════════════════════════════════════════════
class USBService:
    def __init__(self):
        self.root = tk.Tk()
        self.root.withdraw()          # start hidden
        self.root.title("USB Data Copier - Service")
        self.root.configure(bg=C["bg"])
        self.root.attributes("-topmost", True)

        self._prev_mounts: set = set()
        self._window: USBCopierWindow | None = None

        self._poll()   # start polling loop

    def _poll(self):
        try:
            drives = detect_usb_drives()
            mounts = set(d["mount"] for d in drives)

            # New USB plugged in
            new = mounts - self._prev_mounts
            if new:
                self._show_window(drives)

            # All USB removed → hide window
            if not mounts and self._prev_mounts:
                self._hide_window()

            # USB list changed while window open → refresh combo
            if mounts != self._prev_mounts and self._window and self.root.state() != "withdrawn":
                self._window.usb_drives = drives
                self._window._populate_usb_combo()

            self._prev_mounts = mounts
        except Exception:
            pass

        self.root.after(SCAN_INTERVAL_MS, self._poll)

    def _show_window(self, drives):
        if self.root.state() == "withdrawn":
            # First time or re-show: rebuild window content
            if self._window is None:
                self._window = USBCopierWindow(self.root, drives)

            self.root.deiconify()
            self.root.lift()
            self.root.focus_force()
        else:
            # Already visible, just update
            if self._window:
                self._window.usb_drives = drives
                self._window._populate_usb_combo()

    def _hide_window(self):
        self.root.withdraw()

    def run(self):
        self.root.mainloop()


# ══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════
def main():
    if len(sys.argv) > 1:
        global SOURCE_DATA_PATH
        SOURCE_DATA_PATH = sys.argv[1]

    print("USB Data Copier service started.")
    print(f"Data source: {SOURCE_DATA_PATH}")
    print("Waiting for USB drives... (close the window to keep running in background)")

    svc = USBService()
    svc.run()


if __name__ == "__main__":
    main()