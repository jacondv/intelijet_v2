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
        self.root.geometry("960x660")
        self.root.minsize(600, 800)
        self.root.configure(bg=C["bg"])
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
                        arrowcolor=C["muted"],arrowsize=40, borderwidth=0, width=40)
        style.configure("Prog.Horizontal.TProgressbar",
                        troughcolor=C["border"], background=C["accent"],
                        borderwidth=0, thickness=10)
        style.configure("TCombobox", 
                        padding=10, arrowsize=30, font=("Courier New", 16))

    # ── UI BUILD ─────────────────────────────────────────────
    def _build_ui(self):
        # Header
        hdr = tk.Frame(self.root, bg=C["panel"], height=56)
        hdr.pack(fill="x")
        hdr.pack_propagate(False)
        tk.Label(hdr, text="⟢  USB DATA COPIER",
                 bg=C["panel"], fg=C["accent"],
                 font=("Courier New", 15, "bold")).pack(side="left", padx=20)
        self.lbl_usb_dot = tk.Label(hdr, text="● 0 USB connected",
                                     bg=C["panel"], fg=C["danger"],
                                     font=("Courier New", ))
        self.lbl_usb_dot.pack(side="right", padx=20)
        tk.Frame(self.root, bg=C["border"], height=1).pack(fill="x")

        # Body: left | right
        body = tk.Frame(self.root, bg=C["bg"])
        body.pack(fill="both", expand=True)

        left = tk.Frame(body, bg=C["bg"])
        left.pack(side="left", fill="both", expand=True, padx=(16, 8), pady=16)

        right = tk.Frame(body, bg=C["bg"], width=290)
        right.pack(side="right", fill="y", padx=(0, 16), pady=16)
        right.pack_propagate(False)

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
                 font=("Courier New", 8, "bold")).pack(anchor="w", padx=14, pady=(10, 2))
        usb_row = tk.Frame(usb_box, bg=C["panel"])
        usb_row.pack(fill="x", padx=14, pady=(0, 10))
        self.combo_usb = ttk.Combobox(usb_row, state="readonly", font=("Courier New", 14),style="TCombobox")
        self.combo_usb.pack(side="left", fill="x", expand=True)
        self.combo_usb.bind("<<ComboboxSelected>>", self._on_usb_select)
        self._btn(usb_row, "↻", self._refresh_usb, ghost=True, width=3).pack(side="right", padx=(6, 0))

        # ── Breadcrumb path bar
        nav_box = self._panel(parent)
        nav_box.pack(fill="x", pady=(0, 10))
        nav_inner = tk.Frame(nav_box, bg=C["panel"])
        nav_inner.pack(fill="x", padx=14, pady=8)
        self._btn(nav_inner, "⬆ Up", self._go_up, ghost=True).pack(side="left")
        self.lbl_path = tk.Label(nav_inner, text="", bg=C["panel"], fg=C["accent2"],
                                  font=("Courier New", 10), anchor="w")
        self.lbl_path.pack(side="left", padx=(10, 0), fill="x", expand=True)

        # ── Folder / file list header
        hdr_row = tk.Frame(parent, bg=C["bg"])
        hdr_row.pack(fill="x", pady=(0, 5))
        tk.Label(hdr_row, text="SELECT ITEMS TO COPY",
                 bg=C["bg"], fg=C["muted"],
                 font=("Courier New", 14, "bold")).pack(side="left")
        self._btn(hdr_row, "None", self._deselect_all, ghost=True).pack(side="right")
        self._btn(hdr_row, "All",  self._select_all,   ghost=True).pack(side="right", padx=(0, 4))

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
                                font=("Courier New", 9), relief="flat", bd=0,
                                state="disabled", wrap="word",
                                selectbackground=C["border"], padx=8, pady=8)
        log_sb = ttk.Scrollbar(log_box, command=self.log_txt.yview)
        self.log_txt.configure(yscrollcommand=log_sb.set)
        log_sb.pack(side="right", fill="y")
        self.log_txt.pack(fill="both", expand=True)
        for tag, col in [("accent", C["accent"]), ("success", C["success"]),
                         ("warn", C["warn"]), ("err", C["danger"]),
                         ("muted", C["muted"]), ("info", C["text"])]:
            self.log_txt.tag_config(tag, foreground=col)

    def _build_bottom(self):
        bar = tk.Frame(self.root, bg=C["panel"], height=200)
        bar.pack(fill="x", side="bottom")
        bar.pack_propagate(False)

        prog_row = tk.Frame(bar, bg=C["panel"])
        prog_row.pack(fill="x", padx=20, pady=(10, 3))
        self.lbl_progress = tk.Label(prog_row, text="Ready", bg=C["panel"],
                                      fg=C["muted"], font=("Courier New", 9))
        self.lbl_progress.pack(side="left")
        self.lbl_pct = tk.Label(prog_row, text="", bg=C["panel"],
                                 fg=C["accent"], font=("Courier New", 9, "bold"))
        self.lbl_pct.pack(side="right")

        self.progress = ttk.Progressbar(bar, style="Prog.Horizontal.TProgressbar",
                                         mode="determinate")
        self.progress.pack(fill="x", padx=20, pady=(0, 6),ipady=8)

        btn_row = tk.Frame(bar, bg=C["panel"])
        btn_row.pack(fill="x", padx=20, pady=(0, 10))
        self.lbl_size = tk.Label(btn_row, text="", bg=C["panel"],
                                  fg=C["muted"], font=("Courier New", 9))
        self.lbl_size.pack(side="left")
        self.btn_cancel = self._btn(btn_row, "✕  Cancel", self._cancel_copy, danger=True)
        self.btn_cancel.pack(side="right", padx=(6, 0))
        self.btn_cancel.configure(state="disabled")
        self.btn_copy = self._btn(btn_row, "⟶  Start Copy", self._start_copy, accent=True)
        self.btn_copy.pack(side="right")

    # ── WIDGET HELPERS ───────────────────────────────────────
    def _panel(self, parent):
        f = tk.Frame(parent, bg=C["panel"],
                     highlightbackground=C["border"], highlightthickness=1)
        return f

    def _btn(self, parent, text, cmd, accent=False, danger=False, ghost=False, **kw):
        if accent:
            bg, fg, abg = C["accent"], "#0f1117", "#00b894"
        elif danger:
            bg, fg, abg = C["danger"], "white", "#cc3355"
        else:
            bg, fg, abg = C["border"], C["text"], C["hover"]
        b = tk.Button(parent, text=text, command=cmd,
                      bg=bg, fg=fg, activebackground=abg, activeforeground=fg,
                      relief="flat", cursor="hand2",
                      font=("Courier New", 16, "bold" if (accent or danger) else "normal"),
                      padx=10, pady=10, bd=0, **kw)
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
            row.pack(fill="x",pady=10)
            self.folder_widgets[str(item)] = row

            # Hover
            row.bind("<Enter>", lambda e, r=row: r.configure(bg=C["hover"]))
            row.bind("<Leave>", lambda e, r=row: r.configure(bg=C["item"]))

            # Checkbox
            cb = tk.Checkbutton(row, variable=var,
                                 bg=C["item"], activebackground=C["hover"],
                                 selectcolor=C["bg"], fg=C["accent"],
                                 font=("Courier New", 16),
                                 relief="flat", bd=0)
            cb.pack(side="left", padx=(8, 0), pady=6)

            # Icon
            icon_lbl = tk.Label(row, text="📁" if is_dir else "📄",
                                  bg=C["item"], font=("Courier New", 11))
            icon_lbl.pack(side="left", padx=(4, 0))

            # Name
            name_lbl = tk.Label(row, text=item.name,
                                  bg=C["item"], fg=C["text"],
                                  font=("Courier New", 10), anchor="w")
            name_lbl.pack(side="left", padx=(6, 0), fill="x", expand=True)

            # Size (files only)
            if not is_dir:
                sz = tk.Label(row, text=fmt_size(item.stat().st_size),
                               bg=C["item"], fg=C["muted"],
                               font=("Courier New", 8))
                sz.pack(side="right", padx=12)
            else:
                # "Open" button for dirs
                open_btn = tk.Button(row, text="Open →",
                                      bg=C["border"], fg=C["accent2"],
                                      activebackground=C["hover"],
                                      activeforeground=C["accent"],
                                      font=("Courier New", 14), relief="flat",
                                      padx=8, pady=2, cursor="hand2",
                                      command=lambda p=item: self._load_dir(p))
                open_btn.pack(side="right", padx=8)

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