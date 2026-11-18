# Intelijet v3 Operator Manual — Table of Contents

> This manual is written entirely in English (decision 2026-08-18) for field operators and support technicians.
> Planning/briefs for each chapter (in Vietnamese, internal use only) live in `docs/manual_plan/`.
> Any `⚠️ NEEDS CONFIRMATION` block in a chapter is still unanswered — do not delete or guess at it until confirmed by someone with real operating/hardware knowledge.

| # | Chapter | Status |
|---|---------|--------|
| 01 | [System Introduction](01_gioi_thieu.md) | Draft written — pending review |
| 02 | [Safety Warnings](02_canh_bao_an_toan.md) | Nearly complete — only accident contact info and hydraulic pressure-release steps still NEEDS CONFIRMATION |
| 03 | [Initial Installation](03_cai_dat.md) | Draft written — pending review |
| 04 | [Daily Startup & Shutdown](04_khoi_dong_tat.md) | Draft written — pending review |
| 05 | [User Interface Overview](05_giao_dien.md) | Draft written — pending review |
| 06 | [Daily Operating Procedure (Scan SOP)](06_quy_trinh_van_hanh.md) | Draft written — pending review, 2 items NEEDS CONFIRMATION |
| 07 | [Device Monitoring & Alarms](07_giam_sat_canh_bao.md) | Draft written — pending review |
| 08 | [Troubleshooting](08_xu_ly_su_co.md) | Draft written — pending review |
| 09 | [Maintenance & Software Updates](09_bao_tri_cap_nhat.md) | Draft written — pending review, 2 items NEEDS CONFIRMATION |
| 10 | [Data Sync Between Tablets (Syncthing)](10_dong_bo_du_lieu.md) | Draft written — pending review |
| 11 | [USB Data Copier](11_usb_copier.md) | Draft written — pending review |
| 12 | [Appendix](12_phu_luc.md) | Draft written — pending review, hardware specs + support contact NEEDS CONFIRMATION |

All 12 chapters now have a first draft. Images/diagrams: `assets/` (no real photos yet — chapters use `[PHOTO: description]` placeholders).

## Still open across the whole manual

- **Chapter 2** — hydraulic arm pressure-release procedure (user will provide); accident contact name/phone.
- **Chapter 3** — whether a separate hardware installation procedure exists.
- **Chapter 6** — how the hydraulic arm is actually controlled; how to generate a report manually when Auto Report is off. (Corrected 2026-08-18: Compare runs automatically after Post-Scan, producing a concrete-thickness heatmap — the Compare button is not part of the normal flow. The SOP now intentionally ends at step 6 (reviewing the heatmap) — closing the Housing, viewing the report, and reviewing job history are no longer part of this chapter.)
- **Chapter 9** — hardware maintenance schedule; data retention policy.
- **Chapter 12** — hardware specs beyond the scanner; support contact info.

File names stay in Vietnamese/pinyin-free slugs for now (internal identifiers only) — only the written content must be English.
