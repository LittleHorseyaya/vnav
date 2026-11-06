#!/usr/bin/env python3
"""Build the submission-ready LAB5 experiment report from measured metrics."""

import json
from html import escape
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Image,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


HERE = Path(__file__).resolve().parent
PACKAGE = HERE / "feature_tracking"
METRICS = json.loads((PACKAGE / "results" / "metrics.json").read_text())
if len(METRICS["datasets"]) != 2:
    raise RuntimeError("Team PDF requires both real rosbag result tables.")

pdfmetrics.registerFont(TTFont("ArialLab", r"C:\Windows\Fonts\arial.ttf"))
pdfmetrics.registerFont(TTFont("ArialLabBold", r"C:\Windows\Fonts\arialbd.ttf"))

NAVY = colors.HexColor("#17324D")
TEAL = colors.HexColor("#087E8B")
INK = colors.HexColor("#22313F")
MUTED = colors.HexColor("#596A7A")
PALE = colors.HexColor("#EFF6F8")
WHITE = colors.white
PAGE_WIDTH, PAGE_HEIGHT = landscape(A4)
CONTENT_WIDTH = PAGE_WIDTH - 32 * mm

title = ParagraphStyle(
    "Title", fontName="ArialLabBold", fontSize=18, leading=22,
    textColor=NAVY, spaceAfter=4,
)
subtitle = ParagraphStyle(
    "Subtitle", fontName="ArialLab", fontSize=8.5, leading=12,
    textColor=MUTED, spaceAfter=13,
)
heading = ParagraphStyle(
    "Heading", fontName="ArialLabBold", fontSize=11, leading=14,
    textColor=TEAL, spaceBefore=7, spaceAfter=6,
)
body = ParagraphStyle(
    "Body", fontName="ArialLab", fontSize=9.2, leading=13.1,
    textColor=INK, spaceAfter=7,
)
small = ParagraphStyle(
    "Small", parent=body, fontSize=8.5, leading=12, spaceAfter=5,
)
caption = ParagraphStyle(
    "Caption", parent=body, fontSize=8, leading=11,
    textColor=MUTED, spaceAfter=5,
)
table_header = ParagraphStyle(
    "TableHeader", fontName="ArialLabBold", fontSize=7.8, leading=9.2,
    textColor=WHITE, alignment=TA_CENTER,
)
table_cell = ParagraphStyle(
    "TableCell", fontName="ArialLab", fontSize=8.3, leading=10.5,
    textColor=INK, alignment=TA_CENTER,
)
table_left = ParagraphStyle(
    "TableLeft", parent=table_cell, alignment=TA_LEFT,
)


def p(value, style=table_cell):
    return Paragraph(escape(str(value)), style)


def styled_table(headers, rows, widths):
    data = [[Paragraph(h, table_header) for h in headers]]
    data += [
        [p(value, table_left if column == 0 else table_cell)
         for column, value in enumerate(row)]
        for row in rows
    ]
    table = Table(data, colWidths=widths, repeatRows=1, hAlign="LEFT")
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), NAVY),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [WHITE, PALE]),
        ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#D7E3E9")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("TOPPADDING", (0, 0), (-1, -1), 6),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
    ]))
    return table


def scaled_image(path, width):
    source_width, source_height = ImageReader(str(path)).getSize()
    return Image(str(path), width=width,
                 height=width * source_height / source_width)


def footer(canvas, document):
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor("#D9E3EA"))
    canvas.line(16 * mm, 15 * mm, PAGE_WIDTH - 16 * mm, 15 * mm)
    canvas.setFont("ArialLab", 7.6)
    canvas.setFillColor(MUTED)
    canvas.drawString(16 * mm, 9 * mm, "MIT VNAV LAB5  |  Feature tracking comparison")
    canvas.drawRightString(PAGE_WIDTH - 16 * mm, 9 * mm,
                           str(document.page))
    canvas.restoreState()


pair_rows = [
    [
        item["method"], item["keypoints1"], item["keypoints2"],
        item["matches"], item["good"], item["inliers"],
        f'{item["ratio"]:.1f}%',
    ]
    for item in METRICS["pair"]
]
pair_headers = [
    "Method", "Keypoints 1", "Keypoints 2", "Raw matches",
    "Good matches", "Inliers", "Inlier ratio",
]
pair_widths = [95, 99, 99, 105, 105, 90, CONTENT_WIDTH - 593]


def dataset_section(dataset, short_title):
    rows = [
        [
            item["method"], f'{item["keypoints1"]:.1f}',
            f'{item["keypoints2"]:.1f}', f'{item["matches"]:.1f}',
            f'{item["good"]:.1f}', f'{item["inliers"]:.1f}',
            f'{item["ratio"]:.1f}%', item["pairs"],
        ]
        for item in dataset["rows"]
    ]
    headers = [
        "Method", "Avg. KP1", "Avg. KP2", "Avg. matches",
        "Avg. good", "Avg. inliers", "Avg. ratio", "Pairs",
    ]
    widths = [96, 91, 91, 100, 95, 95, 95, CONTENT_WIDTH - 663]
    return [
        Paragraph(short_title, heading),
        Paragraph(
            f'{escape(dataset["name"])}  |  {dataset["image_frames"]} '
            f'RGB frames  |  {escape(dataset.get("playback_note", str(dataset["playback_rate"]) + "x playback"))}',
            caption,
        ),
        styled_table(headers, rows, widths),
    ]


story = [
    Paragraph("LAB5 Feature Tracking Comparison", title),
    Paragraph(
        "VNAV2021 code  |  Ubuntu 20.04 / ROS Noetic  |  OpenCV 3.4.2",
        subtitle,
    ),
    Paragraph("Methods and scoring", heading),
    Paragraph(
        "SIFT and SURF use FLANN two-neighbor matching and the 0.8 distance "
        "ratio. ORB uses Hamming-distance brute force with the same ratio. "
        "Harris+LK tracks corners by pyramidal optical flow. All methods use "
        "fundamental-matrix RANSAC with a 3-pixel threshold and 0.99 confidence "
        "to count geometric inliers.", small,
    ),
    Paragraph("Bundled image pair", heading),
    Paragraph(
        "box.png to box_in_scene.png. SIFT descriptors have 128 float values; "
        "SURF has 64 float values; ORB has 32 bytes.", caption,
    ),
    styled_table(pair_headers, pair_rows, pair_widths),
    Spacer(1, 13),
]

pair_image = scaled_image(
    PACKAGE / "results" / "matches_inliers.png", 380
)
pair_interpretation = Paragraph(
    "<b>Observation.</b> The large viewpoint and scale change makes this "
    "pair harder than adjacent video frames. SURF gave the most RANSAC "
    "inliers (85), while SIFT had the higher inlier fraction (79.8%). "
    "At the supplied 500-feature, one-level setting, ORB retained only "
    "13 unambiguous matches and 7 inliers. Green lines in the figure are "
    "inliers; red lines are rejected matches.",
    body,
)
illustration = Table(
    [[pair_image, pair_interpretation]], colWidths=[395, CONTENT_WIDTH - 395]
)
illustration.setStyle(TableStyle([
    ("VALIGN", (0, 0), (-1, -1), "TOP"),
    ("LEFTPADDING", (0, 0), (-1, -1), 0),
    ("RIGHTPADDING", (0, 0), (-1, -1), 9),
    ("TOPPADDING", (0, 0), (-1, -1), 0),
    ("BOTTOMPADDING", (0, 0), (-1, -1), 0),
]))
story += [illustration, PageBreak()]

evidence_grid = Table([
    [
        [Paragraph("1. SIFT keypoints in box.png", caption),
         scaled_image(PACKAGE / "results" / "keypoints_1.png", 270)],
        [Paragraph("2. SIFT keypoints in box_in_scene.png", caption),
         scaled_image(PACKAGE / "results" / "keypoints_2.png", 270)],
    ],
    [
        [Paragraph("3. Raw nearest-neighbor matches", caption),
         scaled_image(PACKAGE / "results" / "matches_raw.png", 345)],
        [Paragraph("4. Matches after the 0.8 ratio test", caption),
         scaled_image(PACKAGE / "results" / "matches_ratio.png", 345)],
    ],
], colWidths=[CONTENT_WIDTH / 2] * 2)
evidence_grid.setStyle(TableStyle([
    ("VALIGN", (0, 0), (-1, -1), "TOP"),
    ("LEFTPADDING", (0, 0), (-1, -1), 0),
    ("RIGHTPADDING", (0, 0), (-1, -1), 8),
    ("TOPPADDING", (0, 0), (-1, -1), 0),
    ("BOTTOMPADDING", (0, 0), (-1, -1), 13),
]))
story += [
    Paragraph("SIFT detection and matching figures", title),
    Paragraph(
        "The two-image run saves these figures directly from the tracker. "
        "There are 603 and 969 keypoints, 603 raw nearest-neighbor matches, "
        "and 99 matches after the ratio test.", subtitle,
    ),
    evidence_grid,
    Paragraph(
        "The ratio test removes ambiguous correspondences before geometric "
        "verification. The colored inlier/outlier plot appears on page 1.",
        caption,
    ),
    PageBreak(),
]

story += [
    Paragraph("Real flight sequences", title),
    Paragraph(
        "Each table averages all adjacent RGB-frame pairs. Avg. ratio is "
        "the mean per-pair inlier fraction. Playback was slowed as needed "
        "so the VM did not drop images.", subtitle,
    ),
]
story += dataset_section(METRICS["datasets"][0], "Dataset 1: 30fps room sequence")
story += [Spacer(1, 10)]
story += dataset_section(METRICS["datasets"][1], "Dataset 2: smooth trajectory")
story += [
    Spacer(1, 12),
    Paragraph("Interpretation and limits", heading),
    Paragraph(
        "SURF produced the most average inliers on both real sequences "
        "(320 and 610). ORB was weak on the separated box pair, yet exceeded "
        "SIFT on the smooth trajectory (151 versus 81 inliers), showing the "
        "effect of scene and motion. Harris+LK had the highest inlier fraction "
        "but fewer tracks. It assumes small local displacement and roughly "
        "constant brightness; occlusion and appearance changes can break "
        "tracks. Valid tracks count as good matches, with no descriptor ratio "
        "test. Runtime was not measured, so no speed ranking is claimed.",
        small,
    ),
    Paragraph(
        "For the LK visual, see the next page. The ROS Noetic cv_bridge "
        "links system OpenCV 4.2 while the historical lab builds OpenCV 3.4.2; "
        "both modes ran on this VM despite linker warnings.",
        small,
    ),
    PageBreak(),
    Paragraph("Harris + LK track example", title),
    Paragraph(
        "Real RGB frames 300 and 305 from Dataset 1. The five-frame gap "
        "makes short tracks visible; 75 of 78 previous-frame corners were "
        "tracked in this illustration.", subtitle,
    ),
    scaled_image(PACKAGE / "results" / "lk_real" / "lk_tracks.png", 650),
    Spacer(1, 7),
    Paragraph(
        "Green segments show motion between the two frames; red dots mark "
        "tracked positions in the later image. The full-dataset LK statistics "
        "on page 3 use consecutive frames, as do the other methods.",
        caption,
    ),
    Spacer(1, 12),
    Paragraph(
        "Assignment and data: MIT 16.485 VNAV LAB5 Exercises (2023), "
        "vnav.mit.edu/labs_2023/lab5/exercises.html; "
        "github.com/MIT-SPARK/VNAV-lab-data.",
        caption,
    ),
]

output = HERE / "team_report.pdf"
document = SimpleDocTemplate(
    str(output), pagesize=landscape(A4),
    leftMargin=16 * mm, rightMargin=16 * mm,
    topMargin=13 * mm, bottomMargin=20 * mm,
)
document.build(story, onFirstPage=footer, onLaterPages=footer)
print(output)
