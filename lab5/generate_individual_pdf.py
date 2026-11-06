#!/usr/bin/env python3
"""Typeset LAB5 individual derivations as a two-page PDF."""

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


HERE = Path(__file__).resolve().parent
pdfmetrics.registerFont(TTFont("ArialLab", r"C:\Windows\Fonts\arial.ttf"))
pdfmetrics.registerFont(TTFont("ArialLabBold", r"C:\Windows\Fonts\arialbd.ttf"))

NAVY = colors.HexColor("#17324D")
TEAL = colors.HexColor("#087E8B")
INK = colors.HexColor("#22313F")
MUTED = colors.HexColor("#596A7A")
PALE = colors.HexColor("#EFF6F8")
PAGE_WIDTH, PAGE_HEIGHT = A4
CONTENT_WIDTH = PAGE_WIDTH - 36 * mm

title = ParagraphStyle(
    "Title", fontName="ArialLabBold", fontSize=19, leading=23,
    textColor=NAVY, spaceAfter=5,
)
subtitle = ParagraphStyle(
    "Subtitle", fontName="ArialLab", fontSize=9, leading=13,
    textColor=MUTED, spaceAfter=18,
)
heading = ParagraphStyle(
    "Heading", fontName="ArialLabBold", fontSize=12, leading=16,
    textColor=TEAL, spaceBefore=10, spaceAfter=9,
)
body = ParagraphStyle(
    "Body", fontName="ArialLab", fontSize=10.5, leading=16,
    textColor=INK, spaceAfter=9,
)
equation = ParagraphStyle(
    "Equation", parent=body, fontSize=10, leading=15,
    alignment=TA_CENTER, spaceAfter=0,
)


def eq(markup):
    box = Table([[Paragraph(markup, equation)]], colWidths=[CONTENT_WIDTH])
    box.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), PALE),
        ("BOX", (0, 0), (-1, -1), 0.4, colors.HexColor("#D4E8EB")),
        ("LEFTPADDING", (0, 0), (-1, -1), 11),
        ("RIGHTPADDING", (0, 0), (-1, -1), 11),
        ("TOPPADDING", (0, 0), (-1, -1), 10),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 10),
    ]))
    return box


def page_footer(canvas, document):
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor("#D9E3EA"))
    canvas.line(18 * mm, 17 * mm, PAGE_WIDTH - 18 * mm, 17 * mm)
    canvas.setFont("ArialLab", 8)
    canvas.setFillColor(MUTED)
    canvas.drawString(18 * mm, 11 * mm, "MIT VNAV LAB5  |  Individual exercises")
    canvas.drawRightString(PAGE_WIDTH - 18 * mm, 11 * mm, str(document.page))
    canvas.restoreState()


story = [
    Paragraph("LAB5 Individual Exercises", title),
    Paragraph("Perspective projection and vanishing points  |  VNAV2021", subtitle),
    Paragraph("1. Projection of a sphere", heading),
    Paragraph(
        "Take the image plane as z = 1, the optical center as the origin, "
        "and normalized image coordinates (u, v). A ray through (u, v) "
        "has direction w = (u, v, 1). For a sphere centered at "
        "C = (0, 0, d), with radius r and d &gt; r + 1, a boundary ray "
        "is tangent when its distance from C equals r.", body,
    ),
    eq("distance<super>2</super> = "
       "d<super>2</super>(u<super>2</super> + v<super>2</super>) "
       "/ (1 + u<super>2</super> + v<super>2</super>) = r<super>2</super>"),
    Spacer(1, 9),
    Paragraph("Solving for the image boundary gives", body),
    eq("u<super>2</super> + v<super>2</super> = "
       "r<super>2</super> / (d<super>2</super> - r<super>2</super>)."),
    Spacer(1, 9),
    Paragraph(
        "The projection is a filled circle centered at the principal point. "
        "Its radius is r / sqrt(d<super>2</super> - r<super>2</super>). "
        "This is slightly larger than the simple r/d approximation because "
        "the silhouette rays are tangent to the sphere.", body,
    ),
    Paragraph("Off-axis center", heading),
    Paragraph(
        "Now take C = (a, b, d), still with d &gt; r so the sphere stays "
        "in front of the camera. The tangent-ray condition becomes", body,
    ),
    eq("(au + bv + d)<super>2</super> = "
       "(a<super>2</super> + b<super>2</super> + d<super>2</super> "
       "- r<super>2</super>)(u<super>2</super> + v<super>2</super> + 1)."),
    Spacer(1, 9),
    Paragraph(
        "Let ρ = sqrt(a<super>2</super> + b<super>2</super>), "
        "A = d<super>2</super> - r<super>2</super>, D = ρ<super>2</super> + A, "
        "x = (au + bv)/ρ, and y = (-bu + av)/ρ. Completing the square yields", body,
    ),
    eq("(x - ρd/A)<super>2</super> / (r<super>2</super>D/A<super>2</super>) "
       "+ y<super>2</super> / (r<super>2</super>/A) = 1."),
    Spacer(1, 9),
    Paragraph(
        "Thus the off-axis silhouette is an ellipse. Its semiaxes are "
        "r sqrt(D)/A along the displacement direction and r/sqrt(A) "
        "perpendicular to it. When a = b = 0, they become equal and the "
        "ellipse reduces to the on-axis circle.", body,
    ),
    PageBreak(),
    Paragraph("2. Vanishing points", heading),
    Paragraph(
        "Let two distinct 3D lines share direction q = (q<sub>x</sub>, "
        "q<sub>y</sub>, q<sub>z</sub>). Points on line i are "
        "P<sub>i</sub>(λ) = P<sub>i0</sub> + λq. Perspective projection gives", body,
    ),
    eq("u<sub>i</sub>(λ) = (X<sub>i0</sub> + λq<sub>x</sub>) "
       "/ (Z<sub>i0</sub> + λq<sub>z</sub>),<br/>"
       "v<sub>i</sub>(λ) = (Y<sub>i0</sub> + λq<sub>y</sub>) "
       "/ (Z<sub>i0</sub> + λq<sub>z</sub>)."),
    Spacer(1, 9),
    Paragraph(
        "If q<sub>z</sub> ≠ 0, taking λ to infinity gives the same "
        "finite point for both lines, regardless of their starting points:", body,
    ),
    eq("(u<sub>∞</sub>, v<sub>∞</sub>) = "
       "(q<sub>x</sub>/q<sub>z</sub>, q<sub>y</sub>/q<sub>z</sub>)."),
    Spacer(1, 9),
    Paragraph(
        "This is the vanishing point. In homogeneous image coordinates "
        "it is [q<sub>x</sub> : q<sub>y</sub> : q<sub>z</sub>].", body,
    ),
    Paragraph("When do the projections remain parallel?", heading),
    Paragraph(
        "If q<sub>z</sub> = 0, the depth of each point on a given line "
        "is constant. Its image trajectory is", body,
    ),
    eq("(u<sub>i</sub>(λ), v<sub>i</sub>(λ)) = "
       "(X<sub>i0</sub>/Z<sub>i0</sub>, Y<sub>i0</sub>/Z<sub>i0</sub>) "
       "+ λ(q<sub>x</sub>/Z<sub>i0</sub>, q<sub>y</sub>/Z<sub>i0</sub>)."),
    Spacer(1, 9),
    Paragraph(
        "Both image directions are proportional to (q<sub>x</sub>, "
        "q<sub>y</sub>), so the projected lines remain parallel "
        "(or coincide in a degenerate view). Their vanishing point is "
        "at infinity, [q<sub>x</sub> : q<sub>y</sub> : 0]. Therefore "
        "the required condition is q<sub>z</sub> = 0: the 3D direction "
        "is parallel to the image plane.", body,
    ),
    Spacer(1, 18),
    Paragraph(
        "Problem source: MIT 16.485 VNAV, LAB5 Exercises (2023), "
        "vnav.mit.edu/labs_2023/lab5/exercises.html.", subtitle,
    ),
]

output = HERE / "individual_solutions.pdf"
document = SimpleDocTemplate(
    str(output), pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm,
    topMargin=17 * mm, bottomMargin=22 * mm,
)
document.build(story, onFirstPage=page_footer, onLaterPages=page_footer)
print(output)
