#!/usr/bin/env python3

import os
import sys
import time
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.pdfgen import canvas

class ManualCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.pages = []
        
    def showPage(self):
        self.pages.append(dict(self.__dict__))
        self._startPage()
        
    def save(self):
        num_pages = len(self.pages)
        for page in self.pages:
            self.__dict__.update(page)
            # Suppress header/footer on cover/first page if desired
            if self._pageNumber > 1:
                # Top Header
                self.setFont("Helvetica-Bold", 8)
                self.setFillColor(colors.HexColor("#1A365D"))
                self.drawString(54, 750, "ROS 2 GLOBAL PATH PLANNING BENCHMARK MANUAL")
                self.setStrokeColor(colors.HexColor("#BDC3C7"))
                self.setLineWidth(0.5)
                self.line(54, 742, 558, 742)
                
                # Bottom Footer
                self.line(54, 50, 558, 50)
                self.setFont("Helvetica", 8)
                self.setFillColor(colors.HexColor("#7F8C8D"))
                self.drawString(54, 38, "Technical Architecture & Operational Guide")
                self.drawRightString(558, 38, f"Page {self._pageNumber} of {num_pages}")
            super().showPage()
        super().save()

def generate_pdf(out_pdf_path):
    doc = SimpleDocTemplate(
        out_pdf_path,
        pagesize=letter,
        leftMargin=54,
        rightMargin=54,
        topMargin=72,
        bottomMargin=72
    )
    
    styles = getSampleStyleSheet()
    
    # Custom Styles
    title_style = ParagraphStyle(
        'ManualTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=24,
        leading=28,
        textColor=colors.HexColor("#1A365D"),
        spaceAfter=10
    )
    
    subtitle_style = ParagraphStyle(
        'ManualSubtitle',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=12,
        leading=16,
        textColor=colors.HexColor("#4A5568"),
        spaceAfter=25
    )
    
    h1_style = ParagraphStyle(
        'SectionHeader',
        parent=styles['Heading1'],
        fontName='Helvetica-Bold',
        fontSize=16,
        leading=20,
        textColor=colors.HexColor("#1A365D"),
        spaceBefore=15,
        spaceAfter=10,
        keepWithNext=True
    )

    h2_style = ParagraphStyle(
        'SubSectionHeader',
        parent=styles['Heading2'],
        fontName='Helvetica-Bold',
        fontSize=12,
        leading=16,
        textColor=colors.HexColor("#2C5282"),
        spaceBefore=10,
        spaceAfter=6,
        keepWithNext=True
    )
    
    body_style = ParagraphStyle(
        'BodyTextCustom',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=10,
        leading=14,
        textColor=colors.HexColor("#2D3748"),
        spaceAfter=8
    )

    code_style = ParagraphStyle(
        'CodeStyleCustom',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=8.5,
        leading=11,
        textColor=colors.HexColor("#1A202C"),
        backColor=colors.HexColor("#EDF2F7"),
        borderColor=colors.HexColor("#E2E8F0"),
        borderWidth=0.5,
        borderPadding=6,
        spaceAfter=8
    )

    table_cell_style = ParagraphStyle(
        'TableCellText',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=9,
        leading=12,
        textColor=colors.HexColor("#2D3748")
    )

    table_header_style = ParagraphStyle(
        'TableHeaderText',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=9,
        leading=12,
        textColor=colors.white
    )

    story = []
    
    # ------------------ COVER PAGE / TITLE ------------------
    story.append(Spacer(1, 40))
    story.append(Paragraph("ROS 2 Global Path Planning<br/>Benchmarking Suite Manual", title_style))
    story.append(Paragraph("A Technical Manual on Modular 3-Node Architecture, Node Lifecycles, and Operational Workflows", subtitle_style))
    
    # Decorative line
    t_line = Table([[""]], colWidths=[504])
    t_line.setStyle(TableStyle([
        ('LINEBELOW', (0,0), (-1,-1), 3.0, colors.HexColor("#1A365D")),
        ('BOTTOMPADDING', (0,0), (-1,-1), 0),
        ('TOPPADDING', (0,0), (-1,-1), 0),
    ]))
    story.append(t_line)
    story.append(Spacer(1, 20))
    
    # Metadata Block
    meta_text = (
        "<b>Prepared for:</b> ROAR Lab Benchmarking & Evaluation<br/>"
        "<b>Architecture:</b> Decoupled 3-Node Life Cycle (Topic-based Communication)<br/>"
        "<b>Target System:</b> ROS 2 Humble / Iron / Jazzy on Ubuntu 22.04 LTS<br/>"
        "<b>Date:</b> July 2026<br/>"
        "<b>Status:</b> Fully Verified & Operational<br/>"
    )
    story.append(Paragraph(meta_text, body_style))
    story.append(Spacer(1, 20))
    
    # ------------------ EXECUTIVE SUMMARY ------------------
    story.append(Paragraph("Executive Summary", h1_style))
    story.append(Paragraph(
        "This manual details the design, implementation, and operations of the refactored, decoupled ROS 2 "
        "Global Path Planning Benchmarking Suite. The system transition from a legacy monolithic architecture "
        "to a robust 3-node system (comprising an <b>Algorithmic Node</b>, a <b>Benchmarking Node</b>, and a "
        "<b>Testing Orchestrator Node</b>) guarantees clean test isolation, resource safety, and customizable "
        "performance metrics extraction. All stale files and legacy codebases have been successfully purged, leaving "
        "a clean, standards-compliant workspace.",
        body_style
    ))
    
    story.append(PageBreak())
    
    # ------------------ SECTION 1 ------------------
    story.append(Paragraph("1. Modular ROS 2 System Architecture", h1_style))
    story.append(Paragraph(
        "A common point of confusion in ROS 2 development is why multiple separate nodes are colocated in a single "
        "package and folder. This section addresses this architecture design.",
        body_style
    ))
    
    story.append(Paragraph("Why are the 3 nodes in one folder/package together?", h2_style))
    story.append(Paragraph(
        "In ROS 2, a <b>package</b> represents the unit of compilation and release. Although the path planner, "
        "evaluator, and orchestrator serve completely distinct roles, they represent a unified functional suite. "
        "Colocating the source Python files inside a single directory (<code>global_path_benchmarking/</code>) "
        "offers several critical benefits:<br/>"
        "• <b>Shared Configuration:</b> They read from the same <code>config/scenarios.yaml</code> and "
        "<code>config/benchmark_config.yaml</code> files, ensuring zero config duplication.<br/>"
        "• <b>Shared Imports:</b> They share ROS 2 message types, PIL/image reading helpers, and path utility functions.<br/>"
        "• <b>Ease of Compilation:</b> The developer only needs to run <code>colcon build --packages-select global_path_benchmarking</code> "
        "to compile the entire benchmarking suite.",
        body_style
    ))

    story.append(Paragraph("Are they separate processes?", h2_style))
    story.append(Paragraph(
        "<b>Yes.</b> Although the source files reside in the same folder, they are defined as separate entry points "
        "in <code>setup.py</code>. When ROS 2 launches them, each node is spawned in its own isolated OS subprocess "
        "with a dedicated Python interpreter. They do not share memory or state, and communicate exclusively via standard "
        "ROS 2 DDS topics (<code>/start_pose</code>, <code>/goal_pose</code>, <code>/planned_path</code>, <code>/global_costmap/costmap</code>, and <code>/test_name</code>).",
        body_style
    ))
    
    story.append(Paragraph("Decoupled Node Lifecycle Flow", h2_style))
    story.append(Paragraph(
        "The Testing Orchestrator (<code>testing_node</code>) acts as the supervisor. For each scenario in the test configuration:<br/>"
        "1. It clean-spawns <code>algo_node</code> and <code>benchmarking_node</code> as OS subprocesses.<br/>"
        "2. It publishes scenario data to trigger planning and evaluation.<br/>"
        "3. It waits for the planned path to be published or for a timeout.<br/>"
        "4. It terminates (sends SIGTERM/SIGKILL to) both subprocesses, ensuring a clean slate (no cached maps or pose states) for the next run.",
        body_style
    ))
    
    # ------------------ SECTION 2 ------------------
    story.append(Paragraph("2. File Implementations & API Specifications", h1_style))
    story.append(Paragraph(
        "Here we document the specific implementations of the three active Python files in the package.",
        body_style
    ))
    
    # algo_node.py
    story.append(Paragraph("A. algo_node.py (The Path Planner)", h2_style))
    story.append(Paragraph(
        "This node is responsible for implementing the core planning logic. It runs indefinitely and acts as a standard "
        "topic-based path planner.",
        body_style
    ))
    story.append(Paragraph(
        "<b>Subscribers:</b><br/>"
        "• <code>/global_costmap/costmap</code> (<code>nav_msgs/msg/OccupancyGrid</code>): Caches the environmental costmap grid.<br/>"
        "• <code>/start_pose</code> (<code>geometry_msgs/msg/PoseStamped</code>): Caches the robot's initial position.<br/>"
        "• <code>/goal_pose</code> (<code>geometry_msgs/msg/PoseStamped</code>): Triggers the planning computation.<br/>"
        "<b>Publishers:</b><br/>"
        "• <code>/planned_path</code> (<code>nav_msgs/msg/Path</code>): Publishes the generated sequence of poses.<br/>"
        "• <code>/planner_internal_time</code> (<code>std_msgs/msg/Float32</code>): Publishes the internal execution time of the solver (in seconds).<br/>"
        "<b>Key Logic:</b><br/>"
        "• Parameters: <code>use_astar</code> (toggles between A* search and Straight Line fallback).<br/>"
        "• A* Solver: Operates on an 8-connected grid. It penalizes cells with higher costmap values (e.g. slopes), guiding the path away from high-cost terrains.",
        body_style
    ))
    
    story.append(PageBreak())
    
    # benchmarking_node.py
    story.append(Paragraph("B. benchmarking_node.py (The Evaluator)", h2_style))
    story.append(Paragraph(
        "This node runs persistently and monitors all topics to score the planner's performance against standard criteria.",
        body_style
    ))
    story.append(Paragraph(
        "<b>Subscribers:</b><br/>"
        "• <code>/test_name</code> (<code>std_msgs/msg/String</code>): Switches the active test scenario ID (defaults to 'general').<br/>"
        "• <code>/global_costmap/costmap</code>: Reads the active map (including injected dynamic obstacles).<br/>"
        "• <code>/start_pose</code> & <code>/goal_pose</code>: Triggers the round-trip planning duration timer.<br/>"
        "• <code>/planned_path</code>: Receives the computed path and triggers the scoring module.<br/>"
        "<b>Key Logic:</b><br/>"
        "• Reads weights, limits, and score thresholds from <code>config/benchmark_config.yaml</code>.<br/>"
        "• Computes path length, average traversed cell cost, and safety margin (minimum distance from path to any obstacle).<br/>"
        "• Accumulates blocked cell footprints (collisions) based on the robot radius.<br/>"
        "• For dynamic scenarios, it captures the first path (static) and the second path (re-planned after obstacle injection) to compute detour success.<br/>"
        "• Outputs a temporary JSON file (<code>reports/temp_&lt;scenario_id&gt;.json</code>) and a comparison plot PNG.",
        body_style
    ))
    
    # testing_node.py
    story.append(Paragraph("C. testing_node.py (The Orchestrator)", h2_style))
    story.append(Paragraph(
        "The orchestrator runs the entire benchmark suite. It manages files, processes, and aggregations.",
        body_style
    ))
    story.append(Paragraph(
        "<b>Lifecycle Logic:</b><br/>"
        "• Parses command line arguments (e.g. <code>--scenario_id</code>, <code>--verify</code>, <code>--clean</code>, <code>--use_astar</code>).<br/>"
        "• If <code>--clean</code> is passed, it kills old processes before launching.<br/>"
        "• If maps or configuration YAMLs are missing, it dynamically generates synthetic png maps and default scenario files.<br/>"
        "• Spawns <code>algo_node</code> and <code>benchmarking_node</code>, publishes topics, coordinates dynamic obstacle map updates, and terminates the nodes on path completion or timeout.<br/>"
        "• Aggregates temporary JSON files into a consolidated <code>reports/results.json</code>, and compiles the final Markdown and PDF report document.",
        body_style
    ))
    
    story.append(Spacer(1, 10))
    
    # ------------------ SECTION 3 ------------------
    story.append(Paragraph("3. How to Use the Benchmarking Suite", h1_style))
    story.append(Paragraph(
        "Follow these standard terminal instructions to run the verified suite. Ensure your workspace is sourced before launching.",
        body_style
    ))
    
    story.append(Paragraph("Command Reference Table", h2_style))
    
    # Table of commands
    cmd_data = [
        [Paragraph("<b>Objective</b>", table_header_style), Paragraph("<b>Command Line</b>", table_header_style)],
        [
            Paragraph("Pre-Run Verification", table_cell_style),
            Paragraph("<code>ros2 launch global_path_benchmarking benchmark.launch.py verify:=true</code>", table_cell_style)
        ],
        [
            Paragraph("Run Full Suite (A*)", table_cell_style),
            Paragraph("<code>ros2 launch global_path_benchmarking benchmark.launch.py clean:=true</code>", table_cell_style)
        ],
        [
            Paragraph("Run Full Suite (Straight-line)", table_cell_style),
            Paragraph("<code>ros2 launch global_path_benchmarking benchmark.launch.py clean:=true use_astar:=false</code>", table_cell_style)
        ],
        [
            Paragraph("Run Specific Scenario", table_cell_style),
            Paragraph("<code>ros2 launch global_path_benchmarking benchmark.launch.py scenario_id:=crater_field clean:=true</code>", table_cell_style)
        ]
    ]
    t_cmd = Table(cmd_data, colWidths=[154, 350])
    t_cmd.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1A365D")),
        ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E0")),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F7FAFC")]),
        ('TOPPADDING', (0,0), (-1,-1), 6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6),
    ]))
    story.append(t_cmd)
    
    story.append(Spacer(1, 15))
    story.append(Paragraph("Independent Monitor Node Mode", h2_style))
    story.append(Paragraph(
        "You can run the <code>benchmarking_node</code> (Evaluator / Monitor) completely independently "
        "of the orchestrator. This allows you to share the monitor node with others so they can test and compare "
        "their custom planners.<br/>"
        "• <b>Independent Launch:</b> Run <code>ros2 run global_path_benchmarking benchmarking_node</code> in a terminal.<br/>"
        "• <b>Planner Compatibility:</b> The monitor is fully decoupled. Any planner node (custom A*, Dijkstra, Nav2, etc.) "
        "can be used, as long as it subscribes to <code>/start_pose</code>, <code>/goal_pose</code>, and "
        "<code>/global_costmap/costmap</code>, and publishes the planned trajectory path to <code>/planned_path</code>.<br/>"
        "• <b>Terminal Output:</b> As paths are generated, the monitor prints evaluation metrics (success status, "
        "planning times, path length, collisions, average cell costs, safety margins) directly to the console.<br/>"
        "• <b>Automated Reports on Shutdown:</b> When the monitor node is killed or stopped (using Ctrl+C / SIGINT / SIGTERM), "
        "it compiles all recorded runs from that session and automatically outputs two files:<br/>"
        "  1. <code>reports/benchmark_history.csv</code> (A tabular spreadsheet of all runs).<br/>"
        "  2. <code>reports/benchmark_report.pdf</code> (A professionally formatted PDF summary report).",
        body_style
    ))
    
    story.append(Spacer(1, 15))
    story.append(Paragraph("Verification of Purged Legacy Architecture", h2_style))
    story.append(Paragraph(
        "All legacy action-based evaluator and mock planning files (<code>benchmarker.py</code> and <code>mock_planner.py</code>) "
        "have been completely deleted from the workspace. Setup entry points have been updated. Running the modular "
        "system compiles cleanly with zero deprecated references.",
        body_style
    ))
    
    doc.build(story, canvasmaker=ManualCanvas)
    print(f"[INFO] Technical manual PDF successfully compiled at: {out_pdf_path}")

if __name__ == "__main__":
    out_pdf = "reports/GPP_Benchmarking_Technical_Explanation.pdf"
    generate_pdf(out_pdf)
