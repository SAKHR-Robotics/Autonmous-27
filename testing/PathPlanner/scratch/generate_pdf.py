#!/usr/bin/env python3
import os
import sys
import time
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.pdfgen import canvas

class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_elements(num_pages)
            super().showPage()
        super().save()

    def draw_page_elements(self, page_count):
        # Draw header (except on cover page)
        if self._pageNumber > 1:
            self.saveState()
            self.setFont("Helvetica-Bold", 8)
            self.setFillColor(colors.HexColor("#2C3E50"))
            self.drawString(54, 750, "ROS 2 PATH PLANNING BENCHMARKING MANUAL")
            self.setStrokeColor(colors.HexColor("#BDC3C7"))
            self.setLineWidth(0.5)
            self.line(54, 742, 558, 742)
            
            # Footer
            self.line(54, 50, 558, 50)
            self.setFont("Helvetica", 8)
            self.setFillColor(colors.HexColor("#7F8C8D"))
            self.drawString(54, 38, "Confidential - For Academic & Professional Revision")
            page_text = f"Page {self._pageNumber} of {page_count}"
            self.drawRightString(558, 38, page_text)
            self.restoreState()

def create_pdf(filename):
    doc = SimpleDocTemplate(
        filename,
        pagesize=letter,
        leftMargin=54,
        rightMargin=54,
        topMargin=72,
        bottomMargin=72
    )
    
    styles = getSampleStyleSheet()
    
    # Custom styles
    title_style = ParagraphStyle(
        'CoverTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=28,
        leading=34,
        textColor=colors.HexColor("#1A365D"),
        alignment=0,
        spaceAfter=15
    )
    
    subtitle_style = ParagraphStyle(
        'CoverSubtitle',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=13,
        leading=18,
        textColor=colors.HexColor("#4A5568"),
        spaceAfter=30
    )
    
    h1_style = ParagraphStyle(
        'Header1',
        parent=styles['Heading1'],
        fontName='Helvetica-Bold',
        fontSize=16,
        leading=20,
        textColor=colors.HexColor("#1A365D"),
        spaceBefore=16,
        spaceAfter=8,
        keepWithNext=True
    )

    h2_style = ParagraphStyle(
        'Header2',
        parent=styles['Heading2'],
        fontName='Helvetica-Bold',
        fontSize=11,
        leading=14,
        textColor=colors.HexColor("#2B6CB0"),
        spaceBefore=10,
        spaceAfter=4,
        keepWithNext=True
    )
    
    body_style = ParagraphStyle(
        'BodyTextCustom',
        parent=styles['BodyText'],
        fontName='Helvetica',
        fontSize=9.5,
        leading=13.5,
        textColor=colors.HexColor("#2D3748"),
        spaceAfter=6
    )

    code_style = ParagraphStyle(
        'CodeStyle',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=8.0,
        leading=11,
        textColor=colors.HexColor("#2D3748"),
        backColor=colors.HexColor("#F7FAFC"),
        borderColor=colors.HexColor("#CBD5E0"),
        borderWidth=0.5,
        borderPadding=6,
        spaceBefore=4,
        spaceAfter=6
    )
    
    table_cell_style = ParagraphStyle(
        'TableCell',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=8.5,
        leading=11,
        textColor=colors.HexColor("#2D3748")
    )

    table_header_style = ParagraphStyle(
        'TableHeader',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=8.5,
        leading=11,
        textColor=colors.white
    )

    story = []
    
    # ------------------ COVER PAGE ------------------
    story.append(Spacer(1, 40))
    story.append(Paragraph("ROS 2 Global Path Planning<br/>Benchmarking Suite", title_style))
    story.append(Paragraph("A Technical Deep-Dive & Architecture Reference Manual", subtitle_style))
    story.append(Spacer(1, 10))
    
    # Context Box Table
    metadata_data = [
        [Paragraph("<b>Author:</b> Antigravity AI Pair Partner", table_cell_style), 
         Paragraph("<b>Target Framework:</b> ROS 2 Humble", table_cell_style)],
        [Paragraph("<b>Status:</b> Production Completed & Verified", table_cell_style), 
         Paragraph("<b>Date:</b> July 2026", table_cell_style)]
    ]
    t_meta = Table(metadata_data, colWidths=[250, 254])
    t_meta.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#EDF2F7")),
        ('BOX', (0,0), (-1,-1), 1, colors.HexColor("#CBD5E0")),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 10),
        ('BOTTOMPADDING', (0,0), (-1,-1), 10),
        ('LEFTPADDING', (0,0), (-1,-1), 15),
        ('RIGHTPADDING', (0,0), (-1,-1), 15),
    ]))
    story.append(t_meta)
    
    story.append(Spacer(1, 40))
    
    intro_p = (
        "This manual compiles the complete system architecture, codebase design patterns, "
        "and logical math definitions for the ROS 2 Global Path Planning Benchmarking Suite. "
        "Use this reference to understand package dependencies, launch orchestrations, costmap conversions, "
        "A* planning execution, reference path criteria, dynamic obstacle testing, and report generation."
    )
    story.append(Paragraph(intro_p, body_style))
    story.append(PageBreak())
    
    # ------------------ PART 1 ------------------
    story.append(Paragraph("Part 1: Build & Package Configuration", h1_style))
    story.append(Paragraph(
        "ROS 2 relies on structured configurations to manage dependencies and system integration. "
        "This harness uses three files at its package root to declare library packages, executable entrypoints, and library flags.",
        body_style
    ))
    
    story.append(Paragraph("1. package.xml", h2_style))
    story.append(Paragraph(
        "This configuration file lists the dependencies required to build and execute the benchmarking nodes. "
        "Key nodes run on rclpy (ROS 2 Python Client Library) and communicate using standard messages and actions. "
        "The dependencies include: "
        "<b>rclpy</b>, <b>nav_msgs</b> (for costmap OccupancyGrids and planning Paths), and <b>nav2_msgs</b> "
        "(for the compute_path_to_pose Nav2 Action).",
        body_style
    ))
    
    story.append(Paragraph("2. setup.py", h2_style))
    story.append(Paragraph(
        "Written using setuptools, this script handles node installation and maps internal Python modules to ROS 2 executable targets. "
        "It performs two critical roles:<br/>"
        "• <b>Data Files Mapping:</b> Copies the config templates and ROS 2 launch scripts into the installation share folder so that ROS 2 can find them globally.<br/>"
        "• <b>Entry Points Registration:</b> Binds command-line targets to functions. It registers <code>benchmarker</code> to <code>benchmarker.py:main</code>, and <code>mock_planner</code> to <code>mock_planner.py:main</code>.",
        body_style
    ))
    
    story.append(Spacer(1, 10))
    
    # ------------------ PART 2 ------------------
    story.append(Paragraph("Part 2: Launch Orchestration Pipeline", h1_style))
    story.append(Paragraph(
        "The launch script (<code>benchmark.launch.py</code>) orchestrates the setup and execution lifecycles. "
        "It acts as a synchronous supervisor. The launch sequence proceeds as follows:",
        body_style
    ))
    
    seq_steps = (
        "<b>1. Declare Launch Parameters:</b> Exposes user-configurable CLI flags including <code>verify</code>, <code>use_astar</code>, <code>scenario_id</code>, and <code>clean</code>.<br/>"
        "<b>2. Stale Node Termination:</b> If <code>clean:=true</code>, it issues a synchronous system pkill command. It scans for and forcefully terminates lingering simulator processes (Gazebo, gzserver) or Nav2 planning containers from previous crashes. This ensures a clean workspace and prevents port collisions.<br/>"
        "<b>3. Config Path Resolution:</b> Automatically resolves relative paths to absolute locations so configuration scenarios can be successfully loaded.<br/>"
        "<b>4. Execution Routing:</b> If <code>verify:=true</code>, it skips starting the planner node entirely and launches only the benchmarker in validation mode. Otherwise, it spawns both the planning server and the benchmarker nodes simultaneously."
    )
    story.append(Paragraph(seq_steps, body_style))
    
    # ------------------ PART 3 ------------------
    story.append(Paragraph("Part 3: Map Translation & Scenario Configuration", h1_style))
    story.append(Paragraph(
        "The benchmarking system uses a configuration-driven approach where maps and start/goal positions are declared in a YAML schema and converted dynamically.",
        body_style
    ))
    
    story.append(Paragraph("1. Configuration Schema (scenarios.yaml)", h2_style))
    story.append(Paragraph(
        "Each test case specifies a <code>map_image</code>, the physical coordinate <code>resolution</code> (meters per pixel), "
        "the <code>origin</code> coordinate offset, and physical parameters like <code>robot_radius</code> (e.g., 0.3m for Jackal/rover physical sizing). "
        "It also details the start and goal positions, the perfect <code>reference_path</code> coordinates, and optional dynamic obstacle insertion triggers.",
        body_style
    ))
    
    story.append(Paragraph("2. Converting 2D Grayscale PNGs to ROS OccupancyGrids", h2_style))
    story.append(Paragraph(
        "To translate static image layouts into costmaps for planning nodes, the benchmarker processes the pixels:<br/>"
        "• Black pixels (value 0) are converted to lethal cost <b>100</b>.<br/>"
        "• White pixels (value 255) are converted to free space <b>0</b>.<br/>"
        "• Gray values (1-254) are scaled linearly to intermediate costs (1-99).<br/>"
        "• The matrix is flipped vertically using <code>np.flipud</code> because image indices start top-left, whereas ROS frames start bottom-left.",
        body_style
    ))
    
    story.append(Paragraph("3. Reference Path (Ground Truth) Definition", h2_style))
    story.append(Paragraph(
        "To evaluate if the planner's length is optimal, we compare it against a ground-truth <code>reference_path</code>. "
        "This path is defined via three primary methodologies:<br/>"
        "• <b>Offline Global Optimization:</b> Using computationally heavy planners (like Dijkstra or CHOMP) offline with infinite time budgets to calculate the absolute shortest clearance line.<br/>"
        "• <b>Expert-Mapped Corridors:</b> Manual coordinate selection based on expert navigation rules for Marsyard slope avoidance.<br/>"
        "• <b>Euclidean Baselines:</b> Straight-line vectors between start and goal used in obstacle-free tests (e.g., <code>empty_straight</code>).",
        body_style
    ))
    story.append(PageBreak())
    
    # ------------------ PART 4 ------------------
    story.append(Paragraph("Part 4: Mock Planner Algorithms", h1_style))
    story.append(Paragraph(
        "The package provides a mock planner (<code>mock_planner.py</code>) implementing the standard Nav2 Action interface. "
        "It provides a straight-line driver and an A* planner to verify benchmark scoring.",
        body_style
    ))
    
    story.append(Paragraph("1. Straight-Line Mode (Lower-Quality Benchmark Target)", h2_style))
    story.append(Paragraph(
        "Directly calculates the vector between start and goal, adding waypoints every 10cm. It completely ignores obstacle data, "
        "simulating a failing or colliding planner.",
        body_style
    ))
    
    story.append(Paragraph("2. A* Pathfinding Mode (High-Quality Benchmark Target)", h2_style))
    story.append(Paragraph(
        "The A* algorithm converts world coordinates to grid coordinates. It runs an 8-connected search (4 orthogonal moves at cost 1.0, "
        "and 4 diagonal moves at cost 1.414). Cells with a value >= 100 are treated as obstacles and skipped. "
        "For clear cells, the algorithm penalizes non-lethal costs using:<br/>"
        "<code>Move Cost = cost_multiplier * (1.0 + cell_cost / 10.0)</code><br/>"
        "This forces A* to detour around rough gray regions rather than just running alongside obstacles. "
        "If A* fails (e.g. goal trapped), it falls back to a straight line path to return a result.",
        body_style
    ))
    
    # ------------------ PART 5 ------------------
    story.append(Paragraph("Part 5: The Benchmarker Engine & Scorer", h1_style))
    story.append(Paragraph(
        "The engine manages the test sequence and evaluates the resulting paths against standard scoring equations. "
        "For each run, it queries a static path, computes safety metrics, injects dynamic obstacles into the occupancy grid, "
        "requests a replanned detour, and saves visual, JSON, and PDF reports.",
        body_style
    ))
    
    story.append(Paragraph("1. The Importance of Dynamic Obstacles", h2_style))
    story.append(Paragraph(
        "Planetary navigation is highly unpredictable. Shifting slopes, debris, or other rovers can block path options. "
        "By injecting dynamic obstacles, the benchmarker measures the planner's <b>reactivity</b>, <b>replanning latency</b>, "
        "and <b>collision avoidance</b>. Planners that are slow to replan or fail to compute detours are penalized to 0.0 on the replanning score.",
        body_style
    ))
    
    story.append(Paragraph("2. Automated Directory Cleanliness & Purging", h2_style))
    story.append(Paragraph(
        "To prevent stale or duplicate records from previous runs, the benchmarker runs automated purging:<br/>"
        "• **Reports Cleanup:** Deletes previous files (`results.json`, `numerical_report.md`, `numerical_report.pdf`, and `result_scenario_*.png`) at the start of any test execution.<br/>"
        "• **Synthetic Map Re-Generation:** Systematically deletes and recreates synthetic PNGs to reflect the current code configuration.",
        body_style
    ))

    story.append(Paragraph("3. Single-Scenario Filtering & Report Scaling", h2_style))
    story.append(Paragraph(
        "When filtering execution to a single map using the <code>scenario_id</code> flag, the report generator "
        "dynamically hides skipped scenarios. The PDF summary table and detailed plots automatically resize to display "
        "only the metrics for the active run.",
        body_style
    ))
    
    story.append(Paragraph("4. The Scoring Formulas & Passing Threshold", h2_style))
    story.append(Paragraph(
        "The total score is a weighted combination of six sub-scores out of 100. "
        "The **Pass Condition** is defined as a final **Path Planning Score >= 85.0/100**:",
        body_style
    ))
    
    # Math Box Table
    math_data = [
        [Paragraph("<b>Score Component</b>", table_header_style), 
         Paragraph("<b>Weight</b>", table_header_style), 
         Paragraph("<b>Mathematical Logic / Formula</b>", table_header_style)],
        
        [Paragraph("Planning Success (S_success)", table_cell_style), 
         Paragraph("25%", table_cell_style), 
         Paragraph("100 if path returned successfully; 0 otherwise.", table_cell_style)],
        
        [Paragraph("Planning Time (S_time)", table_cell_style), 
         Paragraph("15%", table_cell_style), 
         Paragraph("If T <= 2.0s: 100. If T >= 10.0s: 0. Else: 100 * (1 - (T - 2.0)/8.0)", table_cell_style)],
        
        [Paragraph("Obstacle Avoidance (S_obstacle)", table_cell_style), 
         Paragraph("25%", table_cell_style), 
         Paragraph("100 if robot footprint radius never intersects cell >= 100; else 0.", table_cell_style)],
         
        [Paragraph("Path Cost (S_cost)", table_cell_style), 
         Paragraph("15%", table_cell_style), 
         Paragraph("100 - C_avg, where C_avg is the average cost of cells traversed.", table_cell_style)],
         
        [Paragraph("Path Length (S_length)", table_cell_style), 
         Paragraph("10%", table_cell_style), 
         Paragraph("Ratio R = Planned/Reference. If R <= 1.0: 100. If R >= 1.35: 0. Else: 100 * (1 - (R-1.0)/0.35)", table_cell_style)],
         
        [Paragraph("Replanning (S_replan)", table_cell_style), 
         Paragraph("10%", table_cell_style), 
         Paragraph("100 if detour path computed in <= 2.0s with 0 collisions; else 0.", table_cell_style)]
    ]
    
    t_math = Table(math_data, colWidths=[140, 50, 314])
    t_math.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1A365D")),
        ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E0")),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F7FAFC")]),
        ('TOPPADDING', (0,0), (-1,-1), 8),
        ('BOTTOMPADDING', (0,0), (-1,-1), 8),
        ('LEFTPADDING', (0,0), (-1,-1), 10),
        ('RIGHTPADDING', (0,0), (-1,-1), 10),
    ]))
    story.append(t_math)
    story.append(PageBreak())

    # ------------------ PART 6 (USAGE GUIDE) ------------------
    story.append(Paragraph("Part 6: Operations & Usage Guide", h1_style))
    story.append(Paragraph(
        "This section details the commands needed to compile the workspace, launch the nodes, "
        "verify ground-truth paths, and evaluate custom third-party path planning servers.",
        body_style
    ))

    story.append(Paragraph("1. Configuration Modification (Maps, Starts, Goals)", h2_style))
    story.append(Paragraph(
        "To modify map inputs, change coordinates, or edit reference paths, edit the config file: "
        "<code>config/scenarios.yaml</code>. Below are the key parameter mappings:",
        body_style
    ))
    
    # Config parameters description table
    config_params_data = [
        [Paragraph("<b>YAML Key</b>", table_header_style), 
         Paragraph("<b>Expected Format & Details</b>", table_header_style), 
         Paragraph("<b>Purpose</b>", table_header_style)],
        
        [Paragraph("<code>map_image</code>", table_cell_style), 
         Paragraph("<code>\"maps/filename.png\"</code>", table_cell_style), 
         Paragraph("Selects the structural environment layout.", table_cell_style)],
         
        [Paragraph("<code>start</code>", table_cell_style), 
         Paragraph("<code>[X_coord, Y_coord]</code> (float)", table_cell_style), 
         Paragraph("Sets the starting location in meters.", table_cell_style)],
         
        [Paragraph("<code>goal</code>", table_cell_style), 
         Paragraph("<code>[X_coord, Y_coord]</code> (float)", table_cell_style), 
         Paragraph("Sets the target location in meters.", table_cell_style)],
         
        [Paragraph("<code>robot_radius</code>", table_cell_style), 
         Paragraph("<code>float</code> (e.g., <code>0.35</code>)", table_cell_style), 
         Paragraph("Specifies the rover footprint boundary.", table_cell_style)],
         
        [Paragraph("<code>reference_path</code>", table_cell_style), 
         Paragraph("List of coordinates <code>[[x1,y1], [x2,y2]]</code>", table_cell_style), 
         Paragraph("Defines the ground-truth optimal line.", table_cell_style)]
    ]
    t_cfg = Table(config_params_data, colWidths=[110, 150, 244])
    t_cfg.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#2B6CB0")),
        ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E0")),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F7FAFC")]),
        ('TOPPADDING', (0,0), (-1,-1), 6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6),
        ('LEFTPADDING', (0,0), (-1,-1), 8),
        ('RIGHTPADDING', (0,0), (-1,-1), 8),
    ]))
    story.append(t_cfg)
    story.append(Spacer(1, 10))

    story.append(Paragraph("2. Available Test Scenarios (Simple & Hard)", h2_style))
    story.append(Paragraph(
        "Nine scenarios are pre-configured to evaluate planner robustness:<br/>"
        "• <code>empty_straight</code>: Straight-line path in open field (tests basic capability).<br/>"
        "• <code>scattered_rocks_detour</code>: Obstacles in path, dynamic obstacle triggers detour (tests planning speed/replanning).<br/>"
        "• <code>canyon_gate_passage</code>: Navigates a narrow gate corridor (tests narrow path alignment).<br/>"
        "• <code>marsyard_rough_slopes</code>: **[HARD]** High-cost slope region in middle. Planner must detour around slopes to maintain low path cost.<br/>"
        "• <code>marsyard_labyrinth</code>: **[HARD]** Winding maze walls. Tests footprint safety; planners without obstacle inflation will crash in tight corridors.<br/>"
        "• <code>canyon_gate_blocked</code>: **[HARD]** Dynamic obstacle completely blocks the only gap. Tests planning failure and graceful abort handling.<br/>"
        "• <code>marsyard_snake_passage</code>: **[HARD]** Winding narrow corridor S-curve (tests tight path smoothing, fine grid resolution, and footprint collision checking).<br/>"
        "• <code>dead_end_trap</code>: **[HARD]** Dead-end U-shaped trap blocking the direct route. Tests heuristic trap escape and global detouring.<br/>"
        "• <code>crater_field</code>: **[HARD]** Scattered slope crater field (variable cost gray levels). Tests cost-awareness and cost-weighted search optimization.",
        body_style
    ))

    story.append(Paragraph("3. Build and Sourcing Setup", h2_style))
    story.append(Paragraph(
        "colcon build --packages-select global_path_benchmarking<br/>"
        "source install/setup.bash",
        code_style
    ))
    story.append(Paragraph(
        "<b>Output:</b> Compiled C++ and Python binaries, environment setup scripts.<br/>"
        "<b>Saved Location:</b> Binaries stored in <code>build/</code>, symlinks and setup paths created in <code>install/</code>.",
        body_style
    ))

    story.append(Paragraph("4. Reference Path Footprint Verification", h2_style))
    story.append(Paragraph(
        "ros2 launch global_path_benchmarking benchmark.launch.py verify:=true",
        code_style
    ))
    story.append(Paragraph(
        "<b>Output:</b> Grayscale maps with plotted reference lines and transparent circular footprints.<br/>"
        "<b>Saved Location:</b> Saved as <code>reports/verify_scenario_&lt;scenario_id&gt;.png</code>.",
        body_style
    ))
    story.append(PageBreak())

    story.append(Paragraph("5. Running the Benchmark Suite (Standard A*)", h2_style))
    story.append(Paragraph(
        "ros2 launch global_path_benchmarking benchmark.launch.py",
        code_style
    ))
    story.append(Paragraph(
        "<b>Output:</b> Complete evaluation log, scoring breakdown, comparative plots, and Markdown summary.<br/>"
        "<b>Saved Locations:</b><br/>"
        "• Raw JSON metrics: <code>reports/results.json</code><br/>"
        "• Printable Report: <code>reports/numerical_report.md</code> (contains PASS/FAIL status table)<br/>"
        "• PDF Evaluation Report: <code>reports/numerical_report.pdf</code> (publication-ready compiled results)<br/>"
        "• Result plots: <code>reports/result_scenario_&lt;scenario_id&gt;.png</code> (colored box showing PASS in green/FAIL in red).",
        body_style
    ))

    story.append(Paragraph("6. Running with the Straight-Line Mock Planner", h2_style))
    story.append(Paragraph(
        "ros2 launch global_path_benchmarking benchmark.launch.py use_astar:=false",
        code_style
    ))
    story.append(Paragraph(
        "<b>Output:</b> Evaluation metrics showing high collision rates, high cost, and low scores due to obstacle intersections.<br/>"
        "<b>Saved Locations:</b> Outputs saved to <code>reports/results.json</code>, <code>reports/numerical_report.md</code>, <code>reports/numerical_report.pdf</code>, and corresponding comparison PNGs.",
        body_style
    ))

    story.append(Paragraph("7. Targeted Scenario Runs & System Cleanup", h2_style))
    story.append(Paragraph(
        "ros2 launch global_path_benchmarking benchmark.launch.py scenario_id:=canyon_gate_blocked clean:=true",
        code_style
    ))
    story.append(Paragraph(
        "<b>Output:</b> Spawns only the chosen scenario. Clears memory and kills orphan simulator/ROS nodes. Automatically purges all stale report artifacts from unrelated scenarios.<br/>"
        "<b>Saved Location:</b> Outputs logged under <code>reports/</code> (only for the selected ID).",
        body_style
    ))

    story.append(Paragraph("8. Benchmarking a Custom Third-Party Path Planner", h2_style))
    story.append(Paragraph(
        "ros2 run global_path_benchmarking benchmarker --config config/scenarios.yaml",
        code_style
    ))
    story.append(Paragraph(
        "<b>Output:</b> Communicates with your path planning action server, scoring it against loaded configurations.<br/>"
        "<b>Saved Location:</b> Results logged to <code>reports/results.json</code>, compiled into <code>reports/numerical_report.md</code>, and built as <code>reports/numerical_report.pdf</code>.",
        body_style
    ))

    # Build Document
    doc.build(story, canvasmaker=NumberedCanvas)
    print("PDF Manual compiled successfully with detailed outputs.")

if __name__ == "__main__":
    output_pdf = "/home/saif/Desktop/testing/ROS2_Path_Planning_Benchmarking_Manual.pdf"
    create_pdf(output_pdf)
