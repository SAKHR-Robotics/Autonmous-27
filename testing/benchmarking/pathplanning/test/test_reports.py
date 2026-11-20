"""Unavailable evidence must survive aggregation and every exported report."""
import csv

import pytest

from global_path_benchmarking.report_generator import (
    REPORTLAB_AVAILABLE,
    aggregate_statistics,
    generate_csv_report,
    generate_html_report,
    generate_pdf_report,
)


def incomplete_record(name='missing_inputs'):
    return {'test_name': name, 'success': None, 'outcome': 'INCOMPLETE',
            'planning_time_s': None, 'safety_margin_m': None, 'path_cost': None}


def test_aggregation_excludes_unavailable_measurements():
    history = [incomplete_record(), {'test_name': 'observed', 'success': True,
               'outcome': 'PASS', 'planning_time_s': 2.0, 'safety_margin_m': 0.4}]
    stats = aggregate_statistics(history)
    assert stats['comp_time']['avg'] == pytest.approx(2.0)
    assert stats['clearance']['min'] == pytest.approx(0.4)
    assert stats['path_cost']['avg'] is None


@pytest.mark.parametrize('aggregate', [False, True])
def test_csv_exports_unavailable_without_fabrication(tmp_path, aggregate):
    history = [incomplete_record()]
    if aggregate:
        history.append(incomplete_record('another_trial'))
    output = tmp_path / 'report.csv'
    assert generate_csv_report(history, str(output))
    with output.open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == len(history)
    assert rows[0]['Planning Time (s)'] == 'N/A'
    assert rows[0]['Nearest Obstacle Distance (m)'] == 'N/A'
    assert rows[0]['Success'] == 'N/A'
    assert rows[0]['Outcome'] == 'INCOMPLETE'


@pytest.mark.parametrize('aggregate', [False, True])
def test_html_displays_incomplete_evidence(tmp_path, aggregate):
    history = [incomplete_record()]
    if aggregate:
        history.append(incomplete_record('another_trial'))
    output = tmp_path / 'report.html'
    assert generate_html_report(history, str(output))
    text = output.read_text()
    assert 'N/A' in text
    assert 'INCOMPLETE' in text
    assert 'SYSTEM STABLE' not in text


@pytest.mark.skipif(not REPORTLAB_AVAILABLE, reason='ReportLab is required for PDF export')
@pytest.mark.parametrize('aggregate', [False, True])
def test_pdf_exports_unavailable_evidence(tmp_path, aggregate):
    history = [incomplete_record()]
    if aggregate:
        history.append(incomplete_record('another_trial'))
    output = tmp_path / 'report.pdf'
    assert generate_pdf_report(history, str(output))
    assert output.read_bytes().startswith(b'%PDF-')


def test_missing_pdf_dependency_reports_export_unavailable(tmp_path, monkeypatch):
    from global_path_benchmarking import report_generator
    monkeypatch.setattr(report_generator, 'REPORTLAB_AVAILABLE', False)
    assert generate_pdf_report([incomplete_record()], str(tmp_path / 'missing.pdf')) is False
    assert not (tmp_path / 'missing.pdf').exists()
