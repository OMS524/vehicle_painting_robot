"""Offline regression checks for unchanged CSV import and completion."""
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest

from trajectory_import import import_trajectory, validate_csv
import workstation_backend as backend


def csv_source():
    stream = io.StringIO()
    writer = csv.writer(stream)
    writer.writerow(["row_index", "point_index", "s", "position_x", "position_y", "position_z",
                     "orientation_x", "orientation_y", "orientation_z", "orientation_w", "paint", "frame_id"])
    for row in [2, 7]:
        for point in range(4):
            writer.writerow([row, point, point * .04, point * .04, row * .1, .3, 0, 0, 0, 1, int(point in [1, 2]), "world"])
    return stream.getvalue().encode()


class ImportTests(unittest.TestCase):
    def test_valid(self):
        self.assertEqual(validate_csv(csv_source()), {"rowCount": 2, "pointCount": 8, "controlPointCount": 0})
        self.assertEqual(validate_csv(b"\xef\xbb\xbf" + csv_source())["pointCount"], 8)

    def test_invalid_or_nonfinite_rejected(self):
        for source in [b"x,y,z\n0,1,2\n", csv_source().replace(b"0.04", b"nan", 1),
                       csv_source().replace(b"0.3,0,0,0,1", b"0.3,0,0,0,0", 1),
                       csv_source().splitlines()[0] + b"\n"]:
            with self.subTest(source=source[:40]), self.assertRaises(ValueError):
                validate_csv(source)

    def test_duplicate_and_out_of_order_rejected(self):
        lines = csv_source().splitlines(keepends=True)
        for source in [b"".join(lines + [lines[1]]), b"".join(lines[:2] + [lines[1]] + lines[2:]),
                       b"".join(lines[:1] + lines[5:] + lines[1:5])]:
            with self.assertRaises(ValueError):
                validate_csv(source)

    def test_import_complete_without_scan_or_regeneration_is_byte_exact(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            session = root / "session"; session.mkdir()
            planner = root / "planner"; planner.mkdir()
            source = root / "my_path.csv"; source.write_bytes(csv_source())
            result = import_trajectory({"sessionDir": str(session), "csvPath": str(source), "fileName": source.name})
            self.assertEqual(result["pointCount"], 8)
            self.assertFalse((session / backend.STATE_FILE_NAME).exists())
            self.assertEqual(json.loads((session / backend.CONTROL_POINTS_FILE_NAME).read_text())["points"], [])
            saved = backend.complete({"plannerRoot": str(planner), "sessionDir": str(session), "scanPath": None})
            self.assertEqual(saved["scanFile"], "")
            self.assertEqual((Path(saved["outputDirectory"]) / "painting_trajectory.csv").read_bytes(), source.read_bytes())
            self.assertTrue((Path(saved["outputDirectory"]) / "imported_trajectory.json").is_file())

    def test_invalid_import_does_not_write_generated_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp)
            source = session / "invalid.csv"; source.write_text("xyz\n1\n")
            with self.assertRaises(ValueError):
                import_trajectory({"sessionDir": tmp, "csvPath": str(source), "fileName": source.name})
            self.assertFalse((session / "generated").exists())


if __name__ == "__main__":
    unittest.main()
