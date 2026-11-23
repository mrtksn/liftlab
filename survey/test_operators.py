"""The operator catalogue and the preset files must describe a graph the dashboard can run."""
import json
from pathlib import Path
import tempfile
import unittest

from node_params import validate
import operators

HERE = Path(__file__).resolve().parent


class OperatorTests(unittest.TestCase):
    def test_catalogue_types_and_controls(self):
        catalog = json.loads(json.dumps(operators.catalog()))
        types = catalog["types"]
        ids = [o["id"] for o in catalog["operators"]]
        self.assertEqual(len(ids), len(set(ids)))
        for op in catalog["operators"]:
            for port in op["inputs"] + op["outputs"]:
                for t in port["types"]:
                    self.assertIn(t, types, f"{op['id']}.{port['name']}")
            keys = [p["key"] for p in op["params"]]
            self.assertEqual(len(keys), len(set(keys)), op["id"])
            # Defaults must pass the operator's own validation.
            self.assertEqual(validate(op["params"], {}), {p["key"]: p["default"] for p in op["params"]})
            for p in op["params"]:
                if p["kind"] == "choice":
                    self.assertIn(p["default"], [o[0] for o in p["options"]], f"{op['id']}.{p['key']}")
                for condition in ("when", "label_by"):
                    if condition in p:
                        self.assertIn(p[condition]["param"], keys)

    def test_variants_keep_default_folders(self):
        defaults = validate(operators.GAUSSIAN_PARAMS, {})
        self.assertEqual(operators.variant(defaults, operators.GAUSSIAN_PARAMS), "")
        changed = operators.variant(dict(defaults, steps=3000), operators.GAUSSIAN_PARAMS)
        self.assertRegex(changed, r"^[0-9a-f]{8}$")

    def test_clearance_settings_get_their_own_folder(self):
        import tempfile
        import clearance
        with tempfile.TemporaryDirectory() as directory:
            run = root = Path(directory)
            default = clearance.artifact_dir(run, root)
            self.assertEqual(clearance.artifact_dir(run, root, {"views": 120}), default)
            self.assertNotEqual(clearance.artifact_dir(run, root, {"views": 60}), default)

    def test_mesh_scale_from_range_readings(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory)/"report.json"
            report.write_text(json.dumps({"tof": {"candidate_metres_per_unit": 0.338, "median_error_m": 0.157,
                                                  "p80_error_m": 0.435, "hits": 36, "eligible": 46}}))
            scale = operators.range_scale(report, "relative")
            self.assertEqual((scale["metres_per_unit"], scale["median_error_m"], scale["hits"]), (0.338, 0.157, 36))
            self.assertIsNone(operators.range_scale(report, "m"))      # a measured map is in metres already
            for tof in ({"reason": "Insufficient range variation"}, {"candidate_metres_per_unit": -1},
                        {"candidate_metres_per_unit": "1"}):
                report.write_text(json.dumps({"tof": tof}))
                self.assertIsNone(operators.range_scale(report, "relative"))
            report.write_text("not json")
            self.assertIsNone(operators.range_scale(report, "relative"))
            self.assertIsNone(operators.range_scale(Path(directory)/"missing.json", "relative"))

    def test_every_type_declares_a_display(self):
        kinds = {"tables", "table", "frames", "pixels", "map", "path", "points", "polygons", "triangles", "splats", "report"}
        for name, spec in operators.TYPES.items():
            self.assertIn(spec["display"], kinds, name)

    def test_presets_reference_real_operators_ports_and_controls(self):
        catalog = operators.catalog()
        ops = {o["id"]: o for o in catalog["operators"]}
        compatible = catalog["compatible"]
        accepts = lambda types, t: t in types or any(t in compatible.get(x, []) for x in types)
        files = sorted((HERE/"presets").glob("*.json"))
        self.assertTrue(files)
        for path in files:
            preset = json.loads(path.read_text())
            nodes = {n["id"]: n for n in preset["graph"]["nodes"]}
            shared = {c["key"]: c for c in preset.get("controls", [])}
            def outputs(node):
                return [dict(name="value", types=[node["output"]])] if node["op"] == "code" else \
                       [] if node["op"] == "viewer" else ops[node["op"]]["outputs"]
            for node in nodes.values():
                where = f"{path.name}:{node['id']}"
                self.assertIn(node["op"], ops, where)
                links = node.get("inputs") or ({} if node["op"] != "viewer" else [])
                ports = {p["name"]: [p["type"]] for p in node.get("ports", [])} if node["op"] == "code" else \
                        {p["name"]: p["types"] for p in ops[node["op"]]["inputs"]}
                for name, link in (enumerate(links) if isinstance(links, list) else links.items()):
                    self.assertIn(link["node"], nodes, where)
                    out = next((p for p in outputs(nodes[link["node"]]) if p["name"] == link["port"]), None)
                    self.assertIsNotNone(out, f"{where} → {link}")
                    if name in ports:
                        types = ports[name] if isinstance(ports[name], list) else [ports[name]]
                        self.assertTrue(accepts(types, out["types"][0]), where)
                if node["op"] not in ("code", "viewer"):
                    for p in ops[node["op"]]["inputs"]:
                        self.assertTrue(p.get("optional") or p["name"] in links, f"{where} needs {p['name']}")
                    keys = {p["key"] for p in ops[node["op"]]["params"]}
                    for key, value in (node.get("settings") or {}).items():
                        self.assertIn(key, keys, where)
                        if isinstance(value, dict):
                            self.assertIn(value["var"], shared, where)
            for control in shared.values():
                source = control.get("options_from")
                if source:
                    self.assertIn(source["param"], {p["key"] for p in ops[source["op"]]["params"]}, path.name)
                if control["kind"] == "button":
                    self.assertIn(control["run"], nodes, path.name)
            self.assertIn(preset["graph"]["viewer"], nodes, path.name)
        self.assertEqual(sum(bool(json.loads(p.read_text()).get("default")) for p in files), 1)


if __name__ == "__main__":
    unittest.main()
