from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
import json
import threading
import unittest

import companion
import server

OWN = "http://127.0.0.1:8779"
LIFTLAB = "https://mrtksn.github.io"


class OriginTests(unittest.TestCase):
    def test_allowed_pages(self):
        for origin in (OWN, LIFTLAB, "http://localhost:8000", "http://127.0.0.1:5500", "http://[::1]:8080",
                       "http://localhost"):
            self.assertTrue(companion.origin_ok(origin, OWN), origin)
        for origin in (None, "", "null", "https://evil.example", "http://localhost.evil.example",
                       "http://localhost:80x", "https://mrtksn.github.io.evil.example", "https://localhost:8000"):
            self.assertFalse(companion.origin_ok(origin, OWN), origin)
        self.assertTrue(companion.origin_ok("https://lab.example", OWN, ("https://lab.example",)))

    def test_headers(self):
        self.assertEqual(companion.cors_headers(OWN, OWN), [])
        self.assertEqual(companion.cors_headers("https://evil.example", OWN), [])
        self.assertIn(("Access-Control-Allow-Origin", LIFTLAB), companion.cors_headers(LIFTLAB, OWN))
        self.assertIsNone(companion.preflight_headers("https://evil.example", OWN))
        reply = dict(companion.preflight_headers(LIFTLAB, OWN, private_network=True))
        self.assertEqual(reply["Access-Control-Allow-Private-Network"], "true")
        self.assertNotIn("Access-Control-Allow-Private-Network", dict(companion.preflight_headers(LIFTLAB, OWN)))
        ancestors = companion.frame_ancestors(("https://lab.example",))
        self.assertIn(LIFTLAB, ancestors)
        self.assertIn("https://lab.example", ancestors)
        self.assertNotIn("[", ancestors)


class ServerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.httpd = ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        cls.httpd.origins = ()
        cls.port = cls.httpd.server_port
        threading.Thread(target=cls.httpd.serve_forever, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.httpd.shutdown()
        cls.httpd.server_close()

    def request(self, method, path, origin=None, body=None, headers=()):
        conn = HTTPConnection("127.0.0.1", self.port, timeout=5)
        sent = {"Host": f"127.0.0.1:{self.port}", **dict(headers)}
        if origin:
            sent["Origin"] = origin
        if body is not None:
            sent["Content-Type"] = "application/json"
            body = json.dumps(body)
        conn.request(method, path, body=body, headers=sent)
        response = conn.getresponse()
        data = response.read()
        conn.close()
        return response, data

    def test_liftlab_page_reads_with_cors(self):
        response, data = self.request("GET", "/api.js", LIFTLAB)
        self.assertEqual(response.status, 200)
        self.assertEqual(response.getheader("Access-Control-Allow-Origin"), LIFTLAB)
        self.assertIn(b"companionOrigin", data)
        response, _ = self.request("GET", "/api.js")
        self.assertIsNone(response.getheader("Access-Control-Allow-Origin"))

    def test_other_sites_are_refused(self):
        response, _ = self.request("GET", "/api.js", "https://evil.example")
        self.assertEqual(response.status, 403)
        response, data = self.request("POST", "/delete-preset", "https://evil.example", {"name": "x"})
        self.assertEqual(response.status, 400)
        self.assertIn("Local dashboard required", json.loads(data)["error"])
        response, _ = self.request("OPTIONS", "/operator", "https://evil.example")
        self.assertEqual(response.status, 403)

    def test_preflight(self):
        response, _ = self.request("OPTIONS", "/operator", LIFTLAB,
                                   headers={"Access-Control-Request-Method": "POST",
                                            "Access-Control-Request-Private-Network": "true"})
        self.assertEqual(response.status, 204)
        self.assertEqual(response.getheader("Access-Control-Allow-Origin"), LIFTLAB)
        self.assertEqual(response.getheader("Access-Control-Allow-Private-Network"), "true")
        self.assertIn("Content-Type", response.getheader("Access-Control-Allow-Headers"))

    def test_rebinding_host_is_refused(self):
        response, _ = self.request("OPTIONS", "/operator", LIFTLAB, headers={"Host": "evil.example"})
        self.assertEqual(response.status, 403)
        response, _ = self.request("GET", "/api.js", LIFTLAB, headers={"Host": "evil.example"})
        self.assertEqual(response.status, 403)


if __name__ == "__main__":
    unittest.main()
