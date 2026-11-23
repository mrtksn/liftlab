"""Which web pages may use this computer's survey server (server.py, LiftLab's companion).

The server listens on 127.0.0.1 only. Pages it serves itself use it, as do LiftLab's site (its Room Survey dialog
loads survey/index.html from GitHub Pages and names this server with ?companion=), LiftLab from a local web server
(http://localhost or 127.0.0.1, any port), and any origin added with --origin. Other sites get no CORS headers, so a
browser keeps them from reading anything, and their POSTs are refused. A page opened as a file (Origin "null") is not
accepted: sandboxed frames on any site send that origin too.
"""

LIFTLAB_ORIGINS = ("https://mrtksn.github.io",)
LOCAL_HOSTS = ("http://localhost", "http://127.0.0.1", "http://[::1]")


def origin_ok(origin, own, extra=()):
    """Whether a request's Origin header is a page allowed to use the server ``own`` (``http://HOST:PORT``)."""
    if not origin:
        return False
    if origin == own or origin in LIFTLAB_ORIGINS or origin in extra:
        return True
    for host in LOCAL_HOSTS:
        if origin == host or (origin.startswith(host + ":") and origin[len(host) + 1:].isdigit()):
            return True
    return False


def cors_headers(origin, own, extra=()):
    """Headers that let an allowed page on another origin read a response."""
    if not origin or origin == own or not origin_ok(origin, own, extra):
        return []
    return [("Access-Control-Allow-Origin", origin), ("Vary", "Origin")]


def preflight_headers(origin, own, extra=(), private_network=False):
    """The reply to a CORS preflight (OPTIONS), or None when the page isn't allowed."""
    if not origin_ok(origin, own, extra):
        return None
    headers = [("Access-Control-Allow-Origin", origin), ("Vary", "Origin"),
               ("Access-Control-Allow-Methods", "GET, POST"), ("Access-Control-Allow-Headers", "Content-Type"),
               ("Access-Control-Max-Age", "600")]
    if private_network:
        # Chrome asks before a public site reaches this computer (Private/Local Network Access).
        headers.append(("Access-Control-Allow-Private-Network", "true"))
    return headers


def frame_ancestors(extra=()):
    """The pages that may show a companion page in a frame (the LingBot-Map viewer inside the survey page)."""
    # CSP host sources have no IPv6 literals, so [::1] is left out.
    return " ".join(("'self'",) + LIFTLAB_ORIGINS + tuple(extra) + tuple(h + ":*" for h in LOCAL_HOSTS if "[" not in h))
