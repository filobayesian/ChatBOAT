"""WebSocket server bridge between llm2control pipeline and browser dashboard.

Runs a websockets server in a background daemon thread. The main thread stays
fully synchronous — communication is via queue.Queue (commands in) and
loop.call_soon_threadsafe (state out).
"""

import asyncio
import json
import logging
import queue
import threading
from pathlib import Path

import websockets
from websockets.asyncio.server import ServerConnection
from websockets.datastructures import Headers
from websockets.http11 import Response

logger = logging.getLogger(__name__)

# Default dashboard directory (sibling to llm2control package)
_DEFAULT_DASHBOARD_DIR = Path(__file__).resolve().parent.parent / "dashboard"


class DashboardBridge:
    """Thread-safe WebSocket bridge for the dashboard UI.

    Public API (called from main thread):
        start()                          – spawn background server thread
        stop()                           – graceful shutdown
        wait_for_query(timeout=None)     – block until browser sends a query
        broadcast(msg_type, data)        – push JSON to all connected browsers
    """

    def __init__(self, port: int = 8765, dashboard_dir: str | Path | None = None):
        self.port = port
        self.dashboard_dir = Path(dashboard_dir) if dashboard_dir else _DEFAULT_DASHBOARD_DIR
        self._clients: set[ServerConnection] = set()
        self._query_queue: queue.Queue[str] = queue.Queue()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._server = None

    # ── Public API ────────────────────────────────────────────────────────

    def start(self) -> None:
        """Spawn the background asyncio thread with the WebSocket server."""
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        # Wait until the event loop is ready
        while self._loop is None:
            pass
        logger.info("Dashboard bridge started on ws://localhost:%d", self.port)

    def stop(self) -> None:
        """Graceful shutdown."""
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread:
            self._thread.join(timeout=3)

    def wait_for_query(self, timeout: float | None = None) -> str:
        """Block until the browser sends a query string."""
        return self._query_queue.get(timeout=timeout)

    def broadcast(self, msg_type: str, data: dict) -> None:
        """Push a typed JSON message to all connected browsers.

        Serialization happens in the calling thread; only the send is
        scheduled on the asyncio loop.
        """
        if not self._clients or not self._loop:
            return
        msg = json.dumps({"type": msg_type, **data}, default=_json_default)
        self._loop.call_soon_threadsafe(
            asyncio.ensure_future,
            self._broadcast_coro(msg),
        )

    # ── Internals ─────────────────────────────────────────────────────────

    def _run_loop(self) -> None:
        """Entry point for the background thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
        except RuntimeError:
            pass  # Expected when stop() kills the loop

    async def _serve(self) -> None:
        """Start the WebSocket server and run forever."""
        self._server = await websockets.serve(
            self._handler,
            "0.0.0.0",
            self.port,
            process_request=self._process_http,
        )
        print(f"[Dashboard] WebSocket server on ws://localhost:{self.port}")
        print(f"[Dashboard] Open http://localhost:{self.port} in your browser")
        await asyncio.Future()  # run forever

    async def _process_http(self, connection, request):
        """Serve dashboard HTML on plain HTTP GET /.

        WebSocket upgrade requests (with Upgrade header) are passed through.
        """
        # Don't intercept WebSocket upgrade requests
        if request.headers.get("Upgrade", "").lower() == "websocket":
            return None

        if request.path == "/" or request.path == "/index.html":
            html_path = self.dashboard_dir / "index.html"
            if html_path.exists():
                body = html_path.read_bytes()
                return Response(
                    200, "OK",
                    Headers({
                        "Content-Type": "text/html; charset=utf-8",
                        "Content-Length": str(len(body)),
                        "Cache-Control": "no-cache",
                    }),
                    body,
                )
        # Let WebSocket upgrade proceed for all other paths
        return None

    async def _handler(self, websocket: ServerConnection) -> None:
        """Handle a single WebSocket connection."""
        self._clients.add(websocket)
        remote = websocket.remote_address
        logger.info("Dashboard client connected: %s", remote)
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if msg.get("type") == "query":
                    text = msg.get("text", "").strip()
                    if text:
                        self._query_queue.put(text)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self._clients.discard(websocket)
            logger.info("Dashboard client disconnected: %s", remote)

    async def _broadcast_coro(self, msg: str) -> None:
        """Send a message to all connected clients, removing dead ones."""
        dead = set()
        for ws in list(self._clients):
            try:
                await ws.send(msg)
            except websockets.exceptions.ConnectionClosed:
                dead.add(ws)
            except Exception:
                dead.add(ws)
        self._clients -= dead


def _json_default(obj):
    """JSON serializer fallback for numpy types."""
    import numpy as np
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    raise TypeError(f"Object of type {type(obj)} is not JSON serializable")
