"""Serves the web UI and streams sensor readings to it as Server-Sent Events."""
import json
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Lock
from typing import Dict, Set, Tuple

WEB_DIR = Path(__file__).parent / 'web'
KEEPALIVE_S = 5.0
CLIENT_QUEUE_LEN = 200

Event = Tuple[str, dict]


class EventHub:
    """Fans events out to every connected browser.

    'Sticky' events (config, status) are remembered and replayed to browsers that
    connect later, so a page refresh immediately knows the sensor state.
    """

    def __init__(self):
        self._clients: Set[Queue] = set()
        self._sticky: Dict[str, dict] = {}
        self._lock = Lock()

    def subscribe(self) -> Queue:
        client: Queue = Queue(maxsize=CLIENT_QUEUE_LEN)
        with self._lock:
            for event in self._sticky.items():
                client.put_nowait(event)
            self._clients.add(client)
        return client

    def unsubscribe(self, client: Queue) -> None:
        with self._lock:
            self._clients.discard(client)

    def publish(self, event: str, data: dict, sticky: bool = False) -> None:
        with self._lock:
            if sticky:
                self._sticky[event] = data
            clients = list(self._clients)
        for client in clients:
            try:
                client.put_nowait((event, data))
            except Full:
                pass  # a stalled browser just misses readings


class RequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, hub: EventHub, **kwargs):
        self._hub = hub
        super().__init__(*args, directory=str(WEB_DIR), **kwargs)

    def do_GET(self) -> None:
        if self.path == '/events':
            self._stream_events()
        else:
            super().do_GET()

    def end_headers(self) -> None:
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def log_message(self, format: str, *args) -> None:
        pass

    def _stream_events(self) -> None:
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        client = self._hub.subscribe()
        try:
            while True:
                try:
                    event, data = client.get(timeout=KEEPALIVE_S)
                    message = f'event: {event}\ndata: {json.dumps(data)}\n\n'
                except Empty:
                    message = ': keepalive\n\n'
                self.wfile.write(message.encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            self._hub.unsubscribe(client)


def make_server(hub: EventHub, host: str, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), partial(RequestHandler, hub=hub))
    server.daemon_threads = True
    return server
