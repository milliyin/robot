import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


STATE_FILE = "copy_paste_state.json"
SLOTS = ("pc_to_jetson", "jetson_to_pc")


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Clipboard Bridge</title>
  <style>
    :root {
      color-scheme: light dark;
      --bg: #10141f;
      --panel: #171d2b;
      --text: #edf2ff;
      --muted: #9aa7bd;
      --line: #2a3347;
      --accent: #62d2a2;
      --accent2: #7aa7ff;
      --danger: #ff7474;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: system-ui, -apple-system, Segoe UI, sans-serif;
      background: var(--bg);
      color: var(--text);
    }
    main {
      width: min(1100px, calc(100vw - 28px));
      margin: 20px auto;
    }
    header {
      display: flex;
      align-items: flex-end;
      justify-content: space-between;
      gap: 16px;
      padding: 14px 0 18px;
      border-bottom: 1px solid var(--line);
    }
    h1 {
      margin: 0;
      font-size: clamp(1.4rem, 3vw, 2.1rem);
      font-weight: 760;
      letter-spacing: 0;
    }
    .status {
      color: var(--muted);
      font-size: 0.95rem;
      text-align: right;
      white-space: nowrap;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 16px;
      margin-top: 18px;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      min-width: 0;
      overflow: hidden;
    }
    .section-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      padding: 12px 14px;
      border-bottom: 1px solid var(--line);
    }
    h2 {
      margin: 0;
      font-size: 1rem;
      letter-spacing: 0;
    }
    .meta {
      color: var(--muted);
      font-size: 0.82rem;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    textarea {
      width: 100%;
      height: min(44vh, 420px);
      min-height: 260px;
      display: block;
      resize: vertical;
      border: 0;
      outline: 0;
      padding: 14px;
      background: transparent;
      color: var(--text);
      font: 15px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    .actions {
      display: flex;
      gap: 8px;
      padding: 12px 14px 14px;
      border-top: 1px solid var(--line);
      flex-wrap: wrap;
    }
    button, input {
      height: 38px;
      border-radius: 7px;
      border: 1px solid var(--line);
      background: #20283a;
      color: var(--text);
      padding: 0 12px;
      font: inherit;
    }
    button {
      cursor: pointer;
      font-weight: 650;
    }
    button.primary { background: var(--accent); color: #06140f; border-color: transparent; }
    button.copy { background: var(--accent2); color: #071122; border-color: transparent; }
    button.danger { color: var(--danger); }
    input {
      min-width: 220px;
      flex: 1;
    }
    .hint {
      margin-top: 14px;
      color: var(--muted);
      line-height: 1.5;
      font-size: 0.95rem;
    }
    code { color: var(--accent); }
    @media (max-width: 760px) {
      header { display: block; }
      .status { text-align: left; margin-top: 8px; white-space: normal; }
      .grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>Robot Clipboard Bridge</h1>
        <div class="hint">Open this page on both devices. Save text in one panel, copy it from the other device.</div>
      </div>
      <div class="status" id="status">connecting...</div>
    </header>

    <div class="actions" style="padding-left:0; padding-right:0; border:0;">
      <input id="deviceName" placeholder="Device name, e.g. PC or Jetson">
      <button id="saveName">Save name</button>
    </div>

    <div class="grid">
      <section>
        <div class="section-head">
          <h2>PC -> Jetson</h2>
          <div class="meta" id="meta-pc_to_jetson">not saved yet</div>
        </div>
        <textarea id="text-pc_to_jetson" spellcheck="false" placeholder="Paste text here on PC, then press Save. Open this page on Jetson and press Copy."></textarea>
        <div class="actions">
          <button class="primary" data-save="pc_to_jetson">Save</button>
          <button class="copy" data-copy="pc_to_jetson">Copy</button>
          <button class="danger" data-clear="pc_to_jetson">Clear</button>
        </div>
      </section>

      <section>
        <div class="section-head">
          <h2>Jetson -> PC</h2>
          <div class="meta" id="meta-jetson_to_pc">not saved yet</div>
        </div>
        <textarea id="text-jetson_to_pc" spellcheck="false" placeholder="Paste text here on Jetson, then press Save. Open this page on PC and press Copy."></textarea>
        <div class="actions">
          <button class="primary" data-save="jetson_to_pc">Save</button>
          <button class="copy" data-copy="jetson_to_pc">Copy</button>
          <button class="danger" data-clear="jetson_to_pc">Clear</button>
        </div>
      </section>
    </div>

    <p class="hint">
      Browser clipboard rules usually require a click on <code>Copy</code>.
      If automatic clipboard permission appears, allow it.
    </p>
  </main>

  <script>
    const slots = ["pc_to_jetson", "jetson_to_pc"];
    const deviceInput = document.getElementById("deviceName");
    const savedName = localStorage.getItem("robotClipboardDeviceName") || "";
    deviceInput.value = savedName;

    document.getElementById("saveName").addEventListener("click", () => {
      localStorage.setItem("robotClipboardDeviceName", deviceInput.value.trim());
    });

    function deviceName() {
      return deviceInput.value.trim() || "unknown device";
    }

    async function fetchState() {
      const response = await fetch("/api/state", {cache: "no-store"});
      if (!response.ok) throw new Error("state fetch failed");
      return response.json();
    }

    async function saveSlot(slot, text) {
      const response = await fetch("/api/slot", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({slot, text, author: deviceName()})
      });
      if (!response.ok) throw new Error(await response.text());
      return response.json();
    }

    function setStatus(text) {
      document.getElementById("status").textContent = text;
    }

    async function copyText(text) {
      if (navigator.clipboard && navigator.clipboard.writeText) {
        try {
          await navigator.clipboard.writeText(text);
          return true;
        } catch (error) {
          // LAN HTTP pages can hit browser clipboard restrictions. Fall through.
        }
      }

      const scratch = document.createElement("textarea");
      scratch.value = text;
      scratch.setAttribute("readonly", "");
      scratch.style.position = "fixed";
      scratch.style.left = "-9999px";
      document.body.appendChild(scratch);
      scratch.select();
      const copied = document.execCommand("copy");
      document.body.removeChild(scratch);
      return copied;
    }

    function updateUi(state) {
      for (const slot of slots) {
        const item = state[slot] || {};
        const textBox = document.getElementById(`text-${slot}`);
        if (document.activeElement !== textBox && textBox.value !== (item.text || "")) {
          textBox.value = item.text || "";
        }
        const date = item.updated_at ? new Date(item.updated_at * 1000).toLocaleTimeString() : "never";
        document.getElementById(`meta-${slot}`).textContent = `by ${item.author || "nobody"} at ${date}`;
      }
      setStatus(`connected ${new Date().toLocaleTimeString()}`);
    }

    async function poll() {
      try {
        updateUi(await fetchState());
      } catch (error) {
        setStatus(`offline: ${error.message}`);
      }
    }

    document.querySelectorAll("[data-save]").forEach(button => {
      button.addEventListener("click", async () => {
        const slot = button.dataset.save;
        await saveSlot(slot, document.getElementById(`text-${slot}`).value);
        await poll();
      });
    });

    document.querySelectorAll("[data-clear]").forEach(button => {
      button.addEventListener("click", async () => {
        const slot = button.dataset.clear;
        document.getElementById(`text-${slot}`).value = "";
        await saveSlot(slot, "");
        await poll();
      });
    });

    document.querySelectorAll("[data-copy]").forEach(button => {
      button.addEventListener("click", async () => {
        const slot = button.dataset.copy;
        const text = document.getElementById(`text-${slot}`).value;
        const copied = await copyText(text);
        const label = slot.replace(/_/g, " ");
        setStatus(`${copied ? "copied" : "copy failed"} ${label} ${new Date().toLocaleTimeString()}`);
      });
    });

    poll();
    setInterval(poll, 1000);
  </script>
</body>
</html>
"""


class ClipboardStore:
    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()
        self.state = self._load()

    def _default_state(self):
        now = time.time()
        return {
            slot: {"text": "", "author": "", "updated_at": now}
            for slot in SLOTS
        }

    def _load(self):
        if not os.path.exists(self.path):
            return self._default_state()
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            return self._default_state()
        state = self._default_state()
        for slot in SLOTS:
            if isinstance(data.get(slot), dict):
                state[slot].update(data[slot])
        return state

    def get(self):
        with self.lock:
            return json.loads(json.dumps(self.state))

    def set_slot(self, slot, text, author):
        if slot not in SLOTS:
            raise ValueError("unknown slot")
        with self.lock:
            self.state[slot] = {
                "text": str(text),
                "author": str(author or "unknown device"),
                "updated_at": time.time(),
            }
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(self.state, f, ensure_ascii=False, indent=2)
            return dict(self.state[slot])


class ClipboardHandler(BaseHTTPRequestHandler):
    store = None

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/?"):
            self._send_html(HTML)
            return
        if self.path == "/api/state":
            self._send_json(self.store.get())
            return
        self.send_error(404, "not found")

    def do_POST(self):
        if self.path != "/api/slot":
            self.send_error(404, "not found")
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            item = self.store.set_slot(
                payload.get("slot"),
                payload.get("text", ""),
                payload.get("author", "unknown device"),
            )
        except Exception as exc:
            self.send_error(400, str(exc))
            return
        self._send_json(item)

    def log_message(self, fmt, *args):
        print("%s - %s" % (self.address_string(), fmt % args))

    def _send_html(self, text):
        data = text.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, value):
        data = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def build_parser():
    parser = argparse.ArgumentParser(description="Local-network two-way clipboard website")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--state-file", default=STATE_FILE)
    return parser


def main():
    args = build_parser().parse_args()
    ClipboardHandler.store = ClipboardStore(args.state_file)
    server = ThreadingHTTPServer((args.host, args.port), ClipboardHandler)
    print("Clipboard bridge running on http://%s:%d" % (args.host, args.port))
    print("On your PC, open http://JETSON_IP:%d" % args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping clipboard bridge")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
