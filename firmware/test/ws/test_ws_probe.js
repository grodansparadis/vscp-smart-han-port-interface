#!/usr/bin/env node

const WS_HOST = (process.env.WS_HOST || "").trim();
const TIMEOUT_MS = Number(process.env.WS_TIMEOUT_MS || "3000");
const PORTS = (process.env.WS_PROBE_PORTS || "80,8884")
  .split(",")
  .map((s) => Number(s.trim()))
  .filter((n) => Number.isInteger(n) && n > 0);
const PATHS = ["/ws1", "/ws2"];

if (!WS_HOST) {
  console.error("Set WS_HOST to the device IP/hostname (example: WS_HOST=192.168.1.100)");
  process.exit(1);
}

if (!PORTS.length) {
  console.error("WS_PROBE_PORTS must include at least one valid port (example: 80,8884)");
  process.exit(1);
}

async function getWebSocketCtor() {
  if (typeof WebSocket !== "undefined") {
    return WebSocket;
  }

  try {
    const mod = await import("ws");
    return mod.WebSocket || mod.default;
  } catch {
    throw new Error("No WebSocket implementation found. Use Node.js with global WebSocket or run: npm install ws");
  }
}

function probe(WS, host, port, path, timeoutMs = TIMEOUT_MS) {
  return new Promise((resolve) => {
    const url = `ws://${host}:${port}${path}`;
    const startedAt = Date.now();
    const ws = new WS(url);

    let done = false;
    const timer = setTimeout(() => {
      finish(false, "timeout");
    }, timeoutMs);

    function cleanup() {
      clearTimeout(timer);
      ws.removeEventListener?.("open", onOpen);
      ws.removeEventListener?.("error", onError);
      if (typeof ws.off === "function") {
        ws.off("open", onOpen);
        ws.off("error", onError);
      }
      try {
        ws.close();
      } catch {}
    }

    function finish(ok, detail) {
      if (done) return;
      done = true;
      cleanup();
      resolve({ ok, detail, url, ms: Date.now() - startedAt });
    }

    function onOpen() {
      finish(true, "upgrade ok");
    }

    function onError(err) {
      finish(false, err?.message || String(err));
    }

    if (typeof ws.addEventListener === "function") {
      ws.addEventListener("open", onOpen);
      ws.addEventListener("error", onError);
    } else if (typeof ws.on === "function") {
      ws.on("open", onOpen);
      ws.on("error", onError);
    }
  });
}

(async () => {
  const WS = await getWebSocketCtor();
  console.log(`Probing websocket endpoints on ${WS_HOST} (ports: ${PORTS.join(", ")})`);

  let successCount = 0;
  for (const port of PORTS) {
    for (const path of PATHS) {
      const result = await probe(WS, WS_HOST, port, path);
      if (result.ok) {
        successCount += 1;
        console.log(`✓ ${result.url} (${result.ms}ms)`);
      } else {
        console.log(`✗ ${result.url} -> ${result.detail}`);
      }
    }
  }

  if (!successCount) {
    console.error("No websocket endpoint accepted upgrade.");
    process.exit(1);
  }

  console.log(`Probe complete: ${successCount} endpoint(s) accepted websocket upgrade.`);
})().catch((err) => {
  console.error(`✗ Probe failed: ${err.message}`);
  process.exit(1);
});
