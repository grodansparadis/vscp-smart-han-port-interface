const WS_HOST = (process.env.WS_HOST || "").trim();
const WS_PORT = Number(process.env.WS_PORT || "80");
const TIMEOUT_MS = Number(process.env.WS_TIMEOUT_MS || "5000");

function validateEnvironment() {
  if (!WS_HOST) {
    throw new Error("Set WS_HOST to the device IP/hostname (example: WS_HOST=192.168.1.100)");
  }

  if (!Number.isInteger(WS_PORT) || WS_PORT <= 0) {
    throw new Error("WS_PORT must be a positive integer");
  }
}

function wsUrl(path) {
  return `ws://${WS_HOST}:${WS_PORT}${path}`;
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

function connect(WS, url, timeoutMs = TIMEOUT_MS) {
  return new Promise((resolve, reject) => {
    const ws = new WS(url);
    const timer = setTimeout(() => {
      cleanup();
      try {
        ws.close();
      } catch {}
      reject(new Error(`Timeout connecting to ${url}`));
    }, timeoutMs);

    function cleanup() {
      clearTimeout(timer);
      ws.removeEventListener?.("open", onOpen);
      ws.removeEventListener?.("error", onError);
      if (typeof ws.off === "function") {
        ws.off("open", onOpen);
        ws.off("error", onError);
      }
    }

    function onOpen() {
      cleanup();
      resolve(ws);
    }

    function onError(err) {
      cleanup();
      reject(new Error(`Failed to connect to ${url}: ${err?.message || err}`));
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

function receiveText(ws, timeoutMs = TIMEOUT_MS) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      cleanup();
      reject(new Error("Timeout waiting for message"));
    }, timeoutMs);

    function cleanup() {
      clearTimeout(timer);
      ws.removeEventListener?.("message", onMessage);
      ws.removeEventListener?.("error", onError);
      if (typeof ws.off === "function") {
        ws.off("message", onMessage);
        ws.off("error", onError);
      }
    }

    function normalize(payload) {
      if (typeof payload === "string") return payload;
      if (payload instanceof ArrayBuffer) return Buffer.from(payload).toString("utf8");
      if (Buffer.isBuffer(payload)) return payload.toString("utf8");
      if (ArrayBuffer.isView(payload)) return Buffer.from(payload.buffer).toString("utf8");
      return String(payload);
    }

    function onMessage(evtOrData) {
      cleanup();
      const data = evtOrData?.data !== undefined ? evtOrData.data : evtOrData;
      resolve(normalize(data));
    }

    function onError(err) {
      cleanup();
      reject(new Error(`Socket error while receiving: ${err?.message || err}`));
    }

    if (typeof ws.addEventListener === "function") {
      ws.addEventListener("message", onMessage);
      ws.addEventListener("error", onError);
    } else if (typeof ws.on === "function") {
      ws.on("message", onMessage);
      ws.on("error", onError);
    }
  });
}

function closeSocket(ws) {
  try {
    ws.close();
  } catch {}
}

async function testHandshakePath(WS, path) {
  const ws = await connect(WS, wsUrl(path));
  closeSocket(ws);
  console.log(`✓ Handshake test passed for ${path}`);
}

async function testToggleBroadcastDirection(WS, senderPath, receiverPath) {
  const sender = await connect(WS, wsUrl(senderPath));
  const receiver = await connect(WS, wsUrl(receiverPath));

  try {
    const senderWait = receiveText(sender);
    const receiverWait = receiveText(receiver);

    sender.send("toggle");

    const [senderMsg, receiverMsg] = await Promise.all([senderWait, receiverWait]);

    if (!/^\d+$/.test(senderMsg)) {
      throw new Error(`${senderPath} reply is not numeric: ${senderMsg}`);
    }

    if (!/^\d+$/.test(receiverMsg)) {
      throw new Error(`${receiverPath} reply is not numeric: ${receiverMsg}`);
    }

    if (senderMsg !== receiverMsg) {
      throw new Error(
        `Broadcast mismatch ${senderPath}->${receiverPath}: sender=${senderMsg}, receiver=${receiverMsg}`
      );
    }

    console.log(`✓ Toggle broadcast ${senderPath} -> ${receiverPath} passed (value=${senderMsg})`);
  } finally {
    closeSocket(sender);
    closeSocket(receiver);
  }
}

module.exports = {
  WS_HOST,
  WS_PORT,
  validateEnvironment,
  getWebSocketCtor,
  testHandshakePath,
  testToggleBroadcastDirection,
};
