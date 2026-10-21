#!/usr/bin/env node
/**
 * PX4 <-> browser bridge: lets the frontier-fly page (px4.html) be PX4's simulator.
 *
 *   node scripts/px4/bridge.mjs [--tcp 4560] [--ws 4561]
 *
 * PX4 SITL's simulator_mavlink connects out to a simulator on TCP 4560, and a
 * web page cannot accept TCP connections. This script accepts PX4's
 * connection and relays its bytes, unchanged, to one WebSocket client (the
 * page) and back. All MAVLink decoding, physics and sensors run in the page.
 * No dependencies: the WebSocket server is the minimal RFC 6455 subset a
 * browser needs (binary frames, ping/pong, close), and it binds to
 * 127.0.0.1 only.
 */
import { createHash } from 'node:crypto';
import { createServer } from 'node:net';

const args = process.argv.slice(2);
const opt = (name, fallback) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? Number(args[i + 1]) : fallback;
};
const TCP_PORT = opt('tcp', 4560);
const WS_PORT = opt('ws', 4561);
const HOST = '127.0.0.1';
const GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11';

let px4 = null;
let page = null;
/** PX4 bytes that arrived before a page connected. PX4 sends little until the simulator talks first. */
let pending = [];

function log(msg) {
  console.log(`[bridge] ${msg}`);
}

function wsFrame(payload, opcode = 0x2) {
  const len = payload.length;
  const head = len < 126 ? Buffer.from([0x80 | opcode, len]) : len < 65536 ? Buffer.alloc(4) : Buffer.alloc(10);
  if (len >= 126 && len < 65536) {
    head[0] = 0x80 | opcode;
    head[1] = 126;
    head.writeUInt16BE(len, 2);
  } else if (len >= 65536) {
    head[0] = 0x80 | opcode;
    head[1] = 127;
    head.writeBigUInt64BE(BigInt(len), 2);
  }
  return Buffer.concat([head, payload]);
}

function toPage(chunk) {
  if (page) page.socket.write(wsFrame(chunk));
  else pending.push(chunk);
}

/** Parses client frames (always masked) out of a growing buffer. */
function readFrames(state, chunk, onMessage, onClose, onPing) {
  state.buf = Buffer.concat([state.buf, chunk]);
  for (;;) {
    const b = state.buf;
    if (b.length < 2) return;
    const opcode = b[0] & 0x0f;
    let len = b[1] & 0x7f;
    const masked = (b[1] & 0x80) !== 0;
    let off = 2;
    if (len === 126) {
      if (b.length < 4) return;
      len = b.readUInt16BE(2);
      off = 4;
    } else if (len === 127) {
      if (b.length < 10) return;
      len = Number(b.readBigUInt64BE(2));
      off = 10;
    }
    const maskOff = off;
    if (masked) off += 4;
    if (b.length < off + len) return;
    const data = Buffer.from(b.subarray(off, off + len));
    if (masked) for (let i = 0; i < len; i++) data[i] ^= b[maskOff + (i & 3)];
    state.buf = b.subarray(off + len);
    // Fragmented messages are not used by browsers for small binary sends; treat continuation as data.
    if (opcode === 0x2 || opcode === 0x1 || opcode === 0x0) onMessage(data);
    else if (opcode === 0x8) return onClose();
    else if (opcode === 0x9) onPing(data);
  }
}

const tcp = createServer((socket) => {
  if (px4) {
    log('a second PX4 connected; dropping it');
    socket.destroy();
    return;
  }
  socket.setNoDelay(true);
  px4 = socket;
  log(`PX4 connected from ${socket.remoteAddress}:${socket.remotePort}`);
  socket.on('data', toPage);
  socket.on('close', () => {
    log('PX4 disconnected');
    px4 = null;
    pending = [];
    if (page) page.socket.write(wsFrame(Buffer.from('px4-closed'), 0x1));
  });
  socket.on('error', () => {});
  if (page) page.socket.write(wsFrame(Buffer.from('px4-connected'), 0x1));
});

const ws = createServer((socket) => {
  socket.setNoDelay(true);
  let upgraded = false;
  let head = Buffer.alloc(0);
  const state = { buf: Buffer.alloc(0) };
  socket.on('error', () => {});
  socket.on('data', (chunk) => {
    if (upgraded) {
      readFrames(
        state,
        chunk,
        (data) => px4?.write(data),
        () => socket.end(wsFrame(Buffer.alloc(0), 0x8)),
        (data) => socket.write(wsFrame(data, 0xa)),
      );
      return;
    }
    head = Buffer.concat([head, chunk]);
    const end = head.indexOf('\r\n\r\n');
    if (end < 0) return;
    const request = head.subarray(0, end).toString('latin1');
    const key = /^sec-websocket-key:\s*(.+)$/im.exec(request)?.[1]?.trim();
    if (!key || !/^get /i.test(request)) {
      socket.end('HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\n\r\nfrontier-fly PX4 bridge: WebSocket only\n');
      return;
    }
    if (page) {
      socket.end('HTTP/1.1 409 Conflict\r\n\r\n');
      log('a second page tried to connect; refused');
      return;
    }
    const accept = createHash('sha1').update(key + GUID).digest('base64');
    socket.write(
      'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n' +
        `Sec-WebSocket-Accept: ${accept}\r\n\r\n`,
    );
    upgraded = true;
    page = { socket };
    log('page connected');
    socket.write(wsFrame(Buffer.from(px4 ? 'px4-connected' : 'px4-waiting'), 0x1));
    for (const chunk of pending) socket.write(wsFrame(chunk));
    pending = [];
    const rest = head.subarray(end + 4);
    if (rest.length) socket.emit('data', rest);
    socket.on('close', () => {
      log('page disconnected');
      page = null;
      // PX4 keeps its lockstep clock; a reconnecting page would restart time at zero, so drop PX4 too.
      px4?.destroy();
    });
  });
});

tcp.listen(TCP_PORT, HOST, () => log(`waiting for PX4 on tcp://${HOST}:${TCP_PORT}`));
ws.listen(WS_PORT, HOST, () => log(`waiting for the page on ws://${HOST}:${WS_PORT}`));
