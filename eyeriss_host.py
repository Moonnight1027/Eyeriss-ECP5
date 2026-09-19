"""
Host library for the Eyeriss ECP5 accelerator (see Eyeriss_SoC_Top.sv).

    acc = Eyeriss(SerialTransport('COM3'))      # or Eyeriss(EmulatedDevice())
    out = acc.conv_layer(x_u8, w_i8, bias_i32, shift=6, relu=True, padding=1)

Number format
    ifmap  : uint8 with zero point 128 (value = byte - 128)
    weight : int8
    bias   : int32 in the psum domain (scale = s_x * s_w)
    output : uint8, max(128, clamp(psum >> shift, -128, 127) + 128) with ReLU
"""
import struct
import time

import numpy as np

CMD_WRITE_SRAM = 0x01
CMD_START_MAC  = 0x02
CMD_CONFIG     = 0x03
CMD_WRITE_WORD = 0x04
CMD_READ_WORD  = 0x05
CMD_LAYER_CFG  = 0x06
CMD_PING       = 0xFF
RSP_ACK        = 0x06
RSP_NACK       = 0xEE

ADDR_W    = 13
MEM_WORDS = 1 << ADDR_W
MIN_W, MAX_W = 3, 32
MAX_G = 16
MAX_P = 4
CH_PER_WORD = 4

FLAG_RELU = 0x01
FLAG_BIAS = 0x02


# ==========================================================================
# Layout / golden model
# ==========================================================================
def layout(w, g, p):
    """SRAM layout for one run; returns None if it does not fit."""
    f = w - 2
    filter_base = g * w * w
    bias_base = filter_base + g * p * 9
    psum_base = bias_base + p
    total = psum_base + p * f * f
    if total > MEM_WORDS:
        return None
    return dict(filter_base=filter_base, bias_base=bias_base,
                psum_base=psum_base, total=total)


def max_tile_width(g, p):
    for w in range(MAX_W, MIN_W - 1, -1):
        if layout(w, g, p) is not None:
            return w
    return None


def conv_psum(x_u8, w_i8, bias=None):
    """Integer 3x3 valid conv: x [C,H,W] uint8, w [P,C,3,3] int8 -> [P,H-2,W-2] int64."""
    x = x_u8.astype(np.int64) - 128
    win = np.lib.stride_tricks.sliding_window_view(x, (3, 3), axis=(1, 2))  # C,F,F,3,3
    psum = np.einsum('cyxij,pcij->pyx', win, w_i8.astype(np.int64), optimize=True)
    if bias is not None:
        psum += np.asarray(bias, dtype=np.int64)[:, None, None]
    return psum


def ppu(psum, shift, relu=True):
    q = np.clip(np.asarray(psum, dtype=np.int64) >> shift, -128, 127) + 128
    if relu:
        q = np.maximum(q, 128)
    return q.astype(np.uint8)


def pick_shift(psum):
    """Smallest right shift that keeps the largest |psum| inside int8."""
    peak = int(np.max(np.abs(psum)))
    shift = 0
    while (peak >> shift) > 127 and shift < 31:
        shift += 1
    return shift


def pad_zero_point(x_u8, padding):
    if padding == 0:
        return x_u8
    return np.pad(x_u8, ((0, 0), (padding, padding), (padding, padding)), constant_values=128)


def golden_conv_layer(x_u8, w_i8, bias=None, shift=0, relu=True, padding=0):
    return ppu(conv_psum(pad_zero_point(x_u8, padding), w_i8, bias), shift, relu)


# ==========================================================================
# Packing
# ==========================================================================
def pack_ifmap(x_u8):
    """[C,W,W] uint8 -> G*W*W uint32 words (group, row, col), 4 channels per word."""
    c, h, w = x_u8.shape
    g = -(-c // CH_PER_WORD)
    buf = np.full((g * CH_PER_WORD, h, w), 128, dtype=np.uint8)
    buf[:c] = x_u8
    buf = buf.reshape(g, CH_PER_WORD, h, w).transpose(0, 2, 3, 1)   # g,h,w,4
    return np.ascontiguousarray(buf).view('<u4').reshape(-1)


def pack_filters(w_i8, g):
    """[P,C,3,3] int8 -> G*P*9 uint32 words (group, filter, row, tap)."""
    p, c = w_i8.shape[:2]
    buf = np.zeros((p, g * CH_PER_WORD, 3, 3), dtype=np.int8)
    buf[:, :c] = w_i8
    buf = buf.reshape(p, g, CH_PER_WORD, 3, 3).transpose(1, 0, 3, 4, 2)  # g,p,3,3,4
    return np.ascontiguousarray(buf).view('<u4').reshape(-1)


def unpack_word_bytes(words):
    return np.asarray(words, dtype='<u4').view(np.uint8).reshape(-1, CH_PER_WORD)


# ==========================================================================
# Transports
# ==========================================================================
class SerialTransport:
    def __init__(self, port, timeout=5):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=timeout, write_timeout=timeout)
        self.ser.dtr = self.ser.rts = True
        time.sleep(1)
        self.ser.reset_input_buffer()

    def write(self, data):
        self.ser.write(bytes(data))

    def read(self, n):
        return self.ser.read(n)

    def flush(self):
        """Drop late replies so the next command starts in sync."""
        time.sleep(0.5)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()


class EmulatedDevice:
    """Bit-accurate software model of the SoC protocol (no USB, no timing)."""

    def __init__(self):
        self.mem = np.zeros(MEM_WORDS, dtype=np.uint32)
        self.out = bytearray()
        self.inbuf = bytearray()
        self.w, self.g, self.p = 5, 1, 1
        self.shift, self.relu, self.bias = 0, True, False

    def close(self):
        pass

    def flush(self):
        self.out.clear()

    def read(self, n):
        data, self.out = bytes(self.out[:n]), self.out[n:]
        return data

    def write(self, data):
        self.inbuf += bytes(data)
        while self._step():
            pass

    def _take(self, n):
        if len(self.inbuf) < n:
            return None
        data, self.inbuf = bytes(self.inbuf[:n]), self.inbuf[n:]
        return data

    def _step(self):
        if not self.inbuf:
            return False
        cmd = self.inbuf[0]
        need = {CMD_WRITE_SRAM: 3, CMD_CONFIG: 3, CMD_WRITE_WORD: 5,
                CMD_READ_WORD: 5, CMD_LAYER_CFG: 6}.get(cmd, 1)
        if len(self.inbuf) < need:
            return False
        hdr = bytes(self.inbuf[:need])
        if cmd == CMD_WRITE_SRAM:
            n = hdr[1] | hdr[2] << 8
            if len(self.inbuf) < need + n:
                return False
            self._take(need)
            self.mem[:n] = np.frombuffer(self._take(n), dtype=np.uint8)
        elif cmd == CMD_WRITE_WORD:
            adr, n = struct.unpack('<HH', hdr[1:])
            if len(self.inbuf) < need + 4 * n:
                return False
            self._take(need)
            adr &= MEM_WORDS - 1
            self.mem[adr:adr + n] = np.frombuffer(self._take(4 * n), dtype='<u4')
        elif cmd == CMD_READ_WORD:
            self._take(need)
            adr, n = struct.unpack('<HH', hdr[1:])
            adr &= MEM_WORDS - 1
            self.out += self.mem[adr:adr + n].astype('<u4').tobytes()
        elif cmd == CMD_CONFIG:
            self._take(need)
            w = hdr[1] if MIN_W <= hdr[1] <= MAX_W else self.w
            self.w, self.g, self.p = w, 1, 1
            self.shift, self.relu, self.bias = hdr[2] & 31, True, False
        elif cmd == CMD_LAYER_CFG:
            self._take(need)
            w, g, p, shift, flags = hdr[1:]
            ok = (MIN_W <= w <= MAX_W and 1 <= g <= MAX_G and 1 <= p <= MAX_P
                  and shift <= 31 and layout(w, g, p) is not None)
            if ok:
                self.w, self.g, self.p, self.shift = w, g, p, shift
                self.relu, self.bias = bool(flags & FLAG_RELU), bool(flags & FLAG_BIAS)
            self.out.append(RSP_ACK if ok else RSP_NACK)
        elif cmd == CMD_START_MAC:
            self._take(need)
            self._run()
        else:
            self._take(need)
            self.out.append(cmd)
        return True

    def _run(self):
        w, g, p = self.w, self.g, self.p
        lay = layout(w, g, p)
        f = w - 2
        x = unpack_word_bytes(self.mem[:g * w * w]).reshape(g, w, w, 4)
        x = x.transpose(0, 3, 1, 2).reshape(g * 4, w, w)
        k = unpack_word_bytes(self.mem[lay['filter_base']:lay['bias_base']]).view(np.int8)
        k = k.reshape(g, p, 3, 3, 4).transpose(1, 0, 4, 2, 3).reshape(p, g * 4, 3, 3)
        bias = self.mem[lay['bias_base']:lay['psum_base']].view(np.int32).astype(np.int64)
        psum = conv_psum(x, k, bias if self.bias else None)
        psum = ((psum + 2**31) % 2**32) - 2**31          # int32 wrap like the PE
        out = ppu(psum, self.shift, self.relu).transpose(1, 2, 0).reshape(-1)
        self.mem[lay['psum_base']:lay['total']] = out
        self.out += out.tobytes()


class RecordingTransport:
    """Wraps a transport and logs the byte stream (for testbench vectors)."""

    def __init__(self, inner):
        self.inner = inner
        self.log = []          # ('W', bytes) / ('R', bytes)

    def write(self, data):
        self.log.append(('W', bytes(data)))
        self.inner.write(data)

    def read(self, n):
        data = self.inner.read(n)
        self.log.append(('R', data))
        return data

    def close(self):
        self.inner.close()


# ==========================================================================
# Accelerator API
# ==========================================================================
class Eyeriss:
    def __init__(self, transport):
        self.t = transport
        self._cfg = None

    def close(self):
        self.t.close()

    def _read_exact(self, n):
        data = self.t.read(n)
        if len(data) != n:
            self._cfg = None
            if hasattr(self.t, 'flush'):
                self.t.flush()
            raise TimeoutError(f"RX timeout, received {len(data)} / {n} bytes")
        return data

    # ---- low level ----
    def ping(self):
        self.t.write(bytes([CMD_PING]))
        reply = self.t.read(1)
        if reply != bytes([CMD_PING]):
            raise RuntimeError(f"PING failed, got {reply!r}")

    def configure_legacy(self, w, shift):
        self.t.write(bytes([CMD_CONFIG, w, shift]))
        self._cfg = None

    def configure(self, w, g, p, shift, relu=True, bias=False):
        cfg = (w, g, p, shift, relu, bias)
        if cfg == self._cfg:
            return
        flags = (FLAG_RELU if relu else 0) | (FLAG_BIAS if bias else 0)
        self.t.write(bytes([CMD_LAYER_CFG, w, g, p, shift, flags]))
        reply = self._read_exact(1)[0]
        if reply != RSP_ACK:
            self._cfg = None
            raise ValueError(f"config {cfg} rejected (0x{reply:02X})")
        self._cfg = cfg

    def write_words(self, addr, words):
        words = np.asarray(words, dtype='<u4')
        for i in range(0, len(words), 0xFFFF):
            chunk = words[i:i + 0xFFFF]
            self.t.write(struct.pack('<BHH', CMD_WRITE_WORD, addr + i, len(chunk)) + chunk.tobytes())

    def read_words(self, addr, n):
        self.t.write(struct.pack('<BHH', CMD_READ_WORD, addr, n))
        return np.frombuffer(self._read_exact(4 * n), dtype='<u4')

    def write_legacy(self, payload):
        self.t.write(bytes([CMD_WRITE_SRAM]) + struct.pack('<H', len(payload)) + bytes(payload))

    def run(self, n_out):
        self.t.write(bytes([CMD_START_MAC]))
        return np.frombuffer(self._read_exact(n_out), dtype=np.uint8)

    # ---- single run ----
    def conv_block(self, x_u8, w_i8, bias=None, shift=0, relu=True, upload_ifmap=True):
        """One hardware run: x [C,W,W], w [P,C,3,3] with P <= 4 -> [P,F,F] uint8."""
        c, h, wd = x_u8.shape
        p = w_i8.shape[0]
        assert h == wd and w_i8.shape[1] == c and 1 <= p <= MAX_P
        g = -(-c // CH_PER_WORD)
        lay = layout(wd, g, p)
        if lay is None or g > MAX_G:
            raise ValueError(f"W={wd} C={c} P={p} does not fit the GLB")
        self.configure(wd, g, p, shift, relu, bias is not None)
        if upload_ifmap:
            self.write_words(0, pack_ifmap(x_u8))
        words = pack_filters(w_i8, g)
        if bias is not None:
            words = np.concatenate([words, np.asarray(bias, dtype=np.int64).astype('<i4').view('<u4')])
        self.write_words(lay['filter_base'], words)
        f = wd - 2
        out = self.run(p * f * f)
        return out.reshape(f, f, p).transpose(2, 0, 1)

    # ---- full layer ----
    def conv_layer(self, x_u8, w_i8, bias=None, shift=0, relu=True, padding=0,
                   tile=None, progress=None):
        """
        3x3 conv layer of any size: pads with the zero point, splits the input
        into square tiles that fit the GLB and the filters into batches of 4.
        x [C,H,W] uint8, w [Cout,C,3,3] int8, bias [Cout] int32 -> [Cout,Fh,Fw] uint8
        """
        cout, c = w_i8.shape[:2]
        assert x_u8.shape[0] == c
        g = -(-c // CH_PER_WORD)
        if g > MAX_G:
            raise ValueError(f"{c} input channels > {MAX_G * CH_PER_WORD}")
        xp = pad_zero_point(x_u8, padding)
        fh, fw = xp.shape[1] - 2, xp.shape[2] - 2
        tmax = (tile or max_tile_width(g, min(cout, MAX_P))) - 2
        t = min(tmax, fh, fw)
        if t < 1:
            raise ValueError("input too small")

        def offsets(n):
            offs = list(range(0, n - t + 1, t))
            if offs[-1] + t < n:
                offs.append(n - t)
            return offs

        out = np.zeros((cout, fh, fw), dtype=np.uint8)
        jobs = [(oy, ox) for oy in offsets(fh) for ox in offsets(fw)]
        for n_job, (oy, ox) in enumerate(jobs):
            xt = np.ascontiguousarray(xp[:, oy:oy + t + 2, ox:ox + t + 2])
            for b in range(0, cout, MAX_P):
                wb = w_i8[b:b + MAX_P]
                bb = None if bias is None else np.asarray(bias)[b:b + MAX_P]
                out[b:b + len(wb), oy:oy + t, ox:ox + t] = self.conv_block(
                    xt, wb, bb, shift, relu, upload_ifmap=(b == 0))
            if progress:
                progress(n_job + 1, len(jobs))
        return out
