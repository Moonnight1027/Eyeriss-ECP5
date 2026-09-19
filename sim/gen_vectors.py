"""
Generate sim/layer_vectors.txt: host byte stream (W xx) and expected device
replies (R xx) produced by the host library against the software emulator.
Replayed by sim/tb_layer.sv.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import eyeriss_host as eh  # noqa: E402


def main():
    rng = np.random.default_rng(7)
    rec = eh.RecordingTransport(eh.EmulatedDevice())
    acc = eh.Eyeriss(rec)

    def rand_case(c, cout, h, w, bias, pad):
        x = rng.integers(0, 256, (c, h, w), dtype=np.uint8)
        k = rng.integers(-128, 128, (cout, c, 3, 3)).astype(np.int8)
        b = rng.integers(-20000, 20000, cout) if bias else None
        s = eh.pick_shift(eh.conv_psum(eh.pad_zero_point(x, pad), k, b))
        return x, k, b, s

    acc.ping()

    # Legacy protocol (0x03 / 0x01), W=5
    x, k, _, s = rand_case(1, 1, 5, 5, False, 0)
    acc.configure_legacy(5, s)
    acc.write_legacy(x.tobytes() + k.tobytes())
    acc.run(9)

    # G=1, P=4, bias, no ReLU
    x, k, b, s = rand_case(3, 4, 6, 6, True, 0)
    acc.conv_block(x, k, b, s, relu=False)

    # G=2, P=4 then P=2, padding, forced 6x6 tiles with overlap
    x, k, b, s = rand_case(5, 6, 7, 7, True, 1)
    acc.conv_layer(x, k, b, s, relu=True, padding=1, tile=6)

    # G=4, P=2, no bias
    x, k, _, s = rand_case(16, 2, 5, 5, False, 0)
    acc.conv_block(x, k, None, s)

    # G=16 (64 channels), P=3
    x, k, b, s = rand_case(64, 3, 4, 4, True, 0)
    acc.conv_block(x, k, b, s)

    # Raw word read-back of the ifmap
    acc.read_words(0, 3)

    # Rejected configs keep the previous layout
    for bad in [(5, 17, 1, 0), (5, 1, 5, 0), (2, 1, 1, 0), (32, 16, 4, 0)]:
        try:
            acc.configure(*bad)
            raise AssertionError(f"{bad} accepted")
        except ValueError:
            pass
    acc.run(3 * 2 * 2)

    # Back to legacy after layer mode
    x, k, _, s = rand_case(1, 1, 6, 6, False, 0)
    acc.configure_legacy(6, s)
    acc.write_legacy(x.tobytes() + k.tobytes())
    acc.run(16)

    path = os.path.join(os.path.dirname(__file__), 'layer_vectors.txt')
    n_w = n_r = 0
    with open(path, 'w') as fp:
        for kind, data in rec.log:
            for byte in data:
                fp.write(f"{kind} {byte:02x}\n")
            n_w += len(data) if kind == 'W' else 0
            n_r += len(data) if kind == 'R' else 0
    print(f"{path}: {n_w} host bytes, {n_r} expected reply bytes")


if __name__ == '__main__':
    main()
