"""
Hardware regression for the Eyeriss ECP5 accelerator (see eyeriss_host.py).

Usage:
    python test_eyeriss.py [COM3]       # real board
    python test_eyeriss.py --emulate    # software model only
"""
import sys
import time

import numpy as np

import eyeriss_host as eh


def legacy_cases():
    skin = np.array([[150, 155, 160, 155, 150],
                     [160, 180, 190, 185, 155],
                     [155, 195, 220, 190, 160],
                     [150, 185, 195, 180, 155],
                     [145, 150, 155, 150, 145]], dtype=np.uint8)
    k = lambda rows: np.array(rows, dtype=np.int8)
    cases = [
        ("identity",  skin, k([[0, 0, 0], [0, 1, 0], [0, 0, 0]]), 0),
        ("laplacian", skin, k([[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]]), 0),
        ("negative",  skin, k([[-3, -2, -1], [-2, -1, -1], [-1, -1, -1]]), 0),
    ]
    rng = np.random.default_rng(seed=2026)
    for w, shift in [(3, 9), (16, 10), (32, 10)]:
        cases.append((f"W={w}", rng.integers(0, 256, (w, w), dtype=np.uint8),
                      rng.integers(-128, 128, (3, 3)).astype(np.int8), shift))
    return cases


def layer_cases():
    """(name, C, Cout, H, W, bias, relu, padding)"""
    return [
        ("C=1  Cout=4  bias",        1,  4, 12, 12, True,  True,  0),
        ("C=3  Cout=4  no-ReLU",     3,  4, 10, 10, True,  False, 0),
        ("C=8  Cout=6  pad",         8,  6, 16, 16, True,  True,  1),
        ("C=16 Cout=2  32x32",      16,  2, 32, 32, False, True,  0),
        ("C=64 Cout=8  tiled 40x40", 64, 8, 40, 40, True,  True,  1),
    ]


def main():
    emulate = '--emulate' in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    port = args[0] if args else 'COM3'
    try:
        transport = eh.EmulatedDevice() if emulate else eh.SerialTransport(port)
        acc = eh.Eyeriss(transport)
        acc.ping()
    except Exception as err:
        print(f"[!] {err}")
        return 1
    print(f"[+] PING OK ({'emulator' if emulate else port})")

    results = []

    def check(name, golden, run):
        try:
            t0 = time.perf_counter()
            hw = run()
            dt = (time.perf_counter() - t0) * 1e3
        except Exception as err:
            print(f"[ERROR] {name:26s} {err}")
            results.append(False)
            return
        ok = np.array_equal(golden, hw)
        results.append(ok)
        extra = "" if ok else f"  {int(np.sum(golden != hw))} / {golden.size} mismatches"
        print(f"[{'PASS' if ok else 'FAIL'}] {name:26s} {golden.size:6d} px {dt:8.1f} ms{extra}")

    print("\n-- legacy protocol (0x01 / 0x03)")
    for name, ifmap, filt, shift in legacy_cases():
        w = ifmap.shape[0]

        def run_legacy(ifmap=ifmap, filt=filt, shift=shift, w=w):
            acc.configure_legacy(w, shift)
            acc.write_legacy(ifmap.tobytes() + filt.tobytes())
            return acc.run((w - 2) ** 2).reshape(w - 2, w - 2)

        check(name, eh.golden_conv_layer(ifmap[None], filt[None, None], shift=shift)[0], run_legacy)

    print("\n-- layer protocol (0x04 / 0x05 / 0x06)")
    rng = np.random.default_rng(seed=7)
    for name, c, cout, h, w, use_bias, relu, pad in layer_cases():
        x = rng.integers(0, 256, (c, h, w), dtype=np.uint8)
        k = rng.integers(-128, 128, (cout, c, 3, 3)).astype(np.int8)
        b = rng.integers(-50000, 50000, cout) if use_bias else None
        shift = eh.pick_shift(eh.conv_psum(eh.pad_zero_point(x, pad), k, b))
        golden = eh.golden_conv_layer(x, k, b, shift, relu, pad)
        check(name, golden, lambda: acc.conv_layer(x, k, b, shift, relu, pad))

    # Word read-back of the last uploaded ifmap tile
    words = acc.read_words(0, 4)
    ok = words.dtype == np.dtype('<u4') and len(words) == 4
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {'read_words':26s} {[hex(v) for v in words]}")

    # Configs that do not fit must be rejected
    try:
        acc.configure(32, 16, 4, 0)
        ok = False
    except ValueError:
        ok = True
    results.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {'reject oversized layout':26s}")

    acc.close()
    print(f"\n[*] Result: {sum(results)} / {len(results)} passed")
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
