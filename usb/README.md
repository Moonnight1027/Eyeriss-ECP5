# USB CDC IP (third-party)

These files are **not** part of the original work in this repository.
They come from open-source projects licensed under Apache-2.0 (see `LICENSE` in this folder):

- Luke Valenti, [TinyFPGA-Bootloader](https://github.com/tinyfpga/TinyFPGA-Bootloader) — USB full-speed protocol engine
- Lawrie Griffiths / David Williams, [tinyfpga_bx_usbserial](https://github.com/davidthings/tinyfpga_bx_usbserial) — USB CDC (serial) adaptation

Changes made for this project:

- `usb_uart_ecp5.v`: ECP5 wrapper exposing a plain valid/ready byte interface (`usb_uart_np`).
- `usb_uart_bridge_ep.v`: sends a zero-length packet when a bulk IN transfer ends on a
  full 32-byte packet. Without it, Windows `usbser.sys` never completes reads whose length
  is a multiple of 32 bytes. Covered by `sim/tb_zlp.sv`.
