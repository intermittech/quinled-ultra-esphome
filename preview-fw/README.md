# Preview firmware for the QuinLED Dig2Analog-Plus-5ch (rev 0.4)

`alpha.json` is the channel manifest and `quinled-dig2analog-plus-5ch-0.7.3-alpha.3-rev04-update.bin` the data-line
update image it lists (a 64-byte descriptor and the app body, no bootloader). The image needs bootloader 4 on the
board. The ESPHome `update:` entity reads them with `verify_signature: false`: these files carry no signature, so use
them only from your own source.

```yaml
http_request:
  buffer_size_rx: 2048

update:
  - platform: ultra_line
    id: board_fw
    ultra_line_id: ultra
    name: Board firmware
    source: https://raw.githubusercontent.com/intermittech/quinled-ultra-esphome/d2a-plus-preview/preview-fw/{channel}.json
    channel: alpha
    verify_signature: false
    public_keys: test
    inventory:
      - board: 1
        boot_identity: dig2analog_plus_5ch_rev04_bl4

button:
  - platform: template
    name: Install board firmware
    on_press:
      - update.perform: board_fw
```

`board: 1` is the first board after the controller (boards count from 1, like `num_boards`).

The image is 0.7.3-alpha.3: the 0.7.2 app with the pulse-skip rate setting (`pulse_skip_rate`) and the board's own
100 ns minimum pulse. Settings the board stored before stay after the update (a 200 ns minimum pulse from an older
build, too): set `min_pulse: 100ns` on the `ultra_line` to change it.

A board refuses an image whose version is equal to or older than the one it runs. Every later preview image therefore
carries a higher version (0.7.3-alpha.4, ...).
