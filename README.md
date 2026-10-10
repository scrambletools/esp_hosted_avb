# PTP custom-RPC over ESP-Hosted

A tiny additive RPC channel that lets a host MCU running gPTP issue
Wi-Fi-radio operations on a coprocessor over the ESP-Hosted SDIO/SPI
link, without forking ESP-Hosted itself.

## Why

In a P4+C6 split (the canonical AVB-over-Wi-Fi topology: ESP32-P4 host
runs Ethernet + gPTP, ESP32-C6 coprocessor owns the Wi-Fi radio), the
host needs to drive a few Wi-Fi calls on the coprocessor side that
ESP-Hosted's upstream RPC catalogue does not expose. The most pressing
one is `esp_wifi_set_vendor_ie()` — required to publish IEEE
802.1AS-2020 §12.7 `FollowUpInformation` in the AP's beacon Vendor IE.

The host's local `libnet80211` has no radio behind it, so calling
`esp_wifi_set_vendor_ie()` from host code is a silent no-op — beacons
go out with zero Vendor IE bytes from our OUI.

Rather than forking ESP-Hosted, this component uses the upstream's
own additive escape hatch: the **peer_data_transfer / Custom RPC**
framework. The host sends a packed buffer via
`esp_hosted_send_custom_data(msg_id, ...)`; the coprocessor's
registered callback unpacks and dispatches to the matching
`esp_wifi_*` call locally on the C6. Managed-component sources are
untouched.

## How it works

```
   Host (P4) — gPTP, AVB stack          Coprocessor (C6) — Wi-Fi radio
   ────────────────────────────         ──────────────────────────────
   esp_hosted_send_custom_data(    →    set_vendor_ie_callback() →
     PTP_RPC_MSG_SET_VENDOR_IE_REQ,       esp_wifi_set_vendor_ie(...)
     packed buf)                          [installs IE in beacons]
                                          ↓
   handler reads ack ← PTP_RPC_MSG_SET_VENDOR_IE_ACK (esp_err_t)
```

Wire format is defined in `include/ptp_rpc_proto.h` (one short header,
both sides include it). The coprocessor-side handler in
`ptp_custom_rpc.c` registers itself at startup via a `__attribute__
((constructor))` so the coprocessor application does not need to call
`ptp_custom_rpc_init()` from `app_main` — it just works once this
component is linked in.

## Messages implemented

| ID | Name | Direction | Payload |
| --- | --- | --- | --- |
| `0x50545001` | `PTP_RPC_MSG_SET_VENDOR_IE_REQ` | host → coprocessor | 4-byte header + full `vendor_ie_data_t` buffer |
| `0x50545002` | `PTP_RPC_MSG_SET_VENDOR_IE_ACK` | coprocessor → host | `int32_t esp_err_t` |

`0x5054_5000` (`"PTP\0"`) is the message-ID base; the low byte is the
opcode. Reserved for future expansion (e.g. FTM-frame Vendor IE
install when Espressif exposes that path).

## Building

### Host side

Add the dependency to your project's `main/idf_component.yml`:

```yaml
dependencies:
  scrambletools/esp_ptp_rpc: "*"
```

Then `#include "ptp_rpc_proto.h"` and call
`esp_hosted_send_custom_data()` with the message IDs above. Most
host-side consumers will not call this component directly —
`esp_ptp/ptp_beacon_ie.c` is the canonical user and handles the host
side of the protocol on behalf of the AVB stack.

### Coprocessor side

The coprocessor build also adds this component (e.g. via a stub
manifest in `components/`) and additionally enables the handler:

```
CONFIG_PTP_RPC_BUILD_COPROCESSOR_HANDLER=y
CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER=y
```

The handler self-registers at startup; the coprocessor app does not
need to know about this component.

## Status

Vendor-IE install is live and over-air-verified. Future
opcodes will land here as the AVB-over-Wi-Fi plan needs them; the
wire format is versioned only via opcode additions to keep host and
coprocessor sides loosely coupled.

## License

MIT.
