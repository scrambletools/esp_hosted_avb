# esp_hosted_avb — see README.md.
#
# Always exports the wire-format header. The coprocessor-side handler
# is gated on CONFIG_HOSTED_AVB_BUILD_COPROCESSOR_HANDLER so host-side
# consumers don't drag in esp_wifi. The peer data calls build wherever
# esp-hosted peer data is on, host or coprocessor.

set(srcs "")
if(CONFIG_HOSTED_AVB_BUILD_COPROCESSOR_HANDLER)
  list(APPEND srcs "hosted_avb.c")
endif()
if(CONFIG_ESP_HOSTED_HOST_FEAT_PEER_DATA OR CONFIG_ESP_HOSTED_CP_FEAT_PEER_DATA)
  list(APPEND srcs "hosted_avb_peer.c")
endif()
if(CONFIG_ESP_HOSTED_HOST)
  list(APPEND srcs "hosted_avb_coprocessor_fw.c")
endif()
if(CONFIG_HOSTED_AVB_CLOCK_LINK)
  list(APPEND srcs "hosted_avb_clock_link.c")
endif()
if(CONFIG_HOSTED_AVB_WIFI_TIME)
  list(APPEND srcs "hosted_avb_wifi_time.c")
endif()

# PRIV_REQUIRES must be unconditional: IDF resolves component deps
# before CONFIG_* is populated, so gating this on Kconfig yields an
# empty list and the build fails with "esp_wifi not in requirements".
# These deps are only actually used when the handler source compiles.
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_wifi esp_event log esp_app_format bootloader_support
                  esp_driver_gpio esp_driver_gptimer esp_hw_support esp_timer
    WHOLE_ARCHIVE
)

# esp-hosted's headers, on the host and the coprocessor; a station has
# none.
if(srcs AND NOT CMAKE_BUILD_EARLY_EXPANSION)
  idf_build_get_property(build_components BUILD_COMPONENTS)
  if(espressif__esp_hosted IN_LIST build_components)
    target_link_libraries(${COMPONENT_LIB} PRIVATE idf::espressif__esp_hosted)
  endif()
endif()

# The clock link brackets register reads between two latches of its
# timer under the timer driver's own lock; this file, built into the
# GPTimer driver's library, can reach the driver's private handle.
if(CONFIG_HOSTED_AVB_CLOCK_LINK)
  idf_component_get_property(hosted_avb_gptimer_lib esp_driver_gptimer COMPONENT_LIB)
  target_sources(${hosted_avb_gptimer_lib} PRIVATE
                 "${CMAKE_CURRENT_LIST_DIR}/hosted_avb_timer_bracket.c")
endif()
