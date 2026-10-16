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

# PRIV_REQUIRES must be unconditional: IDF resolves component deps
# before CONFIG_* is populated, so gating this on Kconfig yields an
# empty list and the build fails with "esp_wifi not in requirements".
# These deps are only actually used when the handler source compiles.
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_wifi esp_event log
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
