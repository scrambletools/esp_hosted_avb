# esp_hosted_avb — see README.md.
#
# Always exports the wire-format header. The coprocessor-side handler
# is gated on CONFIG_HOSTED_AVB_BUILD_COPROCESSOR_HANDLER so host-side
# consumers don't drag in esp_wifi.

set(srcs "")
if(CONFIG_HOSTED_AVB_BUILD_COPROCESSOR_HANDLER)
  list(APPEND srcs "hosted_avb.c")
endif()

# PRIV_REQUIRES must be unconditional: IDF resolves component deps
# before CONFIG_* is populated, so gating this on Kconfig yields an
# empty list and the build fails with "esp_wifi not in requirements".
# These deps are only actually used when the handler source compiles.
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_wifi log
    WHOLE_ARCHIVE
)
