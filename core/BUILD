load("@rules_cc//cc:defs.bzl","cc_library")
package(default_visibility = ["//visibility:public"])  

cc_library(
    name = "oscc_lib",

    srcs = [
        "src/oscc.c",
        "src/internal/oscc.h",
    ],

    hdrs = glob([
        "include/*.h",
        "include/can_protocols/*.h",
        "include/vehicles/*.h", 
    ]),

    copts = [
      "-DKIA_NIRO",
      "-DVEHICLE=kia_niro",
      "-Icore/include",
      "-Icore/include/can_protocols",
      "-Icore/include/vehicles",
    ],
)