load("@rules_cc//cc:defs.bzl","cc_library")

cc_library(
    name = "oscc_lib",
    srcs = [
      "src/oscc.c",
      "src/internal/oscc.h",
      ] + glob([
        "include/can_protocols/*.h", 
        "inluces/vehicles/*.h", 
        "include/*.h"
    ]),

    hdrs = ["include/oscc.h"] + glob([
      "include/can_protocols/*.h", 
    ]),

    includes = ["include",
                "include/vehicles",
                "include/can_protocols",
                "src/internal"],
    # defines = [
    #   # "-DVEHICLE=kia_niro"
    # ],
    # local_defines = [
    #   "VEHICLE=kia_niro",
    #   "KIA_NIRO",
    # ],
    copts = [
      "-Iapi/include/",
      "-Iapi/include/vehicles/",
      "-Iinclude/vehicles",
      "-DKIA_NIRO",
      "-DVEHICLE=kia_niro",
    ],

    visibility = ["//src:__pkg__"],
)