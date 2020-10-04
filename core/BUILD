load("@rules_cc//cc:defs.bzl","cc_library")
load('//:shared_variables.bzl', "COPTS")
package(default_visibility = ["//visibility:public"])  

cc_library(
    name = "oscc_lib",

    srcs = [
        "src/oscc.cc",
        "src/internal/oscc.h",
    ],

    hdrs = glob([
        "include/*.h",
        "include/can_protocols/*.h",
        "include/vehicles/*.h", 
    ]),

    copts = COPTS + [
        "-Icore/include",
        "-Icore/include/can_protocols",
        "-Icore/include/vehicles",
    ],
)