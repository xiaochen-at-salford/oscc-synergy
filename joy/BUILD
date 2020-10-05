load("@rules_cc//cc:defs.bzl","cc_library")
load("//:shared_variables.bzl", "COPTS")
package(default_visibility = ["//visibility:public"])  

cc_library(
    name = "joy_lib",
    srcs = [
        # "niro_jscmd.cc",
        "src/commander.cc",
        "src/joystick.cc",
    ],

    hdrs = [
        "include/joystick.h",
        "include/commander.h",
    ],

    deps = [
        "//core:oscc_lib",
        "@sdl2//:lib",
    ],

    copts = COPTS + [
        "-Icore/include",
        # "-Icore/include/can_protocols",
        # "-Icore/include/vehicles",
        "-Ijoy/include",
    ],
)
