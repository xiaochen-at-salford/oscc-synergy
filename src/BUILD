load("@rules_cc//cc:defs.bzl","cc_binary")
load("//:shared_variables.bzl", "COPTS")

cc_binary(
    name = "oscc_jscmd",
    srcs = [
        "main.c",
        "commander.h",
        "commander.c",
        "joystick.h",
        "joystick.c",
    ],

    deps = [
        "//core:oscc_lib",
        "@sdl2//:lib",
    ],

    copts = COPTS + [
        "-Icore/include",
        "-Icore/include/can_protocols",
        "-Icore/include/vehicles",
    ],
)
