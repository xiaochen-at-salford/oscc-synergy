load("@rules_cc//cc:defs.bzl","cc_binary")

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

    copts = [
        "-DKIA_NIRO",
        "-DVEHICLE=kia_niro",
        "-Icore/include",
        "-Icore/include/can_protocols",
        "-Icore/include/vehicles",
    ],
)
