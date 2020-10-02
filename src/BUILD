load("@rules_cc//cc:defs.bzl","cc_binary")

cc_binary(
    name = "oscc_jscmd2",
    srcs = [
        "main.c",
        "commander.h",
        "commander.c",
        "joystick.h",
        "joystick.c",
        "kia_niro.h",
    ],
    # hdrs = glob(["can_protocols/*.h"]),
    deps = [
        "//api:oscc_lib",
        "@sdl2//:lib",
    ],

    copts = [
        "-Iapi/include/can_protocols",
    ],

    includes = [
        "api/include/can_protocols",
    ],
    # copts = [
    #     "--libs` -lSDL2 -lSDL2_mixer -lSDL2_image -lSDL2_ttf",
    #     # "sdl2-config --cflags --libs` -lSDL2 -lSDL2_mixer -lSDL2_image -lSDL2_ttf"
    # ]
)

cc_binary(
    name = "oscc_jscmd",
    srcs = [
        "main.c",
        "commander.h",
        "commander.c",
        "joystick.h",
        "joystick.c",
        "kia_niro.h",
    ] + glob(["can_protocols/*.h"]),
    # hdrs = glob(["can_protocols/*.h"]),
    deps = [
        "//api:oscc_lib",
        "@sdl2//:lib",
    ],

    copts = [
        "-Iapi/include/can_protocols",
    ],

    includes = [
        "api/include/can_protocols",
    ],
    # copts = [
    #     "--libs` -lSDL2 -lSDL2_mixer -lSDL2_image -lSDL2_ttf",
    #     # "sdl2-config --cflags --libs` -lSDL2 -lSDL2_mixer -lSDL2_image -lSDL2_ttf"
    # ]
)