load("@rules_cc//cc:defs.bzl","cc_binary")
load("//:shared_variables.bzl", "COPTS")

cc_binary(
    name = "niro_jscmd",
    srcs = [
        "niro_jscmd.cc",
    ],

    deps = [
        "//core:oscc_lib",
        "//joy:joy_lib",
        "@sdl2//:lib",
    ],

    copts = COPTS + [
        "-Icore/include",
        "-Ijoy/include",
        "-DKIA_NIRO",
        "-DVEHICLE=kia_niro",
    ],
)
