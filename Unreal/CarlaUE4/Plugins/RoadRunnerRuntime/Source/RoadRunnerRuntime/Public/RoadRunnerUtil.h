// Copyright 2022 The MathWorks, Inc.
#pragma once

#include "Runtime/Launch/Resources/Version.h"

#define RR_VERSION_UP_TO(major, minor) \
(ENGINE_MAJOR_VERSION < major || (ENGINE_MAJOR_VERSION == major && ENGINE_MINOR_VERSION <= minor))

#define RR_VERSION_STARTING_AT(major, minor) \
(ENGINE_MAJOR_VERSION > major || (ENGINE_MAJOR_VERSION == major && ENGINE_MINOR_VERSION >= minor))
