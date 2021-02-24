# Configuration to set defaults for make targets

# Tells the unpack script where it's being run, so it knows whether or not to
# install tx2-specific things.
UNPACK_ENV=dev  # rover|dev

# If true, nothing is built while unpacking (helpful if building the docker
# image when the code contains a compile error).
UNPACK_SKIP_BUILD=false  # true|false

# If true, the build command copies headers and libs to .docker-*-mount/
# directories where they can be seen outside of the docker container, and
# modifies compile_commands.json to point to the copies. Use this if you are
# running your editor outside of the docker container.
BUILD_COPY_HEADERS_AND_LIBS=false  # true|false

# If true, the build command always performs DSDL generation as a prebuild step.
# If false, this step is always skipped. This is helpful if you will be changing
# the DSDLs a lot and don't want to worry about running `make generate-dsdl`
# every time, but is quite slow and so is disabled by default.
BUILD_GENERATE_DSDL=false  # true|false
