# Configuration to set defaults for make targets

# Tells the unpack script where it's being run, so it knows whether or not to
# install tx2-specific things.
UNPACK_ENV=dev  # rover|dev

# If true, nothing is built while unpacking (helpful if building the docker
# image when the code contains a compile error).
UNPACK_SKIP_BUILD=true  # true|false

# If true, the build command copies headers and libs to .docker-*-mount/
# directories where they can be seen outside of the docker container, and
# modifies compile_commands.json to point to the copies. Use this if you are
# running your editor outside of the docker container.
BUILD_COPY_HEADERS_AND_LIBS=false  # true|false

# If true, the build command always performs dsdl generation as a prebuild step.
# If false, this step is always skipped. I couldn't find a good way to encode
# this as a proper dependency with Make.
BUILD_GENERATE_DSDL=false  # true|false
