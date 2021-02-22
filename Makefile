scripts_dir=scripts

.PHONY: all
all: build

#### Stuff to build and run the docker image

.PHONY: build-docker
build-docker:
	docker-compose build

.PHONY: run-docker
run-docker:
	docker-compose run spear

.PHONY: run-docker-with-can
run-docker-with-can:
	./$(scripts_dir)/docker-with-can.bash

.PHONY: run-docker-with-vcan
run-docker-with-vcan:
	./$(scripts_dir)/docker-with-can.bash --vcan

#### Stuff to run inside the docker container (or on rover)

# Compilation

.PHONY: build
build:
	./$(scripts_dir)/build.bash

# Testing

.PHONY: rostest
rostest:
	for filepath in tests/test/*.test; do \
		filename=$$(basename $$filepath); \
		rostest tests $$filename; \
	done

# Unpacking, bootstrapping, etc.

.PHONY: init-submodules
init-submodules:
	git submodule update --init --recursive

.PHONY: generate-dsdl
generate-dsdl: init-submodules
	./$(scripts_dir)/generate_dsdl.sh

.PHONY: unpack
unpack: generate-dsdl init-submodules
	./$(scripts_dir)/unpack.sh dev

.PHONY: unpack-rover
unpack-rover: generate_dsdl
	./$(scripts_dir)/unpack.sh rover

# Stuff for initializing CAN

.PHONY: setup-can
setup-can:
	./$(scripts_dir)/setup-can.bash

.PHONY: setup-vcan
setup-vcan:
	./$(scripts_dir)/setup-vcan.bash

# Linting

.PHONY: lint-catkin
lint-catkin:
	./$(scripts_dir)/catkin_lint.bash

.PHONY: lint-xml
lint-xml:
	./$(scripts_dir)/xml_lint.bash
