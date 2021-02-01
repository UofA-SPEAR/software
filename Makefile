.PHONY: build
build:
	./build.bash

.PHONY: unpack
unpack:
	./unpack.sh dev

.PHONY: rostest
rostest:
	for filepath in tests/test/*.test; do \
		filename=$$(basename $$filepath); \
		rostest tests $$filename; \
	done
