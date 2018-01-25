all: serial_utils

install:
	cd build && make install

uninstall:
	cd build && make uninstall

serial_utils:
	@mkdir -p build
	-mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif

clean:
	-cd build && make clean
	rm -rf build bin lib

.PHONY: doc
doc:
	@doxygen doc/Doxyfile
ifeq ($(UNAME),Darwin)
	@open doc/html/index.html
endif

.PHONY: test
test:
	@mkdir -p build
	@mkdir -p bin
	cd build && cmake $(CMAKE_FLAGS) -DSERIAL_UTILS_BUILD_TESTS=1 -DSERIAL_UTILS_BUILD_EXAMPLES=1 ..
ifneq ($(MAKE),)
	cd build && $(MAKE)
else
	cd build && make
endif
	cd bin && ./serial_listener_tests