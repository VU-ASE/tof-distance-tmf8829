# Makefile in accordance with the docs on git management (to use in combination with meta)
.PHONY: build start clean test fetch-roverlib-c edit-headers build-docker

BUILD_DIR=bin/
BINARY_NAME=tof

# If not using VSCode's devcontainers, with docker installed you can run this command to
# build the service inside the container.
build-docker:
	docker build -t ase-service-c -f .devcontainer/Dockerfile .
	docker run -it --cap-add=SYS_PTRACE \
		--security-opt seccomp=unconfined \
		--privileged \
		--user=dev:dev \
		-v "`pwd`":/workspaces/work \
		-w /workspaces/work \
		ase-service-c bash -ic 'make build -C /workspaces/work'

# This target clones roverlib-c so that it can be compiled alongside the service
fetch-roverlib-c:
	@echo "Removing existing roverlib-c and cloning fresh copy..."
	@rm -rf lib
	@git clone https://github.com/VU-ASE/roverlib-c.git lib

# The following target changes a single line in two header files because of an annoying bug
# where the system installation of hashtable for C took priority over the locally installed one
edit-headers:
	@echo "> editing ./lib/include/roverlib/bootinfo.h"
	@echo "     changing line: '#include <hashtable.h>'"
	@echo "     to:            '#include \"/usr/local/include/hashtable.h\"'"
	@sed -i "s/#include <hashtable.h>/#include \"\/usr\/local\/include\/hashtable.h\"/" ./lib/include/roverlib/bootinfo.h

	@echo "> editing ./lib/include/roverlib/configuration.h"
	@echo "     changing line: '#include <hashtable.h>'"
	@echo "     to:            '#include \"/usr/local/include/hashtable.h\"'"
	@sed -i "s/#include <hashtable.h>/#include \"\/usr\/local\/include\/hashtable.h\"/" ./lib/include/roverlib/configuration.h

build: fetch-roverlib-c edit-headers
	@mkdir -p build
	@mkdir -p build/obj
	@mkdir -p build/obj/rovercom/outputs
	@mkdir -p build/obj/rovercom/tuning
	@mkdir -p build/obj/service
	@mkdir -p $(BUILD_DIR)

	# Compile roverlib-c files
	@for file in ./lib/src/*.c; do \
		basename=$$(basename $$file); \
		gcc -c -fPIC -o ./build/obj/$${basename%.c}.o $$file \
		-I/usr/include/cjson -I./lib/include -g; \
	done

	# Compile rovercom/outputs/*.c files
	@for file in ./lib/src/rovercom/outputs/*.c; do \
		basename=$$(basename $$file); \
		gcc -c -fPIC -o ./build/obj/rovercom/outputs/$${basename%.c}.o $$file \
		-I/usr/include/cjson -I./lib/include -g; \
	done

	# Compile rovercom/tuning/*.c files
	@for file in ./lib/src/rovercom/tuning/*.c; do \
		basename=$$(basename $$file); \
		gcc -c -fPIC -o ./build/obj/rovercom/tuning/$${basename%.c}.o $$file \
		-I/usr/include/cjson -I./lib/include -g; \
	done

	# Create static library from all roverlib .o files
	@ar rcs ./build/librover.a ./build/obj/*.o ./build/obj/rovercom/outputs/*.o ./build/obj/rovercom/tuning/*.o

	# Compile TMF8829 service files
	@for file in ./src/*.c; do \
		basename=$$(basename $$file); \
		gcc -c -fPIC -Wall -Wextra -O2 \
		-DENABLE_JSON_LOGGING -DENABLE_HISTOGRAM -DENABLE_KEYSTONE \
		-o ./build/obj/service/$${basename%.c}.o $$file \
		-I/usr/include/cjson -I./lib/include -I./src -g; \
	done

	# Link final C binary
	@gcc -o $(BUILD_DIR)$(BINARY_NAME) \
		./build/obj/service/*.o ./build/librover.a \
		-lm -lpthread -lz -lcjson -lzmq -lprotobuf-c -lhashtable -llist \
		-I/usr/include/cjson -I./lib/include -I./src -g

start: build
	@echo "starting ${BINARY_NAME}"
	# Add /usr/local/lib as a possible location for shared libraries, fix for when
	# working in a devcontainer
	@LD_LIBRARY_PATH=/usr/local/lib:$$LD_LIBRARY_PATH ./${BUILD_DIR}${BINARY_NAME}

debug: build
	@echo "Starting program in debug mode with injected bootspec"
	@ASE_SERVICE=$$(cat boot.json) && export ASE_SERVICE && echo "Starting program" && gdb ./${BUILD_DIR}${BINARY_NAME}

clean:
	@echo "Cleaning all targets for ${BINARY_NAME}"
	rm -rf $(BUILD_DIR)
	rm -rf build
	rm -rf lib

test:
	@echo "No tests configured"