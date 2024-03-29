TARGET_HOST ?= guppy
TARGET_USERNAME ?= $$USER
TARGET_HOST_USER ?= $(TARGET_USERNAME)@$(TARGET_HOST)

REMOTE_DIRECTORY ?= ~
DEB_BUILD_PATH ?= target/debian/guppy_*.deb

TARGET_ARCH := aarch64-unknown-linux-musl
RELEASE_BINARY_PATH := target/release/guppy
RELEASE_CROSS_BINARY_PATH := ./target/${TARGET_ARCH}/release/guppy

TARGET_PATH := ~/src/guppy_rust/

.PHONY: build
build:
	cargo build \
		--release \
		--no-default-features \
		--bin guppy

.PHONY: deploy-binary
deploy-binary: build
	rsync -c ${RELEASE_BINARY_PATH} ${TARGET_HOST}:${TARGET_PATH}

.PHONY: build-deb
build-deb: build
	cargo deb --no-build

.PHONE: install
install: build-deb
	sudo dpkg -i $(DEB_BUILD_PATH)

.PHONY: install-dependencies
install-dependencies:
	sudo apt update && sudo apt install libasound2-dev libudev-dev liblzma-dev libclang-dev protobuf-compiler -y
	cargo install cargo-deb

.PHONY: build-docker
build-docker:
	rm -rf docker_out
	mkdir docker_out
	DOCKER_BUILDKIT=1 docker build --tag guppy-builder --file Dockerfile --output type=local,dest=docker_out .

.PHONY: push-docker
push-docker: build-docker
	rsync -avz --delete docker_out/* $(TARGET_HOST_USER):/home/$(TARGET_USERNAME)/guppy/
	rsync -avz --delete scripts/add_udev_rules $(TARGET_HOST_USER):/home/$(TARGET_USERNAME)/guppy/guppy_add_udev_rules
	rsync -avz --delete scripts/install_wait_loop $(TARGET_HOST_USER):/home/$(TARGET_USERNAME)/guppy/install_wait_loop

.PHONY: deploy-docker
deploy-docker: push-docker
	@echo "Installing guppy on $(TARGET_HOST)"
	mosquitto_pub -h homepi -t "guppy/build" -n
	mosquitto_pub -h homepi -t 'home_speak/say/eleven/voice/Natasha' -m "Guppy deployed"
