.DEFAULT_GOAL := build

ifdef release 
	RELEASE=--release
endif

ifdef run 
	MODE=run
else
	MODE=build
endif

build:
	make release=1 native

native:
	cargo $(MODE) $(RELEASE)

pi: 
	cargo $(MODE) --target armv7-unknown-linux-gnueabihf $(RELEASE) --features "pi"
