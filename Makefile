PROTOC := ./nanopb-0.3.5/generator-bin/protoc.exe
COPY := cp
PROTO_DIR := proto
PROTO_FILES := $(wildcard $(PROTO_DIR)/*.proto)
COMPILED_PROTO_FILES := $(wildcard $(PROTO_DIR)/proto/*)

default: protoc
# Build rule for the main program

protoc: 
	git submodule update --remote --merge
	$(foreach proto,$(PROTO_FILES), $(PROTOC) --nanopb_out=./proto ./$(proto); )
	$(foreach proto,$(COMPILED_PROTO_FILES), $(COPY) $(proto)  $(shell pwd;); ) 
