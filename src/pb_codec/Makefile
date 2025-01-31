# generate protobuf codec

all: high_to_low.pb.c low_to_high.pb.c

high_to_low.pb.c: high_to_low.proto
	@echo build high_to_low protobuf
	@nanopb_generator.py -L "#include <nanopb/%s>" -I . -D . high_to_low.proto

low_to_high.pb.c: low_to_high.proto
	@echo build low_to_hight protobuf
	@nanopb_generator.py -L "#include <nanopb/%s>" -I . -D . low_to_high.proto
