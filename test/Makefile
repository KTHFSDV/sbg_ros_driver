.PHONY: run_all tests

LAUNCH_DIR := ./launch

tests := $(wildcard $(LAUNCH_DIR)/*)

run_all: tests 


run_all:
	$(foreach test, $(tests), rostest $(test);)




