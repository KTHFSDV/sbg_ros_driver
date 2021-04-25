.PHONY: run_all tests

LAUNCH_DIR := ./launch

tests := $(wildcard $(LAUNCH_DIR)/*)

run_tests: tests 


run_tests:
	$(foreach test, $(tests), rostest $(test);)




