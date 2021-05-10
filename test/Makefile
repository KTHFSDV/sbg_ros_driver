.PHONY: run_tests

tests := $(wildcard $(shell rospack find test)/launch/*.test)

run_tests:
	$(foreach test, $(tests), rostest $(test) || exit 1;)
