#
# Detect this Makefile's directory
#
TOP := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

#
# Ensure all commands run in TOP directory
#
SHELL := $(shell which bash)
.ONESHELL:
.SHELLFLAGS := -euc

.PHONY: all config configs gencheck checkall checks clean list
.SILENT:

#
# Default target: show available configurations
#
all: list

#
# Generate all .cfg files for different configurations
#
config: generate_configs.sh
	cd $(TOP)
	./generate_configs.sh > /dev/null

configs: config

#
# Generate and run checks for a specific configuration
# Usage: make icache_check, make dcache_check, etc.
#
%_check: config
	cd $(TOP)
	python3 ../../checks/genchecks.py $*_checks
	$(MAKE) -k -C $*_checks

#
# Generate checks for all configurations
#
gencheck: config
	cd $(TOP)
	for cfg in *_checks.cfg; do
		name=$${cfg%.cfg}
		echo "Generating checks for $$name..."
		python3 ../../checks/genchecks.py $$name
	done

#
# Run all generated check directories
#
checkall: gencheck
	cd $(TOP)
	for dir in *_checks/; do
		echo ""
		echo "=== Running $$dir ==="
		$(MAKE) -C $$dir || exit 1
	done

%_summary:
	cd $(TOP)
	bash cexdata.sh $*_checks 2> /dev/null
	echo "=== Configuration: $* ==="
	echo "=== Warnings ==="
	cat $*_checks/cexdata/warnings.txt
	echo ""
	echo "=== Status ==="
	cat $*_checks/cexdata/status.txt

clean:
	cd $(TOP)
	rm -rf *_checks/ *_checks.cfg

list: config
	cd $(TOP)
	echo "# Available configurations"
	for cfg in *_checks.cfg; do
		name=$${cfg%_checks.cfg}
		echo "make $${name}_check"
	done
