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

#
# Default target: show available configurations
#
all: list

#
# Generate all .cfg files for different configurations
#
config: generate_configs.sh
	@cd $(TOP)
	@./generate_configs.sh > /dev/null

configs: config

#
# Generate and run checks for the baseline configuration (sram_fwd)
#
checks: config
	cd $(TOP)
	python3 ../../checks/genchecks.py checks_sram_fwd
	$(MAKE) -C checks_sram_fwd

#
# Generate and run checks for a specific configuration
# Usage: make sram_fwd_check, make sram_sc_check, etc.
#
%_check: config
	cd $(TOP)
	python3 ../../checks/genchecks.py checks_$*
	$(MAKE) -C checks_$*

#
# Generate checks for all configurations
#
gencheck: config
	cd $(TOP)
	for cfg in checks_*.cfg; do
		name=$${cfg%.cfg}
		echo "Generating checks for $$name..."
		python3 ../../checks/genchecks.py $$name
	done

#
# Run all generated check directories
#
checkall: gencheck
	cd $(TOP)
	for dir in checks_*/; do
		echo ""
		echo "=== Running $$dir ==="
		$(MAKE) -C $$dir || exit 1
	done

%_summary:
	cd $(TOP)
	bash cexdata.sh checks_$*
	echo "=== Configuration: $* ==="
	echo "=== Warnings ==="
	cat checks_$*/cexdata/warnings.txt
	echo ""
	echo "=== Status ==="
	cat checks_$*/cexdata/status.txt

clean:
	cd $(TOP)
	rm -rf checks_*/ checks_*.cfg

list: config
	@cd $(TOP)
	@echo "# Available configurations"
	@for cfg in checks_*.cfg; do
		name=$${cfg#checks_}
		name=$${name%.cfg}
		echo "make $${name}_check"
	done
