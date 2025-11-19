.PHONY: checks check clean summary

checks:
	python3 ../../checks/genchecks.py
	$(MAKE) -C checks

check: checks
	bash cexdata.sh
	cat cexdata/warnings.txt
	cat cexdata/status.txt

summary:
	bash cexdata.sh
	@echo "=== Warnings ==="
	@cat cexdata/warnings.txt
	@echo ""
	@echo "=== Status ==="
	@cat cexdata/status.txt

clean:
	rm -rf checks/ cexdata/
