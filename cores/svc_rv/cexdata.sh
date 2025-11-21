#!/usr/bin/env bash

set -ex

#
# Process counterexample data from a checks directory
#
# Usage: cexdata.sh [checks_dir]
#   checks_dir: Directory containing .sby checks (default: checks)
#
CHECKS_DIR="${1:-checks}"

if [ ! -d "$CHECKS_DIR" ]; then
	echo "Error: Checks directory '$CHECKS_DIR' does not exist"
	exit 1
fi

CEXDATA_DIR="$CHECKS_DIR/cexdata"
rm -rf "$CEXDATA_DIR"
mkdir -p "$CEXDATA_DIR"

#
# Copy failed traces
#
for x in $CHECKS_DIR/*/FAIL; do
	test -f $x || continue
	x=${x%/FAIL}
	y=$(basename $x)
	cp $x/logfile.txt $CEXDATA_DIR/$y.log
	if test -f $x/engine_*/trace.vcd; then
		cp $x/engine_*/trace.vcd $CEXDATA_DIR/$y.vcd
	fi
done

#
# Extract warnings
#
sed -re '/WARNING|[Ww]arning/ ! d; s/^([^:]|:[^ ])*: //;' $CHECKS_DIR/*/logfile.txt | sort -Vu > $CEXDATA_DIR/warnings.txt

#
# Generate status summary
#
for x in $CHECKS_DIR/*.sby; do
	test -f $x || continue
	x=${x%.sby}
	if [ -f $x/PASS ]; then
		printf "%-30s %s %10s\n" $(basename $x) "  pass  " $(sed '/Elapsed process time/ { s/.*\]: //; s/ .*//; p; }; d;' $x/logfile.txt)
	elif [ -f $x/FAIL ]; then
		printf "%-30s %s %10s\n" $(basename $x) "**FAIL**" $(sed '/Elapsed process time/ { s/.*\]: //; s/ .*//; p; }; d;' $x/logfile.txt)
	else
		printf "%-30s %s\n" $(basename $x) unknown
	fi
done | awk '{ print gensub(":", "", "g", $3), $0; }' | sort -n | cut -f2- -d' ' > $CEXDATA_DIR/status.txt
