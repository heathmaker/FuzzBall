#!/bin/bash
# Compiles network.nod.xml/network.edg.xml into the net.xml SUMO actually
# loads. Deterministic and near-instant, so it isn't checked in -- run
# this once (or any time the .nod.xml/.edg.xml source files change)
# before running examples/sumo_acc.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
netconvert --node-files=network.nod.xml --edge-files=network.edg.xml -o net.xml
echo "Wrote $SCRIPT_DIR/net.xml"
