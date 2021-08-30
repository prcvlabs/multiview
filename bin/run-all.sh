#!/bin/bash

PPWD="$(cd "$(dirname "$0")" ; pwd)"
"$PPWD/all-testcases.sh" --fps 15

