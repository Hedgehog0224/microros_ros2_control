#!/bin/bash
pgrep -f "ros" | awk '{print "kill -9 " $1}' | sh