#!/bin/bash

name=""
while read line
do
	case "$line" in
		"diff --git"*)
			name=$(echo "$line" | sed "s#.* b/##");echo "start : $line -> $name"
			mkdir -p $(dirname "$name")
			;;
		*) echo "$line">>"$name"
	esac
done <diff_all.txt
