#!/usr/bin/env bash
set -e
shopt -s globstar

script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$script_dir/../data"

for file in **/*.db3; do
    echo -n "$file	"
    sqlite3 $file 'SELECT count(*) FROM messages;'
done
