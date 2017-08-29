#!/bin/bash

repos=( "$@" )
valid_repos=( )

if (( ${#repos[@]} < 1 )); then
    echo "Not enough arguments" >&2
    exit 1
fi

if (( ${#repos[@]} == 1 )); then
    repos=( "${repos[0]}" ${repos[0]}/* )
fi

for repo in ${repos[@]}; do
    if [[ -d ${repo}/.git ]]; then
        valid_repos=( "${valid_repos[@]}" "$repo" )
    fi
done

if (( ${#valid_repos[@]} == 0 )); then
    echo "No git repo found" >&2
    exit 1
fi

for repo in ${valid_repos[@]}; do
    echo "> $( basename $repo )"
    ( cd $repo && git pull origin )
done
