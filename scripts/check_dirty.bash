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
    res="$( cd $repo && \
        [[ -z $( git status --short ) ]] && \
        echo true || echo false )"
    unpushed_changes="$( cd $repo && \
        [[ -n $( git status | \
            grep 'ahead of [^ ]* by [0-9]* commit' ) ]] && \
        echo true || echo false )"
    if $res; then
        echo -n -e "\e[32mClean\e[0m"
    else
        echo -n -e "\e[31mDirty\e[0m"
    fi
    if $unpushed_changes; then
        echo -n -e " \e[31m(Unpushed commits)\e[0m"
    fi
    echo
done
