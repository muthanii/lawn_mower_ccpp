#!/usr/bin/env bash
# Gives the container its own SSH key for GitHub. ~/.ssh is a Docker volume,
# so the key is made one time and kept across rebuilds. Safe to run again.
set -euo pipefail

key=~/.ssh/id_ed25519
chmod 700 ~/.ssh

if [ ! -f "$key" ]; then
    ssh-keygen -t ed25519 -N "" -C "$(whoami)@ros2-devcontainer" -f "$key" -q
    echo "Made a new SSH key at $key"
fi

# Without a gh login, `gh auth login` (Git protocol: SSH) offers to upload the key
if ! gh auth status > /dev/null 2>&1; then
    echo "Run 'gh auth login' and select SSH to add this key to GitHub."
    exit 0
fi

# GitHub's host keys, read over HTTPS, so the first `ssh` does not ask to trust the host
if ! grep -q '^github.com ' ~/.ssh/known_hosts 2> /dev/null; then
    gh api meta --jq '.ssh_keys[] | "github.com " + .' >> ~/.ssh/known_hosts
fi

pub_key=$(cut -d' ' -f2 "$key.pub")
if ! gh ssh-key list 2> /dev/null | grep -qF "$pub_key"; then
    gh ssh-key add "$key.pub" --title "ros2-devcontainer"
fi
