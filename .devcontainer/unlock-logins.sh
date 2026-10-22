#!/usr/bin/env bash
# Unlocks the Claude Code login, the gh login and the SSH key. They are kept
# encrypted with gocryptfs in a Docker volume (~/.vault-cipher) and show as
# plain files in ~/.vault only while it is unlocked. ~/.claude, ~/.config/gh
# and ~/.ssh link into ~/.vault. ~/.bashrc runs this in the first terminal
# after each start. Safe to run again.
set -euo pipefail

cipher=~/.vault-cipher
plain=~/.vault
legacy=~/.vault-legacy

if ! mountpoint -q "$cipher"; then
    echo "No vault volume at $cipher (Codespaces?). Nothing to unlock."
    exit 0
fi

if ! mountpoint -q "$plain"; then
    if [ ! -f "$cipher/gocryptfs.conf" ]; then
        echo "Pick a passphrase for your logins. If you forget it, delete the vault and sign in again."
        gocryptfs -init -q "$cipher"
    fi
    # gocryptfs refuses a mount point with files in it, so nothing is hidden under the mount
    for try in 1 2 3; do
        echo "Passphrase to unlock the Claude Code and gh logins:"
        if gocryptfs -q -nosyslog "$cipher" "$plain"; then
            break
        fi
        [ "$try" -lt 3 ] || { echo "Logins stay locked. Run 'bash $0' to try again."; exit 1; }
    done
fi

# link PATH NAME: points PATH at ~/.vault/NAME and moves in any files already at PATH
link() {
    local path=$1 target=$plain/$2
    mkdir -p "$target"
    if [ -L "$path" ]; then
        ln -sfn "$target" "$path"
        return
    fi
    if mountpoint -q "$path"; then
        echo "$path is still a volume mount. Rebuild the container first." >&2
        exit 1
    fi
    if [ -d "$path" ]; then
        cp -a --update=none "$path/." "$target/"
        rm -rf "$path"
    fi
    mkdir -p "$(dirname "$path")"
    ln -s "$target" "$path"
}

link ~/.claude claude
link ~/.config/gh gh
link ~/.ssh ssh
chmod 700 "$plain/ssh"

# One time: moves the logins from the old plain-text volumes into the vault, then empties them
for name in claude gh ssh; do
    old=$legacy/$name
    if mountpoint -q "$old" && [ -n "$(ls -A "$old" 2> /dev/null)" ]; then
        cp -a "$old/." "$plain/$name/"
        find "$old" -mindepth 1 -delete
        echo "Moved the old $name files into the vault and emptied the plain-text volume."
    fi
done

bash "$(dirname "$0")/setup-ssh.sh"
