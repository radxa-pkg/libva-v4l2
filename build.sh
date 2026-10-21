#!/bin/sh
# Build libva-v4l2 and collect the Debian packages in ./output.
# Usage: ./build.sh [extra dpkg-buildpackage flags]
set -e
cd "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"

if ! missing=$(dpkg-checkbuilddeps 2>&1 >/dev/null); then
    echo "missing build dependencies:" >&2
    echo "  $missing" >&2
    echo "install them with:" >&2
    echo "  sudo apt install build-essential debhelper meson ninja-build pkg-config \\" >&2
    echo "    libva-dev libdrm-dev libegl1-mesa-dev libgles-dev libgbm-dev" >&2
    exit 1
fi

dpkg-buildpackage -us -uc -b "$@"

mkdir -p output
mv -f ../libva-v4l2_*.deb ../libva-v4l2_*.buildinfo ../libva-v4l2_*.changes output/
echo "packages written to output/:"
ls -l output/
