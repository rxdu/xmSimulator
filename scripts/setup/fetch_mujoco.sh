#!/usr/bin/env bash
#
# Fetch a prebuilt MuJoCo release into third_party/mujoco (gitignored).
#
# MuJoCo is Apache-2.0; xmSimulator owns the family MuJoCo dependency (host-x86 sim).
# it is NOT a submodule — the sim is host-x86 only, never cross-compiled to the
# armv7 base target, so the .so is fetched on demand rather than vendored in git.
#
#   scripts/setup/fetch_mujoco.sh [VERSION]   (default 3.9.0; linux x86_64/aarch64, macos universal2)
#
# Mirrors legged_controller/scripts/setup/fetch_mujoco.sh so the family uses one
# MuJoCo version.
#
# Copyright (c) 2026 Ruixiang Du (rdu)
set -euo pipefail

VERSION="${1:-3.9.0}"
OS="$(uname -s)"
ARCH="$(uname -m)"

REPO_ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
DEST="${REPO_ROOT}/third_party/mujoco"

if [ -f "${DEST}/include/mujoco/mujoco.h" ]; then
  echo "fetch_mujoco: MuJoCo already present at ${DEST} — nothing to do."
  echo "             (delete the directory to re-fetch.)"
  exit 0
fi

BASE_URL="https://github.com/google-deepmind/mujoco/releases/download/${VERSION}"

case "$OS" in
  Linux)
    case "$ARCH" in
      x86_64)  MJARCH="x86_64" ;;
      aarch64) MJARCH="aarch64" ;;
      *) echo "fetch_mujoco: unsupported arch '$ARCH' (need x86_64 or aarch64)" >&2; exit 1 ;;
    esac
    echo "fetch_mujoco: downloading MuJoCo ${VERSION} (linux ${MJARCH}) ..."
    mkdir -p "$DEST"
    curl -fsSL "${BASE_URL}/mujoco-${VERSION}-linux-${MJARCH}.tar.gz" \
      | tar xz -C "$DEST" --strip-components=1
    ;;

  Darwin)
    # Upstream ships macOS as a .dmg holding mujoco.framework, not the
    # include/ + lib/ tarball layout the CMake expects — mount it and remap.
    # universal2 covers both arm64 and x86_64, so ARCH is not part of the URL.
    DMG_URL="${BASE_URL}/mujoco-${VERSION}-macos-universal2.dmg"
    TMPDMG="$(mktemp -t mujoco-XXXXXX).dmg"
    MNT="$(mktemp -d -t mujoco-mnt-XXXXXX)"
    cleanup() {
      hdiutil detach "$MNT" -quiet 2>/dev/null || true
      rm -rf "$TMPDMG" "$MNT"
    }
    trap cleanup EXIT

    echo "fetch_mujoco: downloading MuJoCo ${VERSION} (macos universal2) ..."
    curl -fsSL "$DMG_URL" -o "$TMPDMG"
    hdiutil attach "$TMPDMG" -nobrowse -readonly -mountpoint "$MNT" -quiet

    FW="$(find "$MNT" -maxdepth 3 -name 'mujoco.framework' -print -quit)"
    [ -n "$FW" ] || { echo "fetch_mujoco: mujoco.framework not found in dmg" >&2; exit 1; }

    mkdir -p "${DEST}/include/mujoco" "${DEST}/lib"
    cp -R "${FW}/Headers/." "${DEST}/include/mujoco/"

    # The framework binary is the dylib; name it so CMake's -lmujoco resolves.
    # Shipped as Versions/A/libmujoco.<version>.dylib (the framework's real
    # binary); glob it rather than assuming the version in the filename.
    FWBIN="$(find "${FW}/Versions" -name 'libmujoco*.dylib' -type f -print -quit)"
    [ -n "$FWBIN" ] || { echo "fetch_mujoco: libmujoco dylib not found under ${FW}" >&2; exit 1; }
    cp "$FWBIN" "${DEST}/lib/libmujoco.dylib"
    chmod u+w "${DEST}/lib/libmujoco.dylib"
    # Its install name still points at @rpath/mujoco.framework/...; repoint it
    # at the copy so consumers link and load without the framework present.
    install_name_tool -id "${DEST}/lib/libmujoco.dylib" "${DEST}/lib/libmujoco.dylib" 2>/dev/null
    # install_name_tool invalidates the signature; arm64 refuses to load an
    # unsigned dylib, so re-sign ad-hoc.
    codesign --force --sign - "${DEST}/lib/libmujoco.dylib"
    ;;

  *)
    echo "fetch_mujoco: unsupported OS '$OS' (need Linux or Darwin)" >&2; exit 1 ;;
esac

echo "fetch_mujoco: installed to ${DEST} ($(grep -m1 mjVERSION_HEADER "${DEST}/include/mujoco/mujoco.h" 2>/dev/null || echo '?'))"
