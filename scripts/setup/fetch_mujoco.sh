#!/usr/bin/env bash
#
# Fetch a prebuilt MuJoCo release into third_party/mujoco (gitignored).
#
# MuJoCo is Apache-2.0; xmSimulator owns the family MuJoCo dependency (host-x86 sim).
# it is NOT a submodule — the sim is host-x86 only, never cross-compiled to the
# armv7 base target, so the .so is fetched on demand rather than vendored in git.
#
#   scripts/setup/fetch_mujoco.sh [VERSION]   (default 3.9.0, linux x86_64)
#
# Mirrors legged_controller/scripts/setup/fetch_mujoco.sh so the family uses one
# MuJoCo version.
#
# Copyright (c) 2026 Ruixiang Du (rdu)
set -euo pipefail

VERSION="${1:-3.9.0}"
ARCH="$(uname -m)"
case "$ARCH" in
  x86_64)  MJARCH="x86_64" ;;
  aarch64) MJARCH="aarch64" ;;
  *) echo "fetch_mujoco: unsupported arch '$ARCH' (need x86_64 or aarch64)" >&2; exit 1 ;;
esac

REPO_ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
DEST="${REPO_ROOT}/third_party/mujoco"

if [ -f "${DEST}/include/mujoco/mujoco.h" ]; then
  echo "fetch_mujoco: MuJoCo already present at ${DEST} — nothing to do."
  echo "             (delete the directory to re-fetch.)"
  exit 0
fi

URL="https://github.com/google-deepmind/mujoco/releases/download/${VERSION}/mujoco-${VERSION}-linux-${MJARCH}.tar.gz"
echo "fetch_mujoco: downloading MuJoCo ${VERSION} (${MJARCH}) ..."
mkdir -p "$DEST"
curl -fsSL "$URL" | tar xz -C "$DEST" --strip-components=1
echo "fetch_mujoco: installed to ${DEST} (libmujoco.so $(cat "${DEST}/include/mujoco/mujoco.h" 2>/dev/null | grep -m1 mjVERSION_HEADER || echo '?'))"
