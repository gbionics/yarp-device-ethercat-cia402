#!/usr/bin/env bash
# ───────────────────────────────────────────────────────────────────────
# build_docs.sh — build the Doxygen documentation
#
# Usage:  cd doc && ./build_docs.sh
#
# What it does:
#   1. Downloads the cppreference Doxygen tag file (if not already cached)
#      so that std:: types link to https://en.cppreference.com.
#      The tag file is ~1 MB and is NOT committed to the repo.
#   2. Runs Doxygen.
# ───────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TAG_FILE="cppreference-doxygen-web.tag.xml"
TAG_URL="https://upload.cppreference.com/mwiki/images/f/f8/cppreference-doxygen-web.tag.xml"
# Fallback: Wayback Machine snapshot (the canonical URL is sometimes down).
TAG_URL_FALLBACK="https://web.archive.org/web/2024/https://upload.cppreference.com/mwiki/images/f/f8/cppreference-doxygen-web.tag.xml"

# ── 1. Fetch cppreference tag file ────────────────────────────────────
if [ ! -f "$TAG_FILE" ]; then
    echo "⬇  Downloading cppreference tag file …"
    if ! curl -fsSL -o "$TAG_FILE" "$TAG_URL" 2>/dev/null; then
        echo "   Primary URL failed, trying Wayback Machine …"
        curl -fsSL -o "$TAG_FILE" "$TAG_URL_FALLBACK"
    fi
    echo "   ✓ Saved $TAG_FILE ($(wc -c < "$TAG_FILE") bytes)"
else
    echo "✓  cppreference tag file already present."
fi

# ── 2. Run Doxygen ────────────────────────────────────────────────────
echo "🔨 Running Doxygen …"
doxygen Doxyfile

echo ""
echo "✅ Documentation built in _build/html/"
echo "   Open _build/html/index.html in a browser to view."
