#!/usr/bin/env bash
set -eux	# -e: break instantly on errors

echo "Searching for latest release..."

# Fetch the latest release tag from GitHub API
JSON=$(curl -s https://api.github.com/repos/wpilibsuite/vscode-wpilib/releases/latest)

ALL_URLS=$(echo "$JSON" | jq '[.assets.[] | {url: .browser_download_url}]')
URL=$(echo "$ALL_URLS" | grep '.vsix' | cut -d '"' -f 4)

echo "Downloading WPILib from URL $URL..."

cd /opt
wget "$URL" -O wpilib.vsix

echo "WPILib downloaded at /opt/wpilib.vsix."