#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
readonly WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly SERVICE_NAME="luxi-d455-web.service"
readonly SERVICE_SOURCE="${WORKSPACE}/scripts/systemd/${SERVICE_NAME}"
readonly SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"

if [[ ! -x "${WORKSPACE}/scripts/start_luxi_boot.sh" ]]; then
    echo "Boot launcher is not executable: ${WORKSPACE}/scripts/start_luxi_boot.sh" >&2
    exit 1
fi
if [[ ! -r "${SERVICE_SOURCE}" ]]; then
    echo "Missing service unit: ${SERVICE_SOURCE}" >&2
    exit 1
fi

sudo install -o root -g root -m 0644 "${SERVICE_SOURCE}" "${SERVICE_TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable --now "${SERVICE_NAME}"

echo "Installed and enabled ${SERVICE_NAME}."
echo "Status: sudo systemctl status ${SERVICE_NAME}"
echo "Logs:   journalctl -u ${SERVICE_NAME} -f"

