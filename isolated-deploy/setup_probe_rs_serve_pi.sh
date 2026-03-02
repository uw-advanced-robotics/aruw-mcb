#!/bin/bash
set -euo pipefail

# Installs probe-rs with remote serving support on a Raspberry Pi and creates
# a systemd service for wireless flashing from the dev machine.
#
# This script is intended to be used alongside the existing ARUW J-Link setup:
# - jlink-remote-server.service
# - rtt-manager.service
#
# This setup intentionally does not enforce systemd Conflicts with the J-Link
# services, so probe-rs serve can coexist with your current RTT/J-Link manager
# behavior and rely on runtime ownership handoff.

RUN_AS_USER="${RUN_AS_USER:-aruw}"
HOME_DIR="/home/${RUN_AS_USER}"
PROBE_RS_VERSION="${PROBE_RS_VERSION:-}"
PROBE_RS_PORT="${PROBE_RS_PORT:-3000}"
PROBE_RS_TOKEN="${PROBE_RS_TOKEN:-aruw}"

if [[ ! -d "${HOME_DIR}" ]]; then
    echo "ERROR: Home directory does not exist for user '${RUN_AS_USER}': ${HOME_DIR}"
    exit 1
fi

echo "Installing system packages..."
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    ca-certificates \
    curl \
    git \
    pkg-config \
    libssl-dev

if [[ ! -x "${HOME_DIR}/.cargo/bin/cargo" ]]; then
    echo "Installing rustup toolchain for ${RUN_AS_USER}..."
    sudo -u "${RUN_AS_USER}" -H bash -lc \
        "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y"
fi

echo "Ensuring default Rust toolchain is configured..."
sudo -u "${RUN_AS_USER}" -H bash -lc "\
    source \"${HOME_DIR}/.cargo/env\" && \
    if ! rustup show active-toolchain >/dev/null 2>&1; then \
        rustup default stable; \
    fi"

INSTALL_CMD="source \"${HOME_DIR}/.cargo/env\" && cargo install probe-rs-tools --locked --features remote --force"
if [[ -n "${PROBE_RS_VERSION}" ]]; then
    INSTALL_CMD="${INSTALL_CMD} --version ${PROBE_RS_VERSION}"
fi

echo "Installing probe-rs tools (with remote support)..."
sudo -u "${RUN_AS_USER}" -H bash -lc "${INSTALL_CMD}"

echo "Creating probe-rs configuration..."
sudo -u "${RUN_AS_USER}" -H mkdir -p "${HOME_DIR}/.config/probe-rs"
sudo -u "${RUN_AS_USER}" -H tee "${HOME_DIR}/.config/probe-rs/probe-rs.toml" > /dev/null <<EOF
[server]
host = "0.0.0.0"
port = ${PROBE_RS_PORT}

[[server.users]]
username = "aruw"
password = "${PROBE_RS_TOKEN}"
EOF

echo "Creating systemd service..."
sudo tee /etc/systemd/system/probe-rs-serve.service > /dev/null <<EOF
[Unit]
Description=probe-rs remote server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${RUN_AS_USER}
Group=${RUN_AS_USER}
WorkingDirectory=${HOME_DIR}
Environment=PATH=${HOME_DIR}/.cargo/bin:/usr/local/bin:/usr/bin:/bin
ExecStart=${HOME_DIR}/.cargo/bin/probe-rs serve --config ${HOME_DIR}/.config/probe-rs/probe-rs.toml
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "Enabling probe-rs remote service..."
sudo systemctl daemon-reload
sudo systemctl enable probe-rs-serve.service

echo ""
echo "Setup complete."
echo ""
echo "Start probe-rs remote service:"
echo "  sudo systemctl start probe-rs-serve.service"
echo ""
echo "Check status:"
echo "  sudo systemctl status probe-rs-serve.service"
echo "  sudo journalctl -u probe-rs-serve.service -f"
