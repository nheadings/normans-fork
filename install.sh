#!/bin/bash
# IOL Dashboard Installation Script for Raspberry Pi 5
# Big-Beautiful-Box Project

set -Eeuo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${CYAN}[STEP]${NC} $1"; }

on_error() {
    local exit_code="$1"
    local line_no="$2"
    log_error "Installer failed at line ${line_no}: ${BASH_COMMAND}"
    exit "$exit_code"
}

trap 'on_error $? $LINENO' ERR

append_if_missing() {
    local file="$1"
    local line="$2"
    grep -Fqx "$line" "$file" || echo "$line" | sudo tee -a "$file" > /dev/null
}

ensure_cmdline_arg() {
    local file="$1"
    local arg="$2"

    [ -f "$file" ] || return 0

    sudo python3 - "$file" "$arg" <<'PY'
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
arg = sys.argv[2]
parts = path.read_text().strip().split()
if arg not in parts:
    parts.append(arg)
path.write_text(" ".join(parts) + "\n")
PY
}

copy_if_needed() {
    local src="$1"
    local dst="$2"

    if [ ! -e "$src" ]; then
        log_warn "Missing expected file: $src"
        return 1
    fi

    if [ "$(realpath "$src")" = "$(realpath -m "$dst")" ]; then
        return 0
    fi

    cp "$src" "$dst"
}

copy_tree_contents_if_needed() {
    local src_dir="$1"
    local dst_dir="$2"

    if [ ! -d "$src_dir" ]; then
        log_warn "Missing expected directory: $src_dir"
        return 1
    fi

    if [ "$(realpath "$src_dir")" = "$(realpath -m "$dst_dir")" ]; then
        return 0
    fi

    cp -r "$src_dir/"* "$dst_dir/"
}

install_boot_logo() {
    local image_path="$1"
    local theme_name="trailersync"
    local theme_dir="/usr/share/plymouth/themes/${theme_name}"
    local plymouth_file="${theme_dir}/${theme_name}.plymouth"
    local script_file="${theme_dir}/${theme_name}.script"
    local image_name="Trailersync.png"
    local cmdline_file=""
    local candidate

    if [ ! -f "$image_path" ]; then
        log_warn "Boot logo image missing; skipping Plymouth theme install."
        return
    fi

    if ! command -v update-initramfs >/dev/null 2>&1 || ! command -v update-alternatives >/dev/null 2>&1; then
        log_warn "Plymouth tools missing; skipping boot logo install."
        return
    fi

    sudo mkdir -p "$theme_dir"
    sudo cp "$image_path" "${theme_dir}/${image_name}"

    sudo tee "$plymouth_file" > /dev/null <<EOF
[Plymouth Theme]
Name=TrailerSync
Description=TrailerSync boot splash with a centered fading logo
ModuleName=script

[script]
ImageDir=${theme_dir}
ScriptFile=${script_file}
EOF

    sudo tee "$script_file" > /dev/null <<'EOF'
Window.SetBackgroundTopColor(0.0, 0.0, 0.0);
Window.SetBackgroundBottomColor(0.0, 0.0, 0.0);

logo_image = Image("Trailersync.png");
logo_sprite = Sprite(logo_image);

screen_width = Window.GetWidth();
screen_height = Window.GetHeight();
logo_width = logo_image.GetWidth();
logo_height = logo_image.GetHeight();

logo_sprite.SetX((screen_width - logo_width) / 2);
logo_sprite.SetY((screen_height - logo_height) / 2);
logo_sprite.SetZ(100);
logo_sprite.SetOpacity(0.0);

opacity = 0.0;
fade_step = 0.02;

fun refresh_callback() {
    if (opacity < 1.0) {
        opacity += fade_step;

        if (opacity > 1.0) {
            opacity = 1.0;
        }

        logo_sprite.SetOpacity(opacity);
    }
}

Plymouth.SetRefreshFunction(refresh_callback);
EOF

    sudo chmod 644 "${theme_dir}/${image_name}" "$plymouth_file" "$script_file"

    sudo update-alternatives --install \
        /usr/share/plymouth/themes/default.plymouth \
        default.plymouth \
        "$plymouth_file" \
        220 >/dev/null

    sudo update-alternatives --set \
        default.plymouth \
        "$plymouth_file" >/dev/null

    for candidate in /boot/firmware/cmdline.txt /boot/cmdline.txt; do
        if [ -f "$candidate" ]; then
            cmdline_file="$candidate"
            break
        fi
    done

    if [ -n "$cmdline_file" ]; then
        if ! grep -qw quiet "$cmdline_file" || ! grep -qw splash "$cmdline_file"; then
            sudo cp "$cmdline_file" "${cmdline_file}.bak.$(date +%Y%m%d%H%M%S)"
            sudo python3 - "$cmdline_file" <<'PY'
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
parts = path.read_text().strip().split()
for flag in ("quiet", "splash"):
    if flag not in parts:
        parts.append(flag)
path.write_text(" ".join(parts) + "\n")
PY
        fi
    else
        log_warn "No cmdline.txt found; skipped quiet/splash enforcement."
    fi

    sudo update-initramfs -u
    log_info "Installed TrailerSync Plymouth boot logo."
}

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    log_error "Do not run as root. Run as the pi user."
    exit 1
fi

INSTALL_USER=$(whoami)
INSTALL_HOME=$HOME
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="$INSTALL_HOME/Big-Beautiful-Box"
OPT_DIR="/opt"
REPO_URL="https://github.com/RotorSync/Big-Beautiful-Box.git"
SOFTWARE_VERSION="$(cat "$SCRIPT_DIR/VERSION" 2>/dev/null || echo "unknown")"
SOURCE_BRANCH="$(git -C "$SCRIPT_DIR" symbolic-ref --quiet --short HEAD 2>/dev/null || true)"
SOURCE_COMMIT="$(git -C "$SCRIPT_DIR" rev-parse --verify HEAD 2>/dev/null || true)"

if [ "$INSTALL_USER" != "pi" ]; then
    log_error "This installer currently expects to run as the pi user."
    exit 1
fi

echo ""
log_info "=========================================="
log_info "IOL Dashboard Installation"
log_info "=========================================="
echo ""
log_info "User: $INSTALL_USER"
log_info "Install directory: $INSTALL_DIR"
log_info "Software version: $SOFTWARE_VERSION"
echo ""

# Confirm installation
read -p "Continue with installation? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    log_info "Installation cancelled."
    exit 0
fi

# Step 1: Update system
log_step "1/7: Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Step 2: Install dependencies
log_step "2/7: Installing dependencies..."
sudo apt install -y \
    openssh-server \
    python3 \
    python3-tk \
    python3-serial \
    python3-lgpio \
    python3-spidev \
    python3-pil \
    python3-pil.imagetk \
    git \
    build-essential \
    cmake \
    libgpiod-dev \
    gpiod \
    netcat-openbsd \
    python3-pip \
    python3-yaml \
    bluez \
    bluez-tools \
    zstd \
    plymouth \
    plymouth-themes

sudo usermod -a -G dialout $INSTALL_USER
sudo systemctl enable ssh
sudo systemctl start ssh

# Install Python packages used by the BLE/Rotorsync stack.
# Rotorsync runs as a systemd service via /usr/bin/python3, so these must be
# available to the system interpreter rather than only the pi user's site-packages.
# Do not self-upgrade distro-managed pip here. Ubuntu's deb-packaged pip does
# not carry uninstall metadata that pip expects for a replace-in-place upgrade.
sudo python3 -m pip install --break-system-packages --ignore-installed bleak bumble

# Step 3: Install vendored IOL-HAT
log_step "3/7: Setting up IOL-HAT..."
if [ ! -d "$SCRIPT_DIR/iol-hat" ]; then
    log_error "Vendored iol-hat directory is missing from this repo."
    exit 1
fi

rm -rf "$INSTALL_HOME/iol-hat"
mkdir -p "$INSTALL_HOME/iol-hat"
cp -r "$SCRIPT_DIR/iol-hat/"* "$INSTALL_HOME/iol-hat/"
log_info "Installed vendored iol-hat snapshot from BBB repo"

# Build IOL-HAT
if [ -d "$INSTALL_HOME/iol-hat/src-master-application" ]; then
    cd "$INSTALL_HOME/iol-hat/src-master-application"
    make clean 2>/dev/null || true
    make debug || log_warn "IOL-HAT build failed (may need hardware)"
fi

# Step 4: Configure hardware
log_step "4/7: Configuring hardware..."

# Apply tracked BBB boot/display settings
BBB_BOOT_CFG="/boot/firmware/config.txt"
BBB_BOOT_SNIPPET="$SCRIPT_DIR/deploy/boot-firmware-bbb.conf"
BBB_VIDEO_ARG="video=HDMI-A-1:1920x1080M@30D"
UART_CHANGED=0
if [ -f "$BBB_BOOT_SNIPPET" ]; then
    sudo cp "$BBB_BOOT_CFG" "${BBB_BOOT_CFG}.bbb.bak" 2>/dev/null || true
    while IFS= read -r line; do
        [ -z "$line" ] && continue
        if ! grep -Fqx "$line" "$BBB_BOOT_CFG"; then
            echo "$line" | sudo tee -a "$BBB_BOOT_CFG" > /dev/null
            case "$line" in
                enable_uart=*|dtparam=uart0=*|dtoverlay=disable-bt)
                    UART_CHANGED=1
                    ;;
            esac
        fi
    done < "$BBB_BOOT_SNIPPET"
fi

# Explicitly enforce BBB UART settings for the switch box serial link.
for uart_line in "enable_uart=0" "dtparam=uart0=on" "dtoverlay=disable-bt"; do
    if ! grep -Fqx "$uart_line" "$BBB_BOOT_CFG"; then
        append_if_missing "$BBB_BOOT_CFG" "$uart_line"
        UART_CHANGED=1
    fi
done

if [ "$UART_CHANGED" -eq 1 ]; then
    log_warn "Applied UART boot settings for /dev/ttyAMA0. A reboot is required before the switch box serial link will work."
fi

# Force the desktop/KMS stack to boot the primary HDMI output at 1080p30.
for cmdline_file in /boot/firmware/current/cmdline.txt /boot/firmware/cmdline.txt /boot/cmdline.txt; do
    ensure_cmdline_arg "$cmdline_file" "$BBB_VIDEO_ARG"
done

# Step 5: Install dashboard and Rotorsync files
log_step "5/7: Installing dashboard files..."

if git -C "$SCRIPT_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    if [ "$(realpath "$SCRIPT_DIR")" = "$(realpath "$INSTALL_DIR")" ]; then
        log_info "Using current BBB checkout in $INSTALL_DIR"
        git -C "$INSTALL_DIR" remote set-url origin "$REPO_URL" >/dev/null 2>&1 || true
    else
        [ -d "$INSTALL_DIR" ] && mv "$INSTALL_DIR" "${INSTALL_DIR}.pre-install-backup.$(date +%Y%m%d-%H%M%S)"
        log_info "Cloning BBB locally from $SCRIPT_DIR into $INSTALL_DIR"
        git clone "$SCRIPT_DIR" "$INSTALL_DIR"
        git -C "$INSTALL_DIR" remote set-url origin "$REPO_URL"
        if [ -n "$SOURCE_BRANCH" ]; then
            git -C "$INSTALL_DIR" checkout -f "$SOURCE_BRANCH" >/dev/null 2>&1 || \
                log_warn "Could not switch cloned checkout to branch $SOURCE_BRANCH; continuing with cloned HEAD"
        fi
        if [ -n "$SOURCE_COMMIT" ]; then
            git -C "$INSTALL_DIR" reset --hard "$SOURCE_COMMIT" >/dev/null 2>&1 || \
                log_warn "Could not align cloned checkout to source commit $SOURCE_COMMIT"
        fi
    fi
else
    log_error "Installer must be run from a real git checkout so the installed box has a working GitHub updater."
    log_error "Clone ${REPO_URL} first, then rerun ./install.sh"
    exit 1
fi

if ! git -C "$INSTALL_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    log_error "Installed BBB directory is not a git checkout. Refusing to continue."
    exit 1
fi

mkdir -p "$INSTALL_DIR/src"
mkdir -p "$INSTALL_DIR/RPi"
mkdir -p "$INSTALL_DIR/mopeka"
mkdir -p "$INSTALL_DIR/deploy"
sudo mkdir -p "$OPT_DIR/src"
sudo mkdir -p "$OPT_DIR/mopeka"

# Copy dashboard/runtime files
copy_if_needed "$SCRIPT_DIR/dashboard.py" "$INSTALL_DIR/dashboard.py"
copy_if_needed "$SCRIPT_DIR/bbb_maint_agent.py" "$INSTALL_DIR/bbb_maint_agent.py"
copy_if_needed "$SCRIPT_DIR/config.py" "$INSTALL_DIR/config.py"
copy_if_needed "$SCRIPT_DIR/VERSION" "$INSTALL_DIR/VERSION"
copy_if_needed "$SCRIPT_DIR/iolhat.py" "$INSTALL_DIR/iolhat.py"
copy_if_needed "$SCRIPT_DIR/start_iol_dashboard.sh" "$INSTALL_DIR/start_iol_dashboard.sh"
copy_tree_contents_if_needed "$SCRIPT_DIR/src" "$INSTALL_DIR/src"
copy_tree_contents_if_needed "$SCRIPT_DIR/RPi" "$INSTALL_DIR/RPi"
copy_tree_contents_if_needed "$SCRIPT_DIR/mopeka" "$INSTALL_DIR/mopeka"
copy_tree_contents_if_needed "$SCRIPT_DIR/deploy" "$INSTALL_DIR/deploy"
copy_if_needed "$SCRIPT_DIR/install.sh" "$INSTALL_DIR/install.sh"
[ -f "$SCRIPT_DIR/Trailersync.png" ] && copy_if_needed "$SCRIPT_DIR/Trailersync.png" "$INSTALL_DIR/Trailersync.png"

# Copy Rotorsync runtime files to /opt to match service paths
sudo cp "$SCRIPT_DIR/rotorsync_bumble.py" "$OPT_DIR/rotorsync_bumble.py"
sudo cp "$SCRIPT_DIR/bbb_maint_agent.py" "$OPT_DIR/bbb_maint_agent.py"
sudo cp "$SCRIPT_DIR/rotorsync_watchdog.py" "$OPT_DIR/rotorsync_watchdog.py"
sudo cp "$SCRIPT_DIR/src/__init__.py" "$OPT_DIR/src/"
sudo cp "$SCRIPT_DIR/src/maintenance_protocol.py" "$OPT_DIR/src/maintenance_protocol.py"
sudo cp "$SCRIPT_DIR/src/mopeka_converter.py" "$OPT_DIR/src/mopeka_converter.py"
for mopeka_file in "$SCRIPT_DIR"/mopeka/*; do
    [ -f "$mopeka_file" ] || continue
    base_name="$(basename "$mopeka_file")"
    if [ "$base_name" = "mopeka_config.json" ] && [ -f "$OPT_DIR/mopeka/mopeka_config.json" ]; then
        log_info "Preserving existing /opt/mopeka/mopeka_config.json"
        continue
    fi
    sudo cp "$mopeka_file" "$OPT_DIR/mopeka/"
done
sudo chmod 755 "$OPT_DIR/rotorsync_bumble.py" "$OPT_DIR/bbb_maint_agent.py" "$OPT_DIR/rotorsync_watchdog.py"

# Copy optional files
[ -f "$SCRIPT_DIR/bbb_diagram_rotated.jpeg" ] && copy_if_needed "$SCRIPT_DIR/bbb_diagram_rotated.jpeg" "$INSTALL_DIR/bbb_diagram_rotated.jpeg"
[ -f "$SCRIPT_DIR/thumbs_up.png" ] && copy_if_needed "$SCRIPT_DIR/thumbs_up.png" "$INSTALL_DIR/thumbs_up.png"
[ -f "$SCRIPT_DIR/thumbs_up.png" ] && copy_if_needed "$SCRIPT_DIR/thumbs_up.png" "$INSTALL_HOME/thumbs_up.png"
[ -f "$SCRIPT_DIR/README.md" ] && copy_if_needed "$SCRIPT_DIR/README.md" "$INSTALL_DIR/README.md"

chmod +x "$INSTALL_DIR/start_iol_dashboard.sh"
chmod +x "$INSTALL_DIR/dashboard.py"

# Step 6: Install systemd services
log_step "6/7: Installing systemd service..."

sudo cp "$SCRIPT_DIR/iol_dashboard.service" /etc/systemd/system/
sudo cp "$SCRIPT_DIR/bbb-maint-agent.service" /etc/systemd/system/
sudo cp "$SCRIPT_DIR/rotorsync.service" /etc/systemd/system/
sudo cp "$SCRIPT_DIR/rotorsync_watchdog.service" /etc/systemd/system/
if [ -f "$SCRIPT_DIR/deploy/bbb-maint-agent-sudoers" ]; then
    sudo cp "$SCRIPT_DIR/deploy/bbb-maint-agent-sudoers" /etc/sudoers.d/bbb-maint-agent
    sudo chmod 440 /etc/sudoers.d/bbb-maint-agent
    sudo visudo -cf /etc/sudoers.d/bbb-maint-agent >/dev/null
fi
sudo systemctl daemon-reload
sudo systemctl enable iol_dashboard.service
sudo systemctl enable bbb-maint-agent.service
sudo systemctl enable rotorsync.service
sudo systemctl enable rotorsync_watchdog.service

# Step 7: Configure auto-login, screen, and log retention
log_step "7/7: Configuring display settings and log retention..."

# GDM3 auto-login / X11 display manager settings
if [ -f "$SCRIPT_DIR/deploy/gdm3-custom.conf" ]; then
    sudo mkdir -p /etc/gdm3
    sudo cp /etc/gdm3/custom.conf /etc/gdm3/custom.conf.bbb.bak 2>/dev/null || true
    sudo cp "$SCRIPT_DIR/deploy/gdm3-custom.conf" /etc/gdm3/custom.conf
fi

# Disable idle timeout
if ! grep -q "IdleAction=ignore" /etc/systemd/logind.conf 2>/dev/null; then
    echo -e "\nIdleAction=ignore\nIdleActionSec=0" | sudo tee -a /etc/systemd/logind.conf > /dev/null
fi

# Cap noisy BBB logs while preserving seasonal fill history logs.
sudo mkdir -p /home/pi/bug_reports
if [ -f "$SCRIPT_DIR/deploy/bbb-logrotate.conf" ]; then
    sudo cp "$SCRIPT_DIR/deploy/bbb-logrotate.conf" /etc/logrotate.d/bbb
fi
if [ -f "$SCRIPT_DIR/deploy/bbb-logrotate.service" ] && [ -f "$SCRIPT_DIR/deploy/bbb-logrotate.timer" ]; then
    sudo cp "$SCRIPT_DIR/deploy/bbb-logrotate.service" /etc/systemd/system/bbb-logrotate.service
    sudo cp "$SCRIPT_DIR/deploy/bbb-logrotate.timer" /etc/systemd/system/bbb-logrotate.timer
    sudo systemctl daemon-reload
    sudo systemctl enable --now bbb-logrotate.timer || log_warn "Could not enable bbb-logrotate.timer"
    sudo systemctl start bbb-logrotate.service || log_warn "Could not start bbb-logrotate.service"
fi

# Cap journald disk usage for unattended deployments.
if [ -f "$SCRIPT_DIR/deploy/journald-bbb.conf" ]; then
    sudo mkdir -p /etc/systemd/journald.conf.d
    sudo cp "$SCRIPT_DIR/deploy/journald-bbb.conf" /etc/systemd/journald.conf.d/bbb.conf
    sudo systemctl restart systemd-journald || log_warn "Could not restart systemd-journald"
fi

# Disable unattended package updates for in-season stability.
if [ -f "$SCRIPT_DIR/deploy/20auto-upgrades-bbb" ]; then
    sudo cp "$SCRIPT_DIR/deploy/20auto-upgrades-bbb" /etc/apt/apt.conf.d/20auto-upgrades
fi
if [ -f "$SCRIPT_DIR/deploy/10periodic-bbb" ]; then
    sudo cp "$SCRIPT_DIR/deploy/10periodic-bbb" /etc/apt/apt.conf.d/10periodic
fi
sudo systemctl disable --now unattended-upgrades.service apt-daily.timer apt-daily-upgrade.timer >/dev/null 2>&1 || true

# Force Bumble-only Bluetooth ownership for BBB. BlueZ can be D-Bus activated by
# the desktop stack unless it is both disabled and masked.
sudo systemctl disable --now bluetooth.service >/dev/null 2>&1 || true
sudo systemctl mask bluetooth.service >/dev/null 2>&1 || true

# Install TrailerSync Plymouth boot logo if the bundled asset is present.
if [ -f "$INSTALL_DIR/Trailersync.png" ]; then
    install_boot_logo "$INSTALL_DIR/Trailersync.png" || \
        log_warn "Boot logo install failed; continuing"
fi

# Start services now so the system is usable immediately after install.
sudo systemctl restart iol_dashboard.service || log_warn "Could not start iol_dashboard.service"
sudo systemctl restart bbb-maint-agent.service || log_warn "Could not start bbb-maint-agent.service"
sudo systemctl restart rotorsync.service || log_warn "Could not start rotorsync.service"
sudo systemctl restart rotorsync_watchdog.service || log_warn "Could not start rotorsync_watchdog.service"

# Done!
echo ""
log_info "=========================================="
log_info "Installation Complete!"
log_info "=========================================="
echo ""
log_info "Dashboard installed to: $INSTALL_DIR"
log_info "Rotorsync installed to: $OPT_DIR"
log_info "Services enabled: iol_dashboard.service, bbb-maint-agent.service, rotorsync.service, rotorsync_watchdog.service"
echo ""
log_info "Next steps:"
log_info "  1. Reboot: sudo reboot"
log_info "  2. Check status: systemctl status iol_dashboard.service"
log_info "  3. Check BLE status: systemctl status rotorsync.service"
log_info "  4. Check maintenance sidecar: systemctl status bbb-maint-agent.service"
log_info "  5. After reboot, confirm serial: ls -l /dev/ttyAMA0"
log_info "  6. View logs: tail -f ~/iol_dashboard.log"
echo ""

read -p "Reboot now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo reboot
fi
