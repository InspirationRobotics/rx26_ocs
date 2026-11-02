#!/usr/bin/env bash
# install_robocommand_sim.sh — the SIMULATED RoboNation server
#
# Builds the third machine in the Gate G3 bench: a Linux box that pretends to
# be RoboCommand. It holds 192.168.65.2, serves DHCP on one ethernet interface,
# and runs RoboNation's own docker stub. Nothing else.
#
# It reproduces what the competition hands you: an RJ-45 cable and a DHCP lease
# on the course subnet. That matters more than it sounds. The handbook forbids
# a static address on the OCS's RoboCommand-facing interface, so "we always ran
# it static on the bench" is how you discover at Alpha that something hardcodes
# an IP.
#
#   sim box  192.168.65.2  ── straight ethernet ──  OCS  ── wifi ──  team router
#   (this script)                                                    192.168.8.0/24
#
# THE SIM BOX MUST NOT BE ON THE TEAM NETWORK AT THE SAME TIME. Two interfaces
# plus the ip_forward=1 that Docker sets for you is a router, and a router
# between the course subnet and the team subnet is precisely the topology the
# handbook prohibits -- the boat would reach RoboCommand through this machine.
# Pull the images while you have internet, then take the wifi down. --check
# tests for it.
#
# Usage:
#   bash install_robocommand_sim.sh --list             # candidate interfaces
#   bash install_robocommand_sim.sh --iface enp3s0     # build it
#   bash install_robocommand_sim.sh --iface enp3s0 --check   # verify only
#
# Idempotent: safe to re-run; it only writes what is missing or wrong.
set -euo pipefail

SUBNET="192.168.65"
SELF="${SUBNET}.2"
POOL_LO="${SUBNET}.100"
POOL_HI="${SUBNET}.200"
CONF="/etc/dnsmasq.d/robocommand-course.conf"
REPO_DIR="${ROBOCOMMAND_DIR:-$HOME/robocommand}"

iface=""
mode="install"

while [ $# -gt 0 ]; do
    case "$1" in
        --list)  mode="list" ;;
        --check) mode="check" ;;
        --iface) iface="${2:-}"; shift ;;
        -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

say()  { printf '\n=== %s\n' "$*"; }
ok()   { printf '  ok    %s\n' "$*"; }
warn() { printf '  WARN  %s\n' "$*"; }
bad()  { printf '  FAIL  %s\n' "$*"; }

[ "$(uname -s)" = "Linux" ] || { echo "Linux only." >&2; exit 1; }

# --list
default_iface="$(ip route show default 2>/dev/null | awk '{print $5; exit}')"

if [ "$mode" = "list" ] || [ -z "$iface" ]; then
    say "wired interfaces on this machine"
    printf '  %-14s %-10s %-18s %s\n' NAME STATE ADDRESS NOTE
    for d in /sys/class/net/*; do
        n="$(basename "$d")"
        case "$n" in lo|docker*|veth*|br-*|virbr*|wl*) continue ;; esac
        state="$(cat "$d/operstate" 2>/dev/null || echo '?')"
        addr="$(ip -4 -br addr show "$n" 2>/dev/null | awk '{print $3}')"
        note=""
        [ "$n" = "$default_iface" ] && note="<- your internet/SSH. DO NOT USE."
        printf '  %-14s %-10s %-18s %s\n' "$n" "$state" "${addr:--}" "$note"
    done
    echo
    echo "Pick the one wired to the OCS, then re-run with --iface <name>."
    [ "$mode" = "list" ] && exit 0
    exit 2
fi

[ -e "/sys/class/net/$iface" ] || { echo "no such interface: $iface" >&2; exit 1; }

# Refusing this is the whole reason the check exists: reconfiguring the
# interface that carries your SSH session ends the session, and on a headless
# box ends your access to the machine.
if [ "$iface" = "$default_iface" ]; then
    bad "$iface carries the default route -- it is your internet and possibly your SSH."
    echo "  Use the OTHER wired port. --list shows them." >&2
    exit 1
fi

# --check
if [ "$mode" = "check" ]; then
    rc=0
    say "address"
    if ip -4 addr show "$iface" | grep -q "$SELF"; then ok "$iface holds $SELF"
    else bad "$iface does not hold $SELF"; rc=1; fi

    say "dhcp"
    if systemctl is-active --quiet dnsmasq; then ok "dnsmasq is running"
    else bad "dnsmasq is not running (journalctl -u dnsmasq)"; rc=1; fi
    if [ -f "$CONF" ]; then ok "$CONF present"; else bad "$CONF missing"; rc=1; fi
    if [ -f /var/lib/misc/dnsmasq.leases ] && [ -s /var/lib/misc/dnsmasq.leases ]; then
        ok "leases issued:"; sed 's/^/        /' /var/lib/misc/dnsmasq.leases
    else
        warn "no leases yet -- normal until the OCS is plugged in and its link is up"
    fi

    say "broker"
    if command -v docker >/dev/null && docker compose version >/dev/null 2>&1; then
        ok "docker + compose present"
    else bad "docker or the compose plugin is missing"; rc=1; fi
    if ss -lntp 2>/dev/null | grep -q ':1883'; then
        ok "something is listening on 1883"
        ss -lnt | awk '/:1883/ && $4 ~ /^127\./ {print "        BOUND TO LOOPBACK: " $4}'
    else
        warn "nothing on 1883 -- start the stub (docker compose up) in $REPO_DIR/RobotX_2026"
    fi

    say "isolation -- this is the one that matters"
    others="$(ip -4 -br addr show up 2>/dev/null \
        | awk -v i="$iface" '$1!="lo" && $1!=i && $3!="" && $1 !~ /^docker|^br-|^veth/ {print $1" "$3}')"
    if [ -n "$others" ]; then
        bad "this machine is on another network as well:"
        echo "$others" | sed 's/^/        /'
        echo "        Take it down. With ip_forward=1 (Docker sets this) the boat"
        echo "        can reach RoboCommand through this box." >&2
        rc=1
    else
        ok "only $iface is up -- no path between the course and anything else"
    fi

    say "$([ $rc -eq 0 ] && echo 'sim box READY' || echo 'sim box NOT ready')"
    exit $rc
fi

# install
say "plan"
cat <<PLAN
  interface   $iface
  address     $SELF/24        (static; this box is the server)
  dhcp pool   $POOL_LO - $POOL_HI   (the OCS leases from here)
  dnsmasq     $CONF        (DHCP only, DNS disabled)
  stub        $REPO_DIR

  Untouched: every other interface, your default route, and your DNS.
PLAN
printf '\nProceed? [y/N] '
read -r reply
case "$reply" in y|Y|yes) ;; *) echo "aborted."; exit 0 ;; esac

say "packages"
sudo apt-get update -qq
sudo apt-get install -y dnsmasq git ca-certificates curl
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    ok "docker already present"
else
    say "installing docker from get.docker.com"
    curl -fsSL https://get.docker.com | sudo sh
    sudo usermod -aG docker "$USER"
    warn "log out and back in for group docker to apply, or use sudo docker until then"
fi

say "address on $iface"
if command -v nmcli >/dev/null 2>&1 && nmcli -t device status 2>/dev/null \
        | awk -F: -v i="$iface" '$1==i{print $3}' | grep -qv unmanaged; then
    # NetworkManager owns it, so nmcli must be the one to configure it --
    # writing netplan underneath NM produces a config that loses on reboot.
    nmcli connection delete "course-$iface" >/dev/null 2>&1 || true
    sudo nmcli connection add type ethernet ifname "$iface" con-name "course-$iface" \
        ipv4.method manual ipv4.addresses "$SELF/24" \
        ipv4.never-default yes ipv6.method ignore \
        connection.autoconnect yes >/dev/null
    sudo nmcli connection up "course-$iface" >/dev/null
    ok "nmcli profile course-$iface"
else
    net="/etc/netplan/99-robocommand-course.yaml"
    sudo tee "$net" >/dev/null <<YAML
# Written by install_robocommand_sim.sh. The course segment only.
# never-default: this link must never become the default route -- it goes
# nowhere, and stealing the default route would cut this box off entirely.
network:
  version: 2
  ethernets:
    $iface:
      dhcp4: false
      addresses: [$SELF/24]
      link-local: []
YAML
    sudo chmod 600 "$net"
    sudo netplan apply
    ok "netplan $net"
fi

say "dhcp on $iface"
sudo tee "$CONF" >/dev/null <<CONFEOF
# Written by install_robocommand_sim.sh
# Serves the simulated course segment and NOTHING else.

# DHCP only. port=0 disables dnsmasq's DNS server, which would otherwise fight
# systemd-resolved for port 53 and fail to start -- the usual reason dnsmasq
# refuses to come up on Ubuntu.
port=0

# Answer on this interface alone. Without both lines dnsmasq listens on every
# interface, and a stray DHCP server on the team network is a bad afternoon.
interface=$iface
bind-interfaces
except-interface=lo

dhcp-range=$POOL_LO,$POOL_HI,12h

# No default route and no DNS handed to the client. The OCS reaches this subnet
# directly and must keep its own route to the team network; pushing a gateway
# from here is how you kill the operator's link to the boat.
dhcp-option=3
dhcp-option=6

log-dhcp
CONFEOF
sudo systemctl enable dnsmasq >/dev/null 2>&1 || true
sudo systemctl restart dnsmasq
ok "dnsmasq restarted"

say "robocommand stub"
if [ -d "$REPO_DIR/.git" ]; then
    git -C "$REPO_DIR" pull --ff-only || warn "could not update $REPO_DIR"
else
    git clone https://github.com/robonation/robocommand "$REPO_DIR"
fi
ok "$REPO_DIR"

say "pull the images NOW, while this box still has internet"
( cd "$REPO_DIR/RobotX_2026" && sudo docker compose pull ) \
    || warn "pull failed; 'docker compose up --build' will fetch on first run"

cat <<NEXT

Done. Next:

  1. Take this machine OFF the team network / wifi. It must reach nothing but
     the OCS. Then:
         bash install_robocommand_sim.sh --iface $iface --check

  2. Start the stub:
         cd $REPO_DIR/RobotX_2026 && docker compose up --build
     Confirm 1883 is NOT bound to 127.0.0.1 -- a broker on loopback is
     invisible from the OCS:
         ss -lnt | grep 1883

  3. Cable this box to the OCS. On the OCS, confirm it leased an address in
     ${SUBNET}.0/24 -- leased, not static. Then in ocs/bridge.toml:
         [robocommand]
         host = "$SELF"

  4. From the JETSON:  ping $SELF   ->  this MUST fail.
     If it succeeds you are non-compliant. See docs/G3_robocommand_bench.md B4.
NEXT
