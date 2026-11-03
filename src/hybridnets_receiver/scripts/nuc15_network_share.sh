#!/bin/bash
# Host-only IPv4 forwarding for the single Jetson peer. Keep Docker rules intact.
set -euo pipefail
peer=192.168.1.20/32
gpu_if=enxec1ac30270b6
cell_if=enxec1ac30270b0
forward_chain=NEUPAN-JETSON-FWD
nat_chain=NEUPAN-JETSON-NAT
ipt() { /usr/sbin/iptables -w 5 "$@"; }
ensure() {
    local table=$1 chain=$2
    shift 2
    ipt -t "$table" -C "$chain" "$@" 2>/dev/null || ipt -t "$table" -A "$chain" "$@"
}
case "${1:-}" in
    start)
        ipt -nL DOCKER-USER >/dev/null
        ipt -nL "$forward_chain" >/dev/null 2>&1 || ipt -N "$forward_chain"
        ipt -t nat -nL "$nat_chain" >/dev/null 2>&1 || ipt -t nat -N "$nat_chain"
        # wl+ matches both the built-in wlo1 and USB wlx... Wi-Fi interfaces.
        # Routing metrics select Wi-Fi first, then the cellular interface.
        for uplink in wl+ "$cell_if"; do
            ensure filter "$forward_chain" -i "$gpu_if" -s "$peer" -o "$uplink" -j ACCEPT
            ensure filter "$forward_chain" -i "$uplink" -o "$gpu_if" -d "$peer" \
                -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT
            ensure nat "$nat_chain" -s "$peer" -o "$uplink" -j MASQUERADE
        done
        ipt -C DOCKER-USER -j "$forward_chain" 2>/dev/null || ipt -I DOCKER-USER 1 -j "$forward_chain"
        ensure nat POSTROUTING -s "$peer" -j "$nat_chain"
        ;;
    stop)
        ipt -C DOCKER-USER -j "$forward_chain" 2>/dev/null && ipt -D DOCKER-USER -j "$forward_chain" || true
        ipt -t nat -C POSTROUTING -s "$peer" -j "$nat_chain" 2>/dev/null && \
            ipt -t nat -D POSTROUTING -s "$peer" -j "$nat_chain" || true
        # Flush only this service's own chains, never the global firewall.
        ipt -F "$forward_chain" 2>/dev/null || true
        ipt -X "$forward_chain" 2>/dev/null || true
        ipt -t nat -F "$nat_chain" 2>/dev/null || true
        ipt -t nat -X "$nat_chain" 2>/dev/null || true
        ;;
    *) echo "Usage: $0 start|stop" >&2; exit 2 ;;
esac
