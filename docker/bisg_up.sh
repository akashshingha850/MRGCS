#!/usr/bin/env bash
# bisg_up.sh — start the GCS onboard stack (rosbridge, ZED map relay, gcs_control) against a running bisg_isaac sim.
# Takes ROS_DOMAIN_ID from the sim's MAVROS container (bisg-mavros-<DRONE_ID>), so it follows
# bisg_isaac's per-machine `auto` domain. Also starts the Kinetix bridge (GCS Kinetix tab) when the
# kinetix:jazzy image and the Kinetix repo (KINETIX_DIR, default ../kinetix next to this repo) exist.
# Then open http://localhost:8090 (or the Electron app on ws://localhost:9090).
#
#   docker/bisg_up.sh          start (builds the image on first use)
#   docker/bisg_up.sh down     stop
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

COMPOSE=(docker compose -f docker/compose.yaml --profile bisg)

KINETIX_DIR="$(cd "${KINETIX_DIR:-../kinetix}" 2>/dev/null && pwd || true)"
export KINETIX_DIR KX_UID="$(id -u)" KX_GID="$(id -g)"

if [[ "${1:-}" == down ]]; then
  ROS_DOMAIN_ID=0 "${COMPOSE[@]}" --profile kinetix rm -sf gcs-bisg kinetix-bridge
  exit 0
fi

MAVROS_CT="bisg-mavros-${DRONE_ID:-1}"
if ! domain=$(docker exec "$MAVROS_CT" printenv ROS_DOMAIN_ID 2>/dev/null); then
  echo "[bisg_up] $MAVROS_CT is not running — start the sim first (bisg_isaac: ./bisg all)" >&2
  exit 1
fi

SERVICES=(gcs-bisg)
if [[ -n "$KINETIX_DIR" ]] && docker image inspect kinetix:jazzy >/dev/null 2>&1; then
  COMPOSE+=(--profile kinetix); SERVICES+=(kinetix-bridge)
else
  echo "[bisg_up] Kinetix bridge skipped: needs the kinetix:jazzy image and the repo at KINETIX_DIR"
fi

echo "[bisg_up] ROS_DOMAIN_ID=$domain (from $MAVROS_CT), DRONE_NS=/drone_${DRONE_ID:-1}"
ROS_DOMAIN_ID="$domain" DRONE_NS="/drone_${DRONE_ID:-1}" "${COMPOSE[@]}" up -d "${SERVICES[@]}"
echo "[bisg_up] GCS web app: http://localhost:${GCS_WEB_PORT:-8090}   (rosbridge ws://localhost:9090)"
