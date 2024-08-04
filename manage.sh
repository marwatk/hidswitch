#!/bin/bash

set -eo pipefail

ROOT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
readonly ROOT_DIR

readonly DEVICE_FILE="${ROOT_DIR}/devices.txt"
readonly FLASH_HOST="${FLASH_HOST:-avrdude}"
readonly ESP_VERSION=5.2.1

function copy_source {
  local src="$1"
  local dest="$2"
  cp -rp "${src}/main" "${dest}/"
  cp -rp shared/main "${dest}/"
  cp \
    "${src}/CMakeLists.txt" \
    "${src}/dependencies.lock" \
    "${src}/sdkconfig" \
    "${dest}/"
}

function run {
  local device="$1"
  local port
  port="$(device_port "${device}")"
  local build_folder="${ROOT_DIR}/.docker_build/${1}"
  mkdir -p "${build_folder}"
  chmod a+w "${build_folder}"
  rm -rf "${build_folder:?}/main"
  if [[ "${device}" == "MSTR" ]]; then
    copy_source "${ROOT_DIR}/master" "${build_folder}"
  else
    copy_source "${ROOT_DIR}/slave" "${build_folder}"
    # Set the slave index define in the source
    sed -i \
      "s/#define SLAVE_IDX .*/#define SLAVE_IDX ${device#SLV}/" \
      "${build_folder}/main/main.c"
  fi
  docker run \
    --rm \
    -it \
    -v "${build_folder}:/project" \
    --name "hidswitch_${device}" \
    -w /project \
    espressif/idf:v${ESP_VERSION} \
      idf.py \
        --port "rfc2217://${FLASH_HOST}:${port}?ign_set_control" \
        -DIDF_TARGET=esp32s3 \
        "${@:2}"
}

function device_serial_number {
  local device="$1"
  if ! grep "^${device} " "${DEVICE_FILE}" | awk '{print $3}'; then
    >&2 echo "Couldn't find ${device} in ${DEVICE_FILE}"
    exit 1
  fi
}

function device_port {
  local device="$1"
  if ! grep "^${device} " "${DEVICE_FILE}" | awk '{print $2}'; then
    >&2 echo "Couldn't find ${device} in ${DEVICE_FILE}"
    exit 1
  fi
}

function start_serial {
  local device="$1"
  local serial_number
  serial_number="$(device_serial_number "${device}")"
  local port
  port="$(device_port "${device}")"
  local tty
  if ! tty="$(ssh "${FLASH_HOST}" dmesg \
        | grep -A1 "${serial_number}" \
        | grep -B1 tty \
        | tail -n 1 \
        | grep -Eo "ttyACM[[:digit:]]+")"; then
    >&2 echo "Couldn't determine tty of ${device} with serial ${serial_number}"
    exit 1
  fi
  >&2 echo "Starting remote serial port for ${serial_number} on port ${port}"
  ssh "${FLASH_HOST}" \
    esp_rfc2217_server.py \
      -p "${port}" \
      "/dev/${tty}"
}

function main {
  local device="$1"
  local steps=("${@:2}")

  if [[ "${steps[0]}" == "serial" ]]; then
    start_serial "${device}"
  else
    run "${device}" "${steps[@]}"
  fi
}

main "$@"