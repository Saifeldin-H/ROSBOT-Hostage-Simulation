#!/usr/bin/env bash
set -euo pipefail

models=(
  adjtable/3
  bathroomsink/1
  coffeetable/1
  deskchair/1
  drawer/1
  fridge/1
  mopcart2/1
  mopcart3/1
  officechairblack/1
  squareshelf/2
  table/3
  toilet/1
  trashbin/1
  whitecabinet/1
  woodenchair/1
)

for model in "${models[@]}"; do
  ign fuel download \
    --url "https://fuel.gazebosim.org/1.0/openrobotics/models/${model}"
done
