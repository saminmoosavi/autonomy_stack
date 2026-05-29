#!/bin/bash
# UID is readonly in bash, so we forward as USER_UID/USER_GID instead
USER_UID=$(id -u) USER_GID=$(id -g) docker compose build
