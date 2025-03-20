#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Preloads potential rover operation areas using MapProxy

mapproxy-seed -f mapproxy.yaml -s seed.yaml -i

# TODO: Set this up in Docker