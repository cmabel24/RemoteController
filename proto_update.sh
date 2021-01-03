#!/usr/bin/env bash

PROTO="serialCommunications"
GENERATOR="~/Documents/src/nanopb/generator-bin/nanopb_generator"

eval "${GENERATOR} ${PROTO}.proto"
eval "mv ${PROTO}.pb.h include/"
eval "mv ${PROTO}.pb.c src/"