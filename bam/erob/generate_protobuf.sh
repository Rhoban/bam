#!/bin/bash

# Re-generates the python protobuf
protoc --python_out=. etherban.proto
