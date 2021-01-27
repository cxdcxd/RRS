#!/bin/bash

echo "path: "  $1
cd $1

./consul agent -dev -config-dir=data -client="0.0.0.0"