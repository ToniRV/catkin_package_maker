.PHONY: help build

help:
	@cat README.md

build:
	@python make_package.py
