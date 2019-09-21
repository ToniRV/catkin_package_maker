.PHONY: help build

.DEFAULT_GOAL := default

default:
	@make build project_name='project_name'

build:
	@python make_package.py --project_name $(project_name)

help:
	@python make_package.py --help

