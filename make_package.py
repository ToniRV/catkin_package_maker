#!/usr/bin/env python

import sys
import os
import re
import argparse
from shutil import copytree, ignore_patterns

# Group of Different functions for different styles
if sys.platform.lower() == "win32":
    os.system('color')
RED = '\033[31m'
GREEN = '\033[32m'
RESET = '\033[0m'


def configure_file(template_file, environment):  # noqa: D402
    """
    Evaluate a .in template file used in CMake with configure_file().
    :param template_file: path to the template, ``str``
    :param environment: dictionary of placeholders to substitute,
      ``dict``
    :returns: string with evaluates template
    :raises: KeyError for placeholders in the template which are not
      in the environment
    """
    with open(template_file, 'r') as f:
        template = f.read()
        return configure_string(template, environment)


def configure_string(template, environment):
    """
    Substitute variables enclosed by @ characters.
    :param template: the template, ``str``
    :param environment: dictionary of placeholders to substitute,
      ``dict``
    :returns: string with evaluates template
    :raises: KeyError for placeholders in the template which are not
      in the environment
    """
    def substitute(match):
        var = match.group(0)[1:-1]
        return environment[var]
    return re.sub('\@[a-zA-Z0-9_]+\@', substitute, template)


def parser():
    basic_desc = "Build catkin project with specified project_name."
    parser = argparse.ArgumentParser(add_help=True, description="{}".format(basic_desc))
    parser.add_argument("--project_name", help="Name of the project.", default="project_name")
    return parser


if __name__ == '__main__':
    # Parse command line flags
    args = parser().parse_args()
    project_name = args.project_name

    #0. Create clean package
    project_path = os.path.join('./', project_name)
    if not os.path.isdir(project_path):
        print(GREEN + 'Creating new project at: ' + project_path + RESET)
        copytree('.', project_path, ignore=ignore_patterns('*.pyc', '*git*'))
    else:
        ('.', project_path, ignore=ignore_patterns('*.pyc', '*git*'))


    project_properties = {'project_name': project_name}

    #A. Rename package.xml
    configure_file(os.path.join(project_path, 'package.xml'), project_properties)
    #B. Rename CMakeLists.txt
    configure_file(os.path.join(project_path, 'CMakeLists.txt'), project_properties)
    #C. Rename include filesystem
    os.rename(os.path.join(project_path, 'include/project_name/main.h'),
              os.path.join(project_path, 'include/', project_name, project_name + '.h'))
    #D. Rename src internals
    configure_file(os.path.join(project_path, 'src/main.cpp'), project_properties)
    #E. Rename src filesystem
    os.rename(os.path.join(project_path, 'src/main.cpp'),
              os.path.join(project_path, 'src/', project_name + '.cpp'))

    # Finally, rename package path
    os.rename(project_path, './' + project_name)
