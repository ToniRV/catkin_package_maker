#node fileso!/usr/bin/env python

import sys
import os
import re
import argparse
from shutil import copytree, rmtree, ignore_patterns

# Group of Different functions for different styles
if sys.platform.lower() == "win32":
    os.system('color')
RED = '\033[31m'
GREEN = '\033[32m'
RESET = '\033[0m'

# Python 2/3 compatibility
try:
    input = raw_input
except NameError:
    pass


def prompt_val(msg="enter a value:"):
    return input(msg + "\n")


def confirm(msg="enter 'y' to confirm or any other key to cancel", key='y'):
    if input(msg + "\n") != key:
        return False
    else:
        return True


def check_and_confirm_overwrite(file_path):
    if os.path.isfile(file_path):
        logger.warning(file_path + " exists, overwrite?")
        return confirm("enter 'y' to overwrite or any other key to cancel")
    else:
        return True


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


def write_file(filepath, contents):
    dirname = os.path.dirname(filepath)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    with open(filepath, 'w') as fh:
        fh.write(contents.encode())
        print('Created file %s' % os.path.relpath(filepath))


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
    ignore = ignore_patterns('*.pyc', '*git*', 'make_package.py', 'CATKIN_IGNORE')
    if not os.path.isdir(project_path):
        print(GREEN + 'Creating new project: ' + project_name + RESET)
        copytree('./template', project_path, ignore=ignore)
    else:
        if check_and_confirm_overwrite(project_path):
            print(GREEN + 'Overwritting project: ' + project_name + RESET)
            rmtree(project_path)
            copytree('./template', project_path, ignore=ignore)

    project_properties = {'project_name': project_name}

    #A. Rename package.xml
    package_xml_path = os.path.join(project_path, 'package.xml')
    write_file(package_xml_path, configure_file(package_xml_path, project_properties))

    #B. Rename CMakeLists.txt
    cmakelists_path = os.path.join(project_path, 'CMakeLists.txt')
    write_file(cmakelists_path, configure_file(cmakelists_path, project_properties))

    #C. Rename include filesystem
    os.rename(os.path.join(project_path, 'include/project_name'),
              os.path.join(project_path, 'include/', project_name))
    os.rename(os.path.join(project_path, 'include/', project_name, 'main.h'),
              os.path.join(project_path, 'include/', project_name, project_name + '.h'))

    #D. Rename src internals
    main_path = os.path.join(project_path, 'src/main.cpp')
    new_main_path = os.path.join(project_path, 'src/', project_name + '.cpp')
    write_file(new_main_path, configure_file(main_path, project_properties))
    os.remove(main_path)

    #E. Rename launch filesystem with project_name
    launch_path = os.path.join(project_path, 'launch/', project_name + '.launch')
    os.rename(os.path.join(project_path, 'launch/project_name.launch'), launch_path)

    #F. Update launch file with project_name
    write_file(launch_path, configure_file(launch_path, project_properties))

    #G. Rename install filesystem with project_name
    os.rename(os.path.join(project_path, 'install/template.rosinstall'),
              os.path.join(project_path, 'install/', project_name + '.rosinstall'))

    #H. Update readme with project_name
    readme_path = os.path.join(project_path, 'README.md')
    write_file(readme_path, configure_file(readme_path, project_properties))

    # Python related renamings
    #A. Update setup.py with project_name
    setup_py_path = os.path.join(project_path, 'setup.py')
    write_file(setup_py_path, configure_file(setup_py_path, project_properties))

    #B. Rename src internals
    os.rename(os.path.join(project_path, 'src/project_name'),
              os.path.join(project_path, 'src/', project_name))

    #C. Rename scripts internals
    project_name_script_path = os.path.join(project_path, 'scripts/project_name')
    write_file(project_name_script_path,
               configure_file(project_name_script_path, project_properties))
    ## We add `_script` below to make sure the name does not collide with the cpp node name.
    ## otw calling `rosrun project_name project_name` would be ambiguous (python or cpp?).
    os.rename(os.path.join(project_path, 'scripts/project_name'),
              os.path.join(project_path, 'scripts/', project_name, '_script'))

    # Finally, rename package path
    os.rename(project_path, './' + project_name)
