# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import os
from colorama import Fore, Back, Style


def error(message):
    print(Style.BRIGHT + Fore.RED + message + Style.RESET_ALL)


def bright(message):
    print("")
    print(Style.BRIGHT + message + Style.RESET_ALL)


def run_or_fail(cmd):
    if os.system(cmd) != 0:
        die("Error while running " + cmd)


def emphasis(text):
    return Fore.BLUE + text + Fore.RESET


def yellow(text):
    return Fore.YELLOW + text + Fore.RESET


def red(text):
    return Fore.RED + text + Fore.RESET


def success(text):
    return Fore.GREEN + text + Fore.RESET


def die(message):
    print(error(message))
    exit()

def print_parameter(name, value, infos = None, warning=None):
    result = f"- {success(name)}: {yellow(str(value))}"

    if infos is not None:
        result += f" ({infos})"
    if warning is not None:
        result += red(f" ({warning})") 
    
    print(result)

