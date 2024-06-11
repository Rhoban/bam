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

