import json
import csv
import sys
import os

EMPTY_STRING = ""
IS_PR_DISABLED2 = "PartialRun_API::isTechDisabledByPartialRun"
TECH_TYPE_FORM = "PartialRun::PRTechType::"
IS_PR_DISABLED1 = "RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN"
UNUSED_SEPS_CSV = "unused_SEPs.csv"
PARTIAL_TECHS = "pr_techs"
HELP2 = "--h"
HELP1 = "-h"
EMPTY_DIR = "FOV3 has no SEPs"
SUPPORTED_SEPS = "SUPPORTED SEPS:"
UNSUPPORTED_SEPS = "UNSUPPORTED SEPS:"
GIVEN_DIR = "ME_Ex"
CLOSE_BRACKET = ")"
OPEN_BRACKET = "("
UNDERSCORE = "_"
N_A = "N/A"
SEP_DECLARATION = "void SEP"
SEP_CSV = "SEPs.csv"
TECHNOLOGY_TYPE_ID = "Partial Run Technology Type ID"
TECHNOLOGY_TYPE = "Partial Run Technology Type"
PARTIAL_RUN_SUPPORTED = "Partial Run Supported"
BRAIN_TYPE = "Brain Type"
SEP = "SEP"

MONO_PATH = "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Mono/brain/day_gsf_app.tmp.c"
FOV_PATH = "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/2FOV/day_gsf_app.tmp.c"
WONO_PATH = "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Wono/brain/day_gsf_app.tmp.c"

TECHS_PATH = "ME_Ex/partialRun/python/pr_techs_list.json"

MONO = "MONO"
FOV2 = "2FOV"
FOV3 = "3FOV"
WONO = "WONO"

WRITE = 'w'
READ = 'r'


def store_seps():
    """
    Extracts every sep in the code base, and its techType.
    :return: Returns a dictionary of {sep : sepTechType}
    """
    seps = {}
    for subdir, dirs, files in os.walk(os.getcwd() + os.sep + GIVEN_DIR):
        for filename in files:
            if filename.startswith(SEP):
                with open(subdir + os.sep + filename, READ) as file:
                    current_sep_type = N_A
                    inner_seps = []
                    for line in file:
                        edited_line = line.rstrip()
                        if edited_line:
                            if SEP_DECLARATION in edited_line:
                                sep_name = extract_word(edited_line, SEP_DECLARATION, UNDERSCORE,
                                                        OPEN_BRACKET, len(EMPTY_STRING))
                                inner_seps.append(sep_name)

                            elif IS_PR_DISABLED1 in edited_line:
                                current_sep_type = extract_word(edited_line, IS_PR_DISABLED1, OPEN_BRACKET,
                                                                CLOSE_BRACKET, len(TECH_TYPE_FORM))

                            elif IS_PR_DISABLED2 in edited_line:
                                current_sep_type = extract_word(edited_line, IS_PR_DISABLED2, OPEN_BRACKET,
                                                                CLOSE_BRACKET, len(TECH_TYPE_FORM))

                    for sep in inner_seps:
                        seps[sep] = current_sep_type
    return seps


def extract_word(string, str_indicator, letter_starter, letter_ender, letters_num_to_skip):
    """
    Extracts a substring from the given string such as
     this substring starts with the letter_starter by skipping (letters_num_to_skip) letters
      and ends with the letter_ender.
    :param string: The string that includes the desired substring
    :param str_indicator: The string that indicates the desired substring
    :param letter_starter: The letter that we starts with
    :param letter_ender: The letter that we stops at
    :param letters_num_to_skip: Number of letters needed to skip
    :return: Returns substring according to the input.
    """
    substring = string[string.find(str_indicator):]
    starting_index = substring.index(letter_starter) + letters_num_to_skip + 1
    ending_index = substring.index(letter_ender)
    return substring[starting_index:ending_index]


def get_techs():
    """
    Gets every optional technology type.
    :return: Returns a list of the partial technologies.
    """
    with open(TECHS_PATH, READ) as file:
        return json.load(file)[PARTIAL_TECHS]


def get_tech(sep_name, techs):
    """
    Gets the tech of the given sep
    :param sep_name: The name of the sep
    :param techs: List of every possible technology type
    :return: Returns the sep technology type. N/A in case none were found.
    """
    tech = N_A
    for technology in techs:
        if technology in sep_name:
            tech = technology
    return tech


def get_id(tech, techs):
    """
    Gets the tech-id based on the tech.
    :param tech: Sep technology
    :param techs: List of every possible technology type
    :return: Returns the partial run sep technology ID. N/A in case none were found.
    """
    id = N_A
    if tech in techs:
        id = techs.index(tech)
    return id


def read_brain(techs, brain_path, code_base_seps, brain_type):
    """
    Reads the given brain file. Finds the declared seps in it, their brainType, technology type, their id,
     and whether the sep is supported or not.
    :param techs: List of every possible technology type
    :param brain_path: Brain file path
    :param code_base_seps: dictionary of every sep in the code base with its tech
    :param brain_type: string that indicates the brainType
    :return: Returns List of (lists)the rows that must be published in the csv.
     Where each row contains: sep_name, brain_type, supported_status, techType, id.
     Also, it returns the supported seps, and the not-supported seps.
    """
    data = []
    used_seps = set()
    unused_seps = set()
    with open(brain_path, READ) as file:
        for line in file:
            edited_line = line.rstrip()
            if edited_line and SEP_DECLARATION in edited_line:
                sep_name = extract_word(edited_line, SEP_DECLARATION, UNDERSCORE,
                                        OPEN_BRACKET, len(EMPTY_STRING))
                tech = N_A
                supp_stat = False

                if sep_name in code_base_seps.keys():
                    supp_stat = True
                    tech = code_base_seps[sep_name]
                    used_seps.add(sep_name)

                else:
                    unused_seps.add(sep_name)

                if tech == N_A:
                    tech = get_tech(sep_name, techs)

                id = get_id(tech, techs)

                data.append([sep_name, brain_type, supp_stat, tech, id])
    return data, used_seps, unused_seps


def write_seps_from(data):
    """
    Writes every available sep in all of the three brains on a csv file.
    :param data: List of (lists)rows - seps to write with their info
    """
    header = [SEP, BRAIN_TYPE, PARTIAL_RUN_SUPPORTED, TECHNOLOGY_TYPE,
              TECHNOLOGY_TYPE_ID]
    with open(SEP_CSV, WRITE, newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(data)


def write_unused_seps(techs, code_base_seps, used_seps):
    """
    Writes the seps that are defined in the code base but wasn't used by any of the brains on a csv file.
     Where every row includes sep_name, techType and techType-id.
    :param techs: List of every possible technology type
    :param code_base_seps: dictionary of every sep in the code base with its tech
    :param used_seps: set of used seps in the three brains.
    """
    data = []
    for sep_name in code_base_seps.keys():
        if sep_name not in used_seps:
            tech = code_base_seps[sep_name]
            if tech == N_A:
                tech = get_tech(sep_name, techs)
            id = get_id(tech, techs)
            data.append([sep_name, tech, id])

    header = [SEP, TECHNOLOGY_TYPE, TECHNOLOGY_TYPE_ID]
    with open(UNUSED_SEPS_CSV, WRITE, newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(data)


def do_command_line(args, techs, MONO_used_seps, FOV_used_seps, WONO_used_seps, MONO_unused_seps,
                    FOV_unused_seps, WONO_unused_seps, code_base_seps):
    """
    If the input is -h or --h, it lists every possible technology with its id.
    If the input is one of the brains(MONO/WONO/3FOV/2FOV) it lists the supported seps
    according to the id input if provided, otherwise without restrictions.
    :param args: input
    :param techs: List of every possible technology type
    :param MONO_used_seps: Set of the supported seps in MONO
    :param FOV_used_seps: Set of the supported seps in 2FOV
    :param WONO_used_seps: Set of the supported seps in WONO
    :param MONO_unused_seps: Set of the unsupported seps in MONO
    :param FOV_unused_seps: Set of the unsupported seps in 2FOV
    :param WONO_unused_seps: Set of the unsupported seps in WONO
    :param code_base_seps: dictionary of every sep in the code base with its tech
    """
    if len(args) == 1 and (args[0] == HELP1 or args[0] == HELP2):
        for tech in techs:
            print(get_id(tech, techs), tech)

    else:
        ids = {get_id(tech, techs) for tech in techs}
        if len(args) > 1:
            ids = {int(arg) for arg in args[1:]}

        if args[0] == MONO:
            list_seps(MONO_used_seps, MONO_unused_seps, ids, code_base_seps, techs)

        elif args[0] == FOV2:
            list_seps(FOV_used_seps, FOV_unused_seps, ids, code_base_seps, techs)

        elif args[0] == FOV3:
            print(EMPTY_DIR)

        elif args[0] == WONO:
            list_seps(WONO_used_seps, WONO_unused_seps, ids, code_base_seps, techs)


def list_seps(used_seps, unused_seps, ids, code_base_seps, techs):
    """
    Lists supported seps and unsupported seps that matches the given ids
    :param used_seps: Set of the supported seps
    :param unused_seps: Set of the unsupported seps
    :param ids: set of ids
    :param code_base_seps: dictionary of every sep in the code base with its tech
    :param techs: List of every possible technology type
    :return:
    """
    print(SUPPORTED_SEPS)
    print()
    for sep in used_seps:
        if get_id(code_base_seps[sep], techs) in ids:
            print(sep)
    print()
    print(UNSUPPORTED_SEPS)
    print()
    for sep in unused_seps:
        tech = get_tech(sep, techs)
        if get_id(tech, techs) in ids:
            print(sep)


if __name__ == '__main__':
    code_base_seps = store_seps()
    techs = get_techs()

    MONO_data, MONO_used_seps, MONO_unused_seps = read_brain(techs, MONO_PATH, code_base_seps, MONO)
    FOV_data, FOV_used_seps, FOV_unused_seps = read_brain(techs, FOV_PATH, code_base_seps, FOV2)
    WONO_data, WONO_used_seps, WONO_unused_seps = read_brain(techs, WONO_PATH, code_base_seps, WONO)

    write_seps_from(MONO_data + FOV_data + WONO_data)

    used_seps = set.union(set.union(MONO_used_seps, FOV_used_seps), WONO_used_seps)
    write_unused_seps(techs, code_base_seps, used_seps)

    args = sys.argv[1:]
    do_command_line(args, techs, MONO_used_seps, FOV_used_seps, WONO_used_seps, MONO_unused_seps,
                    FOV_unused_seps, WONO_unused_seps, code_base_seps)
