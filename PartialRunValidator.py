import json
import csv
import sys
import os

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
SEP_CV = "SEPs.csv"
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


def write_seps_from(data):
    header = [SEP, BRAIN_TYPE, PARTIAL_RUN_SUPPORTED, TECHNOLOGY_TYPE,
              TECHNOLOGY_TYPE_ID]
    with open(SEP_CV, WRITE, newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(data)


def read_brain(techs, brain_path, code_base_seps, brain_type):
    data = []
    used_seps = set()
    unused_seps = set()
    with open(brain_path, READ) as file:
        for line in file:
            edited_line = line.rstrip()
            if edited_line and SEP_DECLARATION in edited_line:
                sub_line = edited_line[edited_line.find(SEP_DECLARATION):]
                starting_index = sub_line.index(UNDERSCORE) + 1
                ending_index = sub_line.index(OPEN_BRACKET)
                sep_name = sub_line[starting_index:ending_index]

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


def get_id(tech, techs):
    id = N_A
    if tech in techs:
        id = techs.index(tech)
    return id


def get_tech(sep_name, techs):
    tech = N_A
    for technology in techs:
        if technology in sep_name:
            tech = technology
    return tech


def store_seps():
    """

    :return:
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
                                sub_line = edited_line[edited_line.find(SEP_DECLARATION):]
                                starting_index = sub_line.index(UNDERSCORE) + 1
                                ending_index = sub_line.index(OPEN_BRACKET)
                                sep_name = sub_line[starting_index:ending_index]
                                inner_seps.append(sep_name)

                            elif IS_PR_DISABLED1 in edited_line:
                                sub_line = edited_line[edited_line.find(IS_PR_DISABLED1):]
                                starting_index = sub_line.index(OPEN_BRACKET) + len(TECH_TYPE_FORM) + 1
                                ending_index = sub_line.index(CLOSE_BRACKET)
                                current_sep_type = sub_line[starting_index:ending_index]

                            elif IS_PR_DISABLED2 in edited_line:
                                sub_line = edited_line[edited_line.find(IS_PR_DISABLED2):]

                                starting_index = sub_line.index(OPEN_BRACKET) + len(TECH_TYPE_FORM) + 1
                                ending_index = sub_line.index(CLOSE_BRACKET)
                                current_sep_type = sub_line[starting_index:ending_index]
                    for sep in inner_seps:
                        seps[sep] = current_sep_type

    return seps


def write_unused_seps(techs, code_base_seps, used_seps):
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


def get_techs():
    with open(TECHS_PATH, READ) as file:
        return json.load(file)[PARTIAL_TECHS]


def do_command_line(args, techs, MONO_used_seps, FOV_used_seps, WONO_used_seps, MONO_unused_seps,
                    FOV_unused_seps, WONO_unused_seps, code_base_seps):
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

