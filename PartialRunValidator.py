import json
import csv
import sys
import os

MONO_PATH = "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Mono/brain/day_gsf_app.tmp.c"
FOV_PATH = "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/2FOV/day_gsf_app.tmp.c"
WONO_PATH = "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Wono/brain/day_gsf_app.tmp.c"

TECHS_PATH = "ME_Ex/partialRun/python/pr_techs_list.json"

MONO = "Mono"
FOV = "2FOV"
WONO = "Wono"

WRITE = 'w'
READ = 'r'


def write_seps_from(data):
    header = ["SEP", "Brain Type", "Partial Run Supported", "Partial Run Technology Type",
              "Partial Run Technology Type ID"]
    with open("SEPs.csv", WRITE, newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(data)


def read_brain(techs, brain_path, code_base_seps, brain_type):
    data = []
    used_seps = set()

    with open(brain_path, READ) as file:
        for line in file:
            edited_line = line.rstrip()
            if edited_line and "void SEP" in edited_line:
                sub_line = edited_line[edited_line.find("void SEP"):]
                starting_index = sub_line.index("_") + 1
                ending_index = sub_line.index("(")
                sep_name = sub_line[starting_index:ending_index]
                tech = "N/A"

                supp_stat = False

                if sep_name in code_base_seps.keys():
                    supp_stat = True
                    tech = code_base_seps[sep_name]
                    used_seps.add(sep_name)

                if tech == "N/A":
                    tech = get_tech(sep_name, techs)

                id = get_id(tech, techs)

                data.append([sep_name, brain_type, supp_stat, tech, id])
    return data, used_seps


def get_id(tech, techs):
    id = "N/A"
    if tech in techs:
        id = techs.index(tech)
    return id


def get_tech(sep_name, techs):
    tech = "N/A"
    for technology in techs:
        if technology in sep_name:
            tech = technology
    return tech

def store_seps():
    seps = {}
    for subdir, dirs, files in os.walk(os.getcwd() + os.sep + "ME_Ex"):
        for filename in files:
            if filename.startswith("SEP"):
                with open(subdir + os.sep + filename, READ) as file:
                    current_sep_type = "N/A"
                    inner_seps = []
                    for line in file:
                        edited_line = line.rstrip()
                        if edited_line:
                            if "void SEP" in edited_line:
                                sub_line = edited_line[edited_line.find("void SEP"):]
                                starting_index = sub_line.index("_") + 1
                                ending_index = sub_line.index("(")
                                sep_name = sub_line[starting_index:ending_index]
                                inner_seps.append(sep_name)

                            elif "RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN" in edited_line:
                                sub_line = edited_line[edited_line.find("RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN"):]
                                starting_index = sub_line.index("(") + len("PartialRun::PRTechType::") + 1
                                ending_index = sub_line.index(")")
                                current_sep_type = sub_line[starting_index:ending_index]

                            elif "PartialRun_API::isTechDisabledByPartialRun" in edited_line:
                                sub_line = edited_line[edited_line.find("PartialRun_API::isTechDisabledByPartialRun"):]

                                starting_index = sub_line.index("(") + len("PartialRun::PRTechType::") + 1
                                ending_index = sub_line.index(")")
                                current_sep_type = sub_line[starting_index:ending_index]
                    for sep in inner_seps:
                        seps[sep] = current_sep_type

    return seps


def write_unused_seps(techs, code_base_seps, used_seps):
    data = []
    for sep_name in code_base_seps.keys():
        if sep_name not in used_seps:
            tech = code_base_seps[sep_name]
            if tech == "N/A":
                tech = get_tech(sep_name, techs)
            id = get_id(tech, techs)
            data.append([sep_name, tech, id])
    header = ["SEP", "Partial Run Technology Type", "Partial Run Technology Type ID"]
    with open("unused_SEPs.csv", WRITE, newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(data)


def get_techs():
    with open(TECHS_PATH, READ) as file:
        return json.load(file)["pr_techs"]


if __name__ == '__main__':
    code_base_seps = store_seps()
    techs = get_techs()
    MONO_data, used_seps_1 = read_brain(techs, MONO_PATH, code_base_seps, MONO)
    FOV_data, used_seps_2 = read_brain(techs, FOV_PATH, code_base_seps, FOV)
    WONO_data, used_seps_3 = read_brain(techs, WONO_PATH, code_base_seps, WONO)
    # args = sys.argv[1:]
    write_seps_from(MONO_data + FOV_data + WONO_data)
    used_seps = set.union(set.union(used_seps_1, used_seps_2), used_seps_3)
    write_unused_seps(techs, code_base_seps, used_seps)