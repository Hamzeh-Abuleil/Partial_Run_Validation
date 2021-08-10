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


def write_data(data):
    header = ["SEP", "Brain Type", "Partial Run Supported", "Partial Run Technology Type",
              "Partial Run Technology Type ID"]
    # data = [["S1", "MONO", "True", "UNKNOWN", "21"], ["S3", "FOV", "False", "UNKNOWN", "11"]]
    with open("SEPs.csv", WRITE, newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(data)


def read_brain(brain_path, seps, brain_type):
    data = []
    techs = []
    with open(TECHS_PATH, READ) as file:
        techs = json.load(file)["pr_techs"]

    with open(brain_path, READ) as file:
        for line in file:
            edited_line = line.rstrip()
            if edited_line and "void SEP" in edited_line:
                sub_line = edited_line[edited_line.find("void SEP"):]
                starting_index = sub_line.index("_") + 1
                ending_index = sub_line.index("(")
                sep_name = sub_line[starting_index:ending_index]
                tech = "UNKNOWN"

                supp_stat = False

                if sep_name in seps.keys():
                    supp_stat = True
                    tech = seps[sep_name]

                if tech == "UNKNOWN":
                    for technology in techs:
                        if technology in sep_name:
                            tech = technology
                            break

                data.append([sep_name, brain_type, supp_stat, tech, "UNKNOWN"])
    return data


def store_seps():
    seps = {}
    for subdir, dirs, files in os.walk(os.getcwd() + os.sep + "ME_Ex"):
        for filename in files:
            if filename.startswith("SEP"):
                with open(subdir + os.sep + filename, READ) as file:
                    current_sep_type = "UNKNOWN"
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


if __name__ == '__main__':
    seps = store_seps()
    MONO_data = read_brain(MONO_PATH, seps, MONO)
    FOV_data = read_brain(FOV_PATH, seps, FOV)
    WONO_data = read_brain(WONO_PATH, seps, WONO)
    # args = sys.argv[1:]
    write_data(MONO_data + FOV_data + WONO_data)