import json
import csv
import sys
import os

MONO_PATH = "out/EyeQ4sw_rel/projects/EyeQ/Mono/brain/day_gsf_app.tmp.c"
FOV_PATH = "out/EyeQ4sw_rel/projects/EyeQ/2FOV/day_gsf_app.tmp.c"
WONO_PATH = "out/EyeQ4sw_rel/projects/EyeQ/Wono/brain/day_gsf_app.tmp.c"

TECHS_PATH = "partialRun/python/pr_techs_list.json"

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


def read_brain(brain_path, brain_type):
    data = []
    techs = []
    with open(TECHS_PATH, READ) as file:
        techs = json.load(file)["pr_techs"]

    with open(brain_path, READ) as file:
        for line in file:
            edited_line = line.rstrip()
            if edited_line and "void SEP" in edited_line:
                starting_index = edited_line.index("_") + 1
                ending_index = edited_line.index("(")
                sep_name = edited_line[starting_index:ending_index]
                tech = "UNKNOWN"
                for technology in techs:
                    if technology in sep_name:
                        tech = technology
                        break
                data.append([sep_name, brain_type, "UNKNOWN", tech, "UNKNOWN"])
    return data


def store_seps():
    directories = []
    for filename in os.listdir(os.getcwd()):
        if os.path.isdir(os.getcwd() + os.sep + filename):
            directories.append(filename)
    print(directories)
    # for dr in directories:
        # for subdir, dirs, files in os.walk(os.getcwd() + os.sep + dr):
        #     for filename in files:
        #         if filename.startswith("SEP"):
        #             print(subdir + os.sep + filename)
    # with open(brain_path, READ) as file:
    #     for line in file:
    #         edited_line = line.rstrip()
    #         if edited_line and "void SEP" in edited_line:
    #             starting_index = edited_line.index("_") + 1
    #             ending_index = edited_line.index("(")
    #             sep_name = edited_line[starting_index:ending_index]


if __name__ == '__main__':
    seps = store_seps()
    # MONO_data = read_brain(MONO_PATH, MONO)
    # FOV_data = read_brain(FOV_PATH, FOV)
    # WONO_data = read_brain(WONO_PATH, WONO)
    # args = sys.argv[1:]
    # write_data(MONO_data + FOV_data + WONO_data)