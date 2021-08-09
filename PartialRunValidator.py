import json
import csv
import sys
 # "ME_Ex/partialRun/partialRun_API.h"

MONO = "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Mono/brain/day_gsf_app.tmp.c"
FOV = "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/2FOV/day_gsf_app.tmp.c"
WONO = "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Wono/brain/day_gsf_app.tmp.c"

WRITE = 'w'
READ = 'r'


def write_data():
    header = ["SEP", "Brain Type", "Partial Run Supported", "Partial Run Technology Type",
              "Partial Run Technology Type ID"]
    data = [["S1", "MONO", "True", "UNKNOWN", "21"], ["S3", "FOV", "False", "UNKNOWN", "11"]]
    with open("SEPs.csv", WRITE, newline='') as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(data)


def read_brain(brainType):
    with open(brainType, READ) as file:
        lines = [line.rstrip() for line in file]
        return [line for line in lines if line]
    # RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::<TECH_NAME>)
    # PartialRun_API::isTechDisabledByPartialRun(techType)


if __name__ == '__main__':
    MONO_data = read_brain(MONO)
    FOV_data = read_brain(FOV)
    WONO_data = read_brain(WONO)
    print(MONO_data[0:30])
    args = sys.argv[1:]
    write_data()