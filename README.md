# Summary:
1. Walked through the whole code base, collected every defined SEP using a dictionary that connects SEPs with their technology-type.
2. Stored every possible technology-type in a list.
3. For each brain, stored lists of declared SEPs, whether they are PR_supported, their technology-type, and their techType-Id in a list.
    * If a declared SEP was defined in the code base (apart from the brain itself) we say that it's PR_supported/used.
4. Also, for each brain two sets were used to store the supported SEPs and the unsupported SEPs respectively.
5. Created a csv file that contains the brains SEPs.
6. Created a csv file that conatins all of the unsupported/unused SEPs that are in the code base (apart from the brains).
7. Supported command lines:
    * provided a help command line.
    * provided a list of the supported and unsupported SEPs in the given brain-type(parameter) with the given id(parameter).
---
# Complexity:
1. Having a dictionary of the SEPs and their types allowed us to shuffle a SEP technology-type or id in O(1).
2. Having sets of supported and unsupported SEPs allowed us to check if any SEP is PR_supported in O(1).
* Lists were made when we didn't have an expensive operations, such as searching for an element. In that case, we used set/dictionary.
3. we only opened relevant(SEPs) files when we went through the whole code base. Thus, saved a lot of time.
---
# Implemented Files:
* **PartialRunValidator.py** containts all of the mentioned steps above. Exists in the same folder where ME_Ex exists.
* **SEPs.csv** conatins every available SEP in the three brains. Exists in the same folder where ME_Ex exists.
* **unused_SEPs.csv** containts unsupported & unmentioned SEPs in the code base. Exists in the same folder where ME_Ex exists.
* **README.md** contains description of the process. Exists in the same folder where ME_Ex exists.
----
# Short Answers:
## 1) Paths:

* a. MONO_PATH: *"ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Mono/brain/day_gsf_app.tmp.c"*
* b. 2FOV_PATH: *"ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/2FOV/day_gsf_app.tmp.c"*
* b. 3FOV_PATH: *"ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/3FOV"*
* c. WONO_PATH: *"ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Wono/brain/day_gsf_app.tmp.c"*

## 2) Instances of 'Partial Run Disabled' indicators and 'TECH_TYPES':
* a.
    ````
    Type: OnlineCalibration 
    Indicator: if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::OnlineCalibration))
    ````
* b.
    ````
    Type: WMBC
    Indicator: RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::WMBC)
    ````
* c.
    ````
    Type: TSR
    Indicator: if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::TSR))
    ````
* d.
    ````
    Type: xMotion
    Indicator: RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion)
    ````
    
## 3) Filename:
    SEPs.csv

## 4) Note:
    In case there is no techType or ID for such SEP, N/A was given as value

## 5) Filename:
    unused_SEPs.csv

## 6) Input Format:
    Assume the input is given in the following format: 
    (MONO/WONO/3FOV/2FOV) (ids)
    For example:
        MONO 1 5 2

## 7) Help Command:
    For the input -h or --h, list of all the possible technologies will be printed
