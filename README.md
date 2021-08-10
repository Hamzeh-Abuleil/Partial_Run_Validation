# Partial_Run_Validation, 
#design 
#implementation details
#complexity and space considerations
#testing considerations
#challenges/issues/possible solutions/other possible designs pros and cons


1) Paths:
#a. MONO_PATH: "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Mono/brain/day_gsf_app.tmp.c"
#b. 2FOV_PATH: "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/2FOV/day_gsf_app.tmp.c"
#b. 3FOV_PATH: "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/3FOV"
#c. WONO_PATH: "ME_Ex/out/EyeQ4sw_rel/projects/EyeQ/Wono/brain/day_gsf_app.tmp.c"

#2) instances of 'Partial Run Disabled' indicators and 'TECH_TYPES':
#a. 
    type: OnlineCalibration 
    indicator: if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::OnlineCalibration))
#b. 
    type: WMBC
    indicator: RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::WMBC)
#c.
    type: TSR
    indicator: if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::TSR))
#d.
    type: xMotion
    indicator: RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion)

#3) SEPs.csv

#4) in case there is no techType or ID for the current SEP, N/A was given as value

#5) unused_SEPs.csv

