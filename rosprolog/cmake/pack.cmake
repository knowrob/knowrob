
macro(find_prolog_pack PACK_NAME PACK_FOUND)
    # check if pack is installed
    set(PL_BIN "prolog")
    set(PL_COMMAND "ignore(use_module(library(${PACK_NAME}))),halt.")
    execute_process(
         COMMAND ${PL_BIN} -g "${PL_COMMAND}"
         ERROR_VARIABLE PL_STDERR
         RESULT_VARIABLE PL_RESULT)
    if(PL_STDERR OR NOT PL_RESULT EQUAL "0")
      set(PACK_FOUND 0)
    else()
      set(PACK_FOUND 1)
    endif()
endmacro()

macro(install_prolog_pack PACK_NAME)
    find_prolog_pack(${PACK_NAME} PACK_FOUND)
    if(${PACK_FOUND} EQUAL 0)
        set(PACK_REQUIRED_KEY PACK_${PACK_NAME}_REQUIRED)
        cmake_parse_arguments(
        PACK_${PACK_NAME}
            "REQUIRED" "URL" ""
            ${ARGN}
        )
        # collect args
        set(PACK_ARGS "interactive(false)")
        if(${PACK_URL_KEY})
          set(PACK_ARGS "url('${${PACK_URL_KEY}}'),${PACK_ARGS}")
        endif()
        # call pack_install
        set(PL_BIN "prolog")
        set(PL_COMMAND "use_module(library(prolog_pack))")
        set(PL_COMMAND "${PL_COMMAND},set_setting(prolog_pack:server,'https://www.swi-prolog.org/pack/')")
        set(PL_COMMAND "${PL_COMMAND},ignore(pack_install(${PACK_NAME},[${PACK_ARGS}]))")
        set(PL_COMMAND "${PL_COMMAND},halt.")
        execute_process(
             COMMAND ${PL_BIN} -g "${PL_COMMAND}"
             ERROR_VARIABLE PL_STDERR
             OUTPUT_VARIABLE PL_STDOUT
             RESULT_VARIABLE PL_RESULT)
        if(PL_STDERR OR NOT PL_RESULT EQUAL "0")
          set(PACK_MSG "Unable to locate Prolog pack '${PACK_NAME}'.")
          if(${PACK_REQUIRED_KEY})
            message(SEND_ERROR ${PACK_MSG})
          else()
            message(STATUS ${PACK_MSG})
          endif()
        endif()
    endif()
endmacro()
